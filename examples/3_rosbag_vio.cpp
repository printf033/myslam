/**
 * @file 3_rosbag_vio.cpp
 * @brief 在 rosbag2（.db3）数据集上回放 myslam VIO，接通通用回放管道。
 *
 * 用法：
 * @code
 * 3_rosbag_vio <bag.db3|目录> [选项]
 *    --topic NAME        指定图像 topic（默认自动选 left 视觉相机）
 *    --config FILE       Lua 配置（默认 config/config.lua，不存在则用默认值）
 *    --out FILE.tum      关键帧轨迹输出（TUM 格式）
 *    --start-frame N     从第 N 帧开始
 *    --max-frames N      最多处理 N 帧
 *    --stride N          每 N 帧取一帧
 *    --max-features N    每帧特征数上限
 *    --max-duration SEC  最长回放时长
 *    --fx --fy --cx --cy 覆盖相机内参（无 camera_info 时使用）
 * @endcode
 *
 * 本程序只做「数据源适配」：把 RosBagSource 包装成 @c IReplaySource，其余
 * 时间同步、断档重建、特征提取、估计器装配全部复用 @c ReplayRunner。因此
 * bag 与 SubT 序列走的是同一条回放管道，行为差异只来自数据本身。
 *
 * 内参优先级：命令行覆盖 > bag 内 camera_info > Lua 配置。bag 图像若是
 * 1920x1440 且无 camera_info（如 07_16 / 07_24 系列），必须显式给出内参，
 * 否则回放器会因图像尺寸与标定不符而终止。
 */

#include "app/replay.hpp"
#include "config/lua_config.hpp"
#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include "sensor/rosbag2_source.hpp"
#include "viz/replay_observer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace
{

    using myslam::dataset::ImageFrame;
    using myslam::dataset::RosBagSource;

    void printUsage()
    {
        std::fprintf(stderr, "用法: 3_rosbag_vio <bag.db3|目录> [选项]\n"
                             "  --topic NAME        指定图像 topic（默认自动选 left 视觉相机）\n"
                             "  --config FILE       Lua 配置（默认 config/config.lua）\n"
                             "  --out FILE.tum      轨迹输出\n"
                             "  --start-frame N     起始帧\n"
                             "  --max-frames N      最多处理帧数\n"
                             "  --stride N          帧步长\n"
                             "  --max-features N    每帧特征数上限\n"
                             "  --max-duration SEC  最长回放时长\n"
                             "  --fx --fy --cx --cy 覆盖内参（按针孔模型解释）\n"
                             "  --show / --no-show  强制开/关实时窗口（默认读取 Lua dataset.viz）\n"
                             "  --video FILE        录像输出（如 out.avi / out.mp4）\n"
                             "  --video-fps N       录像帧率（默认跟随相机）\n"
                             "  --playback-rate R   播放倍速（窗口打开时默认 1 = 实时）\n");
    }

    /// 把一帧 bag 图像转成 8 位灰度图（压缩流走 imdecode，原始图按编码转换）
    cv::Mat toGray(const ImageFrame &frame)
    {
        if (frame.data.empty())
        {
            return {};
        }
        auto *raw = const_cast<std::uint8_t *>(frame.data.data());
        if (frame.compressed)
        {
            const cv::Mat encoded(1, static_cast<int>(frame.data.size()), CV_8U, raw);
            return cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
        }
        if (frame.encoding == "mono8" || frame.encoding == "8UC1")
        {
            return cv::Mat(frame.height, frame.width, CV_8UC1, raw, frame.step).clone();
        }
        if (frame.encoding == "bgr8")
        {
            const cv::Mat color(frame.height, frame.width, CV_8UC3, raw, frame.step);
            cv::Mat gray;
            cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
            return gray;
        }
        if (frame.encoding == "rgb8")
        {
            const cv::Mat color(frame.height, frame.width, CV_8UC3, raw, frame.step);
            cv::Mat gray;
            cv::cvtColor(color, gray, cv::COLOR_RGB2GRAY);
            return gray;
        }
        return {};
    }

    /**
     * @brief 把 RosBagSource 适配成回放管道的顺序数据源
     *
     * 时间戳在构造时一次性取出（回放器需要完整的 stamps 列表），IMU 直接
     * 转发 bag 已解析好的记录，图像按序号惰性解码——不把 15GB 的图全部读进内存。
     */
    class RosBagReplaySource final : public myslam::dataset::IReplaySource
    {
    public:
        RosBagReplaySource(const RosBagSource &bag, std::string topic)
            : bag_(bag),
              topic_(std::move(topic)),
              stamps_(bag_.imageStamps(topic_))
        {
        }

        [[nodiscard]] const std::vector<double> &stamps() const override { return stamps_; }

        [[nodiscard]] const std::vector<myslam::dataset::ImuRecord> &imu() const override { return bag_.imu(); }

        [[nodiscard]] cv::Mat loadGray(std::size_t frameIndex) const override
        {
            const auto frame = bag_.imageAt(topic_, frameIndex);
            return frame ? toGray(*frame) : cv::Mat{};
        }

        [[nodiscard]] const std::string &topic() const { return topic_; }

    private:
        const RosBagSource &bag_;
        std::string topic_;
        std::vector<double> stamps_;
    };

    /// 自动挑选主视觉相机：left 优先于 raw，raw 优先于压缩
    std::string pickImageTopic(const RosBagSource &bag)
    {
        const std::string *best = nullptr;
        int bestRank = -1;
        for (const auto &t : bag.topics())
        {
            const bool raw = t.type == "sensor_msgs/msg/Image";
            const bool comp = t.type == "sensor_msgs/msg/CompressedImage";
            if (!raw && !comp)
            {
                continue;
            }
            const bool left = t.name.find("left") != std::string::npos;
            const int rank = (left ? 2 : 0) + (raw ? 1 : 0);
            if (rank > bestRank)
            {
                bestRank = rank;
                best = &t.name;
            }
        }
        return best != nullptr ? *best : std::string{};
    }

    /// 为图像 topic 找同名相机内参：替换最后一段为 camera_info，否则取第一个
    std::string findCameraInfoTopic(const RosBagSource &bag, const std::string &imageTopic)
    {
        const auto slash = imageTopic.rfind('/');
        if (slash != std::string::npos)
        {
            const std::string candidate = imageTopic.substr(0, slash) + "/camera_info";
            if (bag.cameraInfo(candidate))
            {
                return candidate;
            }
        }
        const auto topics = bag.cameraInfoTopics();
        return topics.empty() ? std::string{} : topics.front();
    }

    bool parseDouble(const char *text, double &out)
    {
        char *end = nullptr;
        out = std::strtod(text, &end);
        return end != nullptr && *end == '\0' && std::isfinite(out);
    }

    bool parseInt(const char *text, int &out)
    {
        char *end = nullptr;
        const long value = std::strtol(text, &end, 10);
        if (end == nullptr || *end != '\0')
        {
            return false;
        }
        out = static_cast<int>(value);
        return true;
    }

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printUsage();
        return 2;
    }

    std::string bagPath;
    std::string topic;
    std::filesystem::path configPath = "config/config.lua";
    std::filesystem::path outTum;
    int startFrame = -1;
    int maxFrames = -1;
    int stride = -1;
    int maxFeatures = -1;
    double maxDuration = -1.0;
    std::optional<double> fx;
    std::optional<double> fy;
    std::optional<double> cx;
    std::optional<double> cy;
    std::optional<bool> forceShow;
    std::filesystem::path videoPath;
    int videoFps = -1;
    double playbackRate = -1.0;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        auto needValue = [&](const char *name) -> const char *
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "%s 缺少参数值\n", name);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h")
        {
            printUsage();
            return 0;
        }
        else if (arg == "--topic")
        {
            topic = needValue("--topic");
        }
        else if (arg == "--config")
        {
            configPath = needValue("--config");
        }
        else if (arg == "--out")
        {
            outTum = needValue("--out");
        }
        else if (arg == "--start-frame")
        {
            (void)parseInt(needValue("--start-frame"), startFrame);
        }
        else if (arg == "--max-frames")
        {
            (void)parseInt(needValue("--max-frames"), maxFrames);
        }
        else if (arg == "--stride")
        {
            (void)parseInt(needValue("--stride"), stride);
        }
        else if (arg == "--max-features")
        {
            (void)parseInt(needValue("--max-features"), maxFeatures);
        }
        else if (arg == "--max-duration")
        {
            (void)parseDouble(needValue("--max-duration"), maxDuration);
        }
        else if (arg == "--fx")
        {
            double v = 0.0;
            if (parseDouble(needValue("--fx"), v))
            {
                fx = v;
            }
        }
        else if (arg == "--fy")
        {
            double v = 0.0;
            if (parseDouble(needValue("--fy"), v))
            {
                fy = v;
            }
        }
        else if (arg == "--cx")
        {
            double v = 0.0;
            if (parseDouble(needValue("--cx"), v))
            {
                cx = v;
            }
        }
        else if (arg == "--cy")
        {
            double v = 0.0;
            if (parseDouble(needValue("--cy"), v))
            {
                cy = v;
            }
        }
        else if (arg == "--show")
        {
            forceShow = true;
        }
        else if (arg == "--no-show")
        {
            forceShow = false;
        }
        else if (arg == "--video")
        {
            videoPath = needValue("--video");
        }
        else if (arg == "--video-fps")
        {
            (void)parseInt(needValue("--video-fps"), videoFps);
        }
        else if (arg == "--playback-rate")
        {
            double v = 0.0;
            if (parseDouble(needValue("--playback-rate"), v))
            {
                playbackRate = v;
            }
        }
        else if (!arg.empty() && arg[0] != '-' && bagPath.empty())
        {
            bagPath = arg;
        }
        else
        {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            printUsage();
            return 2;
        }
    }

    if (bagPath.empty())
    {
        std::fprintf(stderr, "[error] 未给出 bag 路径\n");
        printUsage();
        return 2;
    }

    // ---------------------------------------------------------------------
    // Lua 配置：提供 vio 参数、外参、回放默认值与内参兜底
    // 不配置日志后端，保持 mylog 默认的 stdout 输出，便于直接观察效果。
    // ---------------------------------------------------------------------
    myslam::config::ProjectConfig project;
    if (std::filesystem::is_regular_file(configPath))
    {
        if (const auto loaded = myslam::config::load(configPath, project, /*applyLogging=*/false); !loaded)
        {
            std::fprintf(stderr, "[warning] 配置加载失败（%s），使用默认值\n", loaded.error().message.c_str());
        }
    }
    else
    {
        std::fprintf(stderr, "[info] 未找到 %s，使用内置默认配置\n", configPath.string().c_str());
    }

    // ---------------------------------------------------------------------
    // 打开 bag 并选图像 topic
    // ---------------------------------------------------------------------
    RosBagSource bag(bagPath);
    if (const auto rc = bag.load(); !rc)
    {
        std::fprintf(stderr, "[error] %s\n", rc.error().message.c_str());
        return 1;
    }
    if (bag.imu().empty())
    {
        std::fprintf(stderr, "[error] bag 里没有可用的 ins_data（IMU）\n");
        return 1;
    }

    if (topic.empty())
    {
        topic = pickImageTopic(bag);
    }
    if (topic.empty() || bag.imageCount(topic) == 0)
    {
        std::fprintf(stderr, "[error] 找不到可回放的图像 topic（可用 --topic 指定）\n");
        return 1;
    }

    // ---------------------------------------------------------------------
    // 内参：命令行覆盖 > camera_info > Lua 配置
    // ---------------------------------------------------------------------
    myslam::camera::CameraIntrinsics intrinsics = project.intrinsics;
    std::string intrinsicsSource = "配置 dataset.intrinsics";

    const std::string infoTopic = findCameraInfoTopic(bag, topic);
    const auto cameraInfo = infoTopic.empty() ? std::nullopt : bag.cameraInfo(infoTopic);
    if (cameraInfo)
    {
        intrinsics.model = myslam::camera::CameraModel::PINHOLE;
        intrinsics.fx = cameraInfo->k[0];
        intrinsics.fy = cameraInfo->k[4];
        intrinsics.cx = cameraInfo->k[2];
        intrinsics.cy = cameraInfo->k[5];
        intrinsics.imageWidth = cameraInfo->width;
        intrinsics.imageHeight = cameraInfo->height;
        if (cameraInfo->distortion.size() >= 5)
        {
            intrinsics.k1 = cameraInfo->distortion[0];
            intrinsics.k2 = cameraInfo->distortion[1];
            intrinsics.p1 = cameraInfo->distortion[2];
            intrinsics.p2 = cameraInfo->distortion[3];
        }
        intrinsicsSource = std::format("bag {}", infoTopic);
    }

    if (fx)
    {
        intrinsics.fx = *fx;
        intrinsics.fy = fy.value_or(*fx);
        // 命令行只能给 fx/fy/cx/cy，无法给出 xi/k1/k2；按针孔模型解释，
        // 否则会拿 Lua 里 MEI 的球面参数去反解一个普通相机。
        intrinsics.model = myslam::camera::CameraModel::PINHOLE;
        intrinsics.xi = 0.0;
        intrinsics.k1 = 0.0;
        intrinsics.k2 = 0.0;
        intrinsics.p1 = 0.0;
        intrinsics.p2 = 0.0;
        intrinsics.pinholeScale = 1.0;
        intrinsicsSource = "命令行覆盖";
    }
    if (cx)
    {
        intrinsics.cx = *cx;
    }
    if (cy)
    {
        intrinsics.cy = *cy;
    }

    // 图像尺寸是数据的属性，不是标定文件的属性：没有 camera_info 时从首帧
    // 读真实尺寸，否则会用 Lua 里的 640x480 去卡 1920x1440 的图像并误报终止。
    if (const auto firstFrame = bag.imageAt(topic, 0); firstFrame)
    {
        const cv::Mat probe = toGray(*firstFrame);
        if (!probe.empty() && (probe.cols != intrinsics.imageWidth || probe.rows != intrinsics.imageHeight))
        {
            intrinsics.imageWidth = probe.cols;
            intrinsics.imageHeight = probe.rows;
        }
    }

    if (!cameraInfo && !fx)
    {
        std::fprintf(stderr,
                     "[warning] bag 无 camera_info，内参来自 Lua 配置（model=%s）。\n"
                     "          若该相机并非对应模型，请用 --fx/--fy/--cx/--cy 显式给出针孔标定。\n",
                     myslam::camera::cameraModelName(intrinsics.model).data());
    }
    if (!intrinsics.isValid())
    {
        std::fprintf(stderr,
                     "[error] 相机内参无效（fx=%.1f fy=%.1f %dx%d）。\n"
                     "        bag 没有 camera_info，请用 --fx/--fy/--cx/--cy 提供标定。\n",
                     intrinsics.fx, intrinsics.fy, intrinsics.imageWidth, intrinsics.imageHeight);
        return 2;
    }
    (void)intrinsics.normalizeForModel();

    // ---------------------------------------------------------------------
    // 回放配置
    // ---------------------------------------------------------------------
    const double imuRate = bag.estimateImuRate();
    myslam::domain::PipelineConfig vioConfig = project.vio;
    vioConfig.camera.intrinsics = intrinsics;
    if (imuRate > 0.0)
    {
        vioConfig.imu.rateHz = imuRate;
    }
    if (project.input.hasExtrinsic)
    {
        vioConfig.camera.bodyPSensor = project.input.extrinsic;
    }

    // 回放节里与「具体序列」绑定的字段不能继承 SubT 的配置：
    // SubT 的 startFrame=2300 在只有 1149 帧的 bag 上会让循环一次都不执行；
    // outputTum / plyPath 也是 SubT 的输出位置，不能默认沿用。
    myslam::dataset::DatasetRunConfig runConfig = project.replay;
    runConfig.outputTum = outTum; // 空则回放器不写轨迹
    runConfig.plyPath.clear();
    runConfig.startFrame = startFrame >= 0 ? startFrame : 0;
    runConfig.maxFrames = maxFrames >= 0 ? maxFrames : 0;
    runConfig.maxDuration = maxDuration >= 0.0 ? maxDuration : 0.0;
    if (!runConfig.outputTum.empty())
    {
        const auto parent = runConfig.outputTum.parent_path();
        if (!parent.empty())
        {
            std::error_code ec;
            std::filesystem::create_directories(parent, ec);
        }
    }
    if (stride > 0)
    {
        runConfig.stride = stride;
    }
    if (maxFeatures > 0)
    {
        runConfig.maxFeatures = maxFeatures;
    }

    // 可视化：默认沿用 Lua 的 dataset.viz（config.lua 里 showWindow = true），
    // 命令行可显式开/关。showWindow 打开且未指定播放倍速时按实时播放——
    // 默认 0（不限速）会让窗口一闪而过，看不到东西。
    if (forceShow)
    {
        runConfig.viz.showWindow = *forceShow;
    }
    if (!videoPath.empty())
    {
        runConfig.viz.videoPath = videoPath;
    }
    if (videoFps >= 0)
    {
        runConfig.viz.videoFps = videoFps;
    }
    if (playbackRate >= 0.0)
    {
        runConfig.viz.playbackRate = playbackRate;
    }
    if (runConfig.viz.showWindow && runConfig.viz.playbackRate <= 0.0)
    {
        runConfig.viz.playbackRate = 1.0;
    }
    if (runConfig.viz.videoFps <= 0)
    {
        const auto stamps = bag.imageStamps(topic);
        const double span = stamps.size() >= 2 ? stamps.back() - stamps.front() : 0.0;
        const double rate = span > 0.0 ? static_cast<double>(stamps.size() - 1) / span : 0.0;
        if (rate > 0.0)
        {
            runConfig.viz.videoFps = static_cast<int>(std::lround(rate));
        }
    }

    std::printf("bag          : %s\n", bagPath.c_str());
    std::printf("图像 topic   : %s (%zu 帧)\n", topic.c_str(), bag.imageCount(topic));
    std::printf("IMU          : %zu 条, %.1f Hz\n", bag.imu().size(), imuRate);
    std::printf("内参         : %s  fx=%.2f fy=%.2f cx=%.2f cy=%.2f  %dx%d  model=%s\n", intrinsicsSource.c_str(),
                intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy, intrinsics.imageWidth,
                intrinsics.imageHeight, myslam::camera::cameraModelName(intrinsics.model).data());
    std::printf("参考轨迹     : %s\n", bag.referencePoses().empty() ? "无" : bag.poseTopicName().c_str());
    std::printf("估计器       : %s\n", vioConfig.estimator.name.c_str());

    const RosBagReplaySource source(bag, topic);

    // 观察者只在真的要显示/录像时构造；它不认识回放器，只实现 IRunObserver。
    // 观测模型必须与流水线一致，否则去畸变面板和特征点会落在两套像素语义上。
    std::unique_ptr<myslam::vio::IRunObserver> observer;
    if (runConfig.viz.showWindow || !runConfig.viz.videoPath.empty())
    {
        myslam::vio::VioPipeline probe(vioConfig);
        observer = std::make_unique<myslam::viz::ReplayObserver>(runConfig.viz, intrinsics, probe.observationModel());
    }
    std::string vizDesc;
    if (runConfig.viz.showWindow)
    {
        vizDesc = std::format("窗口（倍速 {:.1f}）", runConfig.viz.playbackRate);
    }
    if (!runConfig.viz.videoPath.empty())
    {
        if (!vizDesc.empty())
        {
            vizDesc += " + ";
        }
        vizDesc += std::format("录像 {}", runConfig.viz.videoPath.string());
    }
    std::printf("可视化       : %s\n", vizDesc.empty() ? "关闭" : vizDesc.c_str());

    myslam::dataset::ReplayRunner runner(vioConfig, intrinsics, runConfig, std::move(observer));
    const myslam::dataset::DatasetRunReport report = runner.run(source);

    std::printf("\n===== 回放结果 =====\n");
    std::printf("处理帧 %zu  关键帧 %zu  轨迹点 %zu  IMU 样本 %zu\n", report.framesProcessed, report.keyframes,
                report.trajectorySamples, report.imuSamples);
    std::printf("初始化 %s  回放时长 %.1f s  墙钟 %.1f s\n", report.initialized ? "成功" : "未完成",
                report.durationSeconds, report.elapsedSeconds);
    if (report.segments > 1)
    {
        std::printf("流水线重建 %zu 次（段边界接续，段内独立累积漂移）\n", report.resets);
    }
    if (!runConfig.outputTum.empty())
    {
        std::printf("轨迹已写入 %s\n", runConfig.outputTum.string().c_str());
    }
    return report.initialized ? 0 : 1;
}
