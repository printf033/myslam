/**
 * @file 4_tum_vio.cpp
 * @brief 在 TUM RGB-D 数据集上回放 myslam VIO，并与真值比较轨迹误差。
 *
 * 用法：
 * @code
 * 4_tum_vio <序列目录> [选项]
 *    --inspect           只做数据体检（不跑 VIO）
 *    --config FILE       Lua 配置（默认 config/tum_vi.lua）
 *    --out FILE.tum      关键帧轨迹输出（默认 <序列>.tum）
 *    --max-time-diff S   与真值的时间关联阈值（默认 0.02，与 TUM 官方一致）
 *    --start-frame N --max-frames N --stride N --max-features N --max-duration S
 *    --fx --fy --cx --cy 覆盖 Lua 配置中的相机内参
 *    --show / --no-show --video FILE --video-fps N --playback-rate R
 * @endcode
 *
 * @par 与 rosbag 路径的区别
 *
 * TUM RGB-D **没有陀螺仪**，TumSequence 从真值姿态差分合成 gyro（加速度仍
 * 取 accelerometer.txt 的真实测量）。因此这里的回放验证的是「位置与视觉
 * 融合链路」，不是标准 VIO 的旋转估计能力——程序启动时会明确打印这一点。
 *
 * TUM 相对 rosbag 的优势是自带运动捕捉真值，所以本程序在回放结束后自动
 * 计算 ATE（SE(3) 对齐后的平移 RMSE），给出可量化的结论。
 */

#include "app/replay.hpp"
#include "app/trajectory_eval.hpp"
#include "config/lua_config.hpp"
#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include "sensor/tum_dataset.hpp"
#include "viz/replay_observer.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <vector>

namespace
{

    using myslam::dataset::TumSequence;

    void printUsage()
    {
        std::fprintf(stderr, "用法: 4_tum_vio <序列目录> [选项]\n"
                             "  --inspect           只做数据体检\n"
                             "  --config FILE       Lua 配置（默认 config/tum_vi.lua）\n"
                             "  --out FILE.tum      轨迹输出\n"
                             "  --max-time-diff S   真值关联阈值（默认 0.02）\n"
                             "  --start-frame N --max-frames N --stride N --max-features N\n"
                             "  --max-duration S    最长回放时长\n"
                             "  --fx --fy --cx --cy 覆盖 Lua 中的内参\n"
                             "  --no-undistort      关闭图像去畸变（默认按标定矫正）\n"
                             "  --show / --no-show  开/关实时窗口\n"
                             "  --video FILE        录像输出\n"
                             "  --video-fps N       录像帧率\n"
                             "  --playback-rate R   播放倍速\n");
    }

    /**
     * @brief TumSequence → 顺序回放源
     *
     * TUM 序列的图像带明显径向畸变，而算法层的 @c PinholeCameraModel 只做纯
     * 针孔投影、不应用畸变系数。不矫正的话图像边缘的重投影误差可达几十像素，
     * 三角化会成片失败。因此这里用原始内参+畸变（取自 Lua 配置）预计算重映射表，把图像矫正到「同一个 K 的无畸变图」。
     * 保持 K 不变而不是用 getOptimalNewCameraMatrix，是为了不裁剪视场。
     */
    class TumReplaySource final : public myslam::dataset::IReplaySource
    {
    public:
        TumReplaySource(const TumSequence &sequence, const myslam::camera::CameraIntrinsics &intrinsics, bool undistort)
            : sequence_(sequence)
        {
            const bool hasDistortion = std::abs(intrinsics.k1) > 1e-9 || std::abs(intrinsics.k2) > 1e-9 ||
                                       std::abs(intrinsics.p1) > 1e-9 || std::abs(intrinsics.p2) > 1e-9;
            if (!undistort || !hasDistortion)
            {
                return;
            }
            const cv::Mat k = (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                               intrinsics.cy, 0.0, 0.0, 1.0);
            const cv::Mat d =
                (cv::Mat_<double>(1, 5) << intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2, 0.0);
            cv::initUndistortRectifyMap(k, d, cv::noArray(), k, cv::Size(intrinsics.imageWidth, intrinsics.imageHeight),
                                        CV_32FC1, map1_, map2_);
            active_ = !map1_.empty();
        }

        [[nodiscard]] bool undistorting() const { return active_; }

        [[nodiscard]] const std::vector<double> &stamps() const override { return sequence_.stamps(); }
        [[nodiscard]] const std::vector<myslam::dataset::ImuRecord> &imu() const override { return sequence_.imu(); }
        [[nodiscard]] cv::Mat loadGray(std::size_t frameIndex) const override
        {
            const cv::Mat raw = cv::imread(sequence_.imagePath(frameIndex), cv::IMREAD_GRAYSCALE);
            if (!active_ || raw.empty())
            {
                return raw;
            }
            cv::Mat rectified;
            cv::remap(raw, rectified, map1_, map2_, cv::INTER_LINEAR);
            return rectified;
        }

    private:
        const TumSequence &sequence_;
        cv::Mat map1_;
        cv::Mat map2_;
        bool active_{false};
    };

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

    double median(std::vector<double> values)
    {
        if (values.empty())
        {
            return 0.0;
        }
        const std::size_t mid = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
        return values[mid];
    }

    /// 数据体检：回答「这份数据能不能用、缺什么」
    void inspect(const TumSequence &sequence)
    {
        const auto &stamps = sequence.stamps();
        const auto &gt = sequence.groundTruth();
        const auto &imu = sequence.imu();

        std::printf("\n================ TUM 数据体检 ================\n");
        std::printf("序列目录   : %s\n", sequence.root().string().c_str());
        std::printf("图像       : %zu 帧, %.2f s, %.2f Hz\n", stamps.size(),
                    stamps.empty() ? 0.0 : stamps.back() - stamps.front(), sequence.estimateCameraRate());
        std::printf("深度图     : %zu 帧（本读取器不消费）\n", sequence.depthCount());
        std::printf("真值轨迹   : %zu 条, %.1f Hz\n", gt.size(),
                    gt.size() < 2 ? 0.0
                                  : static_cast<double>(gt.size() - 1) / (gt.back().timestamp - gt.front().timestamp));
        std::printf("原始加速度 : %zu 条\n", sequence.rawAccelerometerCount());
        std::printf("合成 IMU   : %zu 条, %.1f Hz（gyro 由真值差分）\n", imu.size(), sequence.estimateImuRate());
        std::printf("加速度覆盖 : %.1f%% 的相机时间窗\n", 100.0 * sequence.accelerometerCoverage());

        if (gt.size() >= 2)
        {
            double pathLength = 0.0;
            std::vector<double> speed;
            for (std::size_t i = 1; i < gt.size(); ++i)
            {
                const double dt = gt[i].timestamp - gt[i - 1].timestamp;
                const double dist = (gt[i].position - gt[i - 1].position).norm();
                pathLength += dist;
                if (dt > 1e-9)
                {
                    speed.push_back(dist / dt);
                }
            }
            std::printf("真值路径   : %.2f m, 速度中位数 %.3f m/s, 最大 %.3f m/s\n", pathLength, median(speed),
                        speed.empty() ? 0.0 : *std::max_element(speed.begin(), speed.end()));
        }

        if (!imu.empty())
        {
            double sum = 0.0;
            double lo = 1e9;
            double hi = 0.0;
            for (const auto &record : imu)
            {
                const double norm = record.accel.norm();
                sum += norm;
                lo = std::min(lo, norm);
                hi = std::max(hi, norm);
            }
            std::printf("|accel|    : 均值 %.3f, 范围 [%.3f, %.3f] m/s^2（静止时≈9.81）\n",
                        sum / static_cast<double>(imu.size()), lo, hi);
            std::printf("|gyro| max : %.3f rad/s\n",
                        [&imu]
                        {
                            double worst = 0.0;
                            for (const auto &record : imu)
                            {
                                worst = std::max(worst, record.gyro.norm());
                            }
                            return worst;
                        }());
        }
        std::printf("说明       : 无陀螺仪，gyro 由真值差分 → 结果验证位置融合链路，非标准 VIO 精度\n");
        std::printf("=============================================\n");
    }

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printUsage();
        return 2;
    }

    std::string sequencePath;
    bool doInspect = false;
    std::filesystem::path configPath = "config/tum_vi.lua";
    std::filesystem::path outTum;
    int startFrame = -1;
    int maxFrames = -1;
    int stride = -1;
    int maxFeatures = -1;
    int videoFps = -1;
    double maxDuration = -1.0;
    double maxTimeDiff = 0.02;
    double playbackRate = -1.0;
    std::optional<double> fx;
    std::optional<double> fy;
    std::optional<double> cx;
    std::optional<double> cy;
    std::optional<bool> forceShow;
    std::filesystem::path videoPath;
    bool undistort = true;

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
        else if (arg == "--inspect")
        {
            doInspect = true;
        }
        else if (arg == "--config")
        {
            configPath = needValue("--config");
        }
        else if (arg == "--out")
        {
            outTum = needValue("--out");
        }
        else if (arg == "--max-time-diff")
        {
            (void)parseDouble(needValue("--max-time-diff"), maxTimeDiff);
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
        else if (arg == "--no-undistort")
        {
            undistort = false;
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
        else if (!arg.empty() && arg[0] != '-' && sequencePath.empty())
        {
            sequencePath = arg;
        }
        else
        {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            printUsage();
            return 2;
        }
    }

    if (sequencePath.empty())
    {
        std::fprintf(stderr, "[error] 未给出 TUM 序列目录\n");
        printUsage();
        return 2;
    }

    TumSequence sequence(sequencePath);
    if (const auto rc = sequence.load(); !rc)
    {
        std::fprintf(stderr, "[error] %s\n", rc.error().message.c_str());
        return 1;
    }
    if (sequence.size() == 0 || sequence.imu().empty())
    {
        std::fprintf(stderr, "[error] 序列没有图像或 IMU\n");
        return 1;
    }

    if (doInspect)
    {
        inspect(sequence);
        return 0;
    }

    // ---------------------------------------------------------------------
    // Lua 配置：内参、vio 参数与外参（config/tum_vi.lua）。日志后端保持 mylog 默认（stdout）。
    // ---------------------------------------------------------------------
    myslam::config::ProjectConfig project;
    if (!std::filesystem::is_regular_file(configPath))
    {
        std::fprintf(stderr, "[error] 找不到配置文件: %s\n", configPath.string().c_str());
        return 1;
    }
    if (const auto loaded = myslam::config::load(configPath, project, /*applyLogging=*/false); !loaded)
    {
        std::fprintf(stderr, "[error] 配置加载失败: %s\n", loaded.error().message.c_str());
        return 1;
    }

    // ---------------------------------------------------------------------
    // 内参：由 Lua 配置提供，命令行可覆盖
    // ---------------------------------------------------------------------
    myslam::camera::CameraIntrinsics intrinsics = project.intrinsics;
    std::string intrinsicsSource = "lua 配置";

    if (fx)
    {
        intrinsics.fx = *fx;
        intrinsics.fy = fy.value_or(*fx);
        intrinsics.k1 = 0.0;
        intrinsics.k2 = 0.0;
        intrinsics.p1 = 0.0;
        intrinsics.p2 = 0.0;
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
    (void)intrinsics.normalizeForModel();
    if (!intrinsics.isValid())
    {
        std::fprintf(stderr, "[error] 相机内参无效\n");
        return 2;
    }

    // ---------------------------------------------------------------------
    // 回放配置
    // ---------------------------------------------------------------------
    const double imuRate = sequence.estimateImuRate();
    myslam::domain::PipelineConfig vioConfig = project.vio;
    vioConfig.camera.intrinsics = intrinsics;
    if (imuRate > 0.0)
    {
        vioConfig.imu.rateHz = imuRate;
    }

    myslam::dataset::DatasetRunConfig runConfig = project.replay;
    runConfig.outputTum = outTum;
    runConfig.plyPath.clear();
    runConfig.startFrame = startFrame >= 0 ? startFrame : 0;
    runConfig.maxFrames = maxFrames >= 0 ? maxFrames : 0;

    // TUM RGB-D 从运动开始，没有静止段；零初速会让预积分预测系统性偏离
    // 视觉观测，滤波器只能拿零偏去代偿并最终发散。用真值差分给出该时刻
    // 的世界系速度（与合成 gyro 同属「真值注入」，程序开头已声明）。
    if (const auto &gt = sequence.groundTruth(); gt.size() >= 3)
    {
        const std::size_t firstFrame = static_cast<std::size_t>(
            std::clamp(runConfig.startFrame, 0, static_cast<int>(sequence.stamps().size()) - 1));
        const double t0 = sequence.stamps()[firstFrame];
        const auto it =
            std::lower_bound(gt.begin(), gt.end(), t0,
                             [](const myslam::dataset::GroundTruthPose &pose, double t) { return pose.timestamp < t; });
        const std::size_t upper = std::min<std::size_t>(static_cast<std::size_t>(it - gt.begin()), gt.size() - 2);
        const std::size_t lower = upper == 0 ? 0 : upper - 1;
        const std::size_t next = std::min(upper + 1, gt.size() - 1);
        const double span = gt[next].timestamp - gt[lower].timestamp;
        if (span > 1e-6)
        {
            vioConfig.initialization.initialVelocity = (gt[next].position - gt[lower].position) / span;
        }
    }
    runConfig.maxDuration = maxDuration >= 0.0 ? maxDuration : 0.0;
    if (stride > 0)
    {
        runConfig.stride = stride;
    }
    if (maxFeatures > 0)
    {
        runConfig.maxFeatures = maxFeatures;
    }
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
        const double rate = sequence.estimateCameraRate();
        if (rate > 0.0)
        {
            runConfig.viz.videoFps = static_cast<int>(std::lround(rate));
        }
    }
    if (runConfig.outputTum.empty())
    {
        runConfig.outputTum = std::filesystem::path(sequence.root().filename().string() + ".tum");
    }
    if (const auto parent = runConfig.outputTum.parent_path(); !parent.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
    }

    std::printf("序列         : %s\n", sequence.root().string().c_str());
    std::printf("图像         : %zu 帧, %.2f Hz\n", sequence.size(), sequence.estimateCameraRate());
    std::printf("IMU          : %zu 条, %.1f Hz（gyro 由真值差分合成）\n", sequence.imu().size(), imuRate);
    std::printf("真值         : %zu 条（用于 ATE）\n", sequence.groundTruth().size());
    std::printf("内参         : %s  fx=%.2f fy=%.2f cx=%.2f cy=%.2f  %dx%d\n", intrinsicsSource.c_str(), intrinsics.fx,
                intrinsics.fy, intrinsics.cx, intrinsics.cy, intrinsics.imageWidth, intrinsics.imageHeight);
    std::printf("初始速度     : (%.3f, %.3f, %.3f) m/s（真值差分）\n", vioConfig.initialization.initialVelocity.x(),
                vioConfig.initialization.initialVelocity.y(), vioConfig.initialization.initialVelocity.z());
    std::printf("估计器       : %s\n", vioConfig.estimator.name.c_str());
    std::printf("注意         : 本序列无陀螺仪，gyro 来自真值差分；结果验证位置融合链路，"
                "不能等同于标准 VIO 精度\n");

    const TumReplaySource source(sequence, intrinsics, undistort);
    std::printf("去畸变       : %s\n", source.undistorting() ? "开（k1/k2/p1/p2 已矫正）" : "关");
    std::unique_ptr<myslam::vio::IRunObserver> observer;
    if (runConfig.viz.showWindow || !runConfig.viz.videoPath.empty())
    {
        myslam::vio::VioPipeline probe(vioConfig);
        observer = std::make_unique<myslam::viz::ReplayObserver>(runConfig.viz, intrinsics, probe.observationModel());
    }

    myslam::dataset::ReplayRunner runner(vioConfig, intrinsics, runConfig, std::move(observer),
                                         project.system.timingCsvPath);
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

    // ---------------------------------------------------------------------
    // ATE：读回轨迹，与真值做 SE(3) 对齐后统计误差
    // ---------------------------------------------------------------------
    if (!runConfig.outputTum.empty())
    {
        std::printf("轨迹已写入 %s\n", runConfig.outputTum.string().c_str());
        const auto track = myslam::app::loadTumTrajectory(runConfig.outputTum);
        if (!track)
        {
            std::printf("[评估] 无法读取轨迹: %s\n", track.error().c_str());
        }
        else if (track->size() < 3)
        {
            std::printf("[评估] 轨迹点不足 3 个，跳过 ATE\n");
        }
        else
        {
            const std::string trajectoryLabel = runConfig.outputTum.stem().string();
            const myslam::app::AteMetrics ate = myslam::app::evaluateAte(*track, sequence.groundTruth(), maxTimeDiff);
            std::printf("\n===== 绝对轨迹误差（SE3 对齐，无尺度）=====\n");
            std::printf("关联点对 %zu / %zu（阈值 %.3f s）\n", ate.matched, track->size(), maxTimeDiff);
            if (ate.matched < 3)
            {
                std::printf("关联不足，无法计算 ATE\n");
            }
            else
            {
                std::printf("RMSE %.3f m   中位数 %.3f m   均值 %.3f m   最大 %.3f m\n", ate.rmse, ate.median, ate.mean,
                            ate.max);
                // 这里没有 NEES：TUM 轨迹文件只存 t x y z qx qy qz qw，不含协方差，
                // 一致性结论请用仿真路径（VioBenchmark 的 RunReport::neesMean）。
                if (ate.rotationSamples > 0)
                {
                    std::printf("旋转 RMSE %.3f°   最大 %.3f°（%zu 个样本）\n", ate.rotationRmse, ate.rotationMax,
                                ate.rotationSamples);
                }
                else
                {
                    std::printf("旋转误差不可用：轨迹或真值缺姿态列\n");
                }

                // RPE：走过 1 m 后的相对位移/旋转误差。滑动窗口系统逐段积累
                // 误差，这个数比 ATE 更能反映局部质量（ATE 会被整体对齐吸收）。
                const myslam::app::RpeMetrics rpe =
                    myslam::app::evaluateRpe(*track, sequence.groundTruth(), maxTimeDiff, 1.0);
                if (rpe.pairs > 0)
                {
                    std::printf("\n===== 相对位姿误差（段长 %.2f m）=====\n", rpe.segmentLength);
                    std::printf("有效段数 %zu   RMSE %.4f m   均值 %.4f m   最大 %.4f m\n", rpe.pairs, rpe.rmse,
                                rpe.mean, rpe.max);
                    if (rpe.rotationPairs > 0)
                    {
                        std::printf("相对旋转 RMSE %.3f°   最大 %.3f°（%zu 段）\n", rpe.rotationRmse, rpe.rotationMax,
                                    rpe.rotationPairs);
                    }
                }
                else
                {
                    std::printf("\n[评估] 轨迹长度不足 %.2f m，无法计算 RPE\n", rpe.segmentLength);
                }

                // 结构化一行：多次运行直接拼成表格，不必手抄终端输出。
                std::printf("\n%s", std::string(myslam::app::markdownHeader()).c_str());
                std::printf("%s", myslam::app::toMarkdownRow(trajectoryLabel, ate, rpe).c_str());
            }
        }
    }

    return report.initialized ? 0 : 1;
}
