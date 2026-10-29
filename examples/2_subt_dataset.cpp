/**
 * @file 2_subt_dataset.cpp
 * @brief 在 SubT-MRS 数据集上回放 myslam VIO，并输出关键帧轨迹。
 *
 * 用法示例：
 * @code
 * 2_subt_dataset --data /path/Handheld1_Folder/cam_0 \
 *                --imu /path/Handheld1_Folder/imu/imu_data.csv \
 *                --stamps /path/Handheld1_Folder/cam_0/timestamps.txt \
 *                --max-frames 500 --output subt_handheld1.tum
 * @endcode
 *
 * 内参默认值与 @c intrinsics_subt_handheld.yaml 一致（MEI 模型），可用
 * @c --fx/--fy/--cx/--cy/--xi/--k1/--k2/--p1/--p2 覆盖。
 */

#include "dataset/dataset_runner.hpp"
#include "dataset/subt_dataset.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "lua_config.hpp"
#include "vio/configHolder.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <filesystem>
#include <format>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <optional>
#include <string>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:gtsam::guardedIsDebug\n";
}
#endif

namespace
{

    void printUsage()
    {
        std::fprintf(stderr,
                     "用法: 2_subt_dataset --data <cam_0 目录> --imu <imu_data.csv> --stamps <timestamps.txt>\n"
                     "                     [--output traj.tum] [--start-frame N] [--max-frames N] [--stride N]\n"
                     "                     [--max-duration S] [--max-features N] [--pinhole-scale S]\n"
                     "                     [--fx F] [--fy F] [--cx F]\n"
                     "                     [--cy F] [--xi F] [--k1 F] [--k2 F] [--p1 F] [--p2 F]\n"
                     "                     [--width W] [--height H] [--ply cloud.ply]\n"
                     "                     [--viz] [--video out.mp4] [--viz-stride N] [--viz-rate R]\n"
                     "                     [--config config/config.lua]\n"
                     "\n"
                     "--config 指定的文件作为默认值基线，命令行参数优先；未指定时尝试读取\n"
                     "当前目录的 config/config.lua（不存在则全部用内置默认值）。\n");
    }

    bool parseDouble(const char *value, double &out)
    {
        char *end = nullptr;
        out = std::strtod(value, &end);
        return end != value && *end == '\0';
    }

    bool parseInt(const char *value, int &out)
    {
        char *end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end == value || *end != '\0')
        {
            return false;
        }
        out = static_cast<int>(parsed);
        return true;
    }

} // namespace

int main(int argc, char **argv)
{
    // 配置文件先行解析：--config 显式指定，否则尝试读当前目录的 config/config.lua。
    // 必须在 main 开头扫描一遍，因为配置是 CLI 参数的基线默认值。
    std::filesystem::path configPath = "config/config.lua";
    bool explicitConfig = false;
    for (int i = 1; i + 1 < argc; ++i)
    {
        if (std::string(argv[i]) == "--config")
        {
            configPath = argv[i + 1];
            explicitConfig = true;
        }
    }

    // MYSLAM_CONSOLE_LOG=1 时保留 mylog 默认的 stdout 输出，便于调试崩溃路径
    // （文件输出带缓冲，异常终止可能丢失尾部日志）；此时也不让配置文件的
    // logger 节改写输出后端。
    const bool consoleLog = std::getenv("MYSLAM_CONSOLE_LOG") != nullptr;

    myslam::config::ProjectConfig project;
    const bool configFileExists = std::filesystem::is_regular_file(configPath);
    const myslam::config::LoadResult loaded =
        configFileExists ? myslam::config::load(configPath, project, /*applyLogging=*/!consoleLog)
                         : myslam::config::LoadResult(std::unexpected(
                               myslam::config::ConfigError{myslam::config::ConfigErrorKind::FileNotFound,
                                                           std::format("配置文件不存在: {}", configPath.string())}));

    if (!loaded && explicitConfig)
    {
        std::fprintf(stderr, "[error] %s (%s)\n", loaded.error().message.c_str(), loaded.error().kindName().data());
        return 2;
    }

    // 未从配置得到日志后端时回退到代码默认（当前工作目录的单文件输出）
    const bool loggingApplied = loaded.has_value() && loaded->logging;
    if (!loggingApplied && !consoleLog)
    {
        myslam::LogConfig logConfig;
        if (const char *level = std::getenv("MYSLAM_LOG_LEVEL"))
        {
            logConfig.level = level;
        }
        if (const auto result = myslam::configureLogging(logConfig); !result)
        {
            std::fprintf(stderr, "[warning] %s，沿用当前日志配置\n", result.error().message.c_str());
        }
    }
    if (!loaded)
    {
        std::fprintf(stderr, "[info] %s，使用内置默认配置\n", loaded.error().message.c_str());
    }

    std::string dataDir;
    std::string imuPath;
    std::string stampPath;
    // 输出路径以配置为基线；命令行显式给出时才能覆盖。原先无条件赋给
    // runConfig.outputTum，未传 --output 时会把配置里的值静默清空。
    const std::string outputFromConfig = project.replay.outputTum.string();
    std::optional<std::string> outputOverride;
    // 配置作为基线，下面的命令行分支逐项覆盖
    myslam::dataset::MeiIntrinsics intrinsics = project.intrinsics;
    myslam::dataset::DatasetRunConfig runConfig = project.replay;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);

        // 无取值的开关先处理，避免被下面的通用「缺少取值」检查误拦
        if (arg == "--viz")
        {
            runConfig.viz.showWindow = true;
            continue;
        }
        if (arg == "--help" || arg == "-h")
        {
            printUsage();
            return 0;
        }

        const bool hasValue = i + 1 < argc;
        if (!hasValue)
        {
            std::fprintf(stderr, "参数 %s 缺少取值\n", arg.c_str());
            printUsage();
            return 2;
        }
        const char *value = argv[i + 1];
        bool consumed = true;

        if (arg == "--data")
        {
            dataDir = value;
        }
        else if (arg == "--config")
        {
            // 已在 main 开头加载，这里仅消费掉取值避免被当作未知参数
        }
        else if (arg == "--imu")
        {
            imuPath = value;
        }
        else if (arg == "--stamps")
        {
            stampPath = value;
        }
        else if (arg == "--output")
        {
            outputOverride = value;
        }
        else if (arg == "--start-frame")
        {
            consumed = parseInt(value, runConfig.startFrame);
        }
        else if (arg == "--max-frames")
        {
            consumed = parseInt(value, runConfig.maxFrames);
        }
        else if (arg == "--stride")
        {
            consumed = parseInt(value, runConfig.stride);
        }
        else if (arg == "--max-duration")
        {
            consumed = parseDouble(value, runConfig.maxDuration);
        }
        else if (arg == "--max-features")
        {
            consumed = parseInt(value, runConfig.maxFeatures);
        }
        else if (arg == "--ply")
        {
            runConfig.plyPath = value;
        }
        else if (arg == "--video")
        {
            runConfig.viz.videoPath = value;
        }
        else if (arg == "--viz-stride")
        {
            consumed = parseInt(value, runConfig.viz.displayStride);
        }
        else if (arg == "--viz-rate")
        {
            consumed = parseDouble(value, runConfig.viz.playbackRate);
        }
        else if (arg == "--pinhole-scale")
        {
            consumed = parseDouble(value, intrinsics.pinholeScale);
        }
        else if (arg == "--fx")
        {
            consumed = parseDouble(value, intrinsics.fx);
        }
        else if (arg == "--fy")
        {
            consumed = parseDouble(value, intrinsics.fy);
        }
        else if (arg == "--cx")
        {
            consumed = parseDouble(value, intrinsics.cx);
        }
        else if (arg == "--cy")
        {
            consumed = parseDouble(value, intrinsics.cy);
        }
        else if (arg == "--xi")
        {
            consumed = parseDouble(value, intrinsics.xi);
        }
        else if (arg == "--k1")
        {
            consumed = parseDouble(value, intrinsics.k1);
        }
        else if (arg == "--k2")
        {
            consumed = parseDouble(value, intrinsics.k2);
        }
        else if (arg == "--p1")
        {
            consumed = parseDouble(value, intrinsics.p1);
        }
        else if (arg == "--p2")
        {
            consumed = parseDouble(value, intrinsics.p2);
        }
        else if (arg == "--width")
        {
            consumed = parseInt(value, intrinsics.imageWidth);
        }
        else if (arg == "--height")
        {
            consumed = parseInt(value, intrinsics.imageHeight);
        }
        else
        {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            printUsage();
            return 2;
        }

        if (!consumed)
        {
            std::fprintf(stderr, "参数 %s 的取值非法: %s\n", arg.c_str(), value);
            return 2;
        }
        ++i;
    }

    // 范围校验：parseInt/parseDouble 只保证“可解析”，不保证语义合法。
    // 例如 --max-features 0 会变成 OpenCV 的“不限制”，与本程序的“每帧上限”
    // 语义相反，必须在这里拦住。
    auto failRange = [](const char *what, long long value, long long lo, long long hi)
    {
        std::fprintf(stderr, "参数 %s 的取值 %lld 超出区间 [%lld, %lld]\n", what, value, lo, hi);
        return 2;
    };
    if (runConfig.startFrame < 0)
    {
        return failRange("--start-frame", runConfig.startFrame, 0, 10'000'000);
    }
    if (runConfig.maxFrames < 0)
    {
        return failRange("--max-frames", runConfig.maxFrames, 0, 10'000'000);
    }
    if (runConfig.stride < 1)
    {
        return failRange("--stride", runConfig.stride, 1, 10'000);
    }
    if (runConfig.maxFeatures < 1)
    {
        return failRange("--max-features", runConfig.maxFeatures, 1, 1'000'000);
    }
    if (runConfig.viz.displayStride < 1)
    {
        return failRange("--viz-stride", runConfig.viz.displayStride, 1, 10'000);
    }
    if (!(runConfig.maxDuration >= 0.0) || runConfig.maxDuration > 86'400.0)
    {
        std::fprintf(stderr, "参数 --max-duration 的取值 %f 超出区间 [0, 86400]\n", runConfig.maxDuration);
        return 2;
    }
    if (!(runConfig.viz.playbackRate >= 0.0) || runConfig.viz.playbackRate > 100.0)
    {
        std::fprintf(stderr, "参数 --viz-rate 的取值 %f 超出区间 [0, 100]\n", runConfig.viz.playbackRate);
        return 2;
    }

    if (dataDir.empty() || imuPath.empty() || stampPath.empty())
    {
        std::fprintf(stderr, "--data、--imu、--stamps 均为必填参数\n");
        printUsage();
        return 2;
    }
    if (!intrinsics.isValid())
    {
        std::fprintf(stderr, "相机内参非法: fx=%f fy=%f size=%dx%d\n", intrinsics.fx, intrinsics.fy,
                     intrinsics.imageWidth, intrinsics.imageHeight);
        return 2;
    }
    // 命令行覆盖优先，否则沿用配置值
    runConfig.outputTum =
        outputOverride.has_value() ? std::filesystem::path(*outputOverride) : std::filesystem::path(outputFromConfig);

    myslam::dataset::SubtSequence sequence(dataDir, imuPath, stampPath);
    if (const auto loadedSequence = sequence.load(); !loadedSequence)
    {
        // 具体原因已由 SubtSequence 记录到日志，这里只给出面向终端的摘要
        std::fprintf(stderr, "[error] %s\n", loadedSequence.error().message.c_str());
        return 1;
    }

    // 后端使用针孔模型重投影，内参必须与去畸变后的虚拟针孔像素坐标一致。
    // 以配置文件的 vio 节为基线再覆盖——原先直接 default 构造 VioConfig，
    // 导致 config/config.lua 里所有 vio.* 参数（门限、窗口、噪声、Huber…）
    // 在这条回放路径上全部静默失效。
    myslam::vio::VioConfig vioConfig = project.vio;
    vioConfig.fx = intrinsics.pinholeFx();
    vioConfig.fy = intrinsics.pinholeFy();
    vioConfig.cx = intrinsics.pinholeCx();
    vioConfig.cy = intrinsics.pinholeCy();
    vioConfig.imageWidth = static_cast<double>(intrinsics.imageWidth);
    vioConfig.imageHeight = static_cast<double>(intrinsics.imageHeight);
    vioConfig.imuRateHz = sequence.estimateImuRate() > 0.0 ? sequence.estimateImuRate() : 200.0;

    // 视频帧率跟随相机实际频率（SubT Handheld 约 24.5 Hz），
    // 否则录制出的视频时长会与真实序列时长不一致。
    const double cameraRate = sequence.estimateCameraRate();
    if (cameraRate > 0.0)
    {
        runConfig.viz.videoFps = static_cast<int>(std::lround(cameraRate));
    }

    // IMU 实际采样率以序列为准：它同时决定噪声离散化与断档判定周期。
    // （回放不用 SystemConfig：主循环由相机时间戳驱动，不由定时器驱动。）

    mylog::info("[回放配置] 外点门限 软/硬/预优化 = {:.1f}/{:.1f}/{:.1f} px, 关键帧视差 {:.1f} px, "
                "固定滞后窗口 {:.2f}s, Huber delta {:.3f}",
                vioConfig.maxReprojectionError, vioConfig.maxReprojectionErrorHard,
                vioConfig.maxReprojectionErrorPreopt, vioConfig.minParallaxPx, vioConfig.slidingWindowLag,
                vioConfig.huberLossDelta);

    // SubT Handheld1 的 T_imu_cam（相机在 IMU 系下的位姿）
    vioConfig.bodyPSensor = gtsam::Pose3(gtsam::Rot3(-0.04279531, -0.00237969, 0.99908103, -0.99880330, -0.02359508,
                                                     -0.04283961, 0.02367534, -0.99971877, -0.00136708),
                                         gtsam::Point3(0.19499356, 0.04340662, -0.01782382));

    myslam::dataset::DatasetRunner runner(vioConfig, intrinsics, runConfig);
    const myslam::dataset::DatasetRunReport report = runner.run(sequence);

    if (!runConfig.outputTum.empty())
    {
        mylog::info("关键帧轨迹已写入: {}", runConfig.outputTum.string());
    }
    return report.initialized ? 0 : 1;
}
