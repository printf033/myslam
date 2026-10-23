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
#include "vio/types.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
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
                     "                     [--viz] [--video out.mp4] [--viz-stride N] [--viz-rate R]\n");
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
    // 默认写入当前目录的日志文件（与 example 1 一致）；设置 MYSLAM_CONSOLE_LOG=1
    // 时改用 mylog 默认的 stdout 输出，便于调试崩溃路径（文件输出带缓冲，异常
    // 终止可能丢失尾部日志）。MYSLAM_LOG_LEVEL 可覆盖日志级别。
    if (std::getenv("MYSLAM_CONSOLE_LOG") == nullptr)
    {
        myslam::LogConfig logConfig;
        if (const char *level = std::getenv("MYSLAM_LOG_LEVEL"))
        {
            logConfig.level = level;
        }
        if (!myslam::configureLogging(logConfig))
        {
            std::fprintf(stderr, "[warning] 日志级别无效，沿用当前日志配置\n");
        }
    }

    std::string dataDir;
    std::string imuPath;
    std::string stampPath;
    std::string outputPath;
    myslam::dataset::MeiIntrinsics intrinsics;
    myslam::dataset::DatasetRunConfig runConfig;

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
            outputPath = value;
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
    runConfig.outputTum = outputPath;

    myslam::dataset::SubtSequence sequence(dataDir, imuPath, stampPath);
    if (!sequence.load())
    {
        return 1;
    }

    // 后端使用针孔模型重投影，内参必须与去畸变后的虚拟针孔像素坐标一致
    myslam::vio::VioConfig vioConfig;
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
