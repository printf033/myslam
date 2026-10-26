/**
 * @file test_lua_config.cpp
 * @brief Lua 配置加载的库级测试：覆盖解析、钳位、交叉校验与失败路径。
 *
 * 与 test_vio.cpp 相同的断言计数风格：逐条断言、末尾统一报告失败数。
 * 每个用例把 Lua 脚本写到临时目录，测完即删，不污染工作区。
 */

#include "log_config.hpp"
#include "lua_config.hpp"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
/**
 * @brief LeakSanitizer 默认抑制规则（与其它测试一致）
 */
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:gtsam::guardedIsDebug\n";
}
#endif

namespace
{
    int g_checks = 0;
    int g_failures = 0;

    void expectTrue(bool condition, const char *what)
    {
        ++g_checks;
        if (!condition)
        {
            ++g_failures;
            std::fprintf(stderr, "FAIL: %s\n", what);
        }
    }

    void expectNear(double actual, double expected, const char *what)
    {
        expectTrue(std::abs(actual - expected) < 1e-12, what);
    }

    /**
     * @brief 把 Lua 脚本写入临时文件
     *
     * @param name 用例名（用于区分文件名）
     * @param body Lua 脚本内容
     * @return 临时文件路径
     */
    std::filesystem::path writeLua(const std::string &name, const std::string &body)
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() / ("myslam_cfg_" + name + "_" + std::to_string(::getpid()) + ".lua");
        std::ofstream out(path);
        out << body;
        return path;
    }

    /// 加载指定脚本（不触碰日志配置），返回报告
    myslam::config::LoadReport loadFrom(const std::filesystem::path &path, myslam::config::ProjectConfig &config)
    {
        return myslam::config::load(path, config, /*applyLogging=*/false);
    }

    /**
     * @brief 合法脚本应覆盖对应字段，未出现的字段保持默认
     */
    void checkOverrides()
    {
        const auto path = writeLua("overrides", R"(
vio = {
    fx = 123.0,
    minTrackingRatio = 0.5,
    minKeyframeInterval = 0.1,
    maxKeyframeInterval = 0.2,
    maxReprojectionError = 5.0,
    maxReprojectionErrorHard = 10.0,
    maxReprojectionErrorPreopt = 20.0,
    extraSmootherIterations = 3,
}
system = { totalDuration = 1.25, cameraDt = 0.05 }
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "合法配置解析成功");
        expectNear(config.vio.fx, 123.0, "vio.fx 覆盖生效");
        expectNear(config.vio.minTrackingRatio, 0.5, "vio.minTrackingRatio 覆盖生效");
        expectTrue(config.vio.extraSmootherIterations == 3, "vio.extraSmootherIterations 覆盖生效");
        expectNear(config.system.totalDuration, 1.25, "system.totalDuration 覆盖生效");
        expectNear(myslam::vio::SystemConfig::imuDtFrom(config.vio.imuRateHz), 0.005, "默认 200 Hz 派生的步长为 5 ms");
        expectNear(config.vio.cy, 240.0, "未出现的字段保持默认值");
        expectNear(config.system.rmseThreshold, 0.20, "未出现的 system 字段保持默认值");
    }

    /**
     * @brief 越界与非法类型不应让配置失败，而是钳位或忽略
     */
    void checkClampingAndTypes()
    {
        const auto path = writeLua("clamp", R"(
vio = {
    minTrackingRatio = 3.5,
    maxSlidingWindowSize = 0,
    fx = "not a number",
    fy = 0 / 0,
    gravityMagnitude = -1.0,
}
system = { cameraDt = 0.05 }
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "越界值钳位后配置仍然有效");
        expectNear(config.vio.minTrackingRatio, 1.0, "minTrackingRatio 钳位到上界 1.0");
        expectTrue(config.vio.maxSlidingWindowSize == 1, "maxSlidingWindowSize 钳位到下界 1");
        expectNear(config.vio.fx, 460.0, "字符串字段被忽略并保持默认");
        expectNear(config.vio.fy, 460.0, "NaN 字段被忽略并保持默认");
        expectNear(config.vio.gravityMagnitude, 1e-3, "负重力钳位到下界");
    }

    /**
     * @brief IMU 采样步长应由 vio.imuRateHz 派生，且旧配置的 imuDt 被忽略
     */
    void checkImuRateDerivation()
    {
        const auto path = writeLua("imu-rate", R"(
vio = { imuRateHz = 100.0 }
system = { cameraDt = 0.05 }
)");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "合法采样率配置解析成功");
        expectNear(config.vio.imuRateHz, 100.0, "vio.imuRateHz 覆盖生效");
        expectNear(myslam::config::ProjectConfig{}.system.cameraDt, 0.05, "未配置的 cameraDt 保持默认");
        expectNear(myslam::vio::SystemConfig::imuDtFrom(config.vio.imuRateHz), 0.01, "100 Hz 派生出 10 ms 步长");

        // 旧配置残留 imuDt：应被忽略（不改变派生结果），且不致失败
        const auto legacy = writeLua("legacy-imu-dt", R"(
vio = { imuRateHz = 200.0 }
system = { imuDt = 0.02, cameraDt = 0.05 }
)");
        myslam::config::ProjectConfig legacyConfig;
        const auto legacyReport = loadFrom(legacy, legacyConfig);
        std::filesystem::remove(legacy);

        expectTrue(legacyReport.project, "旧配置的 imuDt 不影响加载成功");
        expectNear(myslam::vio::SystemConfig::imuDtFrom(legacyConfig.vio.imuRateHz), 0.005,
                   "旧 imuDt 被忽略，仍由采样率派生");
    }

    /**
     * @brief 自相矛盾的组合应整体失败，且不写回调用方配置
     */
    void checkCrossValidation()
    {
        const auto path = writeLua("contradiction", R"(
vio = {
    fx = 999.0,
    minKeyframeInterval = 0.5,
    maxKeyframeInterval = 0.2,
}
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(!report.project, "关键帧间隔矛盾时配置被拒绝");
        expectNear(config.vio.fx, 460.0, "失败时不写回调用方配置");

        const auto path2 = writeLua("camera-faster", R"(
vio = { imuRateHz = 20.0 }
system = { cameraDt = 0.005 }
)");
        myslam::config::ProjectConfig config2;
        const auto report2 = loadFrom(path2, config2);
        std::filesystem::remove(path2);

        expectTrue(!report2.project, "相机快于 IMU 时配置被拒绝");
    }

    /**
     * @brief 拼错的字段名不应导致失败，但也不应影响任何字段
     */
    void checkUnknownKeys()
    {
        const auto path = writeLua("unknown", R"(
vio = { fxx = 1.0, gyroNoiseSigam = 0.1 }
system = { totalDuration = 2.0 }
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "未知键只告警、不失败");
        expectNear(config.vio.fx, 460.0, "拼错的内参字段未生效");
        expectNear(config.vio.gyroNoiseSigma, 3.6e-4, "拼错的噪声字段未生效");
        expectNear(config.system.totalDuration, 2.0, "同节内的合法字段仍生效");
    }

    /**
     * @brief 外参四元数应被单位化后写入位姿
     */
    void checkExtrinsic()
    {
        const auto path = writeLua("extrinsic", R"(
vio = {
    extrinsic = {
        quaternion = { w = 2.0, x = 0.0, y = 0.0, z = 0.0 },
        translation = { x = 1.5, y = -2.0, z = 0.25 },
    },
}
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "合法外参解析成功");
        // 四元数 (2,0,0,0) 单位化后是恒等旋转
        expectTrue(config.vio.bodyPSensor.rotation().equals(gtsam::Rot3(), 1e-12), "外参四元数被单位化为恒等旋转");
        expectNear(config.vio.bodyPSensor.translation().x(), 1.5, "外参平移 x 生效");
        expectNear(config.vio.bodyPSensor.translation().y(), -2.0, "外参平移 y 生效");
        expectNear(config.vio.bodyPSensor.translation().z(), 0.25, "外参平移 z 生效");

        const auto zeroPath = writeLua("extrinsic-zero", R"(
vio = { extrinsic = { quaternion = { w = 0, x = 0, y = 0, z = 0 } } }
)");
        myslam::config::ProjectConfig config2;
        const auto report2 = loadFrom(zeroPath, config2);
        std::filesystem::remove(zeroPath);
        expectTrue(!report2.project, "零模长四元数被拒绝");
    }

    /**
     * @brief 文件缺失、语法错误、节类型错误都应返回失败而不抛异常
     */
    void checkFailurePaths()
    {
        myslam::config::ProjectConfig config;
        const auto missing = loadFrom(std::filesystem::temp_directory_path() / "myslam_cfg_does_not_exist.lua", config);
        expectTrue(!missing.project, "配置文件缺失时返回失败");

        const auto syntax = writeLua("syntax", "vio = { fx = ");
        const auto syntaxReport = loadFrom(syntax, config);
        std::filesystem::remove(syntax);
        expectTrue(!syntaxReport.project, "语法错误的脚本返回失败");

        const auto wrongType = writeLua("wrong-type", "vio = 42\nsystem = { totalDuration = 1.0 }");
        myslam::config::ProjectConfig config2;
        const auto wrongTypeReport = loadFrom(wrongType, config2);
        std::filesystem::remove(wrongType);
        expectTrue(!wrongTypeReport.project, "配置节不是 table 时返回失败");
        expectNear(config2.system.totalDuration, 5.0, "失败时不写回调用方配置");
    }

    /**
     * @brief 空的合法脚本应保持全部默认值
     */
    void checkEmptyScript()
    {
        const auto path = writeLua("empty", "-- 什么都不配置\n");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "空脚本解析成功");
        expectNear(config.vio.fx, 460.0, "空脚本保持默认内参");
        expectNear(config.system.totalDuration, 5.0, "空脚本保持默认时长");
    }

    /**
     * @brief 日志节应被 mylog 接受并真正应用到全局抑制级别
     */
    void checkLoggingSection()
    {
        const auto path = writeLua("logging", "logger = { suppressLevel = \"warn\" }\n");
        myslam::config::ProjectConfig config;
        const auto report = myslam::config::load(path, config, /*applyLogging=*/true);
        std::filesystem::remove(path);

        expectTrue(report.project, "无 vio / system 节时项目配置仍然成功");
        expectTrue(report.logging, "logger 节被 mylog 接受");
        expectTrue(mylog::getCurrentSuppressLevel() == mylog::LEVEL::WARN, "suppressLevel 已真实生效");

        // 恢复级别，避免影响后续用例与测试框架输出
        mylog::setSuppressLevel("info");
    }

    /**
     * @brief 项目自带的默认配置必须能通过全部校验
     */
    void checkShippedConfig()
    {
        const std::filesystem::path candidates[] = {"config/myslam.lua", "../config/myslam.lua"};
        std::filesystem::path found;
        for (const auto &candidate : candidates)
        {
            if (std::filesystem::is_regular_file(candidate))
            {
                found = candidate;
                break;
            }
        }
        expectTrue(!found.empty(), "找到仓库自带的 config/myslam.lua");
        if (found.empty())
        {
            return;
        }

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(found, config);
        expectTrue(report.project, "仓库默认配置通过全部校验");
        expectNear(config.vio.fx, 460.0, "默认配置的 fx 与代码默认值一致");
        expectNear(config.vio.accelNoiseSigma, 0.0057, "默认配置的加速度噪声与代码默认值一致");
        expectNear(config.vio.bodyPSensor.translation().x(), 0.10, "默认配置的外参平移与代码默认值一致");
        expectTrue(config.vio.bodyPSensor.rotation().equals(gtsam::Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), 1e-12),
                   "默认配置的外参旋转与代码默认值一致");
        expectTrue(config.intrinsics.isValid(), "默认配置的 dataset 内参合法");
        expectTrue(config.replay.viz.trailMaxPoints == 20000, "默认配置的可视化参数与代码默认值一致");
    }

    /**
     * @brief dataset 节：纯数据结构、路径可清空、非法内参被拒绝
     */
    void checkDatasetSection()
    {
        const auto path = writeLua("dataset", R"(
dataset = {
    intrinsics = { fx = 500.0, imageWidth = 320, imageHeight = 240 },
    replay = { stride = 3, maxFeatures = 120, outputTum = "out.tum", plyPath = "" },
    viz = { showWindow = true, videoPath = "", playbackRate = 1.0 },
}
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(report.project, "dataset 节解析成功且不影响 VIO 配置");
        expectNear(config.intrinsics.fx, 500.0, "dataset.intrinsics.fx 覆盖生效");
        expectTrue(config.intrinsics.imageWidth == 320, "dataset.intrinsics.imageWidth 覆盖生效");
        expectTrue(config.intrinsics.isValid(), "覆盖后的内参仍然合法");
        expectTrue(config.replay.stride == 3, "dataset.replay.stride 覆盖生效");
        expectTrue(config.replay.maxFeatures == 120, "dataset.replay.maxFeatures 覆盖生效");
        expectTrue(config.replay.outputTum == std::filesystem::path("out.tum"), "outputTum 路径生效");
        expectTrue(config.replay.plyPath.empty(), "空串可清空路径字段");
        expectTrue(config.replay.viz.showWindow, "dataset.viz.showWindow 生效");
        expectNear(config.replay.viz.playbackRate, 1.0, "dataset.viz.playbackRate 生效");
        // dataset 与 vio 是独立节，前者不应污染后者
        expectNear(config.vio.fx, 460.0, "dataset 节不影响 vio 内参");

        const auto badIntrinsics = writeLua("dataset-bad", R"(
dataset = { intrinsics = { fx = 0.0 } }
)");
        myslam::config::ProjectConfig badConfig;
        const auto badReport = loadFrom(badIntrinsics, badConfig);
        std::filesystem::remove(badIntrinsics);
        expectTrue(!badReport.project, "非法内参（fx 被钳到 1.0 之外的 0）被拒绝");
    }

} // namespace

int main()
{
    myslam::LogConfig logConfig;
    myslam::configureLogging(logConfig);

    checkOverrides();
    checkImuRateDerivation();
    checkClampingAndTypes();
    checkCrossValidation();
    checkUnknownKeys();
    checkExtrinsic();
    checkFailurePaths();
    checkEmptyScript();
    checkLoggingSection();
    checkShippedConfig();
    checkDatasetSection();

    if (g_failures == 0)
    {
        std::printf("OK: %d checks passed\n", g_checks);
        return 0;
    }
    std::fprintf(stderr, "FAILED: %d / %d checks\n", g_failures, g_checks);
    return 1;
}
