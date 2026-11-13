/**
 * @file test_lua_config.cpp
 * @brief Lua 配置加载的库级测试：覆盖解析、钳位、交叉校验与失败路径。
 *
 * 与 test_vio.cpp 相同的断言计数风格：逐条断言、末尾统一报告失败数。
 * 每个用例把 Lua 脚本写到临时目录，测完即删，不污染工作区。
 */

#include "config/lua_config.hpp"
#include "log_config.hpp"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
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

    /// 默认外参旋转：机身前向 +X_b 对应相机光轴 +Z_c
    bool defaultBodyRotation(const myslam::domain::Pose &pose)
    {
        Eigen::Matrix3d expected;
        expected << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        return pose.q.toRotationMatrix().isApprox(expected, 1e-12);
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

    /// 加载指定脚本（不触碰日志配置），返回 std::expected 结果
    myslam::config::LoadResult loadFrom(const std::filesystem::path &path, myslam::config::ProjectConfig &config)
    {
        return myslam::config::load(path, config, /*applyLogging=*/false);
    }

    /// 把配置加载的成败折算成布尔，便于逐条断言
    bool succeeded(const myslam::config::LoadResult &result)
    {
        return result.has_value();
    }

    /**
     * @brief 在加载期间把 stderr 重定向到文件，返回捕获到的文本
     *
     * 配置层用 stderr 输出诊断，而“不该有告警”这类断言必须看到真实的
     * 输出才能成立——只断言数值生效会漏掉“值生效了但同时在报假警”。
     */
    template <typename Fn>
    std::string captureStderr(Fn &&fn)
    {
        const std::filesystem::path logPath =
            std::filesystem::temp_directory_path() / ("myslam_cfg_stderr_" + std::to_string(::getpid()) + ".txt");
        std::fflush(stderr);
        const int savedFd = ::dup(::fileno(stderr));

        if (std::freopen(logPath.string().c_str(), "w", stderr) == nullptr)
        {
            if (savedFd >= 0)
            {
                ::close(savedFd);
            }
            return {};
        }

        fn();

        std::fflush(stderr);
        if (savedFd >= 0)
        {
            ::dup2(savedFd, ::fileno(stderr));
            ::close(savedFd);
        }

        std::ifstream in(logPath);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();
        std::filesystem::remove(logPath);
        return text;
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
    initializerMinSamples = 60,
    healthMaxSpeed = 5.0,
    healthAlertStreakThreshold = 5,
    landmarkPriorSigma = 0.5,
}
system = { totalDuration = 1.25, cameraDt = 0.05 }
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(succeeded(report), "合法配置解析成功");
        expectNear(config.vio.camera.intrinsics.fx, 123.0, "vio.fx 覆盖生效");
        expectNear(config.vio.vision.minTrackingRatio, 0.5, "vio.minTrackingRatio 覆盖生效");
        expectTrue(config.vio.estimator.extraSmootherIterations == 3, "vio.extraSmootherIterations 覆盖生效");
        expectTrue(config.vio.initialization.minSamples == 60, "vio.initializerMinSamples 覆盖生效");
        expectNear(config.vio.health.maxSpeed, 5.0, "vio.healthMaxSpeed 覆盖生效");
        expectTrue(config.vio.health.alertStreakThreshold == 5, "vio.healthAlertStreakThreshold 覆盖生效");
        expectNear(config.vio.estimator.landmarkPriorSigma, 0.5, "vio.landmarkPriorSigma 覆盖生效");
        expectNear(config.system.totalDuration, 1.25, "system.totalDuration 覆盖生效");
        expectNear(myslam::domain::SystemConfig::imuDtFrom(config.vio.imu.rateHz), 0.005,
                   "默认 200 Hz 派生的步长为 5 ms");
        expectNear(config.vio.camera.intrinsics.cy, 240.0, "未出现的字段保持默认值");
        expectNear(config.system.rmseThreshold, 0.20, "未出现的 system 字段保持默认值");
    }

    /**
     * @brief 数值越界应钳位（意图清楚，只是量级错）；类型错误必须判失败
     *
     * 两者性质不同：@c minTrackingRatio = 3.5 的意图是“要求更严的留存率”，
     * 钳到 1.0 不会误导；而 @c cameraDt = "0.05"（从 YAML 拷来的带引号数字）
     * 如果静默保留默认值，用户会以为参数已生效，排障时直接走错方向。
     */
    void checkClampingAndTypes()
    {
        // 纯越界：钳位，配置仍然有效
        const auto clampPath = writeLua("clamp", R"(
vio = {
    minTrackingRatio = 3.5,
    maxSlidingWindowSize = 0,
    gravityMagnitude = -1.0,
}
system = { cameraDt = 0.05 }
)");

        myslam::config::ProjectConfig clampConfig;
        const auto clampReport = loadFrom(clampPath, clampConfig);
        std::filesystem::remove(clampPath);

        expectTrue(succeeded(clampReport), "越界值钳位后配置仍然有效");
        expectNear(clampConfig.vio.vision.minTrackingRatio, 1.0, "minTrackingRatio 钳位到上界 1.0");
        expectTrue(clampConfig.vio.estimator.maxSlidingWindowSize == 1, "maxSlidingWindowSize 钳位到下界 1");
        expectNear(clampConfig.vio.imu.gravityMagnitude, 1e-3, "负重力钳位到下界");

        // 类型错误：必须失败（钳位族字段也一样）
        const auto stringPath = writeLua("type-string", R"(
vio = { fx = "not a number" }
)");
        myslam::config::ProjectConfig stringConfig;
        const auto stringReport = loadFrom(stringPath, stringConfig);
        std::filesystem::remove(stringPath);
        expectTrue(!succeeded(stringReport), "字符串类型的数值字段被判失败（不得静默保持默认）");

        const auto nanPath = writeLua("type-nan", R"(
vio = { fy = 0 / 0 }
)");
        myslam::config::ProjectConfig nanConfig;
        const auto nanReport = loadFrom(nanPath, nanConfig);
        std::filesystem::remove(nanPath);
        expectTrue(!succeeded(nanReport), "NaN 数值字段被判失败");

        const auto boolPath = writeLua("type-bool", R"(
vio = { postOptOutlierMarking = "yes" }
)");
        myslam::config::ProjectConfig boolConfig;
        const auto boolReport = loadFrom(boolPath, boolConfig);
        std::filesystem::remove(boolPath);
        expectTrue(!succeeded(boolReport), "非布尔的布尔字段被判失败");

        const auto sysPath = writeLua("type-system", R"(
system = { cameraDt = "0.05" }
)");
        myslam::config::ProjectConfig sysConfig;
        const auto sysReport = loadFrom(sysPath, sysConfig);
        std::filesystem::remove(sysPath);
        expectTrue(!succeeded(sysReport), "system 节的类型错误同样判失败");
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

        expectTrue(succeeded(report), "合法采样率配置解析成功");
        expectNear(config.vio.imu.rateHz, 100.0, "vio.imuRateHz 覆盖生效");
        expectNear(myslam::config::ProjectConfig{}.system.cameraDt, 0.05, "未配置的 cameraDt 保持默认");
        expectNear(myslam::domain::SystemConfig::imuDtFrom(config.vio.imu.rateHz), 0.01, "100 Hz 派生出 10 ms 步长");

        // 旧配置残留 imuDt：应被忽略（不改变派生结果），且不致失败
        const auto legacy = writeLua("legacy-imu-dt", R"(
vio = { imuRateHz = 200.0 }
system = { imuDt = 0.02, cameraDt = 0.05 }
)");
        myslam::config::ProjectConfig legacyConfig;
        const auto legacyReport = loadFrom(legacy, legacyConfig);
        std::filesystem::remove(legacy);

        expectTrue(succeeded(legacyReport), "旧配置的 imuDt 不影响加载成功");
        expectNear(myslam::domain::SystemConfig::imuDtFrom(legacyConfig.vio.imu.rateHz), 0.005,
                   "旧 imuDt 被忽略，仍由采样率派生");

        // system.initialHeightMeters 必须是已接线字段：先前它存在于
        // SystemConfig 却在白名单里缺席，写进配置只会得到“未知键”告警并被忽略。
        const auto height = writeLua("initial-height", R"(
system = { cameraDt = 0.05, initialHeightMeters = 2.5 }
)");
        myslam::config::ProjectConfig heightConfig;
        const auto heightReport = loadFrom(height, heightConfig);
        std::filesystem::remove(height);

        expectTrue(succeeded(heightReport), "system.initialHeightMeters 可被解析");
        expectNear(heightConfig.vio.initialization.initialHeightMeters, 2.5, "system.initialHeightMeters 覆盖生效");
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

        expectTrue(!succeeded(report), "关键帧间隔矛盾时配置被拒绝");
        expectNear(config.vio.camera.intrinsics.fx, 460.0, "失败时不写回调用方配置");

        const auto path2 = writeLua("camera-faster", R"(
vio = { imuRateHz = 20.0 }
system = { cameraDt = 0.005 }
)");
        myslam::config::ProjectConfig config2;
        const auto report2 = loadFrom(path2, config2);
        std::filesystem::remove(path2);

        expectTrue(!succeeded(report2), "相机快于 IMU 时配置被拒绝");
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

        expectTrue(succeeded(report), "未知键只告警、不失败");
        expectNear(config.vio.camera.intrinsics.fx, 460.0, "拼错的内参字段未生效");
        expectNear(config.vio.imu.gyroNoiseSigma, 3.6e-4, "拼错的噪声字段未生效");
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

        expectTrue(succeeded(report), "合法外参解析成功");
        // 四元数 (2,0,0,0) 单位化后是恒等旋转
        expectTrue(config.vio.camera.bodyPSensor.q.toRotationMatrix().isApprox(Eigen::Matrix3d::Identity(), 1e-12),
                   "外参四元数被单位化为恒等旋转");
        expectNear(config.vio.camera.bodyPSensor.t.x(), 1.5, "外参平移 x 生效");
        expectNear(config.vio.camera.bodyPSensor.t.y(), -2.0, "外参平移 y 生效");
        expectNear(config.vio.camera.bodyPSensor.t.z(), 0.25, "外参平移 z 生效");

        const auto zeroPath = writeLua("extrinsic-zero", R"(
vio = { extrinsic = { quaternion = { w = 0, x = 0, y = 0, z = 0 } } }
)");
        myslam::config::ProjectConfig config2;
        const auto report2 = loadFrom(zeroPath, config2);
        std::filesystem::remove(zeroPath);
        expectTrue(!succeeded(report2), "零模长四元数被拒绝");
    }

    /**
     * @brief 文件缺失、语法错误、节类型错误都应返回失败而不抛异常
     */
    void checkFailurePaths()
    {
        myslam::config::ProjectConfig config;
        const auto missing = loadFrom(std::filesystem::temp_directory_path() / "myslam_cfg_does_not_exist.lua", config);
        expectTrue(!succeeded(missing), "配置文件缺失时返回失败");
        expectTrue(missing.error().kind == myslam::config::ConfigErrorKind::FILE_NOT_FOUND,
                   "缺失文件归类为 FileNotFound");
        expectTrue(!missing.error().message.empty(), "失败时携带说明文本");

        const auto syntax = writeLua("syntax", "vio = { fx = ");
        const auto syntaxReport = loadFrom(syntax, config);
        std::filesystem::remove(syntax);
        expectTrue(!succeeded(syntaxReport), "语法错误的脚本返回失败");
        expectTrue(syntaxReport.error().kind == myslam::config::ConfigErrorKind::LUA_RUNTIME,
                   "语法错误归类为 LuaRuntime");

        const auto wrongType = writeLua("wrong-type", "vio = 42\nsystem = { totalDuration = 1.0 }");
        myslam::config::ProjectConfig config2;
        const auto wrongTypeReport = loadFrom(wrongType, config2);
        std::filesystem::remove(wrongType);
        expectTrue(!succeeded(wrongTypeReport), "配置节不是 table 时返回失败");
        expectTrue(wrongTypeReport.error().kind == myslam::config::ConfigErrorKind::INVALID_STRUCTURE,
                   "配置节类型错误归类为 InvalidStructure");
        expectNear(config2.system.totalDuration, 5.0, "失败时不写回调用方配置");

        // 字段校验失败（对比度矛盾）应与结构错误区分开
        const auto contradiction = writeLua("contradiction-kind", R"(
vio = { minKeyframeInterval = 0.5, maxKeyframeInterval = 0.2 }
)");
        myslam::config::ProjectConfig config3;
        const auto contradictionReport = loadFrom(contradiction, config3);
        std::filesystem::remove(contradiction);
        expectTrue(contradictionReport.error().kind == myslam::config::ConfigErrorKind::INVALID_VALUE,
                   "交叉约束失败归类为 InvalidValue");
        expectTrue(contradictionReport.error().kindName() == "InvalidValue", "kindName 与枚举一致");
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

        expectTrue(succeeded(report), "空脚本解析成功");
        expectNear(config.vio.camera.intrinsics.fx, 460.0, "空脚本保持默认内参");
        expectNear(config.system.totalDuration, 5.0, "空脚本保持默认时长");
    }

    /**
     * @brief 相机模型：针孔 / MEI 由 @c vio.model 选择，后端只吃虚拟针孔
     */
    void checkCameraModelSection()
    {
        const auto path = writeLua("camera-model-mei", R"(
vio = {
    model = "mei",
    fx = 700.0,
    fy = 650.0,
    cx = 320.0,
    cy = 240.0,
    pinholeScale = 0.5,
}
)");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(succeeded(report), "MEI 相机模型配置解析成功");
        expectTrue(config.vio.camera.intrinsics.model == myslam::camera::CameraModel::MEI, "vio.model = mei 生效");
        expectNear(config.vio.camera.intrinsics.pinholeFx(), 350.0, "MEI 虚拟针孔焦距为 fx * pinholeScale");
        expectNear(config.vio.camera.intrinsics.pinholeFy(), 325.0, "MEI 虚拟针孔焦距为 fy * pinholeScale");

        const auto badPath = writeLua("camera-model-typo", R"(
vio = { model = "fisheye" }
)");
        myslam::config::ProjectConfig badConfig;
        const auto badReport = loadFrom(badPath, badConfig);
        std::filesystem::remove(badPath);
        expectTrue(!succeeded(badReport), "未知相机模型名被拒绝");

        // pinholeScale 是几何硬前提，不允许钳位：1e-6 被洗成 1e-3 会得到
        // 0.46 px 的虚拟焦距，配置“合法”但整条流水线产出垃圾。
        const auto scalePath = writeLua("camera-model-bad-scale", R"(
vio = { model = "mei", pinholeScale = 1e-6 }
)");
        myslam::config::ProjectConfig scaleConfig;
        const auto scaleReport = loadFrom(scalePath, scaleConfig);
        std::filesystem::remove(scalePath);
        expectTrue(!succeeded(scaleReport), "越界的 pinholeScale 被拒绝而不是钳位");

        // 默认必须是针孔：鱼眼默认会把仿真器的针孔投影当作畸变处理
        expectTrue(myslam::domain::PipelineConfig{}.camera.intrinsics.model == myslam::camera::CameraModel::PINHOLE,
                   "PipelineConfig 默认相机模型为 pinhole");
    }

    /**
     * @brief 数据集内参的模型切换：针孔不得继承 MEI 的球面反投影参数
     */
    void checkDatasetCameraModel()
    {
        // 默认基线是 SubT 的 MEI（xi = 1.583）。把模型改成 pinhole 时，xi 必须
        // 被清零：前端不按模型分支，而是靠 xi = 0 让 MEI 公式退化为针孔。
        const auto path = writeLua("dataset-camera-pinhole", R"(
dataset = { intrinsics = { model = "pinhole", fx = 500.0, fy = 500.0, cx = 320, cy = 240 } }
)");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(succeeded(report), "dataset.intrinsics.model = pinhole 解析成功");
        expectTrue(!config.intrinsics.isMei(), "数据集内参模型为针孔");
        expectNear(config.intrinsics.xi, 0.0, "针孔模型下 xi 被清零");
        expectNear(config.intrinsics.pinholeFx(), 500.0, "针孔模型不使用 pinholeScale");

        // 回归：从 MEI 基线切到 pinhole、且未写 xi 时，归一化是正常结果，
        // 不得报成配置错误——否则每个合法的针孔序列配置都会带一条假告警。
        myslam::config::ProjectConfig silentConfig;
        const auto silentPath = writeLua("dataset-camera-pinhole-silent", R"(
dataset = {
    intrinsics = { model = "pinhole", fx = 500.0, fy = 500.0, cx = 320, cy = 240, k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0 }
}
)");
        const std::string captured = captureStderr([&] { (void)loadFrom(silentPath, silentConfig); });
        std::filesystem::remove(silentPath);
        expectTrue(captured.empty(), "合法的针孔配置不产生任何诊断");

        // 显式写了 xi 却选针孔时，要明确告知该字段不生效（不静默丢弃）
        myslam::config::ProjectConfig xiConfig;
        const auto xiPath = writeLua("dataset-camera-pinhole-xi", R"(
dataset = { intrinsics = { model = "pinhole", fx = 500.0, fy = 500.0, xi = 1.5 } }
)");
        const std::string xiCaptured = captureStderr([&] { (void)loadFrom(xiPath, xiConfig); });
        std::filesystem::remove(xiPath);
        expectTrue(xiCaptured.find("xi") != std::string::npos, "显式给出的 xi 被提示为不生效");
        expectNear(xiConfig.intrinsics.xi, 0.0, "显式 xi 在针孔模型下仍被归一化为 0");

        const auto meiPath = writeLua("dataset-camera-mei", R"(
dataset = { intrinsics = { model = "mei", fx = 800.0, pinholeScale = 0.5 } }
)");
        myslam::config::ProjectConfig meiConfig;
        const auto meiReport = loadFrom(meiPath, meiConfig);
        std::filesystem::remove(meiPath);
        expectTrue(succeeded(meiReport), "dataset.intrinsics.model = mei 解析成功");
        expectNear(meiConfig.intrinsics.pinholeFx(), 400.0, "MEI 模型应用 pinholeScale");
        expectTrue(meiConfig.intrinsics.xi != 0.0, "MEI 模型保留 xi");
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

        expectTrue(succeeded(report), "无 vio / system 节时项目配置仍然成功");
        expectTrue(report->logging, "logger 节被 mylog 接受");
        expectTrue(mylog::getCurrentSuppressLevel() == mylog::LEVEL::WARN, "suppressLevel 已真实生效");

        // 恢复级别，避免影响后续用例与测试框架输出
        mylog::setSuppressLevel("info");
    }

    /**
     * @brief 项目自带的默认配置必须能通过全部校验
     */
    void checkShippedConfig()
    {
        const std::filesystem::path candidates[] = {"config/config.lua", "../config/config.lua"};
        std::filesystem::path found;
        for (const auto &candidate : candidates)
        {
            if (std::filesystem::is_regular_file(candidate))
            {
                found = candidate;
                break;
            }
        }
        expectTrue(!found.empty(), "找到仓库自带的 config/config.lua");
        if (found.empty())
        {
            return;
        }

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(found, config);
        expectTrue(succeeded(report), "仓库默认配置通过全部校验");
        expectNear(config.vio.camera.intrinsics.fx, 460.0, "默认配置的 fx 与代码默认值一致");
        expectNear(config.vio.imu.accelNoiseSigma, 0.0057, "默认配置的加速度噪声与代码默认值一致");
        expectNear(config.vio.camera.bodyPSensor.t.x(), 0.10, "默认配置的外参平移与代码默认值一致");
        expectTrue(defaultBodyRotation(config.vio.camera.bodyPSensor), "默认配置的外参旋转与代码默认值一致");
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

        expectTrue(succeeded(report), "dataset 节解析成功且不影响 VIO 配置");
        expectNear(config.intrinsics.fx, 500.0, "dataset.intrinsics.fx 覆盖生效");
        expectTrue(config.intrinsics.imageWidth == 320, "dataset.intrinsics.imageWidth 覆盖生效");
        expectTrue(config.intrinsics.isValid(), "覆盖后的内参仍然合法");
        expectTrue(config.replay.stride == 3, "dataset.replay.stride 覆盖生效");
        expectTrue(config.replay.maxFeatures == 120, "dataset.replay.maxFeatures 覆盖生效");
        expectTrue(config.replay.outputTum == std::filesystem::path("out.tum"), "outputTum 路径生效");
        expectTrue(config.replay.plyPath.empty(), "空串可清空路径字段");
        expectTrue(config.replay.viz.showWindow, "dataset.viz.showWindow 生效");
        expectNear(config.replay.viz.playbackRate, 1.0, "dataset.viz.playbackRate 生效");
        // IMU 断档阈值属于回放时序参数，必须可从配置覆盖
        expectNear(config.replay.maxImuGapFactor, myslam::dataset::DatasetRunConfig{}.maxImuGapFactor,
                   "未配置 maxImuGapFactor 时保持默认");
        expectTrue(myslam::dataset::DatasetRunConfig{}.maxImuGapFactor > 0.0, "断档倍数默认为正数");
        // 单一真值来源：dataset.replay.maxImuGapFactor 必须真正搬到
        // ImuConfig::maxGapFactor（TimeSynchronizer 读的是后者）。
        // 否则解析成功而行为不变，是最难发现的失效模式。
        expectNear(config.vio.imu.maxGapFactor, config.replay.maxImuGapFactor,
                   "断档倍数单一真值：dataset.replay 的值接入 ImuConfig");
        // dataset 与 vio 是独立节，前者不应污染后者
        expectNear(config.vio.camera.intrinsics.fx, 460.0, "dataset 节不影响 vio 内参");

        const auto gapPath = writeLua("dataset-gap", R"(
dataset = { replay = { maxImuGapFactor = 4.5 } }
)");
        myslam::config::ProjectConfig gapConfig;
        const auto gapReport = loadFrom(gapPath, gapConfig);
        std::filesystem::remove(gapPath);
        expectTrue(succeeded(gapReport), "maxImuGapFactor 可配置");
        expectNear(gapConfig.replay.maxImuGapFactor, 4.5, "maxImuGapFactor 覆盖生效");
        expectNear(gapConfig.vio.imu.maxGapFactor, 4.5,
                   "maxImuGapFactor 真正接入 ImuConfig（TimeSynchronizer 读的是这个）");

        // 倍数小于 2 会把正常采样抖动误判为断档，因此钳位到下界
        const auto tooSmall = writeLua("dataset-gap-small", R"(
dataset = { replay = { maxImuGapFactor = 0.5 } }
)");
        myslam::config::ProjectConfig tooSmallConfig;
        const auto tooSmallReport = loadFrom(tooSmall, tooSmallConfig);
        std::filesystem::remove(tooSmall);
        expectTrue(succeeded(tooSmallReport), "过小的断档倍数不致命");
        expectNear(tooSmallConfig.replay.maxImuGapFactor, 2.0, "过小倍数钳位到 2.0");

        const auto badIntrinsics = writeLua("dataset-bad", R"(
dataset = { intrinsics = { fx = 0.0 } }
)");
        myslam::config::ProjectConfig badConfig;
        const auto badReport = loadFrom(badIntrinsics, badConfig);
        std::filesystem::remove(badIntrinsics);
        expectTrue(!succeeded(badReport), "非法内参（fx 被钳到 1.0 之外的 0）被拒绝");
    }

    /**
     * @brief dataset.sequence / intrinsics.source / mast3r / extrinsic
     *
     * 这四个节是「回放不再依赖命令行参数」的载体：序列位置、内参来源、
     * 估计选项与安装外参都必须能从 Lua 完整表达，否则删掉命令行只会把
     * 功能一起删掉。
     */
    void checkDatasetInputSection()
    {
        const auto path = writeLua("dataset-input", R"(
dataset = {
    sequence = {
        root = "/tmp/subt/Handheld2_Folder",
        cameraDir = "cam_0",
        imuFile = "imu/imu_data.csv",
        stampFile = "cam_0/timestamps.txt",
    },
    intrinsics = { source = "mast3r", file = "/tmp/calib.txt" },
    mast3r = { python = "/opt/conda/bin/python", pairs = 12, device = "cuda", force = true },
    extrinsic = {
        quaternion = { w = 1.0, x = 0.0, y = 0.0, z = 0.0 },
        translation = { x = 0.2, y = 0.0, z = 0.0 },
    },
}
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(succeeded(report), "dataset 输入节解析成功");
        expectTrue(config.input.sequence.configured(), "sequence.root 已配置");
        expectTrue(config.input.sequence.resolveCameraDir() ==
                       std::filesystem::path("/tmp/subt/Handheld2_Folder/cam_0"),
                   "cameraDir 相对 root 解析");
        expectTrue(config.input.sequence.resolveImuFile() ==
                       std::filesystem::path("/tmp/subt/Handheld2_Folder/imu/imu_data.csv"),
                   "imuFile 相对 root 解析");
        expectTrue(config.input.intrinsicsSource == myslam::dataset::IntrinsicsSource::MAST3R,
                   "intrinsics.source = mast3r 生效");
        expectTrue(config.input.intrinsicsFile == std::filesystem::path("/tmp/calib.txt"), "intrinsics.file 生效");
        expectTrue(config.input.mast3r.pairs == 12, "mast3r.pairs 生效");
        expectTrue(config.input.mast3r.device == "cuda", "mast3r.device 生效");
        expectTrue(config.input.mast3r.force, "mast3r.force 生效");
        expectTrue(config.input.mast3r.python == "/opt/conda/bin/python", "mast3r.python 生效");
        expectTrue(config.input.hasExtrinsic, "dataset.extrinsic 被标记为已配置");
        expectNear(config.input.extrinsic.t.x(), 0.2, "回放外参平移生效");

        // 默认：未配置 sequence 与 extrinsic
        const auto minimal = writeLua("dataset-input-minimal", R"(
dataset = { intrinsics = { fx = 500.0 } }
)");
        myslam::config::ProjectConfig minimalConfig;
        const auto minimalReport = loadFrom(minimal, minimalConfig);
        std::filesystem::remove(minimal);
        expectTrue(succeeded(minimalReport), "只有 internal 字段的 dataset 节仍合法");
        expectTrue(!minimalConfig.input.sequence.configured(), "未配置时 sequence.root 为空");
        expectTrue(!minimalConfig.input.hasExtrinsic, "未配置时 hasExtrinsic 为 false");
        expectTrue(minimalConfig.input.intrinsicsSource == myslam::dataset::IntrinsicsSource::CONFIG,
                   "默认内参来源为 config");

        // source = "file" 但没有给文件：属于配置自相矛盾，必须拒绝
        const auto missingFile = writeLua("dataset-input-missing-file", R"(
dataset = { intrinsics = { source = "file", file = "" } }
)");
        myslam::config::ProjectConfig missingConfig;
        const auto missingReport = loadFrom(missingFile, missingConfig);
        std::filesystem::remove(missingFile);
        expectTrue(!succeeded(missingReport), "source = file 但 file 为空时被拒绝");

        // 非法枚举值不得静默退回默认
        const auto badSource = writeLua("dataset-input-bad-source", R"(
dataset = { intrinsics = { source = "opencv" } }
)");
        myslam::config::ProjectConfig badSourceConfig;
        const auto badSourceReport = loadFrom(badSource, badSourceConfig);
        std::filesystem::remove(badSource);
        expectTrue(!succeeded(badSourceReport), "未知 intrinsics.source 被拒绝");

        const auto badDevice = writeLua("dataset-input-bad-device", R"(
dataset = { mast3r = { device = "tpu" } }
)");
        myslam::config::ProjectConfig badDeviceConfig;
        const auto badDeviceReport = loadFrom(badDevice, badDeviceConfig);
        std::filesystem::remove(badDevice);
        expectTrue(!succeeded(badDeviceReport), "未知 mast3r.device 被拒绝");
    }

} // namespace

/* =============================================================================
 * 以下是针对已修复缺陷的回归用例。每个用例都对应一个曾经真实发生过的
 * 故障（崩溃、静默劣化、假成功或假告警），失败即意味着缺陷回流。
 * ========================================================================== */
namespace
{
    /**
     * @brief 回归：零噪声配置必须被拒（不得静默钳位）
     *
     * 0 不是“无噪声”：仿真器会把 0 传给 std::normal_distribution（σ 必须 > 0，
     * libstdc++ 断言 abort）；因子图侧 Isotropic::Sigma(…, 0) 给出无穷信息矩阵，
     * gtsam 抛 IndeterminateSystemException。两者都能直接杀死进程。
     *
     * 即使钳到 1e-12 也不够：信息矩阵会到 1e18 量级，Cholesky 仍然失败
     * （实测会产生大量 “Partial Cholesky on HessianFactor failed”）。
     * 因此与 fx = 0 一样按“不允许自动纠正”处理，而不是洗成一个魔法数值。
     */
    void checkZeroNoiseRejected()
    {
        const char *const fields[] = {"accelNoiseSigma", "gyroNoiseSigma", "accelBiasRw", "gyroBiasRw",
                                      "pixelNoiseSigma"};
        for (const char *field : fields)
        {
            const auto path =
                writeLua(std::string("zero-noise-") + field, std::string("vio = { ") + field + " = 0.0 }\n");
            myslam::config::ProjectConfig config;
            const auto report = loadFrom(path, config);
            std::filesystem::remove(path);
            expectTrue(!succeeded(report), (std::string("零值被拒绝: ") + field).c_str());
        }

        // 极小但物理上合理的噪声仍应被接受，否则会误伤正常配置
        const auto tiny = writeLua("tiny-noise", R"(
vio = { pixelNoiseSigma = 0.01, accelNoiseSigma = 0.001 }
)");
        myslam::config::ProjectConfig tinyConfig;
        const auto tinyReport = loadFrom(tiny, tinyConfig);
        std::filesystem::remove(tiny);
        expectTrue(succeeded(tinyReport), "小但合理的噪声值被接受");
        expectNear(tinyConfig.vio.vision.pixelNoiseSigma, 0.01, "pixelNoiseSigma 小值原样生效");
    }

    /**
     * @brief 回归：超大整数必须钳到上界，不能因转换 UB 反转成下界
     *
     * 旧实现先 static_cast<long long>(1e300) 再钳位，转换本身是 UB，
     * x86-64 上得到 INT64_MIN，于是 maxFrames 被钳到 0 = “不限”，
     * 与“极大上限”的意图正好相反。
     */
    void checkHugeIntegerClampsUp()
    {
        const auto path = writeLua("huge-int", R"(
dataset = { replay = { maxFrames = 1e300, startFrame = 1e300 } }
)");

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(succeeded(report), "超大整数被钳位后配置仍然有效");
        expectTrue(config.replay.maxFrames > 0, "maxFrames 钳到上界而不是反转成 0（0 意为不限）");
        expectTrue(config.replay.maxFrames == 10'000'000, "maxFrames 钳到上界 1e7");
        expectTrue(config.replay.startFrame == 10'000'000, "startFrame 钳到上界 1e7");

        // 下方向同样要正确：负值应钳到 0，而不是乱七八糟的极值
        const auto negPath = writeLua("neg-int", R"(
dataset = { replay = { maxFrames = -1e300 } }
)");
        myslam::config::ProjectConfig negConfig;
        const auto negReport = loadFrom(negPath, negConfig);
        std::filesystem::remove(negPath);
        expectTrue(succeeded(negReport), "极小整数被钳位后配置仍然有效");
        expectTrue(negConfig.replay.maxFrames == 0, "负无穷钳到上界 0");
    }

    /**
     * @brief 回归：固定滞后窗口小于关键帧间隔必须判非法
     *
     * 窗口为 0 时每帧进来就立即边缘化，活跃关键帧恒为 1，实测 RMSE
     * 从 0.04 m 恶化到 0.18 m，而旧实现完全静默。
     */
    void checkSlidingWindowLagGuarded()
    {
        const auto zeroPath = writeLua("lag-zero", R"(
vio = { slidingWindowLag = 0.0 }
)");
        myslam::config::ProjectConfig zeroConfig;
        const auto zeroReport = loadFrom(zeroPath, zeroConfig);
        std::filesystem::remove(zeroPath);
        // 0 先被钳到 1e-3，再因小于 maxKeyframeInterval(0.45) 被交叉校验拒绝
        expectTrue(!succeeded(zeroReport), "slidingWindowLag = 0 被拒绝（不允许静默退化为单帧）");

        const auto tooShort = writeLua("lag-short", R"(
vio = { slidingWindowLag = 0.2, maxKeyframeInterval = 0.45 }
)");
        myslam::config::ProjectConfig shortConfig;
        const auto shortReport = loadFrom(tooShort, shortConfig);
        std::filesystem::remove(tooShort);
        expectTrue(!succeeded(shortReport), "窗口短于关键帧间隔被拒绝");

        const auto ok = writeLua("lag-ok", R"(
vio = { slidingWindowLag = 1.6, maxKeyframeInterval = 0.45 }
)");
        myslam::config::ProjectConfig okConfig;
        const auto okReport = loadFrom(ok, okConfig);
        std::filesystem::remove(ok);
        expectTrue(succeeded(okReport), "合理的窗口配置被接受");
    }

    /**
     * @brief 回归：状态栏高度为 0、画幅过大、主点越界都必须拦住
     */
    void checkVisualAndGeometryGuards()
    {
        const auto panel = writeLua("panel-zero", R"(
dataset = { viz = { showWindow = true, panelHeight = 0 } }
)");
        myslam::config::ProjectConfig panelConfig;
        const auto panelReport = loadFrom(panel, panelConfig);
        std::filesystem::remove(panel);
        expectTrue(succeeded(panelReport), "panelHeight = 0 钳位后配置仍然有效");
        expectTrue(panelConfig.replay.viz.panelHeight >= 1, "panelHeight 钳到下界 1（0 行的 Mat 会断言失败）");

        // 逐维合法但像素总数失控：1e10 像素会让 cv::Mat::zeros 直接 bad_alloc
        const auto huge = writeLua("huge-frame", R"(
dataset = { intrinsics = { imageWidth = 100000, imageHeight = 100000 } }
)");
        myslam::config::ProjectConfig hugeConfig;
        const auto hugeReport = loadFrom(huge, hugeConfig);
        std::filesystem::remove(huge);
        expectTrue(!succeeded(hugeReport), "画幅像素总数超过上限被拒绝");

        // 主点落在画幅外：全部重投影都会越界
        const auto principal = writeLua("principal-out", R"(
vio = { imageWidth = 640.0, imageHeight = 480.0, cx = 900.0 }
)");
        myslam::config::ProjectConfig principalConfig;
        const auto principalReport = loadFrom(principal, principalConfig);
        std::filesystem::remove(principal);
        expectTrue(!succeeded(principalReport), "主点越出画幅被拒绝");
    }

    /**
     * @brief 路标深度有效区间：默认值、可配置性，以及反转区间必须被拒绝
     *
     * 深度区间是几何有效性判据（见 gtsam 适配器的预优化门控）。它一旦被配反，
     * 后端会静默丢弃全部视觉观测，退化成纯 IMU 积分而不报任何错误——正是
     * 那种“跑得通但结果全错”的失效模式，因此必须在这里拦住。
     */
    void checkLandmarkDepthConfig()
    {
        const myslam::domain::PipelineConfig defaults;
        expectNear(defaults.vision.minLandmarkDepth, 0.3, "默认深度下限为 0.3 m");
        expectNear(defaults.vision.maxLandmarkDepth, 20.0, "默认深度上限为 20.0 m");
        expectTrue(defaults.vision.minLandmarkDepth < defaults.vision.maxLandmarkDepth, "默认深度区间非空");

        const auto path = writeLua("depth-override", R"(
vio = { minLandmarkDepth = 0.5, maxLandmarkDepth = 12.5 }
)");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);
        expectTrue(succeeded(report), "合法的深度区间配置解析成功");
        expectNear(config.vio.vision.minLandmarkDepth, 0.5, "vio.minLandmarkDepth 覆盖生效");
        expectNear(config.vio.vision.maxLandmarkDepth, 12.5, "vio.maxLandmarkDepth 覆盖生效");

        // 反转：max <= min 时区间为空，全部观测都会被判几何无效
        const auto reversed = writeLua("depth-reversed", R"(
vio = { minLandmarkDepth = 5.0, maxLandmarkDepth = 2.0 }
)");
        myslam::config::ProjectConfig reversedConfig;
        const auto reversedReport = loadFrom(reversed, reversedConfig);
        std::filesystem::remove(reversed);
        expectTrue(!succeeded(reversedReport), "深度区间反转时配置被拒绝");

        // 相等同样使区间为空：这是边界，不能因为“不小于”就放行
        const auto equal = writeLua("depth-equal", R"(
vio = { minLandmarkDepth = 3.0, maxLandmarkDepth = 3.0 }
)");
        myslam::config::ProjectConfig equalConfig;
        const auto equalReport = loadFrom(equal, equalConfig);
        std::filesystem::remove(equal);
        expectTrue(!succeeded(equalReport), "深度上下限相等时配置被拒绝");
    }

    /**
     * @brief 回归：strict 字段的类型错误不能被当成“字段缺失”而静默放行
     *
     * 旧实现下 dataset.intrinsics.fx = "758.3"（YAML 拷来的带引号数字）会静默
     * 保留默认值并通过加载，与 fx = 0 被判非法的行为不一致。
     */
    void checkStrictTypeErrors()
    {
        const auto str = writeLua("strict-str", R"(
dataset = { intrinsics = { fx = "758.3" } }
)");
        myslam::config::ProjectConfig strConfig;
        const auto strReport = loadFrom(str, strConfig);
        std::filesystem::remove(str);
        expectTrue(!succeeded(strReport), "strict 字段的字符串类型被拒绝");

        const auto nan = writeLua("strict-nan", R"(
dataset = { intrinsics = { fy = 0 / 0 } }
)");
        myslam::config::ProjectConfig nanConfig;
        const auto nanReport = loadFrom(nan, nanConfig);
        std::filesystem::remove(nan);
        expectTrue(!succeeded(nanReport), "strict 字段的 NaN 被拒绝");

        // 字段真的缺失时必须仍然放行（保留默认值）
        const auto missing = writeLua("strict-missing", R"(
dataset = { intrinsics = { cx = 318.0 } }
)");
        myslam::config::ProjectConfig missingConfig;
        const auto missingReport = loadFrom(missing, missingConfig);
        std::filesystem::remove(missing);
        expectTrue(succeeded(missingReport), "strict 字段缺失时保留默认值并放行");
    }

    /**
     * @brief 回归：字节码配置必须被拒（沙箱前提）
     *
     * luaL_loadfile 默认 mode 为 "bt"，会执行 LuaJIT 预编译字节码；
     * 字节码不做任何校验，是官方明示的内存不安全入口。这里用伪造的
     * "\x1bLJ" 头验证 loader 已限制为仅文本。
     */
    void checkBytecodeRejected()
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() / ("myslam_cfg_bytecode_" + std::to_string(::getpid()) + ".lua");
        {
            std::ofstream out(path, std::ios::binary);
            // LuaJIT 字节码魔数：ESC 'L' 'J' 加版本号
            const unsigned char header[] = {0x1b, 0x4c, 0x4a, 0x01, 0x00};
            out.write(reinterpret_cast<const char *>(header), sizeof(header));
            out << "\x01\x00\x00\x00";
        }

        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);

        expectTrue(!succeeded(report), "预编译字节码配置被拒绝");
        if (!report)
        {
            expectTrue(report.error().kind == myslam::config::ConfigErrorKind::LUA_RUNTIME,
                       "字节码被拒时归类为 LuaRuntime");
            // 关键区分：只看“加载失败”不够——一个截断的垃圾 chunk 在任何 mode
            // 下都会失败，那样测试在修复前后都成立，等于没测。必须验证失败原因
            // 是 loader 的“模式不被接受”，而不是“字节码内容坏”。
            const std::string what = report.error().message;
            const bool rejectedByMode = what.find("mode") != std::string::npos;
            expectTrue(rejectedByMode, "拒绝原因必须是 loader mode 限制，而非字节码内容错");
            if (!rejectedByMode)
            {
                std::fprintf(stderr, "  实际错误信息: %s\n", what.c_str());
            }
        }

        // 对照：纯文本源码必须正常接受，证明 loader 本身没被改坏
        const auto textPath = writeLua("bytecode-control", R"(
vio = { fx = 500.0 }
)");
        myslam::config::ProjectConfig textConfig;
        const auto textReport = loadFrom(textPath, textConfig);
        std::filesystem::remove(textPath);
        expectTrue(succeeded(textReport), "文本源码仍被正常接受（对照）");
        expectNear(textConfig.vio.camera.intrinsics.fx, 500.0, "文本源码的值生效（对照）");
    }

    /**
     * @brief 回归：仓库自带配置不得产生任何诊断告警
     *
     * maxImuGapFactor 曾经被解析但漏登记到 known 列表，导致每次启动都打印
     * “未知字段（拼写错误？）”——而该值实际生效。这类假告警会训练用户
     * 忽略真告警，比不报警更危险。
     */
    void checkShippedConfigIsSilent()
    {
        std::filesystem::path shipped;
        for (const char *candidate : {"config/config.lua", "../config/config.lua", "../../config/config.lua"})
        {
            if (std::filesystem::is_regular_file(candidate))
            {
                shipped = candidate;
                break;
            }
        }
        if (shipped.empty())
        {
            std::printf("SKIP: 未找到仓库自带配置，跳过零告警检查\n");
            return;
        }

        myslam::config::ProjectConfig config;
        myslam::config::LoadResult report =
            std::unexpected(myslam::config::ConfigError{myslam::config::ConfigErrorKind::FILE_NOT_FOUND, "not loaded"});
        const std::string captured = captureStderr([&] { report = loadFrom(shipped, config); });

        expectTrue(succeeded(report), "仓库自带配置能被正确加载");
        expectTrue(captured.empty(), "仓库自带配置加载时不产生任何 stderr 诊断");
        if (!captured.empty())
        {
            std::fprintf(stderr, "  实际 stderr 输出:\n%s\n", captured.c_str());
        }
    }

    /**
     * @brief 回归：无日志节的脚本不得把 logging 报成 true
     *
     * mylog::reloadConfig 对“两个节都不存在”的脚本也返回 true，旧实现据此
     * 把 LoadReport::logging 置为 true 并静默把输出后端换成文件——配置里根本
     * 没有日志配置，stdout 上的日志却消失了。
     */
    void checkLoggingFlagRequiresSection()
    {
        const auto noLog = writeLua("no-logging", R"(
vio = { fx = 460.0 }
)");
        myslam::config::ProjectConfig config;
        const auto report = myslam::config::load(noLog, config, /*applyLogging=*/true);
        std::filesystem::remove(noLog);

        expectTrue(succeeded(report), "无日志节的配置加载成功");
        if (report)
        {
            expectTrue(!report->logging, "脚本没有日志节时 logging 必须为 false");
        }
    }

    /**
     * @brief 回归：初始化超时（原 system.initTimeout）可配置且有合法默认值
     */
    void checkInitTimeout()
    {
        expectTrue(myslam::domain::PipelineConfig{}.initialization.timeoutSeconds > 0.0, "初始化超时默认为正数");

        const auto path = writeLua("init-timeout", R"(
system = { initTimeout = 3.5 }
)");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);
        expectTrue(succeeded(report), "initTimeout 可配置");
        expectNear(config.vio.initialization.timeoutSeconds, 3.5, "system.initTimeout 覆盖生效");
    }

    /**
     * @brief 回归：估计器名字来自配置，未知名字在装配期失败
     */
    void checkEstimatorSelection()
    {
        expectTrue(!myslam::domain::PipelineConfig{}.estimator.name.empty(), "默认估计器名字非空（注册表键）");

        const auto path = writeLua("estimator-name", R"(
vio = { estimator = "imu_only" }
)");
        myslam::config::ProjectConfig config;
        const auto report = loadFrom(path, config);
        std::filesystem::remove(path);
        expectTrue(succeeded(report), "vio.estimator 可被解析");
        expectTrue(config.vio.estimator.name == "imu_only", "vio.estimator 覆盖生效");

        // 类型错误必须报错而不是静默保留默认值
        const auto badType = writeLua("estimator-bad-type", R"(
vio = { estimator = 42 }
)");
        myslam::config::ProjectConfig badConfig;
        const auto badReport = loadFrom(badType, badConfig);
        std::filesystem::remove(badType);
        expectTrue(!succeeded(badReport), "vio.estimator 非字符串时配置失败");
        expectTrue(badConfig.vio.estimator.name == myslam::domain::PipelineConfig{}.estimator.name,
                   "类型错误不改写默认估计器");
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
    checkCameraModelSection();
    checkDatasetCameraModel();
    checkLoggingSection();
    checkShippedConfig();
    checkDatasetSection();
    checkDatasetInputSection();

    // 针对已修复缺陷的回归用例
    checkZeroNoiseRejected();
    checkHugeIntegerClampsUp();
    checkSlidingWindowLagGuarded();
    checkVisualAndGeometryGuards();
    checkLandmarkDepthConfig();
    checkStrictTypeErrors();
    checkBytecodeRejected();
    checkShippedConfigIsSilent();
    checkLoggingFlagRequiresSection();
    checkInitTimeout();
    checkEstimatorSelection();

    if (g_failures == 0)
    {
        std::printf("OK: %d checks passed\n", g_checks);
        return 0;
    }
    std::fprintf(stderr, "FAILED: %d / %d checks\n", g_failures, g_checks);
    return 1;
}
