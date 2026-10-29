/**
 * @file test_vio.cpp
 * @brief VIO 最小系统库级冒烟测试
 *
 * 验证公开 API（VioConfig / SystemConfig / VioSystem / RunReport）可用，
 * 并确认默认配置下的平移精度达到验收门限。断言计数风格与 mylog tests 一致。
 */

#include "log_config.hpp"
#include "vio/vio.hpp"
#include <cstdio>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
/**
 * @brief LeakSanitizer 默认抑制规则（与示例一致）
 *
 * gtsam 的全局 debug 标志表 (gtsam/base/debug.cpp) 在进程退出阶段受静态析构
 * 顺序影响会被误报为泄漏；该分配不属于本项目代码，按调用帧抑制。
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
} // namespace

int main()
{
    myslam::LogConfig logConfig;
    myslam::configureLogging(logConfig);

    myslam::vio::VioConfig config;
    myslam::vio::SystemConfig systemConfig;

    expectTrue(config.fx > 0.0 && config.fy > 0.0, "camera intrinsics are positive");
    expectTrue(myslam::vio::SystemConfig::imuDtFrom(config.imuRateHz) > 0.0 && systemConfig.cameraDt > 0.0,
               "sample steps are positive");
    expectTrue(myslam::vio::SystemConfig::imuDtFrom(0.0) == 0.0, "non-positive imu rate yields zero step");
    expectTrue(systemConfig.rmseThreshold > 0.0, "rmse threshold is positive");

    myslam::vio::VioSystem system(config, systemConfig);
    const myslam::vio::RunReport report = system.run();

    expectTrue(report.initialized, "default run completes initialization");
    expectTrue(report.keyframeCount > 0, "keyframes were processed");
    expectTrue(report.rmse >= 0.0, "rmse is non-negative");
    expectTrue(report.maxError >= report.rmse, "max error bounds rmse");
    expectTrue(report.rmse < systemConfig.rmseThreshold, "rmse within acceptance threshold");
    expectTrue(myslam::vio::isAcceptable(report, systemConfig.rmseThreshold),
               "default run is accepted by isAcceptable");

    // 回归：未初始化时绝不得“报告成功”。
    // RunReport::rmse 在无样本时保持 0，如果调用方只看 rmse < 门限，
    // 就会把“零关键帧”误判为“完美精度”——这正是修复前的实际行为：
    // totalDuration 极短时示例仍以 0 退出并打印“演示完毕”。
    {
        myslam::vio::SystemConfig tooShort;
        tooShort.totalDuration = 0.001; // 远不足 40 条 IMU 样本，初始化必然失败
        myslam::vio::VioSystem shortSystem(config, tooShort);
        const myslam::vio::RunReport shortReport = shortSystem.run();

        expectTrue(!shortReport.initialized, "run too short to initialize reports initialized=false");
        expectTrue(shortReport.keyframeCount == 0, "no keyframes when initialization never completed");
        // 关键断言：此时 rmse 看似“满分”（=0），但验收判定必须拒绝它。
        expectTrue(shortReport.rmse < tooShort.rmseThreshold,
                   "empty evaluation leaves rmse at 0 (the misleading value)");
        expectTrue(!myslam::vio::isAcceptable(shortReport, tooShort.rmseThreshold),
                   "empty evaluation must not be accepted despite rmse being 0");
    }

    // 回归：isAcceptable 本身必须拒绝三种“看似成功”的报告
    {
        myslam::vio::RunReport healthy;
        healthy.initialized = true;
        healthy.keyframeCount = 10;
        healthy.rmse = 0.05;
        expectTrue(myslam::vio::isAcceptable(healthy, 0.20), "a genuinely good run is accepted");

        myslam::vio::RunReport notInitialized = healthy;
        notInitialized.initialized = false;
        notInitialized.keyframeCount = 0;
        notInitialized.rmse = 0.0;
        expectTrue(!myslam::vio::isAcceptable(notInitialized, 0.20), "rmse=0 without initialization is rejected");

        myslam::vio::RunReport noKeyframes = healthy;
        noKeyframes.keyframeCount = 0;
        expectTrue(!myslam::vio::isAcceptable(noKeyframes, 0.20), "zero keyframes is rejected");

        // 回归：初始化会无条件压入一条与真值几乎重合的首帧位姿，
        // 只有它一条时 rmse 也接近 0，但零次优化根本没有轨迹。
        myslam::vio::RunReport onlyAnchor = healthy;
        onlyAnchor.keyframeCount = 1;
        onlyAnchor.rmse = 0.0;
        expectTrue(!myslam::vio::isAcceptable(onlyAnchor, 0.20),
                   "single initial-anchor sample must not be accepted as a trajectory");

        myslam::vio::RunReport inaccurate = healthy;
        inaccurate.rmse = 5.0;
        expectTrue(!myslam::vio::isAcceptable(inaccurate, 0.20), "rmse above threshold is rejected");
    }

    // 回归：路标深度有效区间必须非空，且标称运行不应触发流水线重建
    //
    // 深度区间被配反时后端会静默丢弃全部视觉观测（退化为纯 IMU 积分），
    // 配置层已拒绝该组合，这里守住默认值不退化。
    {
        expectTrue(config.minLandmarkDepth > 0.0, "landmark depth lower bound is positive");
        expectTrue(config.maxLandmarkDepth > config.minLandmarkDepth, "landmark depth default interval is non-empty");

        // 标称仿真既不缺 IMU 数据也没有数值退化：重建次数必须为 0。
        // 若此处非 0，说明求解失败被局部恢复吞掉了（轨迹仍连续但帧被丢弃），
        // 或健康监控误判——两种情况都该在测试里暴露而不是留到数据集回放。
        expectTrue(report.resets == 0, "a nominal run performs no pipeline rebuild");
    }

    // 回归：健康监控需要连续异常才降级（单帧抖动不得触发流水线重建）
    {
        myslam::vio::HealthMonitor monitor;
        const gtsam::NavState healthyState(gtsam::Pose3(), gtsam::Vector3::Zero());
        const gtsam::imuBias::ConstantBias healthyBias;
        expectTrue(monitor.inspect(healthyState, healthyBias, 100).isHealthy, "healthy state passes immediately");

        const gtsam::NavState fastState(gtsam::Pose3(), gtsam::Vector3(50.0, 0.0, 0.0));
        for (int i = 1; i < myslam::vio::HealthMonitor::kAlertStreakThreshold; ++i)
        {
            expectTrue(monitor.inspect(fastState, healthyBias, 100).isHealthy,
                       "single-frame anomaly does not trigger degradation");
        }
        expectTrue(!monitor.inspect(fastState, healthyBias, 100).isHealthy,
                   "sustained anomaly triggers degradation once threshold reached");

        monitor.reset();
        expectTrue(monitor.inspect(healthyState, healthyBias, 100).isHealthy, "reset clears the streak");
        expectTrue(monitor.streak() == 0, "streak is zero after recovery");
    }

    // 回归：降级类型必须可区分，否则上层无法判断该不该重建坐标系
    //
    // 视觉中断（图像变暗/遮挡）重建流水线毫无作用：下一帧图像依旧是黑的，
    // 重建只会让坐标系反复归零。实测 732 s 处的黑暗通道因此产生过 52 次
    // 重建、52 次跳回原点。惯性失稳（零偏发散/速度跳变）则确实需要重建。
    {
        using myslam::vio::HealthMonitor;
        const gtsam::NavState okState(gtsam::Pose3(), gtsam::Vector3::Zero());
        const gtsam::imuBias::ConstantBias okBias;

        const auto visual = HealthMonitor::diagnose(okState, okBias, 0);
        expectTrue(!visual.isHealthy, "zero active features is unhealthy");
        expectTrue(visual.degradation == HealthMonitor::Degradation::VISUAL,
                   "observation starvation is classified as VISUAL, not INERTIAL");

        const gtsam::NavState fastState(gtsam::Pose3(), gtsam::Vector3(50.0, 0.0, 0.0));
        const auto inertial = HealthMonitor::diagnose(fastState, okBias, 100);
        expectTrue(!inertial.isHealthy, "absurd speed is unhealthy");
        expectTrue(inertial.degradation == HealthMonitor::Degradation::INERTIAL,
                   "speed jump is classified as INERTIAL, which does require a rebuild");

        const auto healthy = HealthMonitor::diagnose(okState, okBias, 100);
        expectTrue(healthy.isHealthy && healthy.degradation == HealthMonitor::Degradation::NONE,
                   "healthy state carries no degradation kind");
    }

    myslam::flushLogging();

    std::printf("checks=%d failures=%d (keyframes=%zu rmse=%.4f max=%.4f)\n", g_checks, g_failures,
                report.keyframeCount, report.rmse, report.maxError);
    return g_failures == 0 ? 0 : 1;
}
