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

    expectTrue(report.keyframeCount > 0, "keyframes were processed");
    expectTrue(report.rmse >= 0.0, "rmse is non-negative");
    expectTrue(report.maxError >= report.rmse, "max error bounds rmse");
    expectTrue(report.rmse < systemConfig.rmseThreshold, "rmse within acceptance threshold");

    myslam::flushLogging();

    std::printf("checks=%d failures=%d (keyframes=%zu rmse=%.4f max=%.4f)\n", g_checks, g_failures,
                report.keyframeCount, report.rmse, report.maxError);
    return g_failures == 0 ? 0 : 1;
}
