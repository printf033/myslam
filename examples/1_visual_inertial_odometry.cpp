/**
 * @file 1_visual_inertial_odometry.cpp
 * @brief VIO 最小系统使用示例：配置日志与系统、运行、读取精度报告
 *
 * 系统实现位于 @c src/vio/（见 @c vio/vio.hpp 聚合头），本文件只演示
 * 公开 API 的最小用法。默认配置驱动 5 秒自包含仿真（IMU 200Hz /
 * 相机 20Hz），结束时输出与真值对齐的平移 RMSE。
 */

#include "log_config.hpp"
#include "logger.hpp"
#include "vio/vio.hpp"
#include <cstdio>
#include <string>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
/**
 * @brief LeakSanitizer 默认抑制规则
 *
 * gtsam 的全局 debug 标志表 (gtsam/base/debug.cpp) 在进程退出阶段受静态析构
 * 顺序影响，会被 LSan 误报为泄漏；该分配不属于本项目代码，按调用帧抑制。
 */
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:gtsam::guardedIsDebug\n";
}
#endif

int main()
{
    myslam::LogConfig logConfig;
    if (!myslam::configureLogging(logConfig))
    {
        std::fprintf(stderr, "[warning] 日志级别无效，沿用当前日志配置\n");
    }
    else
    {
        std::fprintf(stderr, "[info] 日志由 mylog::FileManager 输出到当前工作目录\n");
    }

    myslam::vio::VioConfig config;
    myslam::vio::SystemConfig systemConfig;

    myslam::vio::VioSystem system(config, systemConfig);
    const myslam::vio::RunReport report = system.run();

    mylog::info("================================================================================");
    mylog::info("✅ VIO 最小系统全流程示例演示完毕。");
    mylog::info("   关键帧样本 {} 帧, 平移 RMSE {:.4f} m, 最大偏差 {:.4f} m", report.keyframeCount, report.rmse,
                report.maxError);
    mylog::info("================================================================================");

    myslam::flushLogging();

    return report.rmse < systemConfig.rmseThreshold ? 0 : 1;
}
