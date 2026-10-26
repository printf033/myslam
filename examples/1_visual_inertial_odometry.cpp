/**
 * @file 1_visual_inertial_odometry.cpp
 * @brief VIO 最小系统使用示例：加载 Lua 配置、运行、读取精度报告
 *
 * 系统实现位于 @c src/vio/（见 @c vio/vio.hpp 聚合头），本文件只演示
 * 公开 API 的最小用法。默认配置驱动 5 秒自包含仿真（IMU 200Hz /
 * 相机 20Hz），结束时输出与真值对齐的平移 RMSE。
 *
 * 用法：
 * @code
 * 1_visual_inertial_odometry                  # 读取 config/myslam.lua，缺失则用内置默认值
 * 1_visual_inertial_odometry my_config.lua    # 显式指定配置，加载或校验失败即退出
 * @endcode
 */

#include "log_config.hpp"
#include "logger.hpp"
#include "lua_config.hpp"
#include "vio/vio.hpp"
#include <cstdio>
#include <filesystem>
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

int main(int argc, char **argv)
{
    const bool explicitConfig = argc > 1;
    const std::filesystem::path configPath =
        explicitConfig ? std::filesystem::path(argv[1]) : std::filesystem::path("config/myslam.lua");

    myslam::config::ProjectConfig project;
    myslam::config::LoadReport loadReport;
    const bool configFileExists = std::filesystem::is_regular_file(configPath);

    if (configFileExists)
    {
        loadReport = myslam::config::load(configPath, project);
    }
    else if (explicitConfig)
    {
        std::fprintf(stderr, "[error] 配置文件不存在: %s\n", configPath.string().c_str());
        return 1;
    }

    if (explicitConfig && !loadReport.project)
    {
        std::fprintf(stderr, "[error] 配置文件 %s 校验失败，未修改任何运行参数\n", configPath.string().c_str());
        return 1;
    }

    // 没有可用的 Lua 日志配置时回退到 mylog 的 C++ API
    if (!loadReport.logging)
    {
        myslam::LogConfig logConfig;
        if (!myslam::configureLogging(logConfig))
        {
            std::fprintf(stderr, "[warning] 日志级别无效，沿用当前日志配置\n");
        }
    }

    if (loadReport.project)
    {
        std::fprintf(stderr, "[info] 已加载配置: %s\n", configPath.string().c_str());
    }
    else if (configFileExists)
    {
        // 文件存在但解析失败：原因已由 config 层输出，这里不重复猜测
        std::fprintf(stderr, "[info] 配置文件 %s 未能加载，使用内置默认配置\n", configPath.string().c_str());
    }
    else
    {
        std::fprintf(stderr, "[info] 未找到 %s，使用内置默认配置\n", configPath.string().c_str());
    }

    myslam::vio::VioSystem system(project.vio, project.system);
    const myslam::vio::RunReport runReport = system.run();

    mylog::info("================================================================================");
    mylog::info("✅ VIO 最小系统全流程示例演示完毕。");
    mylog::info("   关键帧样本 {} 帧, 平移 RMSE {:.4f} m, 最大偏差 {:.4f} m", runReport.keyframeCount, runReport.rmse,
                runReport.maxError);
    mylog::info("================================================================================");

    myslam::flushLogging();

    return runReport.rmse < project.system.rmseThreshold ? 0 : 1;
}
