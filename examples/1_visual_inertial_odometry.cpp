/**
 * @file 1_visual_inertial_odometry.cpp
 * @brief VIO 最小系统使用示例：加载 Lua 配置、运行、读取精度报告
 *
 * 系统实现按层分布在 @c src/ 下（见 @c src/vio.hpp 聚合头），本文件只演示
 * 公开 API 的最小用法。默认配置驱动 5 秒自包含仿真（IMU 200Hz /
 * 相机 20Hz），结束时输出与真值对齐的平移 RMSE。
 *
 * 用法：
 * @code
 * 1_visual_inertial_odometry                  # 读取 config/config.lua，缺失则用内置默认值
 * 1_visual_inertial_odometry --config alt.lua # 显式指定配置，加载或校验失败即退出
 * @endcode
 */

#include "config/lua_config.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "vio.hpp"
#include <cstdio>
#include <expected>
#include <filesystem>
#include <format>
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
    // 唯一的命令行参数与 2_subt_dataset 一致：选哪份 Lua 配置。
    std::filesystem::path configPath = "config/config.lua";
    bool explicitConfig = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            std::fprintf(stderr, "用法: 1_visual_inertial_odometry [--config FILE]\n"
                                 "所有算法与时序参数都在 Lua 配置里（vio.* / system.*）。\n");
            return 0;
        }
        if (arg == "--config")
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "--config 缺少文件路径\n");
                return 2;
            }
            configPath = argv[++i];
            explicitConfig = true;
            continue;
        }
        std::fprintf(stderr, "未知参数: %s（所有算法参数都在 Lua 配置里）\n", arg.c_str());
        return 2;
    }

    myslam::config::ProjectConfig project;
    const bool configFileExists = std::filesystem::is_regular_file(configPath);

    // load() 的失败原因由 ConfigError 携带，调用方无需再猜测文件是否存在
    const myslam::config::LoadResult loaded =
        configFileExists ? myslam::config::load(configPath, project)
                         : myslam::config::LoadResult(std::unexpected(
                               myslam::config::ConfigError{myslam::config::ConfigErrorKind::FileNotFound,
                                                           std::format("配置文件不存在: {}", configPath.string())}));

    if (!loaded && explicitConfig)
    {
        std::fprintf(stderr, "[error] %s (%s)\n", loaded.error().message.c_str(), loaded.error().kindName().data());
        return 1;
    }

    // 没有可用的 Lua 日志配置时回退到 mylog 的 C++ API
    const bool loggingApplied = loaded.has_value() && loaded->logging;
    if (!loggingApplied)
    {
        myslam::LogConfig logConfig;
        if (const auto result = myslam::configureLogging(logConfig); !result)
        {
            std::fprintf(stderr, "[warning] %s，沿用当前日志配置\n", result.error().message.c_str());
        }
    }

    if (loaded)
    {
        std::fprintf(stderr, "[info] 已加载配置: %s\n", configPath.string().c_str());
    }
    else
    {
        std::fprintf(stderr, "[info] %s，使用内置默认配置\n", loaded.error().message.c_str());
    }

    myslam::vio::VioBenchmark system(project.vio, project.system);
    const myslam::domain::RunReport runReport = system.run();

    mylog::info("================================================================================");
    if (!runReport.initialized)
    {
        mylog::error("❌ VIO 最小系统未能完成初始化，本次运行没有可评估的轨迹。");
        mylog::info("================================================================================");
        myslam::flushLogging();
        return 1;
    }

    mylog::info("✅ VIO 最小系统全流程示例演示完毕。");
    mylog::info("   关键帧样本 {} 帧, 平移 RMSE {:.4f} m, 最大偏差 {:.4f} m", runReport.keyframeCount, runReport.rmse,
                runReport.maxError);
    mylog::info("================================================================================");

    myslam::flushLogging();

    // 未初始化时 rmse 保持 0，会把“没有任何结果”误判为“完美精度”，
    // 因此判定统一走 isAcceptable（同时检查 initialized 与关键帧数）。
    return myslam::domain::isAcceptable(runReport, project.system.rmseThreshold) ? 0 : 1;
}
