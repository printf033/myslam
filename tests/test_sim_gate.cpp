/**
 * @file test_sim_gate.cpp
 * @brief 端到端精度门禁：从真实 Lua 配置加载，跑到验收，用退出码表达结论。
 *
 * @par 为什么单独一个目标
 *
 * `test_vio.cpp` 验证的是「代码默认配置下流水线正确」，用的是内存里手工
 * 构造的 `PipelineConfig`。它证明不了另一件事：**配置文件里写的那一套值
 * 能不能跑出可接受的精度**。配置解析、默认值叠加、交叉校验三者都对，
 * 但组合出来的参数让精度掉出门限——这种情况只有端到端跑一次才看得见。
 *
 * 本目标把 `config/config.lua` 当作唯一的参数来源，跑完整仿真，然后：
 *   1. 用 `isAcceptable` 判精度；
 *   2. 用 `isConsistent` 判一致性（NEES 是否在带上）。
 * 任一不通过即非零退出，ctest 因此成为真正的门禁而不只是编译检查。
 *
 * 与 CI 的关系：这条路径是确定性的（合成数据、固定步长、无外部数据），
 * 所以门禁不会因为环境差异而抖动。
 */

#include "adapters/register_builtin.hpp"
#include "app/benchmark.hpp"
#include "config/project_config.hpp"
#include "domain/config.hpp"
#include "log_config.hpp"
#include "logger.hpp"

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA
#include "config/lua_config.hpp"
#endif

#include <cstdio>
#include <string>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
/**
 * @brief LeakSanitizer 抑制：gtsam 的全局 debug 标志表在静态析构阶段误报
 *
 * 与 `test_vio.cpp`、两个示例入口保持同一份抑制规则：该分配不属于本项目
 * 代码，按调用帧抑制。缺了它，Debug + ASan 下本测试恒失败。
 */
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:gtsam::guardedIsDebug\n";
}
#endif

namespace
{

    /// 一致性带：真值期望 NEES 均值为 3，这里留出很宽的回归容差
    constexpr double kNeEsLowerBound = 0.2;
    constexpr double kNeEsUpperBound = 20.0;

} // namespace

int main()
{
    myslam::LogConfig logConfig;
    myslam::configureLogging(logConfig);
    myslam::adapters::registerBuiltinEstimators();

#if !(defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA)
    std::fprintf(stderr, "[sim-gate] 本次构建未启用 Lua，无法从配置文件加载；跳过\n");
    return 0;
#else
    const std::string configPath = "config/config.lua";

    myslam::config::ProjectConfig project;
    const auto loaded = myslam::config::load(configPath, project);
    if (!loaded)
    {
        std::fprintf(stderr, "[sim-gate] 配置加载失败 (%s): %s\n", std::string(loaded.error().kindName()).c_str(),
                     loaded.error().message.c_str());
        return 1;
    }

    myslam::vio::VioBenchmark benchmark(project.vio, project.system);
    const myslam::domain::RunReport report = benchmark.run();

    bool passed = true;

    if (!myslam::domain::isAcceptable(report, project.system.rmseThreshold))
    {
        std::fprintf(stderr, "[sim-gate] 精度门禁失败: 初始化=%d, 关键帧=%zu, RMSE=%.4f m, 门限=%.3f m\n",
                     static_cast<int>(report.initialized), report.keyframeCount, report.rmse,
                     project.system.rmseThreshold);
        passed = false;
    }

    if (report.neesSamples == 0)
    {
        std::fprintf(stderr, "[sim-gate] 一致性门禁失败: 后端未上报任何位姿协方差\n");
        passed = false;
    }
    else if (!myslam::domain::isConsistent(report, kNeEsLowerBound, kNeEsUpperBound))
    {
        std::fprintf(stderr, "[sim-gate] 一致性门禁失败: NEES 均值 %.3f 不在 [%.2f, %.2f] (理论 3)\n", report.neesMean,
                     kNeEsLowerBound, kNeEsUpperBound);
        passed = false;
    }

    if (!passed)
    {
        return 1;
    }

    std::printf("[sim-gate] 通过: 关键帧 %zu, RMSE %.4f m, NEES 均值 %.3f (%zu 样本)\n", report.keyframeCount,
                report.rmse, report.neesMean, report.neesSamples);
    return 0;
#endif
}
