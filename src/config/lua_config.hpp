#pragma once

/**
 * @file lua_config.hpp
 * @brief 沙箱 Lua 配置加载入口：解析 VIO 参数、数据集回放参数与日志配置。
 *
 * 设计目标与子库 mylog 的 @c lua_config.hpp 保持一致：
 * - **沙箱执行**：脚本在未注册标准库的裸 @c lua_State 中运行，io / os / package /
 *   debug 均不可用，配置脚本无法读写文件或启动进程；
 * - **指令预算**：通过 @c lua_sethook 限制脚本执行指令数，避免死循环卡住进程；
 * - **字段校验**：类型不符的字段被忽略并告警，数值越界按安全区间钳位，
 *   未知键（典型为拼写错误）会被点名，自相矛盾的组合直接判定配置非法；
 * - **原子应用**：先解析到临时配置对象，全部校验通过后才写回调用方；
 *   任何一步失败时调用方持有的配置保持原样。
 *
 * 本文件现在只保留"入口 + 原子应用"这一层：
 * - 数据类型见 @c config/project_config.hpp；
 * - 字段取值原语见 @c config/lua_primitives.hpp；
 * - 各配置节的字段与交叉校验见 @c config/lua_sections.hpp。
 *
 * 版本号与项目名由 CMake 生成（见 @c myslam.hpp），不属于运行期配置。
 *
 * 未链接 @c myslam::myslam_lua 时本头文件退化为桩实现，@c load() 恒返回失败，
 * 调用方可以安全地忽略配置文件。
 *
 * @code
 * myslam::config::ProjectConfig cfg;
 * const auto result = myslam::config::load("config/config.lua", cfg);
 * if (!result)
 * {
 *     // 沿用代码内置默认值；result.error() 给出失败类别与说明
 *     std::fprintf(stderr, "[%s] %s\n", result.error().kindName().data(), result.error().message.c_str());
 * }
 * @endcode
 */

#include "config/lua_sections.hpp"
#include "config/project_config.hpp"
#include <cstdio>
#include <expected>
#include <filesystem>
#include <format>
#include <string>

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA
#include <mylog/src/lua_config.hpp>
extern "C"
{
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#endif

namespace myslam::config
{

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA

    /**
     * @brief 从 Lua 脚本加载项目配置与日志配置
     *
     * 执行顺序：
     * 1. 以调用方传入的 @p out 为基线解析 @c vio / @c system / @c dataset 三节到
     *    临时副本，校验全部通过后一次性写回；失败则 @p out 保持不变；
     * 2. 若 @p applyLogging 为真且脚本确实含 @c logger 或 @c fileManager 节，
     *    把同一份脚本交给 @c mylog::reloadConfig 应用，成功后安装 FileManager
     *    共享单文件输出。（@c reloadConfig 只改写配置对象，输出后端仍需调用方
     *    显式安装，mylog 自己的示例也是这么做的；安装后行为与
     *    @c configureLogging 一致。）
     *    两部分各自原子，互不回滚。脚本没有日志节时 @c LoadReport::logging
     *    保持 false，调用方可据此回退到代码默认后端。
     *
     * 未识别的配置节会被忽略，便于在同一个文件里放其它工具的配置。
     *
     * @c dataset 节只包含纯数据（内参、回放与可视化开关），不依赖 OpenCV；
     * 即使本次构建关闭了 @c MYSLAM_BUILD_DATASET_TOOLS 也可以解析。
     *
     * @param config Lua 配置文件路径
     * @param out 待覆盖的项目配置；字段缺失时保留其当前值
     * @param applyLogging 是否同时应用日志配置
     * @return LoadResult 成功时携带 LoadReport；失败时携带 ConfigError 且 @p out 未被修改
     */
    inline LoadResult load(const std::filesystem::path &config, ProjectConfig &out, bool applyLogging = true)
    {
        LoadReport report;

        std::error_code ec;
        if (!std::filesystem::is_regular_file(config, ec))
        {
            detail::reportError("配置文件不存在或不可读: {}", config.string());
            return std::unexpected(
                ConfigError{ConfigErrorKind::FileNotFound, std::format("配置文件不存在或不可读: {}", config.string())});
        }

        lua_State *rawState = luaL_newstate();
        if (rawState == nullptr)
        {
            detail::reportError("无法创建 Lua 状态机（内存不足）");
            return std::unexpected(ConfigError{ConfigErrorKind::LuaRuntime, "无法创建 Lua 状态机（内存不足）"});
        }

        // RAII 持有 lua_State：解析路径里有 std::format / std::string / Values 拷贝
        // 等可能抛出的操作（典型场景是内存耗尽），异常穿出时也必须关闭状态机。
        const auto stateDeleter = [](lua_State *state) { lua_close(state); };
        const std::unique_ptr<lua_State, decltype(stateDeleter)> stateGuard(rawState, stateDeleter);
        lua_State *const L = stateGuard.get();

        lua_sethook(
            L, [](lua_State *state, lua_Debug * /*ar*/)
            { luaL_error(state, "config script aborted: instruction budget exceeded"); }, LUA_MASKCOUNT,
            kInstructionBudget);

        // mode = "t"：只接受文本 chunk。默认的 luaL_loadfile 等价于 mode "bt"，
        // 会执行 LuaJIT 预编译字节码，而字节码不做任何校验（常量槽/栈帧尺寸/
        // 操作数均不验证），是官方明示“不可加载不可信字节码”的内存不安全入口。
        // 一旦放行，本文件头承诺的“脚本无法读写文件或启动进程”就不成立。
        // thirdparty/mylog 的配置加载已采用同一加固。
        if (luaL_loadfilex(L, config.string().c_str(), "t") != LUA_OK || lua_pcall(L, 0, 0, 0) != LUA_OK)
        {
            const char *err = lua_tostring(L, -1);
            const std::string detailText = err != nullptr ? err : "未知 Lua 错误";
            detail::reportError("执行 {} 失败: {}", config.string(), detailText);
            return std::unexpected(
                ConfigError{ConfigErrorKind::LuaRuntime, std::format("执行 {} 失败: {}", config.string(), detailText)});
        }

        ProjectConfig parsed = out;
        bool projectValid = true;
        std::string structureError; // 仅在配置节类型不符时填充

        lua_getglobal(L, "vio");
        if (lua_istable(L, -1))
        {
            const bool vioValid = detail::loadVioSection(L, lua_gettop(L), parsed.vio);
            projectValid = projectValid && vioValid;
        }
        else if (!lua_isnil(L, -1))
        {
            detail::reportError("配置节 vio 应为 table，实际为 {}", detail::typeName(L));
            structureError = std::format("配置节 vio 应为 table，实际为 {}", detail::typeName(L));
            projectValid = false;
        }
        lua_pop(L, 1);

        lua_getglobal(L, "system");
        if (lua_istable(L, -1))
        {
            const bool systemValid =
                detail::loadSystemSection(L, lua_gettop(L), parsed.system, parsed.vio.initialization, parsed.vio);
            projectValid = projectValid && systemValid;
        }
        else if (!lua_isnil(L, -1))
        {
            detail::reportError("配置节 system 应为 table，实际为 {}", detail::typeName(L));
            structureError = std::format("配置节 system 应为 table，实际为 {}", detail::typeName(L));
            projectValid = false;
        }
        lua_pop(L, 1);

        lua_getglobal(L, "dataset");
        if (lua_istable(L, -1))
        {
            const bool datasetValid = detail::loadDatasetSection(L, lua_gettop(L), parsed.intrinsics, parsed.replay,
                                                                 parsed.input, parsed.vio.imu);
            projectValid = projectValid && datasetValid;
        }
        else if (!lua_isnil(L, -1))
        {
            detail::reportError("配置节 dataset 应为 table，实际为 {}", detail::typeName(L));
            structureError = std::format("配置节 dataset 应为 table，实际为 {}", detail::typeName(L));
            projectValid = false;
        }
        lua_pop(L, 1);

        // 探测脚本是否真的写了日志节。mylog::reloadConfig 对“两个节都不存在”
        // 的脚本也返回 true（无字段可校验），若据此就设置 logging 标志，会把
        // “配置文件里没有日志配置”报成“日志已应用”，并静默把输出后端从
        // stdout 改成当前目录下的文件。
        bool hasLoggingSection = false;
        for (const char *sectionName : {"logger", "fileManager"})
        {
            lua_getglobal(L, sectionName);
            hasLoggingSection = hasLoggingSection || lua_istable(L, -1);
            lua_pop(L, 1);
        }

        if (!projectValid)
        {
            const ConfigErrorKind kind =
                structureError.empty() ? ConfigErrorKind::InvalidValue : ConfigErrorKind::InvalidStructure;
            const std::string message =
                structureError.empty() ? std::format("配置文件 {} 的字段校验未通过，详见上方逐条告警", config.string())
                                       : structureError;
            return std::unexpected(ConfigError{kind, message});
        }

        out = parsed;

        if (applyLogging && hasLoggingSection)
        {
            report.logging = mylog::reloadConfig(config);
            if (report.logging)
            {
                // reloadConfig 只改写配置对象，输出后端需要显式安装，否则配置了
                // fileManager 却仍写 stdout。默认后端与 configureLogging 一致。
                mylog::Logger::getInstance().setOutputFunction(mylog::FileManager::createFileOutputFunction());
                mylog::Logger::getInstance().setFlushFunction(mylog::FileManager::createFileFlushFunction());
            }
        }
        return report;
    }

#else // !MYSLAM_HAS_LUA

    /**
     * @brief 未启用 Lua 支持时的桩实现
     *
     * @param config 配置文件路径（仅用于诊断输出）
     * @param out 调用方配置，保持原样
     * @param applyLogging 未使用
     * @return LoadResult 恒为 ConfigError{Unsupported}
     */
    inline LoadResult load(const std::filesystem::path &config, ProjectConfig &out, bool applyLogging = true)
    {
        (void)out;
        (void)applyLogging;
        const std::string message =
            std::format("本构建未启用 Lua 配置支持（MYSLAM_ENABLE_LUA=OFF），无法加载 {}", config.string());
        std::fprintf(stderr, "[myslam] config error: %s\n", message.c_str());
        return std::unexpected(ConfigError{ConfigErrorKind::Unsupported, message});
    }

#endif // MYSLAM_HAS_LUA

} // namespace myslam::config
