#pragma once

/**
 * @file project_config.hpp
 * @brief 项目配置的纯数据类型与加载结果，不含 Lua、不含仿真编排。
 *
 * 这些类型原先定义在 @c lua_config.hpp 里，而该文件同时承载了 Lua 交互原语
 * 与逐节解析，导致任何只想引用 @c ProjectConfig 的代码都要把整份解析器
 * 拉进编译。拆出来后：
 *
 * - @c lua_config.hpp 只保留沙箱执行与入口 @c load()；
 * - @c lua_primitives.hpp 承载 Lua 字段取值原语（怎么安全取一个值）；
 * - @c lua_sections.hpp 承载各配置节的字段与交叉校验（每个节有哪些字段）；
 * - 本文件只依赖数据结构的属主头（@c domain/config.hpp、@c domain/camera_intrinsics.hpp、
 *   @c common/dataset_config.hpp），不依赖 LuaJIT。
 *
 * 这样配置的"形状"与"怎么读"分开，替换配置来源（Lua / JSON / 命令行）时
 * 只需另写一个解析器，@c ProjectConfig 的消费方一行都不用改。
 */

#include "common/dataset_config.hpp"
#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include <expected>
#include <string>
#include <string_view>

namespace myslam::config
{

    /**
     * @brief 一次加载得到的项目配置
     *
     * 字段未出现在 Lua 脚本中时保留调用方传入的当前值，因此可以叠加加载。
     */
    struct ProjectConfig
    {
        domain::PipelineConfig vio{};        ///< 视觉惯性参数
        domain::SystemConfig system{};       ///< 运行时序与验收门限
        dataset::MeiIntrinsics intrinsics{}; ///< 数据集相机内参（可以是针孔或 MEI）；具体数值由 lua 配置文件给出
        dataset::DatasetRunConfig replay{};  ///< 数据集回放与可视化配置
        dataset::DatasetInputConfig input{}; ///< 序列位置、内参来源与 MASt3R 选项
    };

    /**
     * @brief 配置加载失败的类别
     */
    enum class ConfigErrorKind
    {
        FILE_NOT_FOUND,    ///< 路径不存在或不是普通文件
        LUA_RUNTIME,       ///< Lua 虚拟机创建失败、脚本语法错误或执行出错
        INVALID_STRUCTURE, ///< 配置节类型不符（例如 `vio = 42`）
        INVALID_VALUE,     ///< 字段值越界、缺失或违反交叉约束
        UNSUPPORTED,       ///< 本次构建未启用 Lua 支持
    };

    /**
     * @brief 配置加载失败的具体原因
     *
     * 只携带定位问题所需的最小信息；详细告警仍逐条写入 stderr。
     */
    struct ConfigError
    {
        ConfigErrorKind kind{ConfigErrorKind::INVALID_VALUE}; ///< 失败类别
        std::string message;                                  ///< 面向使用者的说明

        /**
         * @brief 类别名称，便于日志与测试断言
         */
        [[nodiscard]] constexpr std::string_view kindName() const noexcept
        {
            switch (kind)
            {
            case ConfigErrorKind::FILE_NOT_FOUND:
                return "FileNotFound";
            case ConfigErrorKind::LUA_RUNTIME:
                return "LuaRuntime";
            case ConfigErrorKind::INVALID_STRUCTURE:
                return "InvalidStructure";
            case ConfigErrorKind::INVALID_VALUE:
                return "InvalidValue";
            case ConfigErrorKind::UNSUPPORTED:
                return "Unsupported";
            }
            return "Unknown";
        }
    };

    /**
     * @brief 加载成功时的附带信息
     *
     * 项目参数已写入调用方的 @c ProjectConfig；日志配置是独立的一部分，
     * 其失败不影响项目参数，因此单独以标志报告而不作为错误返回。
     */
    struct LoadReport
    {
        bool logging{false}; ///< 脚本含 logger / fileManager 节且已由 mylog 应用
    };

    /**
     * @brief 配置加载结果
     *
     * 失败时不修改调用方的 @c ProjectConfig，因此可以安全地沿用旧配置。
     */
    using LoadResult = std::expected<LoadReport, ConfigError>;

    /// 脚本最多可执行的 Lua 指令数，与 mylog 的配置加载保持同一量级
    inline constexpr int kInstructionBudget = 50'000'000;

} // namespace myslam::config
