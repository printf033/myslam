#pragma once

/**
 * @file lua_config.hpp
 * @brief 沙箱 Lua 配置加载：VIO 系统参数（VioConfig / SystemConfig）与日志配置。
 *
 * 设计目标与子库 mylog 的 @c configer_lua.hpp 保持一致：
 * - **沙箱执行**：脚本在未注册标准库的裸 @c lua_State 中运行，io / os / package /
 *   debug 均不可用，配置脚本无法读写文件或启动进程；
 * - **指令预算**：通过 @c lua_sethook 限制脚本执行指令数，避免死循环卡住进程；
 * - **字段校验**：类型不符的字段被忽略并告警，数值越界按安全区间钳位，
 *   未知键（典型为拼写错误）会被点名，自相矛盾的组合直接判定配置非法；
 * - **原子应用**：先解析到临时配置对象，全部校验通过后才写回调用方；
 *   任何一步失败时调用方持有的配置保持原样。
 *
 * 版本号与项目名由 CMake 生成（见 @c myslam.hpp），不属于运行期配置。
 *
 * 未链接 @c myslam::myslam_lua 时本头文件退化为桩实现，@c load() 恒返回失败，
 * 调用方可以安全地忽略配置文件。
 *
 * @code
 * myslam::config::ProjectConfig cfg;
 * const auto result = myslam::config::load("config/myslam.lua", cfg);
 * if (!result)
 * {
 *     // 沿用代码内置默认值；result.error() 给出失败类别与说明
 *     std::fprintf(stderr, "[%s] %s\n", result.error().kindName().data(), result.error().message.c_str());
 * }
 * @endcode
 */

#include "dataset/types.hpp"
#include "vio/params.hpp"
#include "vio/system.hpp"
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <expected>
#include <filesystem>
#include <format>
#include <initializer_list>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA
#include "configer_lua.hpp"
extern "C"
{
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#endif

namespace myslam::config
{

    /**
     * @brief 一次加载得到的项目配置
     *
     * 字段未出现在 Lua 脚本中时保留调用方传入的当前值，因此可以叠加加载。
     */
    struct ProjectConfig
    {
        vio::VioConfig vio{};                ///< 视觉惯性参数
        vio::SystemConfig system{};          ///< 运行时序与验收门限
        dataset::MeiIntrinsics intrinsics{}; ///< 数据集相机内参（MEI 模型）
        dataset::DatasetRunConfig replay{};  ///< 数据集回放与可视化配置
    };

    /**
     * @brief 配置加载失败的类别
     */
    enum class ConfigErrorKind
    {
        FileNotFound,     ///< 路径不存在或不是普通文件
        LuaRuntime,       ///< Lua 虚拟机创建失败、脚本语法错误或执行出错
        InvalidStructure, ///< 配置节类型不符（例如 `vio = 42`）
        InvalidValue,     ///< 字段值越界、缺失或违反交叉约束
        Unsupported,      ///< 本次构建未启用 Lua 支持
    };

    /**
     * @brief 配置加载失败的具体原因
     *
     * 只携带定位问题所需的最小信息；详细告警仍逐条写入 stderr。
     */
    struct ConfigError
    {
        ConfigErrorKind kind{ConfigErrorKind::InvalidValue}; ///< 失败类别
        std::string message;                                 ///< 面向使用者的说明

        /**
         * @brief 类别名称，便于日志与测试断言
         */
        [[nodiscard]] constexpr std::string_view kindName() const noexcept
        {
            switch (kind)
            {
            case ConfigErrorKind::FileNotFound:
                return "FileNotFound";
            case ConfigErrorKind::LuaRuntime:
                return "LuaRuntime";
            case ConfigErrorKind::InvalidStructure:
                return "InvalidStructure";
            case ConfigErrorKind::InvalidValue:
                return "InvalidValue";
            case ConfigErrorKind::Unsupported:
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
        bool logging{false}; ///< logger / fileManager 节已由 mylog 应用
    };

    /**
     * @brief 配置加载结果
     *
     * 失败时不修改调用方的 @c ProjectConfig，因此可以安全地沿用旧配置。
     */
    using LoadResult = std::expected<LoadReport, ConfigError>;

    /// 脚本最多可执行的 Lua 指令数，与 mylog 的配置加载保持同一量级
    inline constexpr int kInstructionBudget = 50'000'000;

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA

    namespace detail
    {

        /**
         * @brief 向标准错误输出配置诊断信息
         *
         * 配置加载可能早于日志系统就绪，因此统一写 stderr 而不经过 mylog；
         * 使用 std::format 避免格式化串注入。
         *
         * @param fmt std::format 格式化字符串（编译期检查）
         * @param args 格式化参数
         */
        template <typename... Args>
        inline void reportError(std::format_string<Args...> fmt, Args &&...args)
        {
            std::fputs(
                std::format("[myslam] config error: {}\n", std::format(fmt, std::forward<Args>(args)...)).c_str(),
                stderr);
        }

        /**
         * @brief 把数值钳位到闭区间 [lo, hi]
         */
        template <typename T>
        [[nodiscard]] inline T clampValue(T value, T lo, T hi)
        {
            return value < lo ? lo : (value > hi ? hi : value);
        }

        /// 取栈顶值的 Lua 类型名，用于诊断信息
        inline const char *typeName(lua_State *L)
        {
            return lua_typename(L, lua_type(L, -1));
        }

        /**
         * @brief 读取一个有限浮点字段，字段缺失或类型错误时返回 false
         *
         * NaN / Inf 一律视为非法：Lua 里一个 @c 0/0 就能写出来，若放行会污染整条
         * 滤波链路且难以定位。
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param out 读取成功时写入
         * @return true 字段存在、类型为数字且为有限值
         * @return false 字段缺失或非法
         */
        inline bool readFiniteDouble(lua_State *L, int idx, std::string_view section, const char *key, double &out)
        {
            bool ok = false;
            lua_getfield(L, idx, key);
            if (lua_type(L, -1) == LUA_TNUMBER)
            {
                const double raw = static_cast<double>(lua_tonumber(L, -1));
                if (std::isfinite(raw))
                {
                    out = raw;
                    ok = true;
                }
                else
                {
                    reportError("{}.{} 不是有限数值，已忽略", section, key);
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.{} 应为数字，实际为 {}，已忽略", section, key, typeName(L));
            }
            lua_pop(L, 1);
            return ok;
        }

        /**
         * @brief 读取浮点字段并钳位到安全区间
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param lo 允许下界（含）
         * @param hi 允许上界（含）
         * @param field 待写入字段；字段缺失或类型错误时保持原值
         */
        inline void loadDouble(lua_State *L, int idx, std::string_view section, const char *key, double lo, double hi,
                               double &field)
        {
            double value = field;
            if (!readFiniteDouble(L, idx, section, key, value))
            {
                return;
            }
            const double clamped = clampValue(value, lo, hi);
            if (clamped != value)
            {
                reportError("{}.{} = {} 超出安全区间 [{}, {}]，已钳位为 {}", section, key, value, lo, hi, clamped);
            }
            field = clamped;
        }

        /**
         * @brief 读取浮点字段但不允许自动纠正：越界即判定配置非法
         *
         * 用于“钳位会洗白用户错误”的字段：例如焦距写 0 若被钳到 1.0，
         * 表面合法但物理上荒谬，几何链路会在运行期才崩。
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param lo 允许下界（含）
         * @param hi 允许上界（含）
         * @param field 待写入字段
         * @return true 字段缺失（保留原值）或取值合法
         * @return false 字段存在但越界
         */
        inline bool loadDoubleStrict(lua_State *L, int idx, std::string_view section, const char *key, double lo,
                                     double hi, double &field)
        {
            double value = field;
            if (!readFiniteDouble(L, idx, section, key, value))
            {
                return true;
            }
            if (value < lo || value > hi)
            {
                reportError("{}.{} = {} 超出合法区间 [{}, {}]（该字段不允许自动纠正）", section, key, value, lo, hi);
                return false;
            }
            field = value;
            return true;
        }

        /**
         * @brief 读取整数型字段但不允许自动纠正：越界即判定配置非法
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param lo 允许下界（含）
         * @param hi 允许上界（含）
         * @param field 待写入字段
         * @return true 字段缺失（保留原值）或取值合法
         * @return false 字段存在但越界
         */
        inline bool loadIntegerStrict(lua_State *L, int idx, std::string_view section, const char *key, long long lo,
                                      long long hi, int &field)
        {
            double value = 0.0;
            if (!readFiniteDouble(L, idx, section, key, value))
            {
                return true;
            }
            const long long raw = static_cast<long long>(value);
            if (raw < lo || raw > hi)
            {
                reportError("{}.{} = {} 超出合法区间 [{}, {}]（该字段不允许自动纠正）", section, key, raw, lo, hi);
                return false;
            }
            field = static_cast<int>(raw);
            return true;
        }

        /**
         * @brief 读取布尔字段
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param field 待写入字段；字段缺失或类型错误时保持原值
         */
        inline void loadBool(lua_State *L, int idx, std::string_view section, const char *key, bool &field)
        {
            lua_getfield(L, idx, key);
            if (lua_type(L, -1) == LUA_TBOOLEAN)
            {
                field = lua_toboolean(L, -1) != 0;
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.{} 应为布尔值，实际为 {}，已忽略", section, key, typeName(L));
            }
            lua_pop(L, 1);
        }

        /**
         * @brief 读取字符串字段；空串视为“未设置”
         *
         * 路径类字段以空串表示“不输出”，因此允许写 @c "" 清空默认值。
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param field 待写入路径；字段缺失或类型错误时保持原值
         */
        inline void loadPath(lua_State *L, int idx, std::string_view section, const char *key,
                             std::filesystem::path &field)
        {
            lua_getfield(L, idx, key);
            if (lua_type(L, -1) == LUA_TSTRING)
            {
                size_t length = 0;
                const char *text = lua_tolstring(L, -1, &length);
                field = std::filesystem::path(std::string(text, length));
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.{} 应为字符串路径，实际为 {}，已忽略", section, key, typeName(L));
            }
            lua_pop(L, 1);
        }

        /**
         * @brief 读取整数型字段并钳位到安全区间
         *
         * @tparam Int 目标整型（int / size_t 等）
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param lo 允许下界（含）
         * @param hi 允许上界（含）
         * @param field 待写入字段；字段缺失或类型错误时保持原值
         */
        template <typename Int>
        inline void loadInteger(lua_State *L, int idx, std::string_view section, const char *key, long long lo,
                                long long hi, Int &field)
            requires std::is_integral_v<Int>
        {
            double value = 0.0;
            if (!readFiniteDouble(L, idx, section, key, value))
            {
                return;
            }
            if (value != std::floor(value))
            {
                reportError("{}.{} = {} 不是整数，按 {} 取整", section, key, value, static_cast<long long>(value));
            }
            const long long raw = static_cast<long long>(value);
            const long long clamped = clampValue(raw, lo, hi);
            if (clamped != raw)
            {
                reportError("{}.{} = {} 超出安全区间 [{}, {}]，已钳位为 {}", section, key, raw, lo, hi, clamped);
            }
            field = static_cast<Int>(clamped);
        }

        /**
         * @brief 点名配置节中不被识别的键
         *
         * 拼写错误的字段被静默忽略是最难排查的配置故障，因此在解析末尾遍历表的
         * 键集合，对未登记的键逐条告警（不视为致命错误）。
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param known 该节合法键名列表
         */
        inline void checkUnknownKeys(lua_State *L, int idx, std::string_view section,
                                     std::initializer_list<std::string_view> known)
        {
            lua_pushnil(L);
            while (lua_next(L, idx) != 0)
            {
                if (lua_type(L, -2) == LUA_TSTRING)
                {
                    const std::string_view key(lua_tostring(L, -2));
                    bool found = false;
                    for (const std::string_view candidate : known)
                    {
                        if (candidate == key)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        reportError("{} 中出现未知字段 \"{}\"（拼写错误？），已忽略", section, key);
                    }
                }
                lua_pop(L, 1); // 仅弹出值，保留键供下一轮 lua_next 使用
            }
        }

        /**
         * @brief 读取四元数的四个分量，要求全部存在且为数字
         *
         * @param L Lua 状态机
         * @param idx 四元数表在栈上的索引
         * @param w 输出实部
         * @param x 输出第一虚部
         * @param y 输出第二虚部
         * @param z 输出第三虚部
         * @return true 四个分量齐全且合法
         * @return false 存在缺失或非法的分量
         */
        inline bool readQuaternion(lua_State *L, int idx, double &w, double &x, double &y, double &z)
        {
            const bool hasW = readFiniteDouble(L, idx, "vio.extrinsic.quaternion", "w", w);
            const bool hasX = readFiniteDouble(L, idx, "vio.extrinsic.quaternion", "x", x);
            const bool hasY = readFiniteDouble(L, idx, "vio.extrinsic.quaternion", "y", y);
            const bool hasZ = readFiniteDouble(L, idx, "vio.extrinsic.quaternion", "z", z);
            return hasW && hasX && hasY && hasZ;
        }

        /**
         * @brief 读取相机相对 IMU 的外参刚体变换
         *
         * Lua 表示：@c extrinsic = { quaternion = { w, x, y, z }, translation = { x, y, z } }，
         * 两个子表可独立省略。四元数按单位化处理，模长近似为零时判定为错误。
         * 采用四元数而非旋转矩阵，是为了让配置不必维护正交性约束。
         *
         * @param L Lua 状态机
         * @param idx 父表在栈上的索引
         * @param pose 待写入外参
         * @return true 解析成功或无外参字段
         * @return false 四元数非法
         */
        inline bool loadExtrinsic(lua_State *L, int idx, gtsam::Pose3 &pose)
        {
            lua_getfield(L, idx, "extrinsic");
            if (!lua_istable(L, -1))
            {
                if (!lua_isnil(L, -1))
                {
                    reportError("vio.extrinsic 应为 table，实际为 {}，已忽略", typeName(L));
                }
                lua_pop(L, 1);
                return true;
            }

            const int extrinsicIdx = lua_gettop(L);
            checkUnknownKeys(L, extrinsicIdx, "vio.extrinsic", {"quaternion", "translation"});
            bool valid = true;
            gtsam::Rot3 rotation = pose.rotation();
            gtsam::Point3 translation = pose.translation();

            lua_getfield(L, extrinsicIdx, "quaternion");
            if (lua_istable(L, -1))
            {
                const int quatIdx = lua_gettop(L);
                checkUnknownKeys(L, quatIdx, "vio.extrinsic.quaternion", {"w", "x", "y", "z"});
                double w = 0.0;
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                if (!readQuaternion(L, quatIdx, w, x, y, z))
                {
                    reportError("vio.extrinsic.quaternion 需要完整的 w / x / y / z 四个分量，已忽略外参旋转");
                    valid = false;
                }
                else
                {
                    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
                    if (norm < 1e-9)
                    {
                        reportError("vio.extrinsic.quaternion 模长近似为零，无法表示旋转，已忽略外参旋转");
                        valid = false;
                    }
                    else
                    {
                        rotation = gtsam::Rot3(gtsam::Quaternion(w / norm, x / norm, y / norm, z / norm));
                    }
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("vio.extrinsic.quaternion 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            lua_getfield(L, extrinsicIdx, "translation");
            if (lua_istable(L, -1))
            {
                const int transIdx = lua_gettop(L);
                checkUnknownKeys(L, transIdx, "vio.extrinsic.translation", {"x", "y", "z"});
                double x = translation.x();
                double y = translation.y();
                double z = translation.z();
                loadDouble(L, transIdx, "vio.extrinsic.translation", "x", -1e3, 1e3, x);
                loadDouble(L, transIdx, "vio.extrinsic.translation", "y", -1e3, 1e3, y);
                loadDouble(L, transIdx, "vio.extrinsic.translation", "z", -1e3, 1e3, z);
                translation = gtsam::Point3(x, y, z);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("vio.extrinsic.translation 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            lua_pop(L, 1); // 弹出 extrinsic 表
            if (valid)
            {
                pose = gtsam::Pose3(rotation, translation);
            }
            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c vio 节并校验交叉约束
         *
         * @param L Lua 状态机
         * @param idx vio 表在栈上的索引
         * @param cfg 待填充配置
         * @return true 解析成功（可能包含钳位告警）
         * @return false 存在无法自动纠正的矛盾配置
         */
        inline bool loadVioSection(lua_State *L, int idx, vio::VioConfig &cfg)
        {
            constexpr std::string_view kSection = "vio";
            checkUnknownKeys(L, idx, kSection,
                             {"fx",
                              "fy",
                              "cx",
                              "cy",
                              "imageWidth",
                              "imageHeight",
                              "extrinsic",
                              "accelNoiseSigma",
                              "gyroNoiseSigma",
                              "accelBiasRw",
                              "gyroBiasRw",
                              "integrationNoise",
                              "imuRateHz",
                              "gravityMagnitude",
                              "pixelNoiseSigma",
                              "huberLossDelta",
                              "maxReprojectionError",
                              "maxReprojectionErrorHard",
                              "maxReprojectionErrorPreopt",
                              "postOptOutlierMarking",
                              "preoptTemporaryRejection",
                              "minParallaxPx",
                              "minTrackingRatio",
                              "minKeyframeInterval",
                              "maxKeyframeInterval",
                              "minTriangulationParallaxPx",
                              "landmarkGateWarmupObs",
                              "extraSmootherIterations",
                              "maxSlidingWindowSize",
                              "slidingWindowLag"});

            bool valid = true;

            // 相机内参
            loadDouble(L, idx, kSection, "fx", 1.0, 1e6, cfg.fx);
            loadDouble(L, idx, kSection, "fy", 1.0, 1e6, cfg.fy);
            loadDouble(L, idx, kSection, "cx", -1e6, 1e6, cfg.cx);
            loadDouble(L, idx, kSection, "cy", -1e6, 1e6, cfg.cy);
            loadDouble(L, idx, kSection, "imageWidth", 0.0, 1e6, cfg.imageWidth);
            loadDouble(L, idx, kSection, "imageHeight", 0.0, 1e6, cfg.imageHeight);

            // 相机相对 IMU 的外参
            if (!loadExtrinsic(L, idx, cfg.bodyPSensor))
            {
                valid = false;
            }

            // IMU 噪声特性与采样率
            loadDouble(L, idx, kSection, "accelNoiseSigma", 0.0, 1.0, cfg.accelNoiseSigma);
            loadDouble(L, idx, kSection, "gyroNoiseSigma", 0.0, 1.0, cfg.gyroNoiseSigma);
            loadDouble(L, idx, kSection, "accelBiasRw", 0.0, 1.0, cfg.accelBiasRw);
            loadDouble(L, idx, kSection, "gyroBiasRw", 0.0, 1.0, cfg.gyroBiasRw);
            loadDouble(L, idx, kSection, "integrationNoise", 0.0, 1e-2, cfg.integrationNoise);
            loadDouble(L, idx, kSection, "imuRateHz", 1.0, 10'000.0, cfg.imuRateHz);
            loadDouble(L, idx, kSection, "gravityMagnitude", 1e-3, 100.0, cfg.gravityMagnitude);

            // 视觉观测噪声与鲁棒核
            loadDouble(L, idx, kSection, "pixelNoiseSigma", 0.0, 100.0, cfg.pixelNoiseSigma);
            loadDouble(L, idx, kSection, "huberLossDelta", 1e-3, 100.0, cfg.huberLossDelta);
            loadDouble(L, idx, kSection, "maxReprojectionError", 0.0, 1e4, cfg.maxReprojectionError);
            loadDouble(L, idx, kSection, "maxReprojectionErrorHard", 0.0, 1e4, cfg.maxReprojectionErrorHard);
            loadDouble(L, idx, kSection, "maxReprojectionErrorPreopt", 0.0, 1e4, cfg.maxReprojectionErrorPreopt);
            loadBool(L, idx, kSection, "postOptOutlierMarking", cfg.postOptOutlierMarking);
            loadBool(L, idx, kSection, "preoptTemporaryRejection", cfg.preoptTemporaryRejection);

            // 关键帧选取策略
            loadDouble(L, idx, kSection, "minParallaxPx", 0.0, 1e4, cfg.minParallaxPx);
            loadDouble(L, idx, kSection, "minTrackingRatio", 0.0, 1.0, cfg.minTrackingRatio);
            loadDouble(L, idx, kSection, "minKeyframeInterval", 0.0, 60.0, cfg.minKeyframeInterval);
            loadDouble(L, idx, kSection, "maxKeyframeInterval", 0.0, 60.0, cfg.maxKeyframeInterval);
            loadDouble(L, idx, kSection, "minTriangulationParallaxPx", 0.0, 1e4, cfg.minTriangulationParallaxPx);
            loadInteger(L, idx, kSection, "landmarkGateWarmupObs", 0, 1'000, cfg.landmarkGateWarmupObs);
            loadInteger(L, idx, kSection, "extraSmootherIterations", 0, 100, cfg.extraSmootherIterations);

            // 滑动窗口
            loadInteger(L, idx, kSection, "maxSlidingWindowSize", 1, 1'000, cfg.maxSlidingWindowSize);
            loadDouble(L, idx, kSection, "slidingWindowLag", 0.0, 60.0, cfg.slidingWindowLag);

            // 交叉约束：单字段合法但组合自相矛盾时不做自动纠正，交由使用者决策
            if (cfg.maxKeyframeInterval < cfg.minKeyframeInterval)
            {
                reportError("vio.maxKeyframeInterval ({}) 小于 minKeyframeInterval ({}): 强制插入间隔不应短于最小间隔",
                            cfg.maxKeyframeInterval, cfg.minKeyframeInterval);
                valid = false;
            }
            if (cfg.maxReprojectionErrorHard < cfg.maxReprojectionError)
            {
                reportError("vio.maxReprojectionErrorHard ({}) 小于 maxReprojectionError ({}): 硬门限应不严于软门限",
                            cfg.maxReprojectionErrorHard, cfg.maxReprojectionError);
                valid = false;
            }
            if (cfg.maxReprojectionErrorPreopt < cfg.maxReprojectionError)
            {
                reportError("vio.maxReprojectionErrorPreopt ({}) 小于 maxReprojectionError ({}): 预优化门控应更宽松",
                            cfg.maxReprojectionErrorPreopt, cfg.maxReprojectionError);
                valid = false;
            }

            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c system 节并校验时序一致性
         *
         * IMU 采样步长不在这里配置：它由 @c vio.imuRateHz 派生
         * （见 @c SystemConfig::imuDtFrom），因此不存在两处不一致的可能。
         * 旧配置里残留的 @c imuDt 会得到一条明确的迁移提示。
         *
         * @param L Lua 状态机
         * @param idx system 表在栈上的索引
         * @param cfg 待填充配置
         * @param vioCfg 已解析的视觉惯性配置，提供 IMU 采样率
         * @return true 解析成功
         * @return false 存在自相矛盾的时序配置
         */
        inline bool loadSystemSection(lua_State *L, int idx, vio::SystemConfig &cfg, const vio::VioConfig &vioCfg)
        {
            constexpr std::string_view kSection = "system";
            checkUnknownKeys(L, idx, kSection, {"imuDt", "cameraDt", "totalDuration", "rmseThreshold"});

            loadDouble(L, idx, kSection, "cameraDt", 1e-6, 10.0, cfg.cameraDt);
            loadDouble(L, idx, kSection, "totalDuration", 1e-3, 86'400.0, cfg.totalDuration);
            loadDouble(L, idx, kSection, "rmseThreshold", 0.0, 1e3, cfg.rmseThreshold);

            // 旧配置兼容：imuDt 已改为派生量，单独提示比笼统的“未知字段”更可操作
            lua_getfield(L, idx, "imuDt");
            if (!lua_isnil(L, -1))
            {
                reportError("system.imuDt 已移除（现由 vio.imuRateHz 派生为 {} s），该字段被忽略",
                            vio::SystemConfig::imuDtFrom(vioCfg.imuRateHz));
            }
            lua_pop(L, 1);

            const double imuDt = vio::SystemConfig::imuDtFrom(vioCfg.imuRateHz);
            if (imuDt <= 0.0)
            {
                reportError("vio.imuRateHz ({}) 非正，无法派生 IMU 采样步长", vioCfg.imuRateHz);
                return false;
            }
            if (cfg.cameraDt < imuDt)
            {
                reportError("system.cameraDt ({}) 小于 vio.imuRateHz ({}) 派生的步长 ({}): 本系统假设 IMU 采样快于相机",
                            cfg.cameraDt, vioCfg.imuRateHz, imuDt);
                return false;
            }
            return true;
        }

        /**
         * @brief 解析 Lua 的 @c dataset.intrinsics 节（MEI 相机内参）
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param cfg 待填充内参
         * @return true 解析成功（含钳位告警）
         */
        inline bool loadIntrinsicsSection(lua_State *L, int idx, dataset::MeiIntrinsics &cfg)
        {
            constexpr std::string_view kSection = "dataset.intrinsics";
            lua_getfield(L, idx, "intrinsics");
            if (!lua_istable(L, -1))
            {
                if (!lua_isnil(L, -1))
                {
                    reportError("dataset.intrinsics 应为 table，实际为 {}，已忽略", typeName(L));
                    lua_pop(L, 1);
                    return false;
                }
                lua_pop(L, 1);
                return true;
            }

            const int tableIdx = lua_gettop(L);
            checkUnknownKeys(
                L, tableIdx, kSection,
                {"fx", "fy", "cx", "cy", "xi", "k1", "k2", "p1", "p2", "pinholeScale", "imageWidth", "imageHeight"});

            loadDouble(L, tableIdx, kSection, "cx", -1e6, 1e6, cfg.cx);
            loadDouble(L, tableIdx, kSection, "cy", -1e6, 1e6, cfg.cy);
            loadDouble(L, tableIdx, kSection, "xi", -10.0, 10.0, cfg.xi);
            loadDouble(L, tableIdx, kSection, "k1", -10.0, 10.0, cfg.k1);
            loadDouble(L, tableIdx, kSection, "k2", -10.0, 10.0, cfg.k2);
            loadDouble(L, tableIdx, kSection, "p1", -10.0, 10.0, cfg.p1);
            loadDouble(L, tableIdx, kSection, "p2", -10.0, 10.0, cfg.p2);
            // 焦距、缩放与图像尺寸是几何链路的硬前提：越界时报错而非钳位，
            // 否则 fx=0 被钳到 1.0 会“表面合法、运行期崩”。
            bool valid = true;
            valid = loadDoubleStrict(L, tableIdx, kSection, "fx", 1.0, 1e6, cfg.fx) && valid;
            valid = loadDoubleStrict(L, tableIdx, kSection, "fy", 1.0, 1e6, cfg.fy) && valid;
            valid = loadDoubleStrict(L, tableIdx, kSection, "pinholeScale", 1e-3, 10.0, cfg.pinholeScale) && valid;
            valid = loadIntegerStrict(L, tableIdx, kSection, "imageWidth", 1, 100'000, cfg.imageWidth) && valid;
            valid = loadIntegerStrict(L, tableIdx, kSection, "imageHeight", 1, 100'000, cfg.imageHeight) && valid;

            lua_pop(L, 1);
            return valid && cfg.isValid();
        }

        /**
         * @brief 解析 Lua 的 @c dataset.replay 与 @c dataset.viz 两节
         *
         * 路径字段允许写空串表示“不输出”，因此用 @c "" 可以清掉代码默认值。
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param cfg 待填充回放配置（含嵌套的可视化配置）
         * @return true 解析成功
         * @return false 存在自相矛盾或类型错误的配置
         */
        inline bool loadReplaySection(lua_State *L, int idx, dataset::DatasetRunConfig &cfg)
        {
            bool valid = true;

            lua_getfield(L, idx, "replay");
            if (lua_istable(L, -1))
            {
                const int tableIdx = lua_gettop(L);
                constexpr std::string_view kSection = "dataset.replay";
                checkUnknownKeys(
                    L, tableIdx, kSection,
                    {"outputTum", "startFrame", "maxFrames", "stride", "maxFeatures", "maxDuration", "plyPath"});

                loadPath(L, tableIdx, kSection, "outputTum", cfg.outputTum);
                loadPath(L, tableIdx, kSection, "plyPath", cfg.plyPath);
                loadInteger(L, tableIdx, kSection, "startFrame", 0, 10'000'000, cfg.startFrame);
                loadInteger(L, tableIdx, kSection, "maxFrames", 0, 10'000'000, cfg.maxFrames);
                loadInteger(L, tableIdx, kSection, "stride", 1, 10'000, cfg.stride);
                loadInteger(L, tableIdx, kSection, "maxFeatures", 1, 1'000'000, cfg.maxFeatures);
                loadDouble(L, tableIdx, kSection, "maxDuration", 0.0, 86'400.0, cfg.maxDuration);
                // 倍数低于 2 会把正常抖动误判为断档，上限 1000 已远超任何合理缺样
                loadDouble(L, tableIdx, kSection, "maxImuGapFactor", 2.0, 1'000.0, cfg.maxImuGapFactor);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("dataset.replay 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            lua_getfield(L, idx, "viz");
            if (lua_istable(L, -1))
            {
                const int tableIdx = lua_gettop(L);
                constexpr std::string_view kSection = "dataset.viz";
                checkUnknownKeys(L, tableIdx, kSection,
                                 {"showWindow", "videoPath", "displayStride", "panelHeight", "trailMaxPoints",
                                  "videoFps", "playbackRate"});

                loadBool(L, tableIdx, kSection, "showWindow", cfg.viz.showWindow);
                loadPath(L, tableIdx, kSection, "videoPath", cfg.viz.videoPath);
                loadInteger(L, tableIdx, kSection, "displayStride", 1, 10'000, cfg.viz.displayStride);
                loadInteger(L, tableIdx, kSection, "panelHeight", 0, 4'000, cfg.viz.panelHeight);
                loadInteger(L, tableIdx, kSection, "trailMaxPoints", 1, 10'000'000, cfg.viz.trailMaxPoints);
                loadInteger(L, tableIdx, kSection, "videoFps", 1, 1'000, cfg.viz.videoFps);
                loadDouble(L, tableIdx, kSection, "playbackRate", 0.0, 100.0, cfg.viz.playbackRate);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("dataset.viz 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            // 窗口与视频都没有启用时，渲染参数无意义但不致命
            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c dataset 节
         *
         * @param L Lua 状态机
         * @param idx dataset 表在栈上的索引
         * @param intrinsics 待填充相机内参
         * @param replay 待填充回放配置
         * @return true 解析成功
         * @return false 存在无法自动纠正的配置错误
         */
        inline bool loadDatasetSection(lua_State *L, int idx, dataset::MeiIntrinsics &intrinsics,
                                       dataset::DatasetRunConfig &replay)
        {
            checkUnknownKeys(L, idx, "dataset", {"intrinsics", "replay", "viz"});

            bool valid = true;
            if (!loadIntrinsicsSection(L, idx, intrinsics))
            {
                reportError("dataset.intrinsics 校验失败：内参或图像尺寸非法");
                valid = false;
            }
            if (!loadReplaySection(L, idx, replay))
            {
                valid = false;
            }
            return valid;
        }

    } // namespace detail

    /**
     * @brief 从 Lua 脚本加载项目配置与日志配置
     *
     * 执行顺序：
     * 1. 以调用方传入的 @p out 为基线解析 @c vio / @c system / @c dataset 三节到
     *    临时副本，校验全部通过后一次性写回；失败则 @p out 保持不变；
     * 2. 若 @p applyLogging 为真，把同一份脚本交给 @c mylog::reloadConfig 应用
     *    @c logger / @c fileManager 两节，成功后安装 FileManager 共享单文件输出。
     *    （@c reloadConfig 只改写配置对象，输出后端仍需调用方显式安装，
     *    mylog 自己的示例也是这么做的；安装后行为与 @c configureLogging 一致。）
     *    两部分各自原子，互不回滚。
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

        lua_State *L = luaL_newstate();
        if (L == nullptr)
        {
            detail::reportError("无法创建 Lua 状态机（内存不足）");
            return std::unexpected(ConfigError{ConfigErrorKind::LuaRuntime, "无法创建 Lua 状态机（内存不足）"});
        }

        lua_sethook(
            L, [](lua_State *state, lua_Debug * /*ar*/)
            { luaL_error(state, "config script aborted: instruction budget exceeded"); }, LUA_MASKCOUNT,
            kInstructionBudget);

        if (luaL_loadfile(L, config.string().c_str()) != 0 || lua_pcall(L, 0, 0, 0) != 0)
        {
            const char *err = lua_tostring(L, -1);
            const std::string detailText = err != nullptr ? err : "未知 Lua 错误";
            detail::reportError("执行 {} 失败: {}", config.string(), detailText);
            lua_close(L);
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
            const bool systemValid = detail::loadSystemSection(L, lua_gettop(L), parsed.system, parsed.vio);
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
            const bool datasetValid = detail::loadDatasetSection(L, lua_gettop(L), parsed.intrinsics, parsed.replay);
            projectValid = projectValid && datasetValid;
        }
        else if (!lua_isnil(L, -1))
        {
            detail::reportError("配置节 dataset 应为 table，实际为 {}", detail::typeName(L));
            structureError = std::format("配置节 dataset 应为 table，实际为 {}", detail::typeName(L));
            projectValid = false;
        }
        lua_pop(L, 1);

        lua_close(L);

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

        if (applyLogging)
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
