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
 * if (!myslam::config::load("config/myslam.lua", cfg))
 * {
 *     // 沿用代码内置默认值
 * }
 * @endcode
 */

#include "vio/system.hpp"
#include "vio/types.hpp"
#include <cmath>
#include <cstddef>
#include <cstdio>
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
        vio::VioConfig vio{};       ///< 视觉惯性参数
        vio::SystemConfig system{}; ///< 运行时序与验收门限
    };

    /**
     * @brief 加载结果：区分项目参数与日志配置各自的成败
     *
     * 两个部分互不回滚：日志配置失败不影响项目参数应用，反之亦然。
     * 需要"全有或全无"语义时检查 @c ok()。
     */
    struct LoadReport
    {
        bool project{false}; ///< 项目参数解析成功并已写入调用方配置
        bool logging{false}; ///< logger / fileManager 节已由 mylog 应用

        /**
         * @brief 两部分是否都成功
         */
        [[nodiscard]] bool ok() const noexcept { return project && logging; }
    };

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

    } // namespace detail

    /**
     * @brief 从 Lua 脚本加载项目配置与日志配置
     *
     * 执行顺序：
     * 1. 以调用方传入的 @p out 为基线解析 @c vio / @c system 两节到临时副本，
     *    校验全部通过后一次性写回；失败则 @p out 保持不变；
     * 2. 若 @p applyLogging 为真，把同一份脚本交给 @c mylog::reloadConfig 应用
     *    @c logger / @c fileManager 两节，成功后安装 FileManager 共享单文件输出。
     *    （@c reloadConfig 只改写配置对象，输出后端仍需调用方显式安装，
     *    mylog 自己的示例也是这么做的；安装后行为与 @c configureLogging 一致。）
     *    两部分各自原子，互不回滚。
     *
     * 未识别的配置节会被忽略，便于在同一个文件里放其它工具的配置。
     *
     * @param config Lua 配置文件路径
     * @param out 待覆盖的项目配置；字段缺失时保留其当前值
     * @param applyLogging 是否同时应用日志配置
     * @return LoadReport 各部分成败
     */
    inline LoadReport load(const std::filesystem::path &config, ProjectConfig &out, bool applyLogging = true)
    {
        LoadReport report;

        std::error_code ec;
        if (!std::filesystem::is_regular_file(config, ec))
        {
            detail::reportError("配置文件不存在或不可读: {}", config.string());
            return report;
        }

        lua_State *L = luaL_newstate();
        if (L == nullptr)
        {
            detail::reportError("无法创建 Lua 状态机（内存不足）");
            return report;
        }

        lua_sethook(
            L, [](lua_State *state, lua_Debug * /*ar*/)
            { luaL_error(state, "config script aborted: instruction budget exceeded"); }, LUA_MASKCOUNT,
            kInstructionBudget);

        if (luaL_loadfile(L, config.string().c_str()) != 0 || lua_pcall(L, 0, 0, 0) != 0)
        {
            const char *err = lua_tostring(L, -1);
            detail::reportError("执行 {} 失败: {}", config.string(), err != nullptr ? err : "未知 Lua 错误");
            lua_close(L);
            return report;
        }

        ProjectConfig parsed = out;
        bool projectValid = true;

        lua_getglobal(L, "vio");
        if (lua_istable(L, -1))
        {
            const bool vioValid = detail::loadVioSection(L, lua_gettop(L), parsed.vio);
            projectValid = projectValid && vioValid;
        }
        else if (!lua_isnil(L, -1))
        {
            detail::reportError("配置节 vio 应为 table，实际为 {}", detail::typeName(L));
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
            projectValid = false;
        }
        lua_pop(L, 1);

        lua_close(L);

        if (projectValid)
        {
            out = parsed;
            report.project = true;
        }

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
     * @return LoadReport 各部分均为 false
     */
    inline LoadReport load(const std::filesystem::path &config, ProjectConfig &out, bool applyLogging = true)
    {
        (void)out;
        (void)applyLogging;
        std::fprintf(stderr, "[myslam] config error: 本构建未启用 Lua 配置支持（MYSLAM_ENABLE_LUA=OFF），无法加载 %s\n",
                     config.string().c_str());
        return LoadReport{};
    }

#endif // MYSLAM_HAS_LUA

} // namespace myslam::config
