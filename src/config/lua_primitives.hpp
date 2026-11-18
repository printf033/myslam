#pragma once

/**
 * @file lua_primitives.hpp
 * @brief 从 Lua 表安全取值的通用原语（@c myslam::config::detail）。
 *
 * 这一层只回答"怎么从 Lua 表里取一个合法值"，不关心任何具体配置节：
 *
 * - 类型不符时忽略该字段并告警，保留调用方已有的默认值；
 * - 数值越界按安全区间钳位，而不是让运行期行为随配置漂移；
 * - 严格变体（@c *Strict）用于"越界必须报错"的场合，如外参四元数；
 * - 未知键会被点名——配置拼写错误是最常见的失效来源，静默忽略最难排查。
 *
 * 具体配置节有哪些字段、彼此的交叉约束是什么，见 @c config/lua_sections.hpp。
 * 新增一个原语改本文件，新增一个节改 section 文件。
 *
 * 未启用 Lua 时整段实现不参与编译，文件退化为空壳。
 */

#include "config/project_config.hpp"
#include "domain/camera_intrinsics.hpp"
#include "domain/types.hpp"
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <format>
#include <initializer_list>
#include <string_view>
#include <type_traits>

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA
// 必须带 `mylog/` 前缀（thirdparty/ 在 include 路径上）：本文件名与 mylog
// 的同名头不冲突，但统一约定避免歧义。
#include <mylog/src/lua_config.hpp>
extern "C"
{
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace myslam::config
{

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
         * @brief 字段类型错误的计数器
         *
         * “数值越界”可以钳位（量级问题，意图仍是清楚的），但“类型不匹配”
         * （例如从 YAML 拷来的 @c cameraDt = "0.05"）是使用者错误：静默保留
         * 默认值会让用户以为参数已经生效，比直接报错危险得多。
         *
         * 这个计数不通过出参传递：需要它的调用点有几十个，而配置解析是
         * 单线程的（每次 @c load() 独占一个 lua_State），用 thread_local
         * 计数即可。每个配置节解析前清零、解析后据此判定该节是否合法。
         */
        inline int &fieldTypeErrors()
        {
            static thread_local int count = 0;
            return count;
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
                    ++fieldTypeErrors();
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.{} 应为数字，实际为 {}，已忽略", section, key, typeName(L));
                ++fieldTypeErrors();
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
         * @brief 严格版数值读取：区分“字段缺失”与“字段存在但非法”
         *
         * @c readFiniteDouble 把两种情形都折叠成 false，对允许保留默认值的普通
         * 字段没问题；但严格校验的字段必须把它们分开——否则 @c fx = "460.0"
         * （带引号的数字）会被当成“字段没写”而静默保留默认值，与
         * @c fx = 0 被判非法的行为不一致。
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param out 读取成功时写入
         * @param present 可选出参：字段是否存在（无论合法与否）
         * @return true 字段缺失（调用方保留原值）或取值合法
         * @return false 字段存在但类型错误或不是有限数值
         */
        [[nodiscard]] inline bool readFiniteDoubleStrict(lua_State *L, int idx, std::string_view section,
                                                         const char *key, double &out, bool *present = nullptr)
        {
            lua_getfield(L, idx, key);
            const int type = lua_type(L, -1);
            if (present != nullptr)
            {
                *present = (type != LUA_TNIL);
            }
            if (type == LUA_TNIL)
            {
                lua_pop(L, 1);
                return true; // 字段缺失：保留原值，不算失败
            }

            bool ok = false;
            if (type == LUA_TNUMBER)
            {
                const double raw = static_cast<double>(lua_tonumber(L, -1));
                if (std::isfinite(raw))
                {
                    out = raw;
                    ok = true;
                }
                else
                {
                    reportError("{}.{} 不是有限数值（该字段不允许自动纠正）", section, key);
                }
            }
            else
            {
                reportError("{}.{} 应为数字，实际为 {}（该字段不允许自动纠正）", section, key, typeName(L));
            }
            lua_pop(L, 1);
            return ok;
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
         * @return false 字段存在但类型错误或越界
         */
        [[nodiscard]] inline bool loadDoubleStrict(lua_State *L, int idx, std::string_view section, const char *key,
                                                   double lo, double hi, double &field)
        {
            double value = field;
            bool present = false;
            if (!readFiniteDoubleStrict(L, idx, section, key, value, &present))
            {
                return false;
            }
            if (!present)
            {
                return true; // 缺失：保留原值（原值已由默认值或基线保证合法）
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
            bool present = false;
            if (!readFiniteDoubleStrict(L, idx, section, key, value, &present))
            {
                return false;
            }
            // 字段缺失时不能用未初始化的 0 去判界：下界为 1 的字段（如图像宽高）
            // 会把“没写这个字段”误判为“写了 0”，合法配置平白被拒。
            if (!present)
            {
                return true;
            }
            // 先在 double 域判界再转换：|value| >= 2^63 时 static_cast 是 UB，
            // 常见平台产出 INT64_MIN，会让“越界”检查依据一个凭空的值。
            if (value < static_cast<double>(lo) || value > static_cast<double>(hi))
            {
                reportError("{}.{} = {} 超出合法区间 [{}, {}]（该字段不允许自动纠正）", section, key, value, lo, hi);
                return false;
            }
            field = static_cast<int>(static_cast<long long>(value));
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
                ++fieldTypeErrors();
            }
            lua_pop(L, 1);
        }

        /**
         * @brief 读取相机模型字段（@c model = "pinhole" / "mei"）
         *
         * 拼错模型名不是可忽略的类型问题：静默退回针孔会让鱼眼图被当作针孔
         * 处理，特征点会堆到镜头边框上（早期实现的真实故障）。因此非法取值
         * 一律判配置非法，而不是保留默认值。
         *
         * @param L Lua 状态机
         * @param tableIdx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param field 待写入模型；字段缺失或非法时保持原值
         * @return 字段缺失或解析成功返回 true；取值非法返回 false
         */
        inline bool loadCameraModel(lua_State *L, int tableIdx, std::string_view section, camera::CameraModel &field)
        {
            bool ok = true;
            lua_getfield(L, tableIdx, "model");
            if (lua_type(L, -1) == LUA_TSTRING)
            {
                std::size_t length = 0;
                const char *modelText = lua_tolstring(L, -1, &length);
                const std::string_view modelName(modelText, length);
                if (const auto parsed = camera::parseCameraModel(modelName))
                {
                    field = *parsed;
                }
                else
                {
                    reportError("{}.model = \"{}\" 不是受支持的相机模型（pinhole / mei）", section, modelName);
                    ok = false;
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.model 应为字符串，实际为 {}，已忽略", section, typeName(L));
                ++fieldTypeErrors();
            }
            lua_pop(L, 1);
            return ok;
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
                // 内嵌 NUL 会让 open(2)/ofstream 在 NUL 处截断：配置里的路径与
                // 实际写入的文件不同名，而这些路径还被用于日志与错误提示。
                if (std::strlen(text) != length)
                {
                    reportError("{}.{} 含内嵌 NUL 字节，路径不合法，已忽略", section, key);
                    ++fieldTypeErrors();
                }
                else
                {
                    field = std::filesystem::path(std::string(text, length));
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.{} 应为字符串路径，实际为 {}，已忽略", section, key, typeName(L));
                ++fieldTypeErrors();
            }
            lua_pop(L, 1);
        }

        /**
         * @brief 读取字符串字段（严格类型，不做数字协变）
         *
         * 不能用 @c lua_isstring：它对数字也返回真，于是 @c source = 42 会静默
         * 变成字符串 "42"，直到装配期才报一个看起来毫无关系的未知来源。
         * 空串是合法取值，语义由调用方定义（通常表示“未设置，用默认”）。
         *
         * @param L Lua 状态机
         * @param idx 配置节表在栈上的索引
         * @param section 节名（用于诊断）
         * @param key 字段名
         * @param field 待写入字符串；字段缺失或类型错误时保持原值
         */
        inline void loadString(lua_State *L, int idx, std::string_view section, const char *key, std::string &field)
        {
            lua_getfield(L, idx, key);
            if (lua_type(L, -1) == LUA_TSTRING)
            {
                std::size_t length = 0;
                const char *text = lua_tolstring(L, -1, &length);
                if (std::strlen(text) != length)
                {
                    reportError("{}.{} 含内嵌 NUL 字节，已忽略", section, key);
                    ++fieldTypeErrors();
                }
                else
                {
                    field.assign(text, length);
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.{} 应为字符串，实际为 {}，已忽略", section, key, typeName(L));
                ++fieldTypeErrors();
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

            // 判界与钳位都在 double 域完成，最后才转整型。
            // 若先 static_cast<long long> 再钳位，则 |value| >= 2^63 时转换本身是 UB；
            // x86-64 上实测得到 INT64_MIN，随后被钳到「下界」——把 maxFrames = 1e300
            // 这种“极大上限”反转成“0 = 不限”，与用户意图正好相反。
            const double loD = static_cast<double>(lo);
            const double hiD = static_cast<double>(hi);
            if (value < loD || value > hiD)
            {
                const double clampedD = clampValue(value, loD, hiD);
                reportError("{}.{} = {} 超出安全区间 [{}, {}]，已钳位为 {}", section, key, value, lo, hi, clampedD);
                value = clampedD;
            }
            else if (value != std::floor(value))
            {
                reportError("{}.{} = {} 不是整数，按 {} 取整", section, key, value, std::floor(value));
            }

            // 此处 value 必落在 [loD, hiD]；调用点传入的上下界都远小于 2^53，转换精确。
            field = static_cast<Int>(static_cast<long long>(value));
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
        inline bool readQuaternion(lua_State *L, int idx, std::string_view section, double &w, double &x, double &y,
                                   double &z)
        {
            const bool hasW = readFiniteDouble(L, idx, section, "w", w);
            const bool hasX = readFiniteDouble(L, idx, section, "x", x);
            const bool hasY = readFiniteDouble(L, idx, section, "y", y);
            const bool hasZ = readFiniteDouble(L, idx, section, "z", z);
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
         * @param section 节名（用于诊断；@c vio.extrinsic 或 @c dataset.extrinsic）
         * @return true 解析成功或无外参字段
         * @return false 四元数非法
         */
        inline bool loadExtrinsic(lua_State *L, int idx, domain::Pose &pose, std::string_view section = "vio.extrinsic")
        {
            lua_getfield(L, idx, "extrinsic");
            if (!lua_istable(L, -1))
            {
                if (!lua_isnil(L, -1))
                {
                    reportError("{}.extrinsic 应为 table，实际为 {}，已忽略", section, typeName(L));
                }
                lua_pop(L, 1);
                return true;
            }

            const int extrinsicIdx = lua_gettop(L);
            checkUnknownKeys(L, extrinsicIdx, section, {"quaternion", "translation"});
            bool valid = true;
            Eigen::Quaterniond rotation = pose.orientation;
            domain::Vec3 translation = pose.position;

            lua_getfield(L, extrinsicIdx, "quaternion");
            if (lua_istable(L, -1))
            {
                const int quatIdx = lua_gettop(L);
                checkUnknownKeys(L, quatIdx, std::format("{}.quaternion", section), {"w", "x", "y", "z"});
                double w = 0.0;
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                if (!readQuaternion(L, quatIdx, std::format("{}.quaternion", section), w, x, y, z))
                {
                    reportError("{}.quaternion 需要完整的 w / x / y / z 四个分量，已忽略外参旋转", section);
                    valid = false;
                }
                else
                {
                    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
                    if (norm < 1e-9)
                    {
                        reportError("{}.quaternion 模长近似为零，无法表示旋转，已忽略外参旋转", section);
                        valid = false;
                    }
                    else
                    {
                        rotation = Eigen::Quaterniond(w / norm, x / norm, y / norm, z / norm);
                    }
                }
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.quaternion 应为 table，实际为 {}，已忽略", section, typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            lua_getfield(L, extrinsicIdx, "translation");
            if (lua_istable(L, -1))
            {
                const int transIdx = lua_gettop(L);
                checkUnknownKeys(L, transIdx, std::format("{}.translation", section), {"x", "y", "z"});
                double x = translation.x();
                double y = translation.y();
                double z = translation.z();
                loadDouble(L, transIdx, std::format("{}.translation", section), "x", -1e3, 1e3, x);
                loadDouble(L, transIdx, std::format("{}.translation", section), "y", -1e3, 1e3, y);
                loadDouble(L, transIdx, std::format("{}.translation", section), "z", -1e3, 1e3, z);
                translation = domain::Vec3(x, y, z);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("{}.translation 应为 table，实际为 {}，已忽略", section, typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            lua_pop(L, 1); // 弹出 extrinsic 表
            if (valid)
            {
                pose.orientation = rotation;
                pose.position = translation;
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

    } // namespace detail

} // namespace myslam::config

#endif // MYSLAM_HAS_LUA
