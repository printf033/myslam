#pragma once

/**
 * @file status.hpp
 * @brief 系统状态、健康结论与恢复语义：编排层与估计器之间的「诊断语言」。
 *
 * @par 为什么恢复动作必须进契约
 *
 * 旧接口里后端只能返回 @c bool，于是「本帧没出位姿」这一事实被迫由编排层
 * 翻译成固定策略：求解失败就重建流水线、健康异常就降级。新后端如果自带
 * 局部重定位或回环，它的处置意图无处表达，只能被上层误读成「重建整条
 * 流水线」，坐标系因此白白归零。
 *
 * 现在后端在 @c StepResult 里直接说明该怎么恢复，编排层只负责执行。
 *
 * 约束：本文件只依赖标准库。
 */

#include <cstddef>
#include <string>
#include <string_view>

namespace myslam::domain
{

    /**
     * @brief 初始化/追踪状态机
     *
     * 没有 @c LOST_DEGRADED 节点：降级不再是一种"停留状态"，而是
     * @c StepResult::recovery 表达的一次事件。旧实现在重建前把状态置为
     * @c LOST_DEGRADED 再立刻重建，任何观察者都只看到一次瞬时跳变，
     * 却要多维护一个永远进不去的状态与一条无分支的转移。
     */
    enum class SystemState : std::uint8_t
    {
        UNINITIALIZED,
        STATIONARY_COLLECTING,
        GRAVITY_ALIGNED,
        TRACKING_OK,
    };

    [[nodiscard]] inline std::string_view systemStateName(SystemState state) noexcept
    {
        switch (state)
        {
        case SystemState::UNINITIALIZED:
            return "UNINITIALIZED";
        case SystemState::STATIONARY_COLLECTING:
            return "STATIONARY_COLLECTING";
        case SystemState::GRAVITY_ALIGNED:
            return "GRAVITY_ALIGNED";
        case SystemState::TRACKING_OK:
            return "TRACKING_OK";
        }
        return "UNKNOWN";
    }

    /**
     * @brief 降级类型：决定上层能否靠「重建坐标系」修复
     *
     * @c INERTIAL 是估计算法自身失稳，重建确实能重置内部状态；
     * @c VISUAL 是外部观测退化（变暗、遮挡、无纹理），重建毫无作用——
     * 下一帧图像依旧是黑的，重建只会让坐标系反复归零。
     */
    enum class Degradation : std::uint8_t
    {
        NONE,
        INERTIAL,
        VISUAL,
    };

    /**
     * @brief 一次健康检查的结论
     */
    struct HealthReport
    {
        bool isHealthy{true};
        Degradation degradation{Degradation::NONE};
        std::string alertMessage;
    };

    /**
     * @brief 单帧处理的结局
     */
    enum class StepStatus : std::uint8_t
    {
        OK,        ///< 产出新位姿
        RECOVERED, ///< 估计器内部已恢复，本帧不产出位姿，坐标系保持连续
        SKIPPED,   ///< 输入不足以构建约束（例如自上个关键帧以来没有 IMU）
        FAILED,    ///< 无法恢复，按 @c recovery 处置
    };

    /**
     * @brief 估计器建议的恢复动作
     */
    enum class RecoveryAction : std::uint8_t
    {
        NONE,            ///< 无需处置，继续下一帧
        RESET_ESTIMATOR, ///< 只重建估计器内部状态（丢弃窗口与路标，坐标系连续）
        RESET_PIPELINE,  ///< 重置整条流水线并重新初始化（坐标系作废）
        RELOCALIZE,      ///< 由估计器自行重定位（上层不得重建坐标系）
    };

    /**
     * @brief 估计器可上报的诊断计数
     */
    struct EstimatorDiagnostics
    {
        std::size_t zuptCount{0};       ///< 零速更新累计生效次数
        std::size_t recoveredFrames{0}; ///< 内部局部恢复次数
        std::size_t activeLandmarks{0}; ///< 当前活跃路标数
    };

} // namespace myslam::domain
