#pragma once

/**
 * @file health_monitor.hpp
 * @brief 健康诊断：零偏发散、速度跳变、观测不足。
 *
 * 诊断分两层：
 *   - @c diagnose 是纯函数，对单帧状态做无状态判定，便于单元测试；
 *   - @c inspect 在诊断之上叠加连续计数滞回，避免单帧噪声触发降级。
 *
 * 结论类型在 @c domain/status.hpp。诊断逻辑本身与估计器无关，因此留在领域层：
 * 任何后端都可以复用同一套阈值与滞回规则。
 *
 * 约束：只依赖领域类型。
 */

#include "domain/config.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include <algorithm>
#include <format>
#include <string>

namespace myslam::domain
{

    /**
     * @brief 带滞回的健康监控
     */
    class HealthMonitor
    {
    public:
        explicit HealthMonitor(const HealthConfig &config = {})
            : config_(config)
        {
        }

        /**
         * @brief 单帧无状态诊断
         */
        [[nodiscard]] static HealthReport diagnose(const KinematicState &state, std::size_t activeFeatures,
                                                   const HealthConfig &config = {})
        {
            const double accBiasNorm = state.bias.accel.norm();
            const double gyroBiasNorm = state.bias.gyro.norm();
            if (accBiasNorm > config.maxAccelBiasNorm)
            {
                return {false, Degradation::INERTIAL,
                        std::format("加速度计偏置发散: {:.3f} m/s^2 > {:.3f}", accBiasNorm, config.maxAccelBiasNorm)};
            }
            if (gyroBiasNorm > config.maxGyroBiasNorm)
            {
                return {false, Degradation::INERTIAL,
                        std::format("陀螺仪偏置发散: {:.4f} rad/s > {:.4f}", gyroBiasNorm, config.maxGyroBiasNorm)};
            }

            const double speed = state.velocity.norm();
            if (speed > config.maxSpeed)
            {
                return {false, Degradation::INERTIAL,
                        std::format("系统速度异常跳变: {:.2f} m/s > {:.2f} m/s", speed, config.maxSpeed)};
            }

            if (static_cast<int>(activeFeatures) < config.minActiveFeatures)
            {
                return {false, Degradation::VISUAL,
                        std::format("活跃观测特征数量过低 ({})，视觉观测暂时中断", activeFeatures)};
            }

            return {true, Degradation::NONE, "状态健康，运行平稳"};
        }

        /**
         * @brief 带滞回的健康检查
         */
        [[nodiscard]] HealthReport inspect(const KinematicState &state, std::size_t activeFeatures)
        {
            const HealthReport report = diagnose(state, activeFeatures, config_);
            if (report.isHealthy)
            {
                inertialStreak_ = 0;
                visualStreak_ = 0;
                return report;
            }

            // 每类降级单独计数：交叉出现的「惯性 1 帧 + 视觉 1 帧 + 惯性 1 帧」
            // 曾共用同一个计数器而凑够阈值，而其中没有任何一类连续成立。
            // 滞回的本意是「同一故障持续多帧」，跨类别求和把它绕过了。
            int &streak = report.degradation == Degradation::INERTIAL ? inertialStreak_ : visualStreak_;
            ++streak;
            if (streak >= config_.alertStreakThreshold)
            {
                return report;
            }
            return {
                true, Degradation::NONE,
                std::format("{} (连续 {}/{} 次，暂不降级)", report.alertMessage, streak, config_.alertStreakThreshold)};
        }

        void reset() noexcept
        {
            inertialStreak_ = 0;
            visualStreak_ = 0;
        }
        /// 两类降级计数中较大者（诊断用）
        [[nodiscard]] int streak() const noexcept { return std::max(inertialStreak_, visualStreak_); }
        [[nodiscard]] const HealthConfig &config() const noexcept { return config_; }

    private:
        HealthConfig config_{};
        int inertialStreak_{0};
        int visualStreak_{0};
    };

} // namespace myslam::domain
