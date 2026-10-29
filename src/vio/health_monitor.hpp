#pragma once

/**
 * @file health_monitor.hpp
 * @brief 系统健康与异常诊断（零偏发散、速度跳变、观测不足）。
 *
 * 诊断分两层：
 *   - @c diagnose 是纯函数，对单帧状态做无状态判定，便于单元测试；
 *   - @c inspect 在诊断之上叠加连续计数滞回，只有连续
 *     @c kAlertStreakThreshold 次异常才升级为“不健康”，避免单帧噪声把
 *     整条流水线推进降级状态。恢复健康时计数立即清零。
 */

#include <format>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <string>

namespace myslam::vio
{

    // =========================================================================
    // 6. 系统健康与异常监控器
    // =========================================================================

    class HealthMonitor
    {
    public:
        /**
         * @brief 降级类型：决定上层能否靠“重建流水线”修复
         *
         * 两者的处置方式必须分开：
         *   - @c INERTIAL 是估计算法自身出了问题（零偏发散、速度跳变），
         *     重建流水线确实能重置内部状态，是正确手段；
         *   - @c VISUAL 是外部观测退化（图像变暗、遮挡、无纹理），
         *     重建流水线完全无助：下一帧图像依旧是黑的，重建只会让坐标系
         *     反复归零，把一段暂时的视觉中断放大成几十次轨迹跳变。
         *     此类异常应当就地等待视觉恢复，靠 IMU 外推维持轨迹。
         */
        enum class Degradation
        {
            NONE,
            INERTIAL,
            VISUAL
        };

        struct HealthReport
        {
            bool isHealthy{true};
            Degradation degradation{Degradation::NONE};
            std::string alertMessage;
        };

        /// 连续异常达到该帧数才判定不健康，之后由上层触发降级/重置
        static constexpr int kAlertStreakThreshold = 3;

        /**
         * @brief 单帧无状态诊断
         *
         * @param state 当前导航状态
         * @param bias 当前 IMU 零偏估计
         * @param activeFeatures 本帧参与优化的视觉观测数
         * @return 诊断结论；健康时 @c alertMessage 为说明文字
         */
        [[nodiscard]] static HealthReport diagnose(const gtsam::NavState &state,
                                                   const gtsam::imuBias::ConstantBias &bias, size_t activeFeatures)
        {
            // 1. 零偏异常发散检测
            double accBiasNorm = bias.accelerometer().norm();
            double gyroBiasNorm = bias.gyroscope().norm();
            if (accBiasNorm > 1.0)
            {
                return {false, Degradation::INERTIAL, std::format("加速度计偏置发散: {:.3f} m/s^2 > 1.0", accBiasNorm)};
            }
            if (gyroBiasNorm > 0.15)
            {
                return {false, Degradation::INERTIAL, std::format("陀螺仪偏置发散: {:.4f} rad/s > 0.15", gyroBiasNorm)};
            }

            // 2. 速度异常检测
            double speed = state.v().norm();
            if (speed > 20.0)
            {
                return {false, Degradation::INERTIAL, std::format("系统速度异常跳变: {:.2f} m/s > 20.0 m/s", speed)};
            }

            // 3. 特征稀疏几何退化检测
            if (activeFeatures < 5)
            {
                return {false, Degradation::VISUAL,
                        std::format("活跃观测特征数量过低 ({})，视觉观测暂时中断", activeFeatures)};
            }

            return {true, Degradation::NONE, "状态健康，运行平稳"};
        }

        /**
         * @brief 带滞回的健康检查
         *
         * 连续异常未达阈值时仍返回健康，但把计数写进提示文字，便于排查
         * “偶发抖动”与“持续故障”。
         */
        [[nodiscard]] HealthReport inspect(const gtsam::NavState &state, const gtsam::imuBias::ConstantBias &bias,
                                           size_t activeFeatures)
        {
            const HealthReport report = diagnose(state, bias, activeFeatures);
            if (report.isHealthy)
            {
                streak_ = 0;
                return report;
            }

            ++streak_;
            if (streak_ >= kAlertStreakThreshold)
            {
                return report;
            }
            return {true, Degradation::NONE,
                    std::format("{} (连续 {}/{} 次，暂不降级)", report.alertMessage, streak_, kAlertStreakThreshold)};
        }

        /// 硬件重置/流水线重建后清除滞回计数
        void reset() { streak_ = 0; }

        /// 当前连续异常帧数
        int streak() const { return streak_; }

    private:
        int streak_{0};
    };

} // namespace myslam::vio
