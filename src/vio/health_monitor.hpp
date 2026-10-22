#pragma once

/**
 * @file health_monitor.hpp
 * @brief 系统健康与异常诊断（零偏发散、速度跳变、观测不足）。
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
        struct HealthReport
        {
            bool isHealthy{true};
            std::string alertMessage;
        };

        static HealthReport inspect(const gtsam::NavState &state, const gtsam::imuBias::ConstantBias &bias,
                                    size_t activeFeatures)
        {
            // 1. 零偏异常发散检测
            double accBiasNorm = bias.accelerometer().norm();
            double gyroBiasNorm = bias.gyroscope().norm();
            if (accBiasNorm > 1.0)
            {
                return {false, std::format("加速度计偏置发散: {:.3f} m/s^2 > 1.0", accBiasNorm)};
            }
            if (gyroBiasNorm > 0.15)
            {
                return {false, std::format("陀螺仪偏置发散: {:.4f} rad/s > 0.15", gyroBiasNorm)};
            }

            // 2. 速度异常检测
            double speed = state.v().norm();
            if (speed > 20.0)
            {
                return {false, std::format("系统速度异常跳变: {:.2f} m/s > 20.0 m/s", speed)};
            }

            // 3. 特征稀疏几何退化检测
            if (activeFeatures < 5)
            {
                return {false, std::format("活跃观测特征数量过低 ({})，存在视觉退化风险", activeFeatures)};
            }

            return {true, "状态健康，运行平稳"};
        }
    };

} // namespace myslam::vio
