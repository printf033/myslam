#pragma once

/**
 * @file propagator.hpp
 * @brief 高频（200Hz）零延迟状态外推器。
 */

#include "vio/params.hpp"
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace myslam::vio
{

    // =========================================================================
    // 5. 高频状态外推器 (Dead Reckoning / 200Hz Output)
    // =========================================================================

    class HighRatePropagator
    {
    public:
        explicit HighRatePropagator(double gravityMag)
            : gravity_(0.0, 0.0, -gravityMag)
        {
        }

        /**
         * @brief 用后端优化完成的最优关键帧状态重置外推基线
         */
        void resetWithOptimizedState(double timestamp, const gtsam::NavState &navState,
                                     const gtsam::imuBias::ConstantBias &bias)
        {
            latestTime_ = timestamp;
            currentState_ = navState;
            currentBias_ = bias;
        }

        /**
         * @brief 随每个 200Hz IMU 到达实时进行高频低延迟数值递推
         */
        gtsam::NavState propagate(const ImuMeasurement &imu)
        {
            if (latestTime_ <= 0.0)
            {
                latestTime_ = imu.timestamp;
                return currentState_;
            }

            double dt = imu.timestamp - latestTime_;
            if (dt <= 0.0 || dt > 0.05)
            {
                dt = 0.005; // 异常时间间隔保护
            }

            // 去零偏
            gtsam::Vector3 unbiasGyro = imu.gyro - currentBias_.gyroscope();
            gtsam::Vector3 unbiasAcc = imu.accel - currentBias_.accelerometer();

            // 姿态前向更新: R_{t+dt} = R_t * Exp(omega * dt)
            gtsam::Rot3 deltaRot = gtsam::Rot3::Expmap(unbiasGyro * dt);
            gtsam::Rot3 newRot = currentState_.pose().rotation() * deltaRot;

            // 比力变换到世界系并加上重力: a_w = R_t * (a_b - ba) + g
            gtsam::Vector3 accWorld = currentState_.pose().rotation().rotate(unbiasAcc) + gravity_;

            // 速度与平移更新
            gtsam::Vector3 newVel = currentState_.v() + accWorld * dt;
            gtsam::Point3 newPos =
                currentState_.pose().translation() + currentState_.v() * dt + 0.5 * accWorld * dt * dt;

            currentState_ = gtsam::NavState(gtsam::Pose3(newRot, newPos), newVel);
            latestTime_ = imu.timestamp;

            return currentState_;
        }

        const gtsam::NavState &latestState() const { return currentState_; }

    private:
        gtsam::Vector3 gravity_;
        double latestTime_{-1.0};
        gtsam::NavState currentState_;
        gtsam::imuBias::ConstantBias currentBias_;
    };

} // namespace myslam::vio
