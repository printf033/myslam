#pragma once

/**
 * @file propagator.hpp
 * @brief 高频（200Hz）零延迟状态外推器。
 */

#include "logger.hpp"
#include "vio/configHolder.hpp"
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
        /// 相邻样本间隔超过该倍数的标称周期即视为时间戳异常
        static constexpr double kMaxDtFactor = 10.0;

        /**
         * @param gravityMag 重力模长 (m/s^2)
         * @param imuRateHz IMU 采样率 (Hz)，用于派生标称采样周期；非正时退回 200 Hz
         */
        explicit HighRatePropagator(double gravityMag, double imuRateHz = 200.0)
            : gravity_(0.0, 0.0, -gravityMag),
              nominalDt_(imuRateHz > 0.0 ? 1.0 / imuRateHz : 0.005)
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
            hasBaseline_ = true;
        }

        /**
         * @brief 随每个 200Hz IMU 到达实时进行高频低延迟数值递推
         */
        gtsam::NavState propagate(const ImuMeasurement &imu)
        {
            // 显式基线标志：不能用 latestTime_ <= 0.0 当哨兵，因为相对时间戳
            // 从 0 开始是合法取值，首帧会被误判为“尚无基线”而丢弃积分。
            if (!hasBaseline_)
            {
                latestTime_ = imu.timestamp;
                hasBaseline_ = true;
                return currentState_;
            }

            double dt = imu.timestamp - latestTime_;
            if (dt <= 0.0 || dt > kMaxDtFactor * nominalDt_)
            {
                // 时间戳倒序或断档：无法确定真实积分区间。退回标称周期而不是
                // 硬编码 0.005 s——后者对应 200 Hz，改采样率后就与配置脱节。
                mylog::debug("[高频外推] 时间间隔异常 ({:.6f}s，标称 {:.6f}s)，按标称周期积分", dt, nominalDt_);
                dt = nominalDt_;
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
        double nominalDt_;
        double latestTime_{0.0};
        bool hasBaseline_{false};
        gtsam::NavState currentState_;
        gtsam::imuBias::ConstantBias currentBias_;
    };

} // namespace myslam::vio
