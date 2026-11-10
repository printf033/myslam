#pragma once

/**
 * @file propagator.hpp
 * @brief 高频（IMU 速率）零延迟状态外推器。
 *
 * 外推只用于优化之间的低延迟输出，不参与因子图。它是可选的：不提供外推的
 * 后端可以让工厂返回空指针，编排层就只输出优化结果。
 *
 * 约束：只依赖领域类型与标准库。
 */

#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <cmath>

namespace myslam::domain
{

    /**
     * @brief 用欧拉积分在 IMU 采样间隔内推进导航状态
     */
    class StatePropagator final : public IStatePropagator
    {
    public:
        explicit StatePropagator(const ImuConfig &config)
            : gravity_(0.0, 0.0, -config.gravityMagnitude),
              nominalDt_(config.rateHz > 0.0 ? 1.0 / config.rateHz : 0.005),
              maxDtFactor_(config.maxGapFactor > 0.0 ? config.maxGapFactor : 10.0)
        {
        }

        void reset(const KinematicState &state) override
        {
            state_ = state;
            latestTime_ = state.timestamp;
            hasBaseline_ = true;
        }

        KinematicState propagate(const ImuSlice &slice) override
        {
            // 显式基线标志：不能用 latestTime_ <= 0 当哨兵，相对时间戳从 0 开始是合法值
            if (!hasBaseline_)
            {
                latestTime_ = slice.sample.timestamp;
                state_.timestamp = slice.sample.timestamp;
                hasBaseline_ = true;
                return state_;
            }

            double dt = slice.dt;
            if (!(dt > 0.0) || dt > maxDtFactor_ * nominalDt_)
            {
                mylog::debug("[高频外推] 积分步长异常 ({:.6f}s，标称 {:.6f}s)，按标称周期积分", dt, nominalDt_);
                dt = nominalDt_;
            }

            const Vec3 unbiasedGyro = slice.sample.gyro - state_.bias.gyro;
            const Vec3 unbiasedAcc = slice.sample.accel - state_.bias.accel;

            // 姿态前向更新: R_{t+dt} = R_t * Exp(omega * dt)
            state_.pose.q = state_.pose.q * exponentialMap(unbiasedGyro * dt);

            // 比力变换到世界系并加上重力: a_w = R * (a_b - ba) + g
            const Vec3 accWorld = state_.pose.q * unbiasedAcc + gravity_;

            state_.pose.t += state_.velocity * dt + 0.5 * accWorld * dt * dt;
            state_.velocity += accWorld * dt;
            state_.timestamp = slice.sample.timestamp;
            latestTime_ = slice.sample.timestamp;
            return state_;
        }

        [[nodiscard]] KinematicState latestState() const { return state_; }

    private:
        /// 罗德里格斯公式的闭式四元数指数映射
        [[nodiscard]] static Eigen::Quaterniond exponentialMap(const Vec3 &rotationVector)
        {
            const double angle = rotationVector.norm();
            if (angle < 1e-12)
            {
                return Eigen::Quaterniond(1.0, 0.5 * rotationVector.x(), 0.5 * rotationVector.y(),
                                          0.5 * rotationVector.z())
                    .normalized();
            }
            return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotationVector / angle));
        }

        Vec3 gravity_;
        double nominalDt_;
        double maxDtFactor_;
        double latestTime_{0.0};
        bool hasBaseline_{false};
        KinematicState state_{};
    };

} // namespace myslam::domain
