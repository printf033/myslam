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
            if (!(dt > 0.0))
            {
                // dt <= 0 表示这段区间实际没有被覆盖（调用方漏填或零长切片）。
                // 不能像旧实现那样回退成标称周期：那会凭空发明一步积分，
                // 而时间戳又不动，之后每一条输出都带着一个错误的位置。
                mylog::debug("[高频外推] 非正积分步长 ({:.6f}s)，本步不推进", dt);
                return state_;
            }
            const double nominalCap = maxDtFactor_ * nominalDt_;
            if (dt > nominalCap)
            {
                // 不截断到标称周期：截断会让状态只前进 nominalDt 而时间戳
                // 跳到样本时刻，时间与内容脱钩且误差随断档长度无界增长。
                // 按实际步长积分（等价于假设该区间内匀速）并把可信度下降
                // 明确告知，比一个自相矛盾的状态好。
                mylog::warn("[高频外推] 积分步长 {:.4f}s 超过标称上限 {:.4f}s，按实际步长积分（该段输出可信度低）", dt,
                            nominalCap);
            }

            const Vec3 unbiasedGyro = slice.sample.gyro - state_.bias.gyro;
            const Vec3 unbiasedAcc = slice.sample.accel - state_.bias.accel;

            // 姿态前向更新: R_{t+dt} = R_t * Exp(omega * dt)
            state_.pose.orientation = state_.pose.orientation * exponentialMap(unbiasedGyro * dt);

            // 比力变换到世界系并加上重力: a_w = R * (a_b - ba) + g
            const Vec3 accWorld = state_.pose.orientation * unbiasedAcc + gravity_;

            state_.pose.position += state_.velocity * dt + 0.5 * accWorld * dt * dt;
            state_.velocity += accWorld * dt;
            // 积分结果对应区间的**终点**（sample 覆盖 [t, t+dt)），
            // 因此时间戳必须取 t + dt；取起点会让每条外推输出恒定滞后一步。
            state_.timestamp = slice.sample.timestamp + dt;
            latestTime_ = state_.timestamp;
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
