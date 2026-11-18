#pragma once

/**
 * @file imu_only_estimator.hpp
 * @brief 纯 IMU 航位推算后端：注册表的第二个实现，用于验证可替换性。
 *
 * 它不做任何视觉约束——视觉观测只被计数用于诊断。存在的意义不是精度，而是
 * 证明「换后端不改流水线」在类型与构建层面都成立：同一份 @c VioPipeline、
 * 同一个 @c domain::ISensorSource，只把 @c estimator.name 从 @c gtsam_fixed_lag 换成
 * @c imu_only，整个程序仍能编译、初始化、产出轨迹。
 *
 * 它也是 A/B 的基线：当某个新优化器表现异常时，切换到本实现可以立刻区分
 * 「估计器的问题」与「数据/时间同步的问题」。
 */

#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/propagator.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <algorithm>
#include <cmath>
#include <span>
#include <string_view>

namespace myslam::adapters
{

    /**
     * @brief 欧拉积分 + 零偏补偿的 IMU 航位推算
     */
    class ImuOnlyEstimator final : public domain::IEstimator
    {
    public:
        explicit ImuOnlyEstimator(const domain::PipelineConfig &config)
            // 复用领域层的高频外推器：此前本类自己写了一份欧拉积分
            // （R←R⊗Exp(ωdt)、a_w=R(a−ba)+g、p += v dt + ½a dt²），与
            // domain::StatePropagator 逐行相同。两份公式一旦需要修正
            // （例如时间戳语义、步长异常处理），必然只改一处。
            : propagator_(config.imu)
        {
        }

        [[nodiscard]] std::string_view name() const noexcept override { return "imu_only"; }

        bool initialize(const domain::Keyframe &firstKeyframe, const domain::KinematicState &initialState,
                        const domain::PoseCovariance &initialCovariance = {}) override
        {
            // 纯 IMU 航位推算不作协方差传播，先验协方差对它没有意义，
            // 显式忽略而不是默默接受一个永远不会被用的值。
            (void)initialCovariance;
            state_ = initialState;
            propagator_.reset(initialState);
            firstKeyframeId_ = firstKeyframe.id;
            keyframeCount_ = 1;
            return true;
        }

        domain::StepResult addKeyframe(const domain::Keyframe &keyframe, std::span<const domain::ImuSlice> imu) override
        {
            domain::StepResult result;

            if (imu.empty())
            {
                result.status = domain::StepStatus::SKIPPED;
                result.detail = "没有 IMU 数据可供积分";
                return result;
            }

            for (const domain::ImuSlice &slice : imu)
            {
                state_ = propagator_.propagate(slice);
            }

            ++keyframeCount_;
            visualObservations_ = keyframe.observations.size();
            if (keyframeCount_ % 20 == 0)
            {
                mylog::debug("[IMU-only] KF #{}: 积分 {} 条 IMU，纯惯性外推，视觉观测 {} 个（未参与约束）", keyframe.id,
                             imu.size(), visualObservations_);
            }

            result.status = domain::StepStatus::OK;
            result.recovery = domain::RecoveryAction::NONE;
            result.state = state_;
            result.health = {true, domain::Degradation::NONE, "纯 IMU 航位推算（无视觉约束）"};
            return result;
        }

        [[nodiscard]] domain::KinematicState latestState() const override { return state_; }

        [[nodiscard]] domain::HealthReport health() const override
        {
            return {true, domain::Degradation::NONE, "纯 IMU 航位推算（无视觉约束）"};
        }

        [[nodiscard]] domain::EstimateSnapshot snapshot() const override
        {
            domain::EstimateSnapshot snapshot;
            snapshot.keyframes.push_back({firstKeyframeId_, state_.timestamp, state_.pose});
            return snapshot;
        }

        [[nodiscard]] domain::EstimatorDiagnostics diagnostics() const override { return {0, 0, 0}; }

    private:
        domain::StatePropagator propagator_;
        domain::KinematicState state_{};
        domain::KeyframeId firstKeyframeId_{0};
        std::size_t keyframeCount_{0};
        std::size_t visualObservations_{0};
    };

} // namespace myslam::adapters
