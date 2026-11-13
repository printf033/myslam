#pragma once

/**
 * @file pipeline.hpp
 * @brief 与传感器来源、估计器实现都无关的 VIO 流水线核心。
 *
 * @par 依赖面
 *
 * 本文件只包含 @c domain/。它不认识 gtsam、OpenCV、gtsam 的 @c Values，
 * 也不认识任何具体估计器或数据源。估计器由工厂按配置名创建，重建时复用
 * 同一个工厂——注入的实现不会被 @c reset 静默换掉。
 *
 * @par 职责边界
 *
 * 属于本类：初始化状态机、关键帧决策、IMU 窗口归属、调用估计器、执行
 * 估计器建议的恢复动作。
 * 不属于本类：IMU/相机时间同步（由 @c TimeSynchronizer 或调用方的固定
 * 步长循环负责）、图像读取与特征提取、可视化、轨迹导出。
 *
 * @par 错误处理
 *
 * 本类不抛异常。估计器内部必须截住第三方异常并转成 @c StepResult；本类
 * 只根据 @c status / @c recovery 决定下一步，不猜「失败就该重建」。
 */

#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/initializer.hpp"
#include "domain/keyframe_selector.hpp"
#include "domain/ports.hpp"
#include "domain/propagator.hpp"
#include "domain/registry.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <algorithm>
#include <cstddef>
#include <format>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace myslam::vio
{

    /**
     * @brief 初始化阶段单帧推进的结果
     */
    struct InitOutcome
    {
        bool aligned{false};
        bool tracking{false};
        bool relaxed{false};
        std::optional<domain::Pose> initialPose;
    };

    /**
     * @brief 追踪阶段单帧处理的结果
     */
    struct FrameOutcome
    {
        enum class Kind
        {
            NOT_TRACKING,
            IGNORED,
            KEYFRAME_SKIPPED,
            KEYFRAME_OPTIMIZED,
            KEYFRAME_RECOVERED,
            KEYFRAME_FAILED,
        };

        Kind kind{Kind::NOT_TRACKING};
        domain::KeyframeDecision decision{};
        std::optional<domain::Pose> pose;
        domain::HealthReport health{};
        /// 估计器建议的恢复动作；编排层据此决定是否重建，而不是一律重置
        domain::RecoveryAction recovery{domain::RecoveryAction::NONE};
        std::string failureReason;

        [[nodiscard]] bool producedPose() const noexcept
        {
            return kind == Kind::KEYFRAME_OPTIMIZED && pose.has_value();
        }
    };

    /**
     * @brief 传感器无关、估计器无关的 VIO 流水线
     */
    class VioPipeline
    {
    public:
        /**
         * @brief 用工厂按配置创建估计器
         *
         * 不接收 @c SystemConfig：相机采样步长、运行时长与验收门限都是编排层
         * 的口径，流水线一个都不消费。此前它作为成员被存下来却从未读取——
         * 这种"看起来依赖"比显式不依赖更容易误导后续修改。
         *
         * @param config 分组后的算法配置
         * @param factory 估计器工厂；为空时按 @c config.estimator.name 查询注册表
         */
        explicit VioPipeline(const domain::PipelineConfig &config, domain::EstimatorFactory factory = {})
            : config_(config),
              factory_(factory ? std::move(factory) : registryFactory()),
              initializer_(config.initialization, config.imu),
              selector_(config.vision),
              propagator_(config.imu)
        {
            std::expected<domain::EstimatorBundle, std::string> bundle =
                std::unexpected(std::string("估计器工厂未执行"));
            try
            {
                bundle = factory_(config_);
            }
            catch (const std::exception &error)
            {
                error_ = std::string("估计器工厂抛出异常: ") + error.what();
                mylog::error("[流水线] {}", error_);
                return;
            }
            if (!bundle)
            {
                error_ = bundle.error();
                mylog::error("[流水线] 估计器装配失败: {}", error_);
                return;
            }
            if (!adoptBundle(std::move(*bundle)))
            {
                return;
            }
            mylog::debug("[流水线] 估计器={}, 观测模型={}", estimator_->name(), model_->name());
        }

        /**
         * @brief 注入估计器实例，并保留工厂用于重建
         *
         * @param estimator 估计器实现，不得为空
         * @param observationModel 该估计器消费的观测模型，不得为空
         * @param factory 重建工厂；为空时 @c reset 会明确失败而不是换回默认实现
         */
        VioPipeline(const domain::PipelineConfig &config, std::unique_ptr<domain::IEstimator> estimator,
                    std::shared_ptr<const domain::ICameraModel> observationModel, domain::EstimatorFactory factory = {})
            : config_(config),
              factory_(std::move(factory)),
              initializer_(config.initialization, config.imu),
              selector_(config.vision),
              propagator_(config.imu),
              estimator_(std::move(estimator)),
              model_(std::move(observationModel))
        {
            if (!estimator_ || !model_)
            {
                error_ = "注入的估计器或观测模型为空";
                mylog::error("[流水线] {}", error_);
                estimator_.reset();
                model_.reset();
            }
        }

        /// 估计器与观测模型都必须存在；缺一个就让流水线明确无效
        [[nodiscard]] bool valid() const noexcept { return estimator_ != nullptr && model_ != nullptr; }

        [[nodiscard]] const std::string &error() const noexcept { return error_; }

        /// 高层是否处于追踪状态
        [[nodiscard]] bool tracking() const noexcept { return tracking_; }

        /// 状态机（初始化阶段由本类维护，追踪后固定为 TRACKING_OK）
        [[nodiscard]] domain::SystemState state() const noexcept
        {
            if (!valid())
            {
                return domain::SystemState::UNINITIALIZED;
            }
            if (tracking_)
            {
                return domain::SystemState::TRACKING_OK;
            }
            return initializer_.state();
        }

        /**
         * @brief 喂入一条带步长的 IMU 测量
         *
         * 初始化阶段只进入采样窗口；追踪阶段累积到「上一个成功关键帧以来」的
         * IMU 窗口（随下一个关键帧交给估计器），并同时驱动高频外推。
         */
        std::optional<domain::KinematicState> feedImu(const domain::ImuSlice &slice)
        {
            if (!tracking_)
            {
                initializer_.feedImu(slice.sample);
                return std::nullopt;
            }

            imuWindow_.push_back(slice);
            noteImuWindowPressure();
            return propagator_.propagate(slice);
        }

        /**
         * @brief 初始化阶段推进一帧
         */
        InitOutcome stepInitialization(domain::KeyframeId keyframeId, const domain::Frame &frame, double timestamp)
        {
            InitOutcome outcome;
            if (!valid())
            {
                return outcome;
            }

            if (initializer_.state() == domain::SystemState::UNINITIALIZED ||
                initializer_.state() == domain::SystemState::STATIONARY_COLLECTING)
            {
                domain::Pose alignedPose;
                domain::Bias alignedBias;

                if (!config_.initialization.gravityAlignment)
                {
                    outcome.aligned = initializer_.skipGravityAlignment(
                        alignedPose, alignedBias, config_.ablationInitialPose(),
                        static_cast<std::size_t>(std::max(1, config_.initialization.minSamples)));
                }
                else
                {
                    if (config_.initialization.timeoutSeconds > 0.0)
                    {
                        if (initDeadline_ < 0.0)
                        {
                            initDeadline_ = timestamp + config_.initialization.timeoutSeconds;
                        }
                        outcome.relaxed = timestamp >= initDeadline_;
                    }
                    outcome.aligned = initializer_.tryGravityAlignment(alignedPose, alignedBias, !outcome.relaxed);
                }
            }

            if (initializer_.state() != domain::SystemState::GRAVITY_ALIGNED)
            {
                return outcome;
            }

            domain::KinematicState initialState;
            initialState.timestamp = timestamp;
            if (initialHint_)
            {
                // 重建后从上一段断裂处继续。否则每段都从原点重新出发，
                // 轨迹在段边界跳回 (0,0,0)，下游拿到的是碎片而非一条轨迹。
                // 位置与速度都取上一段的终态；零偏仍用本段重新对齐的结果，
                // 因为跨段沿用旧零偏会把上一段的误差带入新段。
                initialState.pose = initialHint_->pose;
                initialState.velocity = initialHint_->velocity;
            }
            else
            {
                initialState.pose = initializer_.alignedPose();
                initialState.pose.t.z() = config_.initialization.initialHeightMeters;
                // 默认零，适用于静止起始；运动起始的序列由配置注入真值速度。
                initialState.velocity = config_.initialization.initialVelocity;
            }
            initialState.bias = initializer_.alignedBias();

            domain::Keyframe firstKeyframe;
            firstKeyframe.id = keyframeId;
            firstKeyframe.timestamp = timestamp;
            firstKeyframe.observations = frame.observations;
            firstKeyframe.image = frame.image;

            // 首关键帧退化门控：观测太少或全部挤在图像中心附近时，三角化
            // 病态（深度不可观），初始化后视觉约束不足。保持 GRAVITY_ALIGNED
            // 等下一帧重试，而不是带病启动。
            {
                const auto &obs = frame.observations;
                if (static_cast<int>(obs.size()) < config_.initialization.minInitObservations)
                {
                    mylog::debug("[流水线] 初始化门控：观测数 {} < {}，延迟初始化", obs.size(),
                                 config_.initialization.minInitObservations);
                    return outcome;
                }

                // 射线最大夹角：优先用前端已算好的 normalized，缺失时用相机模型反投影
                std::vector<domain::Vec3> rays;
                rays.reserve(obs.size());
                for (const auto &o : obs)
                {
                    if (o.normalized.has_value())
                    {
                        rays.push_back(*o.normalized);
                    }
                    else if (auto ray = model_->unproject(o.uv))
                    {
                        rays.push_back(*ray);
                    }
                }

                double maxSpreadRad = 0.0;
                for (std::size_t i = 0; i < rays.size(); ++i)
                {
                    for (std::size_t j = i + 1; j < rays.size(); ++j)
                    {
                        const double cosAngle =
                            std::clamp(rays[i].dot(rays[j]) / (rays[i].norm() * rays[j].norm()), -1.0, 1.0);
                        maxSpreadRad = std::max(maxSpreadRad, std::acos(cosAngle));
                    }
                }

                const double minSpreadRad = config_.initialization.minInitSpreadDeg * std::numbers::pi / 180.0;
                if (maxSpreadRad < minSpreadRad)
                {
                    mylog::debug("[流水线] 初始化门控：射线最大夹角 {:.2f}° < {:.2f}°，延迟初始化",
                                 maxSpreadRad * 180.0 / std::numbers::pi, config_.initialization.minInitSpreadDeg);
                    return outcome;
                }
            }

            bool initialized = false;
            try
            {
                initialized = estimator_->initialize(firstKeyframe, initialState);
            }
            catch (const std::exception &error)
            {
                mylog::error("[流水线] 估计器初始化抛出异常: {}（保持 GRAVITY_ALIGNED 等待重试）", error.what());
                return outcome;
            }

            if (!initialized)
            {
                mylog::warn("[流水线] 首帧先验注入失败，保持 GRAVITY_ALIGNED 等待下一帧重试");
                return outcome;
            }

            propagator_.reset(initialState);
            lastState_ = initialState;
            lastKeyframeId_ = keyframeId;
            lastKeyframeObservations_ = frame.observations;
            lastKeyframeTimestamp_ = timestamp;
            clearImuWindow();
            tracking_ = true;
            // 先验只用于本次启动；后续若再次重置，由 reset() 重新采集当前状态。
            initialHint_.reset();

            outcome.tracking = true;
            outcome.initialPose = initialState.pose;
            return outcome;
        }

        /**
         * @brief 追踪阶段处理一帧
         *
         * @param keyframeId 本帧若成为关键帧时写入的编号
         * @param frame 当前帧（含观测与可选图像）
         * @param timestamp 当前时刻 (s)
         * @param requireImuSinceLastKeyframe 是否要求自上个成功关键帧以来至少
         *        有一条 IMU。真实序列存在采集空洞，缺失时估计器无法构建
         *        预积分因子，应跳过本帧。
         */
        FrameOutcome processFrame(domain::KeyframeId keyframeId, const domain::Frame &frame, double timestamp,
                                  bool requireImuSinceLastKeyframe = false)
        {
            FrameOutcome outcome;

            if (!valid() || !tracking_)
            {
                outcome.kind = FrameOutcome::Kind::NOT_TRACKING;
                return outcome;
            }

            if (imuWindowOverdue_)
            {
                // IMU 窗口已超过保留时限：再拖下去要么丢积分前缀（破坏
                // 「窗口覆盖自上个关键帧以来」的契约，预积分区间与关键帧
                // 时间戳错位），要么无界增长。强制插一个关键帧把这段区间
                // 完整交给估计器消费掉。视觉中断时这一帧可以没有观测——
                // 只有 IMU 因子也是有效的一步。
                outcome.decision = {true, "IMU 窗口到达保留时限，强制插入关键帧以完整消费积分区间", 0.0, 1.0};
            }
            else
            {
                outcome.decision = selector_.evaluate(frame, lastKeyframeObservations_, lastKeyframeTimestamp_);
            }
            if (!outcome.decision.isKeyframe)
            {
                outcome.kind = FrameOutcome::Kind::IGNORED;
                return outcome;
            }

            if (requireImuSinceLastKeyframe && imuWindow_.empty())
            {
                outcome.kind = FrameOutcome::Kind::KEYFRAME_SKIPPED;
                return outcome;
            }

            domain::Keyframe keyframe;
            keyframe.id = keyframeId;
            keyframe.timestamp = timestamp;
            keyframe.observations = frame.observations;
            keyframe.image = frame.image;

            domain::StepResult result;
            try
            {
                result = estimator_->addKeyframe(keyframe, imuWindow_);
            }
            catch (const std::exception &error)
            {
                // 契约要求适配器自己截住第三方异常，但"契约要求"不是机制。
                // 任何穿过来的异常在这里转成失败 + 重建，否则它会一路穿到
                // 编排层之外 terminate 整个进程（旧实现在两层都做了兜底）。
                result.status = domain::StepStatus::FAILED;
                result.recovery = domain::RecoveryAction::RESET_PIPELINE;
                result.detail = std::string("估计器抛出异常: ") + error.what();
                mylog::error("[流水线] {}", result.detail);
            }
            outcome.health = result.health;
            outcome.recovery = result.recovery;

            switch (result.status)
            {
            case domain::StepStatus::OK:
                if (!result.state)
                {
                    outcome.kind = FrameOutcome::Kind::KEYFRAME_FAILED;
                    outcome.failureReason = "估计器返回 OK 但未给出状态";
                    outcome.recovery = domain::RecoveryAction::RESET_PIPELINE;
                    return outcome;
                }
                propagator_.reset(*result.state);
                lastState_ = *result.state;
                lastKeyframeId_ = keyframeId;
                // 空观测的关键帧不能覆盖基线：它会抹掉"上一次能比对的特征
                // 集合"，让视觉恢复后既无共视率也无视差可算。保留旧基线，
                // 恢复帧与它比较时共视率自然为 0，从而触发关键帧。
                if (!frame.observations.empty())
                {
                    lastKeyframeObservations_ = frame.observations;
                }
                lastKeyframeTimestamp_ = timestamp;
                clearImuWindow();
                consecutiveLocalRebuilds_ = 0;
                outcome.pose = result.state->pose;
                outcome.kind = FrameOutcome::Kind::KEYFRAME_OPTIMIZED;
                return outcome;

            case domain::StepStatus::RECOVERED:
                // 估计器已局部恢复：坐标系连续，但本帧不产出位姿。
                // 刻意不更新上一关键帧基线，让下一次三角化的基线更长。
                outcome.kind = FrameOutcome::Kind::KEYFRAME_RECOVERED;
                return outcome;

            case domain::StepStatus::SKIPPED:
                outcome.kind = FrameOutcome::Kind::KEYFRAME_SKIPPED;
                return outcome;

            case domain::StepStatus::FAILED:
                outcome.kind = FrameOutcome::Kind::KEYFRAME_FAILED;
                outcome.failureReason = result.detail;
                return outcome;
            }

            outcome.kind = FrameOutcome::Kind::KEYFRAME_FAILED;
            outcome.failureReason = "估计器返回了未知状态";
            return outcome;
        }

        /**
         * @brief 执行估计器建议的恢复动作
         *
         * @param action 动作；@c NONE 与 @c RELOCALIZE 不由本类处理
         */
        /**
         * @brief 注入重建后的初始状态先验（位置 + 速度）
         *
         * 段重建时调用：新段从上一段的终态继续，保证输出的轨迹在时间轴上
         * 连续。不调用时保持旧行为（首次初始化从原点出发）。
         */
        void setInitialStateHint(const domain::KinematicState &state) { initialHint_ = state; }

        /// 清除初始状态先验（回到「从原点开始」的行为）
        void clearInitialStateHint() { initialHint_.reset(); }

        void reset(domain::RecoveryAction action = domain::RecoveryAction::RESET_PIPELINE)
        {
            if (action == domain::RecoveryAction::NONE || action == domain::RecoveryAction::RELOCALIZE)
            {
                return;
            }

            // 就地重置也会回到未初始化状态，必须先把当前状态记下来作为重启先验，
            // 否则这条恢复路径同样会把轨迹拉回原点。
            if (lastState_)
            {
                initialHint_ = *lastState_;
            }

            if (action == domain::RecoveryAction::RESET_ESTIMATOR && factory_ && lastState_)
            {
                if (tryRebuildEstimatorInPlace())
                {
                    return;
                }
                mylog::warn("[流水线] 估计器局部重建失败，退回完整重置");
            }

            initializer_.resetSamples();
            propagator_ = domain::StatePropagator(config_.imu);
            clearImuWindow();
            lastKeyframeObservations_.clear();
            lastKeyframeTimestamp_ = -1.0;
            lastState_.reset();
            tracking_ = false;
            initDeadline_ = -1.0;
            consecutiveLocalRebuilds_ = 0;
            ++resets_;

            if (!factory_)
            {
                error_ = "重建失败：未提供估计器工厂（拒绝静默回退到默认实现）";
                mylog::error("[流水线] {}", error_);
                estimator_.reset();
                model_.reset();
                return;
            }

            try
            {
                auto bundle = factory_(config_);
                if (!bundle)
                {
                    error_ = bundle.error();
                    mylog::error("[流水线] 重建估计器失败: {}", error_);
                    estimator_.reset();
                    model_.reset();
                    return;
                }
                if (adoptBundle(std::move(*bundle)))
                {
                    error_.clear();
                }
            }
            catch (const std::exception &error)
            {
                error_ = std::string("重建时工厂抛出异常: ") + error.what();
                mylog::error("[流水线] {}", error_);
                estimator_.reset();
                model_.reset();
            }
        }

        [[nodiscard]] std::size_t resets() const noexcept { return resets_; }
        [[nodiscard]] std::size_t imuSamplesSinceLastKeyframe() const noexcept { return imuWindow_.size(); }
        [[nodiscard]] bool hasLastKeyframe() const noexcept { return lastKeyframeTimestamp_ >= 0.0; }

        /// 观测模型：编排层构造前端时必须使用同一个实例
        [[nodiscard]] std::shared_ptr<const domain::ICameraModel> observationModel() const noexcept { return model_; }

        /**
         * @brief 当前估计器；无效流水线返回 @c nullptr
         *
         * 返回指针而不是引用：装配失败后流水线仍然存在，任何"解引用即认为
         * 一定有效"的接口都会变成 UB 入口。调用方必须先判空（或先查
         * @c valid()）。
         */
        [[nodiscard]] const domain::IEstimator *estimator() const noexcept { return estimator_.get(); }

        [[nodiscard]] domain::KinematicState latestState() const
        {
            return lastState_.value_or(domain::KinematicState{});
        }

        [[nodiscard]] domain::HealthReport health() const
        {
            return estimator_ ? estimator_->health() : domain::HealthReport{};
        }

        [[nodiscard]] domain::EstimateSnapshot snapshot() const
        {
            return estimator_ ? estimator_->snapshot() : domain::EstimateSnapshot{};
        }

        [[nodiscard]] domain::EstimatorDiagnostics diagnostics() const
        {
            return estimator_ ? estimator_->diagnostics() : domain::EstimatorDiagnostics{};
        }

    private:
        /// 就地重建的连续上限：超过即放弃坐标系连续性
        static constexpr std::size_t kMaxConsecutiveLocalRebuilds = 3;

        /**
         * @brief 接管工厂产出，校验完整性
         *
         * 注册表内部会校验，但 @c VioPipeline 也接受任意调用方工厂；一条
         * 返回 @c {nullptr, model} 的工厂若不被拦下，下一行 @c name() 就会
         * 空解引用。
         */
        bool adoptBundle(domain::EstimatorBundle bundle)
        {
            if (!bundle.estimator || !bundle.observationModel)
            {
                error_ = "工厂返回了不完整的 bundle（估计器或观测模型为空）";
                mylog::error("[流水线] {}", error_);
                estimator_.reset();
                model_.reset();
                return false;
            }
            estimator_ = std::move(bundle.estimator);
            model_ = std::move(bundle.observationModel);
            return true;
        }

        /**
         * @brief 标记 IMU 窗口是否已超过保留时限
         *
         * **不能**丢弃窗口前缀来限内存：估计器把整段窗口当作「自上个关键帧
         * 以来的 IMU」消费（预积分 + CombinedImuFactor），丢前缀会让积分
         * 时长与关键帧时间戳错位，产生一个静默的错误先验。
         *
         * 正确做法是反向施压：窗口超时限就要求下一帧必须成为关键帧，把
         * 整段区间完整交出去。静止或视觉中断导致长时间无关键帧时，
         * 内存只在到达时限前增长，然后被一次强制关键帧清空。
         */
        void noteImuWindowPressure()
        {
            if (imuWindow_.empty())
            {
                return;
            }
            const double keepSeconds = std::max(4.0 * config_.estimator.slidingWindowLag,
                                                config_.imu.maxGapFactor / std::max(1.0, config_.imu.rateHz));
            const double span = imuWindow_.back().sample.timestamp - imuWindow_.front().sample.timestamp;
            if (span > keepSeconds)
            {
                if (!imuWindowOverdue_)
                {
                    mylog::warn("[流水线] IMU 窗口已达 {:.2f}s（上限 {:.2f}s）仍无关键帧，强制下一帧插入", span,
                                keepSeconds);
                }
                imuWindowOverdue_ = true;
            }
        }

        void clearImuWindow()
        {
            imuWindow_.clear();
            imuWindowOverdue_ = false;
        }

        /**
         * @brief 用工厂重建估计器并以当前状态重新初始化（坐标系连续）
         *
         * 与完整重置的区别：保留关键帧编号、观测基线与外推基线。所有失败
         * 路径都返回 @c false 交给调用方退回完整重置，不抛异常。
         */
        bool tryRebuildEstimatorInPlace()
        {
            try
            {
                auto bundle = factory_(config_);
                if (!bundle)
                {
                    mylog::warn("[流水线] 局部重建失败: {}", bundle.error());
                    return false;
                }
                if (!bundle->estimator || !bundle->observationModel)
                {
                    mylog::warn("[流水线] 局部重建的工厂产出不完整（估计器或观测模型为空）");
                    return false;
                }
                // 只在同一种观测模型下替换：前端（例如回放的特征跟踪器）持有
                // model_ 这个 shared_ptr，换参数不同但同名的模型同样会让
                // 「产生观测的模型」与「消费观测的模型」悄悄错开。
                if (bundle->observationModel->name() != model_->name())
                {
                    mylog::warn("[流水线] 局部重建产出了不同的观测模型 ({} -> {})，退回完整重置", model_->name(),
                                bundle->observationModel->name());
                    return false;
                }

                domain::Keyframe anchor;
                anchor.id = lastKeyframeId_;
                anchor.timestamp = lastState_->timestamp;
                anchor.observations = lastKeyframeObservations_;
                if (!bundle->estimator->initialize(anchor, *lastState_))
                {
                    return false;
                }

                estimator_ = std::move(bundle->estimator);
                propagator_.reset(*lastState_);
                // **刻意保留** imuWindow_：新估计器锚定在 lastState_（上一个
                // 成功关键帧）初始化，它还没有消费这段区间。若在此清空，
                // 下一个关键帧只会拿到「重建之后」的尾段切片，而预积分因子
                // 却声称覆盖「锚点 -> 新关键帧」——积分时长与关键帧时间间隔
                // 错位，正是 trimImuWindow 那版实现被移除的原因。
                // 新估计器每次 integrate() 都会重建 pim_，因此重复交付不会
                // 造成重复积分。imuWindowOverdue_ 也一并保留，让这段区间尽快
                // 在下一次相机帧被消费掉。
                ++consecutiveLocalRebuilds_;
                if (consecutiveLocalRebuilds_ > kMaxConsecutiveLocalRebuilds)
                {
                    // 重建后仍持续失败：保留窗口的代价（无界增长 + 每帧完整求解）
                    // 已经超过坐标系连续性的收益。退回完整重置，让调用方重新
                    // 初始化；调用方据 resets() 仍能看到这一段被重启过。
                    mylog::error("[流水线] 连续就地重建 {} 次仍失败，放弃坐标系连续性，改为完整重置",
                                 consecutiveLocalRebuilds_);
                    return false;
                }
                ++resets_;
                mylog::info("[流水线] 估计器已局部重建，坐标系保持连续（锚点 KF #{}，保留 {} 条待积分 IMU）",
                            lastKeyframeId_, imuWindow_.size());
                return true;
            }
            catch (const std::exception &error)
            {
                mylog::warn("[流水线] 局部重建抛出异常: {}", error.what());
                return false;
            }
        }

        /// 默认工厂：按配置里的名字查询注册表
        [[nodiscard]] static domain::EstimatorFactory registryFactory()
        {
            return [](const domain::PipelineConfig &config)
            { return domain::EstimatorRegistry::instance().create(config.estimator.name, config); };
        }

        domain::PipelineConfig config_;
        domain::EstimatorFactory factory_;

        domain::Initializer initializer_;
        domain::KeyframeSelector selector_;
        domain::StatePropagator propagator_;

        std::unique_ptr<domain::IEstimator> estimator_;
        std::shared_ptr<const domain::ICameraModel> model_;

        std::vector<domain::ImuSlice> imuWindow_;
        /// IMU 窗口已超过保留时限：下一帧必须成为关键帧
        bool imuWindowOverdue_{false};
        /// 自上次成功关键帧以来连续就地重建的次数。
        ///
        /// 就地重建保留窗口（正确性所需），但如果重建后每帧都继续失败，
        /// 窗口就会按 IMU 速率无界增长，并且每帧都做一次完整求解。超过上限
        /// 后退化为完整重置：坐标系不得不作废，但内存与算力被封顶。
        std::size_t consecutiveLocalRebuilds_{0};
        std::vector<domain::Observation> lastKeyframeObservations_;
        double lastKeyframeTimestamp_{-1.0};
        domain::KeyframeId lastKeyframeId_{0};
        std::optional<domain::KinematicState> lastState_;

        /// 重建后的起点：只对「下一次初始化」生效，初始化成功后清空
        std::optional<domain::KinematicState> initialHint_;
        double initDeadline_{-1.0};
        bool tracking_{false};
        std::size_t resets_{0};
        std::string error_;
    };

} // namespace myslam::vio
