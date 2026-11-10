#pragma once

/**
 * @file fixed_lag_estimator.hpp
 * @brief gtsam 固定滞后平滑后端：@c IEstimator 的参考实现。
 *
 * @par 从旧 @c VioBackend 的迁移
 *
 * 算法原样保留（预积分 + 投影因子 + iSAM2 固定滞后 + 分层外点门控），
 * 变化只在边界：
 *   - 输入不再逐条流式喂 IMU，而是由编排层把「自上个关键帧以来的 IMU 区间」
 *     随关键帧一起传入；本类在入口处批量预积分。时间同步因此不再属于后端；
 *   - 上一关键帧的观测由本类自己保存，契约里不再出现「两视图」假设；
 *   - 失败用 @c StepResult 表达，并给出恢复建议；
 *   - 所有对外查询返回值，不承诺引用生命周期。
 *
 * 只接受提供针孔标定的模型（@c IPinholeCalibration）。把真实 MEI 模型配给
 * 本实现会在构造期报错，而不是运行期算出错误的雅可比。
 */

#include "adapters/gtsam/types.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/health_monitor.hpp"
#include "domain/ports.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <format>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace myslam::adapters
{

    using gtsam::symbol_shorthand::B; // imuBias::ConstantBias
    using gtsam::symbol_shorthand::L; // Point3 (Landmark)
    using gtsam::symbol_shorthand::V; // Vector3 (Velocity)
    using gtsam::symbol_shorthand::X; // Pose3 (R, p)

    /**
     * @brief 紧耦合固定滞后平滑器（iSAM2 + 真边缘化）
     */
    class FixedLagEstimator final : public domain::IEstimator
    {
    public:
        FixedLagEstimator(const domain::PipelineConfig &config, std::shared_ptr<const domain::ICameraModel> camera)
            : config_(config),
              camera_(std::move(camera)),
              pinhole_(dynamic_cast<const domain::IPinholeCalibration *>(camera_.get())),
              healthMonitor_(config.health)
        {
            if (!camera_ || pinhole_ == nullptr)
            {
                mylog::error("[gtsam后端] 相机模型 '{}' 不提供针孔标定（IPinholeCalibration）；"
                             "该模型只能配给支持全向重投影的估计器。流水线将以未初始化状态运行",
                             camera_ ? std::string(camera_->name()) : std::string("(空)"));
                valid_ = false;
                return;
            }

            K_ = std::make_shared<::gtsam::Cal3_S2>(pinhole_->fx(), pinhole_->fy(), 0.0, pinhole_->principalPoint().x(),
                                                    pinhole_->principalPoint().y());

            auto params = ::gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(config_.imu.gravityMagnitude);
            params->accelerometerCovariance = ::gtsam::I_3x3 * std::pow(config_.imu.accelNoiseSigma, 2);
            params->gyroscopeCovariance = ::gtsam::I_3x3 * std::pow(config_.imu.gyroNoiseSigma, 2);
            params->integrationCovariance = ::gtsam::I_3x3 * config_.imu.integrationNoise;
            params->biasAccCovariance = ::gtsam::I_3x3 * std::pow(config_.imu.accelBiasRw, 2);
            params->biasOmegaCovariance = ::gtsam::I_3x3 * std::pow(config_.imu.gyroBiasRw, 2);
            imuParams_ = params;

            const auto isotropicPixel = ::gtsam::noiseModel::Isotropic::Sigma(2, config_.vision.pixelNoiseSigma);
            robustPixelNoise_ = ::gtsam::noiseModel::Robust::Create(
                ::gtsam::noiseModel::mEstimator::Huber::Create(config_.vision.huberLossDelta), isotropicPixel);

            isamParams_.relinearizeThreshold = 0.05;
            isamParams_.relinearizeSkip = 1;
            isamParams_.findUnusedFactorSlots = true;
            isamParams_.factorization = ::gtsam::ISAM2Params::CHOLESKY;

            createSmoother();
            pim_ = std::make_unique<::gtsam::PreintegratedCombinedMeasurements>(imuParams_, toGtsam(currentBias_));
        }

        [[nodiscard]] std::string_view name() const noexcept override { return "gtsam_fixed_lag"; }

        [[nodiscard]] bool valid() const noexcept { return valid_; }

        bool initialize(const domain::Keyframe &firstKeyframe, const domain::KinematicState &initialState) override
        {
            const domain::KeyframeId firstKeyframeId = firstKeyframe.id;
            if (!valid_)
            {
                return false;
            }

            resetInternal();
            currentBias_ = initialState.bias;
            latestTimestamp_ = initialState.timestamp;
            latestNavState_ = toGtsam(initialState);
            pim_ = std::make_unique<::gtsam::PreintegratedCombinedMeasurements>(imuParams_, toGtsam(currentBias_));

            const double ip = config_.estimator.initialPoseSigma;
            const double ib = config_.estimator.initialBiasSigma;
            auto poseNoise = ::gtsam::noiseModel::Diagonal::Sigmas(
                (::gtsam::Vector(6) << ip, ip, ip, 2.0 * ip, 2.0 * ip, 2.0 * ip).finished());
            auto velNoise = ::gtsam::noiseModel::Isotropic::Sigma(3, config_.estimator.initialVelSigma);
            auto biasNoise = ::gtsam::noiseModel::Diagonal::Sigmas(
                (::gtsam::Vector(6) << ib, ib, ib, 0.4 * ib, 0.4 * ib, 0.4 * ib).finished());

            newFactors_.addPrior(X(firstKeyframeId), latestNavState_.pose(), poseNoise);
            newFactors_.addPrior(V(firstKeyframeId), latestNavState_.velocity(), velNoise);
            newFactors_.addPrior(B(firstKeyframeId), toGtsam(currentBias_), biasNoise);

            newValues_.insert(X(firstKeyframeId), latestNavState_.pose());
            newValues_.insert(V(firstKeyframeId), latestNavState_.velocity());
            newValues_.insert(B(firstKeyframeId), toGtsam(currentBias_));

            ::gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
            timestamps[X(firstKeyframeId)] = initialState.timestamp;
            timestamps[V(firstKeyframeId)] = initialState.timestamp;
            timestamps[B(firstKeyframeId)] = initialState.timestamp;

            try
            {
                smoother_->update(newFactors_, newValues_, timestamps);
                currentEstimate_ = smoother_->calculateEstimate();
            }
            catch (const std::exception &error)
            {
                mylog::error("[gtsam后端] 首帧先验注入失败: {}", error.what());
                newFactors_.resize(0);
                newValues_.clear();
                return false;
            }

            newFactors_.resize(0);
            newValues_.clear();
            windowKeyframes_.push_back(firstKeyframeId);
            // 首个关键帧的观测就是下一次三角化的基线；不保存它会让 KF #1
            // 无法建立任何路标，起步阶段的精度因此明显变差。
            previousObservations_.assign(firstKeyframe.observations.begin(), firstKeyframe.observations.end());
            lastHealth_ = {};

            mylog::info("[gtsam后端初始化] 注入初始帧先验: KF #{}, P=[{:.3f}, {:.3f}, {:.3f}], V=[{:.3f}, {:.3f}, "
                        "{:.3f}]",
                        firstKeyframeId, initialState.pose.t.x(), initialState.pose.t.y(), initialState.pose.t.z(),
                        initialState.velocity.x(), initialState.velocity.y(), initialState.velocity.z());
            return true;
        }

        domain::StepResult addKeyframe(const domain::Keyframe &keyframe, std::span<const domain::ImuSlice> imu) override
        {
            domain::StepResult result;

            if (!valid_)
            {
                result.status = domain::StepStatus::FAILED;
                result.recovery = domain::RecoveryAction::RESET_PIPELINE;
                result.detail = "估计器构造失败（相机模型不提供针孔标定）";
                return result;
            }
            if (windowKeyframes_.empty())
            {
                result.status = domain::StepStatus::FAILED;
                result.recovery = domain::RecoveryAction::RESET_PIPELINE;
                result.detail = "滑动窗口为空（缺少 initialize 或首帧已被边缘化）";
                mylog::error("[gtsam后端] KF #{}: {}", keyframe.id, result.detail);
                return result;
            }
            if (imu.empty())
            {
                result.status = domain::StepStatus::SKIPPED;
                result.detail = "自上个关键帧以来没有 IMU 数据，无法构建预积分因子";
                return result;
            }

            integrate(imu);

            const int prevKfId = windowKeyframes_.back();
            const ::gtsam::NavState propState = seedKeyframe(keyframe.id, prevKfId);
            const PreOptGate gate = gatePreOptimization(keyframe.id, keyframe.observations, propState.pose());

            if (currentEstimate_.exists(X(prevKfId)) && !previousObservations_.empty())
            {
                triangulateNewLandmarks(prevKfId, previousObservations_, keyframe.observations,
                                        currentEstimate_.at<::gtsam::Pose3>(X(prevKfId)), propState.pose());
            }
            else
            {
                mylog::warn("[gtsam后端] 上一关键帧 #{} 的观测或位姿不可用，跳过本次三角化", prevKfId);
            }

            const std::size_t visualFactorCount = addVisualFactors(keyframe.id, keyframe.observations, gate);
            const ::gtsam::FixedLagSmoother::KeyTimestampMap timestamps =
                buildTimestamps(keyframe.id, keyframe.timestamp, keyframe.observations);

            const SolveOutcome solve = solveKeyframe(keyframe.id, keyframe.timestamp, timestamps);
            if (!solve.ok)
            {
                result.status = solve.recovered ? domain::StepStatus::RECOVERED : domain::StepStatus::FAILED;
                result.recovery =
                    solve.recovered ? domain::RecoveryAction::NONE : domain::RecoveryAction::RESET_PIPELINE;
                result.health = lastHealth_;
                result.detail = solve.recovered ? "求解失败，已以锚点局部恢复（坐标系连续）" : "求解失败且无法局部恢复";
                return result;
            }

            // 求解已成功，但 Values::at 仍会抛：平滑器同一次 update 里把刚插入的
            // 关键帧挤出了固定滞后窗口时就是如此（时间戳异常、窗口过短）。
            // 「实现不向外抛异常」是契约，因此这一段自己收口，而不是指望
            // 上层的兜底——上层兜底是最后一道机制，不是唯一的机制。
            ::gtsam::Pose3 optPose;
            ::gtsam::Vector3 optVelocity;
            ::gtsam::imuBias::ConstantBias optBias;
            try
            {
                optPose = currentEstimate_.at<::gtsam::Pose3>(X(keyframe.id));
                optVelocity = currentEstimate_.at<::gtsam::Vector3>(V(keyframe.id));
                optBias = currentEstimate_.at<::gtsam::imuBias::ConstantBias>(B(keyframe.id));
            }
            catch (const std::exception &error)
            {
                mylog::error("[gtsam后端] KF #{} 求解后无法取回状态 ({}), 判为不可恢复", keyframe.id, error.what());
                result.status = domain::StepStatus::FAILED;
                result.recovery = domain::RecoveryAction::RESET_PIPELINE;
                result.detail = std::string("求解后状态缺失: ") + error.what();
                return result;
            }

            latestNavState_ = ::gtsam::NavState(optPose, optVelocity);
            latestTimestamp_ = keyframe.timestamp;

            mylog::info("[gtsam后端优化] KF #{}: 增量优化耗时={:.2f}ms, 活跃变量数={}", keyframe.id, solve.elapsedMs,
                        currentEstimate_.size());

            try
            {
                markOutliersPostOptimization(keyframe.id, keyframe.observations, optPose);
                applyBiasFeedback(keyframe.id, optBias);
                marginalize(keyframe.id, solve.smootherResult);
                updateHealth(visualFactorCount);
            }
            catch (const std::exception &error)
            {
                // 后处理（门控、零偏反馈、边缘化清理）都读 Values；任何一条失败
                // 都意味着内部登记与估计不再一致，交给上层重建。
                mylog::error("[gtsam后端] KF #{} 后处理失败 ({}), 需要重建", keyframe.id, error.what());
                result.status = domain::StepStatus::FAILED;
                result.recovery = domain::RecoveryAction::RESET_PIPELINE;
                result.detail = std::string("后处理失败: ") + error.what();
                return result;
            }

            previousObservations_.assign(keyframe.observations.begin(), keyframe.observations.end());

            result.status = domain::StepStatus::OK;
            // 恢复语义由估计器表达，而不是由编排层猜：
            //   - 惯性失稳（零偏发散、速度跳变）确实需要重建坐标系；
            //   - 视觉中断（变暗/遮挡）重建毫无作用，只会让坐标反复归零，
            //     因此显式给出 NONE，上层不得据此重置。
            result.recovery = (!lastHealth_.isHealthy && lastHealth_.degradation == domain::Degradation::INERTIAL)
                                  ? domain::RecoveryAction::RESET_PIPELINE
                                  : domain::RecoveryAction::NONE;
            result.state = latestState();
            result.health = lastHealth_;
            return result;
        }

        [[nodiscard]] domain::KinematicState latestState() const override
        {
            return fromGtsam(latestTimestamp_, latestNavState_, currentBias_);
        }

        [[nodiscard]] domain::HealthReport health() const override { return lastHealth_; }

        [[nodiscard]] domain::EstimateSnapshot snapshot() const override
        {
            domain::EstimateSnapshot snapshot;
            if (!currentEstimate_.empty())
            {
                for (const auto &[key, pose] : currentEstimate_.extract<::gtsam::Pose3>())
                {
                    const ::gtsam::Symbol symbol(key);
                    if (symbol.chr() == 'x')
                    {
                        snapshot.keyframes.push_back(
                            {static_cast<domain::KeyframeId>(symbol.index()), latestTimestamp_, fromGtsam(pose)});
                    }
                }
                for (const auto &[key, point] : currentEstimate_.extract<::gtsam::Point3>())
                {
                    const ::gtsam::Symbol symbol(key);
                    if (symbol.chr() == 'l')
                    {
                        snapshot.landmarks.push_back(
                            {static_cast<domain::LandmarkId>(symbol.index()), fromGtsam(point)});
                    }
                }
            }
            else
            {
                for (const auto &[id, point] : triangulatedLandmarks_)
                {
                    snapshot.landmarks.push_back({id, fromGtsam(point)});
                }
            }
            return snapshot;
        }

        [[nodiscard]] domain::EstimatorDiagnostics diagnostics() const override
        {
            return {zuptCount_, recoveredFrames_, static_cast<std::size_t>(triangulatedLandmarks_.size())};
        }

    private:
        /// 预优化门控产出
        struct PreOptGate
        {
            std::unordered_set<domain::FeatureId> gatedOut;
            std::unordered_set<domain::FeatureId> geometricallyInvalid;
            int outlierCount{0};
        };

        /// 求解阶段产出
        struct SolveOutcome
        {
            bool ok{false};
            bool recovered{false}; ///< 已局部恢复（本帧不产出位姿，坐标系连续）
            double elapsedMs{0.0};
            ::gtsam::FixedLagSmoother::Result smootherResult;
        };

        void createSmoother()
        {
            smoother_ =
                std::make_unique<::gtsam::IncrementalFixedLagSmoother>(config_.estimator.slidingWindowLag, isamParams_);
        }

        void resetInternal()
        {
            createSmoother();
            windowKeyframes_.clear();
            triangulatedLandmarks_.clear();
            outlierLandmarks_.clear();
            outlierStreak_.clear();
            landmarkObsCount_.clear();
            currentEstimate_.clear();
            newFactors_.resize(0);
            newValues_.clear();
            previousObservations_.clear();
            recentImu_.clear();
            healthMonitor_.reset();
            recoveredFrames_ = 0;
            zuptCount_ = 0;
        }

        /// 把关键帧区间批量送进预积分器，并维护 ZUPT 的近期样本窗口
        void integrate(std::span<const domain::ImuSlice> imu)
        {
            pim_ = std::make_unique<::gtsam::PreintegratedCombinedMeasurements>(imuParams_, toGtsam(currentBias_));
            for (const domain::ImuSlice &slice : imu)
            {
                pim_->integrateMeasurement(toGtsam(slice.sample.accel), toGtsam(slice.sample.gyro), slice.dt);
                recentImu_.push_back(slice.sample);
            }
            while (!recentImu_.empty() &&
                   imu.back().sample.timestamp - recentImu_.front().timestamp > config_.estimator.zuptWindowSeconds)
            {
                recentImu_.pop_front();
            }
        }

        /// 用预积分外推当前关键帧初值，并插入 CombinedImuFactor
        ::gtsam::NavState seedKeyframe(domain::KeyframeId currentKfId, domain::KeyframeId prevKfId)
        {
            const ::gtsam::NavState propState = pim_->predict(latestNavState_, toGtsam(currentBias_));
            newValues_.insert(X(currentKfId), propState.pose());
            newValues_.insert(V(currentKfId), propState.velocity());
            newValues_.insert(B(currentKfId), toGtsam(currentBias_));

            newFactors_.emplace_shared<::gtsam::CombinedImuFactor>(X(prevKfId), V(prevKfId), X(currentKfId),
                                                                   V(currentKfId), B(prevKfId), B(currentKfId), *pim_);
            mylog::debug("[IMU预积分] 插入 CombinedImuFactor: KF #{} -> #{}, 积分时长={:.3f}s", prevKfId, currentKfId,
                         pim_->deltaTij());
            return propState;
        }

        PreOptGate gatePreOptimization(domain::KeyframeId currentKfId,
                                       std::span<const domain::Observation> observations,
                                       const ::gtsam::Pose3 &predBodyPose)
        {
            PreOptGate gate;
            const ::gtsam::Pose3 predCamPose = predBodyPose.compose(toGtsam(config_.camera.bodyPSensor));

            for (const domain::Observation &observation : observations)
            {
                if (!triangulatedLandmarks_.contains(observation.id) || outlierLandmarks_.contains(observation.id) ||
                    !currentEstimate_.exists(L(observation.id)))
                {
                    continue;
                }

                const ::gtsam::Point3 ptW = currentEstimate_.at<::gtsam::Point3>(L(observation.id));
                const ::gtsam::Point3 ptC = predCamPose.transformTo(ptW);

                double residualPx = 0.0;
                bool geometricallyValid = true;
                if (ptC.z() < config_.vision.minLandmarkDepth || ptC.z() > config_.vision.maxLandmarkDepth)
                {
                    geometricallyValid = false;
                }
                else if (const auto predictedUv = camera_->project(fromGtsam(ptC)); predictedUv.has_value())
                {
                    residualPx = (*predictedUv - observation.uv).norm();
                }
                else
                {
                    geometricallyValid = false;
                }

                if (!geometricallyValid)
                {
                    gate.geometricallyInvalid.insert(observation.id);
                    continue;
                }

                if (landmarkObsCount_[observation.id] < config_.vision.landmarkGateWarmupObs)
                {
                    continue;
                }

                if (residualPx > config_.vision.maxReprojectionErrorPreopt)
                {
                    gate.outlierCount++;
                    const double catastrophicPx = config_.vision.maxReprojectionErrorPreopt * 2.0;
                    const char *action = config_.vision.preoptTemporaryRejection ? "已拒绝本帧观测" : "仍送入优化";
                    if (residualPx > catastrophicPx)
                    {
                        mylog::warn("[外点剔除] 特征点 #{} 预优化残差 {:.2f} px 超灾难门限 {:.2f} px，{}",
                                    observation.id, residualPx, catastrophicPx, action);
                    }
                    else
                    {
                        mylog::debug("[外点门控] 特征点 #{} 预优化残差 {:.2f} px 超宽门限 {:.2f} px，{}",
                                     observation.id, residualPx, config_.vision.maxReprojectionErrorPreopt, action);
                    }
                    if (config_.vision.preoptTemporaryRejection)
                    {
                        gate.gatedOut.insert(observation.id);
                    }
                }
            }

            if (!gate.geometricallyInvalid.empty())
            {
                mylog::debug("[外点门控] KF #{}: 本帧 {} 个路标几何无效，已剔除其观测", currentKfId,
                             gate.geometricallyInvalid.size());
            }
            return gate;
        }

        void triangulateNewLandmarks(domain::KeyframeId prevKfId, std::span<const domain::Observation> previous,
                                     std::span<const domain::Observation> current, const ::gtsam::Pose3 &tWb1,
                                     const ::gtsam::Pose3 &tWb2)
        {
            const ::gtsam::Pose3 tWc1 = tWb1.compose(toGtsam(config_.camera.bodyPSensor));
            const ::gtsam::Pose3 tWc2 = tWb2.compose(toGtsam(config_.camera.bodyPSensor));

            std::unordered_map<domain::FeatureId, ::gtsam::Point2> observations1;
            for (const domain::Observation &observation : previous)
            {
                observations1[observation.id] = ::gtsam::Point2(observation.uv.x(), observation.uv.y());
            }

            int triangulatedCount = 0;
            for (const domain::Observation &observation : current)
            {
                if (triangulatedLandmarks_.contains(observation.id))
                {
                    continue;
                }
                const auto it = observations1.find(observation.id);
                if (it == observations1.end())
                {
                    continue;
                }

                const ::gtsam::Point2 p1 = it->second;
                const ::gtsam::Point2 p2(observation.uv.x(), observation.uv.y());
                if ((p1 - p2).norm() < config_.vision.minTriangulationParallaxPx)
                {
                    continue;
                }

                try
                {
                    const std::vector<::gtsam::Pose3> cameraPoses = {tWc1, tWc2};
                    const ::gtsam::Point2Vector measurements = {p1, p2};
                    const ::gtsam::Point3 ptW = ::gtsam::triangulatePoint3(cameraPoses, K_, measurements, 1e-4, true);

                    const ::gtsam::Point3 ptC1 = tWc1.transformTo(ptW);
                    const ::gtsam::Point3 ptC2 = tWc2.transformTo(ptW);

                    const auto uv1 = camera_->project(fromGtsam(ptC1));
                    const auto uv2 = camera_->project(fromGtsam(ptC2));
                    if (!uv1 || !uv2)
                    {
                        continue;
                    }
                    const double reprojErr1 = (*uv1 - domain::Vec2(p1.x(), p1.y())).norm();
                    const double reprojErr2 = (*uv2 - domain::Vec2(p2.x(), p2.y())).norm();
                    if (reprojErr1 > config_.vision.maxReprojectionErrorHard ||
                        reprojErr2 > config_.vision.maxReprojectionErrorHard)
                    {
                        mylog::warn("[三角化剔除] 特征点 #{} 三角化重投影不一致 (帧1 {:.2f} px, 帧2 {:.2f} px)，已丢弃",
                                    observation.id, reprojErr1, reprojErr2);
                        continue;
                    }

                    if (ptC1.z() > config_.vision.minLandmarkDepth && ptC1.z() < config_.vision.maxLandmarkDepth &&
                        ptC2.z() > config_.vision.minLandmarkDepth && ptC2.z() < config_.vision.maxLandmarkDepth)
                    {
                        triangulatedLandmarks_[observation.id] = ptW;
                        landmarkObsCount_[observation.id] = 2;
                        newValues_.insert(L(observation.id), ptW);

                        auto landmarkPriorNoise =
                            ::gtsam::noiseModel::Isotropic::Sigma(3, config_.estimator.landmarkPriorSigma);
                        newFactors_.addPrior(L(observation.id), ptW, landmarkPriorNoise);

                        newFactors_.emplace_shared<
                            ::gtsam::GenericProjectionFactor<::gtsam::Pose3, ::gtsam::Point3, ::gtsam::Cal3_S2>>(
                            p1, robustPixelNoise_, X(prevKfId), L(observation.id), K_,
                            toGtsam(config_.camera.bodyPSensor));
                        triangulatedCount++;
                    }
                }
                catch (const std::exception &)
                {
                    // 几何退化忽略
                }
            }

            if (triangulatedCount > 0)
            {
                mylog::info("[视觉前端] 跨帧三角化成功生成 {} 个新路标点 (当前总活跃路标: {})", triangulatedCount,
                            triangulatedLandmarks_.size());
            }
        }

        std::size_t addVisualFactors(domain::KeyframeId currentKfId, std::span<const domain::Observation> observations,
                                     const PreOptGate &gate)
        {
            std::size_t visualFactorCount = 0;
            for (const domain::Observation &observation : observations)
            {
                const bool landmarkInGraph =
                    currentEstimate_.exists(L(observation.id)) || newValues_.exists(L(observation.id));
                if (!triangulatedLandmarks_.contains(observation.id) || outlierLandmarks_.contains(observation.id) ||
                    gate.gatedOut.contains(observation.id) || gate.geometricallyInvalid.contains(observation.id) ||
                    !landmarkInGraph)
                {
                    continue;
                }

                // 观测自带噪声尺度时用它（例如学习型前端给出的不确定度），
                // 否则退回全局像素噪声。鲁棒核在两种情况下都保留。
                ::gtsam::SharedNoiseModel pixelNoise = robustPixelNoise_;
                if (observation.sigmaPx > 0.0)
                {
                    pixelNoise = ::gtsam::noiseModel::Robust::Create(
                        ::gtsam::noiseModel::mEstimator::Huber::Create(config_.vision.huberLossDelta),
                        ::gtsam::noiseModel::Isotropic::Sigma(2, observation.sigmaPx));
                }
                newFactors_.emplace_shared<
                    ::gtsam::GenericProjectionFactor<::gtsam::Pose3, ::gtsam::Point3, ::gtsam::Cal3_S2>>(
                    ::gtsam::Point2(observation.uv.x(), observation.uv.y()), pixelNoise, X(currentKfId),
                    L(observation.id), K_, toGtsam(config_.camera.bodyPSensor));
                landmarkObsCount_[observation.id]++;
                visualFactorCount++;
            }

            if (visualFactorCount < static_cast<std::size_t>(std::max(0, config_.vision.minVisualFactorsForHealth)) &&
                isStationary())
            {
                auto zuptNoise = ::gtsam::noiseModel::Isotropic::Sigma(3, config_.estimator.zuptVelocitySigma);
                const ::gtsam::Vector3 zeroVelocity = ::gtsam::Vector3::Zero();
                newFactors_.addPrior(V(currentKfId), zeroVelocity, zuptNoise);
                ++zuptCount_;
                if (zuptCount_ % 50 == 1)
                {
                    mylog::warn("[零速更新] KF #{}: 视觉仅剩 {} 个观测且 IMU 判定静止，已注入速度为零先验 (累计 {} 次)",
                                currentKfId, visualFactorCount, zuptCount_);
                }
            }

            mylog::debug("[因子图构建] KF #{}: 新增 IMU 因子 1 个, 视觉投影因子 {} 个", currentKfId, visualFactorCount);
            return visualFactorCount;
        }

        ::gtsam::FixedLagSmoother::KeyTimestampMap
        buildTimestamps(domain::KeyframeId currentKfId, double timestamp,
                        std::span<const domain::Observation> observations) const
        {
            ::gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
            timestamps[X(currentKfId)] = timestamp;
            timestamps[V(currentKfId)] = timestamp;
            timestamps[B(currentKfId)] = timestamp;
            for (const domain::Observation &observation : observations)
            {
                const bool landmarkInGraph =
                    currentEstimate_.exists(L(observation.id)) || newValues_.exists(L(observation.id));
                if (triangulatedLandmarks_.contains(observation.id) && !outlierLandmarks_.contains(observation.id) &&
                    landmarkInGraph)
                {
                    timestamps[L(observation.id)] = timestamp;
                }
            }
            return timestamps;
        }

        SolveOutcome solveKeyframe(domain::KeyframeId currentKfId, double timestamp,
                                   const ::gtsam::FixedLagSmoother::KeyTimestampMap &timestamps)
        {
            SolveOutcome outcome;
            const auto tStart = std::chrono::steady_clock::now();

            try
            {
                outcome.smootherResult = smoother_->update(newFactors_, newValues_, timestamps);
                for (int iter = 0; iter < config_.estimator.extraSmootherIterations; ++iter)
                {
                    const auto extra = smoother_->update();
                    outcome.smootherResult.keysOfDeletedNodes.insert(extra.keysOfDeletedNodes.begin(),
                                                                     extra.keysOfDeletedNodes.end());
                    outcome.smootherResult.expiredPendingKeys.insert(extra.expiredPendingKeys.begin(),
                                                                     extra.expiredPendingKeys.end());
                }
                currentEstimate_ = smoother_->calculateEstimate();
            }
            catch (const std::exception &error)
            {
                mylog::warn("[gtsam后端] KF #{} 平滑器求解失败 ({}), 尝试局部恢复", currentKfId, error.what());
                newFactors_.resize(0);
                newValues_.clear();

                if (!recoverSmoother(timestamp))
                {
                    mylog::error("[gtsam后端] KF #{} 局部恢复失败（窗口无可锚定关键帧）", currentKfId);
                    return outcome;
                }
                outcome.recovered = true;
                ++recoveredFrames_;
                return outcome;
            }

            outcome.elapsedMs =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tStart).count();
            newFactors_.resize(0);
            newValues_.clear();
            outcome.ok = true;
            return outcome;
        }

        bool recoverSmoother(double timestamp)
        {
            if (windowKeyframes_.empty())
            {
                return false;
            }

            const domain::KeyframeId anchor = windowKeyframes_.back();
            ::gtsam::Pose3 anchorPose = latestNavState_.pose();
            ::gtsam::Vector3 anchorVel = latestNavState_.velocity();
            if (currentEstimate_.exists(X(anchor)) && currentEstimate_.exists(V(anchor)))
            {
                anchorPose = currentEstimate_.at<::gtsam::Pose3>(X(anchor));
                anchorVel = currentEstimate_.at<::gtsam::Vector3>(V(anchor));
            }

            createSmoother();

            const double rp = config_.estimator.recoveryPoseSigma;
            const double rv = config_.estimator.recoveryVelSigma;
            const double rb = config_.estimator.recoveryBiasSigma;
            auto poseNoise = ::gtsam::noiseModel::Diagonal::Sigmas(
                (::gtsam::Vector(6) << rp, rp, rp, 2.0 * rp, 2.0 * rp, 2.0 * rp).finished());
            auto velNoise = ::gtsam::noiseModel::Isotropic::Sigma(3, rv);
            auto biasNoise = ::gtsam::noiseModel::Diagonal::Sigmas(
                (::gtsam::Vector(6) << rb, rb, rb, 0.5 * rb, 0.5 * rb, 0.5 * rb).finished());

            ::gtsam::NonlinearFactorGraph anchorFactors;
            ::gtsam::Values anchorValues;
            ::gtsam::FixedLagSmoother::KeyTimestampMap anchorTimestamps;

            anchorFactors.addPrior(X(anchor), anchorPose, poseNoise);
            anchorFactors.addPrior(V(anchor), anchorVel, velNoise);
            anchorFactors.addPrior(B(anchor), toGtsam(currentBias_), biasNoise);
            anchorValues.insert(X(anchor), anchorPose);
            anchorValues.insert(V(anchor), anchorVel);
            anchorValues.insert(B(anchor), toGtsam(currentBias_));
            anchorTimestamps[X(anchor)] = timestamp;
            anchorTimestamps[V(anchor)] = timestamp;
            anchorTimestamps[B(anchor)] = timestamp;

            try
            {
                smoother_->update(anchorFactors, anchorValues, anchorTimestamps);
                currentEstimate_ = smoother_->calculateEstimate();
            }
            catch (const std::exception &error)
            {
                mylog::error("[gtsam后端] 重建平滑器失败: {}", error.what());
                return false;
            }

            windowKeyframes_.clear();
            windowKeyframes_.push_back(anchor);
            triangulatedLandmarks_.clear();
            outlierLandmarks_.clear();
            outlierStreak_.clear();
            landmarkObsCount_.clear();
            // 刻意保留 previousObservations_：它属于锚点关键帧，而锚点仍在
            // 窗口里。清空它会让恢复后的下一个关键帧无处可比，三角化被整体
            // 跳过一帧——正好与「恢复后基线更长、更容易重建路标」相反。
            healthMonitor_.reset();

            mylog::info("[gtsam后端恢复] 平滑器已重建: 锚点 KF #{} P=[{:.3f}, {:.3f}, {:.3f}]，坐标系保持连续", anchor,
                        anchorPose.x(), anchorPose.y(), anchorPose.z());
            return true;
        }

        void markOutliersPostOptimization(domain::KeyframeId currentKfId,
                                          std::span<const domain::Observation> observations,
                                          const ::gtsam::Pose3 &optPose)
        {
            const ::gtsam::Pose3 optCamPose = optPose.compose(toGtsam(config_.camera.bodyPSensor));
            int postOptOutliers = 0;

            for (const domain::Observation &observation : observations)
            {
                if (!triangulatedLandmarks_.contains(observation.id) || outlierLandmarks_.contains(observation.id) ||
                    !currentEstimate_.exists(L(observation.id)) ||
                    landmarkObsCount_[observation.id] < config_.vision.landmarkGateWarmupObs)
                {
                    continue;
                }

                const ::gtsam::Point3 ptW = currentEstimate_.at<::gtsam::Point3>(L(observation.id));
                const ::gtsam::Point3 ptC = optCamPose.transformTo(ptW);
                double residualPx = std::numeric_limits<double>::infinity();
                if (ptC.z() >= config_.vision.minLandmarkDepth && ptC.z() <= config_.vision.maxLandmarkDepth)
                {
                    if (const auto optimizedUv = camera_->project(fromGtsam(ptC)); optimizedUv.has_value())
                    {
                        residualPx = (*optimizedUv - observation.uv).norm();
                    }
                }

                if (!config_.vision.postOptOutlierMarking)
                {
                    continue;
                }
                if (residualPx <= config_.vision.maxReprojectionError)
                {
                    outlierStreak_.erase(observation.id);
                    continue;
                }

                if (residualPx > config_.vision.maxReprojectionErrorHard)
                {
                    outlierLandmarks_.insert(observation.id);
                    outlierStreak_.erase(observation.id);
                    postOptOutliers++;
                    mylog::warn("[外点剔除] 特征点 #{} 优化后重投影残差 {:.2f} px 超硬门限 {:.2f} px，已标记为外点",
                                observation.id, residualPx, config_.vision.maxReprojectionErrorHard);
                    continue;
                }

                int &streak = outlierStreak_[observation.id];
                ++streak;
                mylog::debug("[外点门控] 特征点 #{} 优化后残差 {:.2f} px 超软门限 {:.2f} px (连续 {} 帧)",
                             observation.id, residualPx, config_.vision.maxReprojectionError, streak);
                if (streak >= kOutlierStreakThreshold)
                {
                    outlierLandmarks_.insert(observation.id);
                    outlierStreak_.erase(observation.id);
                    postOptOutliers++;
                    mylog::warn("[外点剔除] 特征点 #{} 连续 {} 帧超软门限 {:.2f} px，已标记为外点", observation.id,
                                kOutlierStreakThreshold, config_.vision.maxReprojectionError);
                }
            }

            if (postOptOutliers > 0)
            {
                mylog::warn("[外点处理] KF #{} 优化后共标记 {} 个外点观测", currentKfId, postOptOutliers);
            }
        }

        void applyBiasFeedback(domain::KeyframeId currentKfId, const ::gtsam::imuBias::ConstantBias &optBias)
        {
            currentBias_ = fromGtsam(optBias);
            pim_->resetIntegrationAndSetBias(toGtsam(currentBias_));

            mylog::info("[零偏更新反馈] KF #{}: 后验加速度偏置=[{:.4f}, {:.4f}, {:.4f}] m/s^2, 陀螺偏置=[{:.5f}, "
                        "{:.5f}, {:.5f}] rad/s",
                        currentKfId, currentBias_.accel.x(), currentBias_.accel.y(), currentBias_.accel.z(),
                        currentBias_.gyro.x(), currentBias_.gyro.y(), currentBias_.gyro.z());
        }

        void marginalize(domain::KeyframeId currentKfId, const ::gtsam::FixedLagSmoother::Result &smootherResult)
        {
            windowKeyframes_.push_back(currentKfId);
            ::gtsam::KeySet removedKeys = smootherResult.keysOfDeletedNodes;
            removedKeys.insert(smootherResult.expiredPendingKeys.begin(), smootherResult.expiredPendingKeys.end());
            if (!removedKeys.empty())
            {
                std::size_t removedKeyframes = 0;
                std::size_t removedLandmarks = 0;
                for (::gtsam::Key key : removedKeys)
                {
                    const ::gtsam::Symbol symbol(key);
                    if (symbol.chr() == 'l')
                    {
                        ++removedLandmarks;
                        const auto id = static_cast<domain::LandmarkId>(symbol.index());
                        triangulatedLandmarks_.erase(id);
                        outlierLandmarks_.erase(id);
                        outlierStreak_.erase(id);
                        landmarkObsCount_.erase(id);
                    }
                    else if (symbol.chr() == 'x')
                    {
                        ++removedKeyframes;
                        const auto id = static_cast<domain::KeyframeId>(symbol.index());
                        windowKeyframes_.erase(std::remove(windowKeyframes_.begin(), windowKeyframes_.end(), id),
                                               windowKeyframes_.end());
                    }
                }
                mylog::info("[滑动窗口边缘化] 边缘化退出变量: 关键帧 {} 个, 路标 {} 个 (活跃关键帧 {}, 活跃变量 {})",
                            removedKeyframes, removedLandmarks, windowKeyframes_.size(), currentEstimate_.size());
            }

            if (windowKeyframes_.size() > config_.estimator.maxSlidingWindowSize)
            {
                mylog::debug("[滑动窗口] 活跃关键帧 {} 个，超过 maxSlidingWindowSize={}（实际窗口由 "
                             "slidingWindowLag={:.2f}s 决定）",
                             windowKeyframes_.size(), config_.estimator.maxSlidingWindowSize,
                             config_.estimator.slidingWindowLag);
            }
        }

        void updateHealth(std::size_t visualFactorCount)
        {
            lastHealth_ = healthMonitor_.inspect(latestState(), visualFactorCount);
            if (!lastHealth_.isHealthy)
            {
                mylog::warn("[健康报警] {}", lastHealth_.alertMessage);
            }
        }

        [[nodiscard]] bool isStationary() const
        {
            if (recentImu_.size() < 10)
            {
                return false;
            }

            domain::Vec3 meanAcc = domain::Vec3::Zero();
            domain::Vec3 meanGyro = domain::Vec3::Zero();
            for (const auto &sample : recentImu_)
            {
                meanAcc += sample.accel;
                meanGyro += sample.gyro;
            }
            const double invCount = 1.0 / static_cast<double>(recentImu_.size());
            meanAcc *= invCount;
            meanGyro *= invCount;

            domain::Vec3 varAcc = domain::Vec3::Zero();
            domain::Vec3 varGyro = domain::Vec3::Zero();
            for (const auto &sample : recentImu_)
            {
                varAcc += (sample.accel - meanAcc).cwiseAbs2();
                varGyro += (sample.gyro - meanGyro).cwiseAbs2();
            }
            varAcc *= invCount;
            varGyro *= invCount;

            const double accStd = std::sqrt(varAcc.sum() / 3.0);
            const double gyroStd = std::sqrt(varGyro.sum() / 3.0);

            const double rateScale = std::sqrt(std::max(1.0, config_.imu.rateHz));
            const double accThreshold = std::max(0.3, 5.0 * config_.imu.accelNoiseSigma * rateScale);
            const double gyroThreshold = std::max(0.05, 5.0 * config_.imu.gyroNoiseSigma * rateScale);
            if (accStd > accThreshold || gyroStd > gyroThreshold)
            {
                return false;
            }

            const double accNormTolerance = std::max(0.2, 3.0 * config_.imu.accelNoiseSigma * rateScale);
            return std::abs(meanAcc.norm() - config_.imu.gravityMagnitude) < accNormTolerance;
        }

        domain::PipelineConfig config_;
        std::shared_ptr<const domain::ICameraModel> camera_;
        const domain::IPinholeCalibration *pinhole_{nullptr};
        bool valid_{true};

        std::shared_ptr<::gtsam::Cal3_S2> K_;
        std::shared_ptr<::gtsam::PreintegratedCombinedMeasurements::Params> imuParams_;
        std::unique_ptr<::gtsam::PreintegratedCombinedMeasurements> pim_;
        ::gtsam::SharedNoiseModel robustPixelNoise_;

        ::gtsam::ISAM2Params isamParams_;
        std::unique_ptr<::gtsam::IncrementalFixedLagSmoother> smoother_;
        ::gtsam::NonlinearFactorGraph newFactors_;
        ::gtsam::Values newValues_;
        ::gtsam::Values currentEstimate_;

        ::gtsam::NavState latestNavState_;
        domain::Bias currentBias_{};
        double latestTimestamp_{0.0};

        std::deque<domain::KeyframeId> windowKeyframes_;
        std::vector<domain::Observation> previousObservations_;
        std::unordered_map<domain::LandmarkId, ::gtsam::Point3> triangulatedLandmarks_;
        std::unordered_set<domain::LandmarkId> outlierLandmarks_;
        std::unordered_map<domain::LandmarkId, int> outlierStreak_;
        std::unordered_map<domain::LandmarkId, int> landmarkObsCount_;

        domain::HealthMonitor healthMonitor_;
        domain::HealthReport lastHealth_{};

        static constexpr int kOutlierStreakThreshold = 3;

        std::size_t recoveredFrames_{0};
        std::deque<domain::ImuSample> recentImu_;
        std::size_t zuptCount_{0};
    };

} // namespace myslam::adapters
