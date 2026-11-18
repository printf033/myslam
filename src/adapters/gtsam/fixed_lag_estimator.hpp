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

#include "adapters/gtsam/inverse_depth_factor.hpp"
#include "adapters/gtsam/types.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/health_monitor.hpp"
#include "domain/ports.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
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
#include <numbers>
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

            useInverseDepth_ = config_.estimator.landmarkRepresentation == "anchored_inverse_depth";
            if (useInverseDepth_)
            {
                mylog::info("[gtsam后端] 路标参数化: 锚定逆深度 (u, v, rho)，锚点为三角化时刻的相机位姿");
            }

            createSmoother();
            pim_ = std::make_unique<::gtsam::PreintegratedCombinedMeasurements>(imuParams_, toGtsam(currentBias_));
        }

        [[nodiscard]] std::string_view name() const noexcept override { return "gtsam_fixed_lag"; }

        [[nodiscard]] bool valid() const noexcept { return valid_; }

        bool initialize(const domain::Keyframe &firstKeyframe, const domain::KinematicState &initialState,
                        const domain::PoseCovariance &initialCovariance = {}) override
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
            ::gtsam::SharedNoiseModel poseNoise = ::gtsam::noiseModel::Diagonal::Sigmas(
                (::gtsam::Vector(6) << ip, ip, ip, 2.0 * ip, 2.0 * ip, 2.0 * ip).finished());

            // 初始化器能给出协方差时用它：固定 sigma 对「静止起步」与
            // 「运动起步」用同一套数字，必然一头过宽、另一头过窄，而先验
            // 过窄会把错误的初始姿态固化进整段轨迹。
            //
            // 逐轴下限必不可少：航向与位置在部分场景下不可观，协方差
            // 相应极大（或数值上退化），直接当信息阵用会让首帧因子
            // 病态。上界则防止一次异常估计把先验压到零。
            if (config_.estimator.initialCovarianceFromInit && initialCovariance.has_value())
            {
                domain::Mat6 covariance = *initialCovariance;
                covariance = 0.5 * (covariance + covariance.transpose());
                constexpr double kMinSigma = 1e-4;
                constexpr double kMaxSigma = 10.0;
                for (int i = 0; i < 6; ++i)
                {
                    const double variance = std::clamp(covariance(i, i), kMinSigma * kMinSigma, kMaxSigma * kMaxSigma);
                    covariance(i, i) = variance;
                }
                poseNoise = ::gtsam::noiseModel::Gaussian::Covariance(covariance);
                mylog::info("[gtsam后端初始化] 采用初始化器协方差: sigma_rot=[{:.4f}, {:.4f}, {:.4f}] rad, "
                            "sigma_pos=[{:.4f}, {:.4f}, {:.4f}] m",
                            std::sqrt(covariance(0, 0)), std::sqrt(covariance(1, 1)), std::sqrt(covariance(2, 2)),
                            std::sqrt(covariance(3, 3)), std::sqrt(covariance(4, 4)), std::sqrt(covariance(5, 5)));
            }

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
            keyframeTimestamps_[firstKeyframeId] = firstKeyframe.timestamp;
            // 首个关键帧的观测就是下一次三角化的基线；不保存它会让 KF #1
            // 无法建立任何路标，起步阶段的精度因此明显变差。
            previousObservations_.assign(firstKeyframe.observations.begin(), firstKeyframe.observations.end());
            lastHealth_ = {};

            mylog::info("[gtsam后端初始化] 注入初始帧先验: KF #{}, P=[{:.3f}, {:.3f}, {:.3f}], V=[{:.3f}, {:.3f}, "
                        "{:.3f}]",
                        firstKeyframeId, initialState.pose.position.x(), initialState.pose.position.y(),
                        initialState.pose.position.z(), initialState.velocity.x(), initialState.velocity.y(),
                        initialState.velocity.z());
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

            const std::size_t visualFactorCount = addVisualFactors(keyframe.id, keyframe.observations, gate, propState);
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

            // 协方差必须在本帧仍在窗口内时读取：边缘化之后该变量就退出了
            // Values，边际协方差也无从计算。
            result.poseCovariance = queryPoseCovariance(keyframe.id);
            if (result.poseCovariance.has_value())
            {
                // 位姿标准差比 6x6 矩阵更适合做一眼可读的健康指示：
                // 协方差没被读出来的时候，下游的 NEES 与 χ² 都是空谈。
                // 花括号不能省：`eig(positionCov)` 会被解析成函数声明（most vexing parse）。
                const domain::Mat3 positionCov = result.poseCovariance->block<3, 3>(3, 3);
                const Eigen::SelfAdjointEigenSolver<domain::Mat3> eig{positionCov};
                const double sigmaMax =
                    eig.eigenvalues().maxCoeff() > 0.0 ? std::sqrt(eig.eigenvalues().maxCoeff()) : 0.0;
                mylog::debug("[协方差] KF #{}: 平移最大边际标准差={:.4f} m", keyframe.id, sigmaMax);
            }
            else
            {
                mylog::debug("[协方差] KF #{}: 本轮未取得边际协方差（publishCovariance={}）", keyframe.id,
                             config_.estimator.publishCovariance);
            }

            try
            {
                markOutliersPostOptimization(keyframe.id, keyframe.observations, optPose, result.poseCovariance);
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
            keyframeTimestamps_[keyframe.id] = keyframe.timestamp;

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
                        const auto id = static_cast<domain::KeyframeId>(symbol.index());
                        // 时间戳从本类维护的表里取：Values 不含时间信息，用最新
                        // 时间戳代替会让导出的轨迹所有点时间相同，评估器的时间
                        // 关联因此全部落空。
                        const auto stampIt = keyframeTimestamps_.find(id);
                        snapshot.keyframes.push_back(
                            {id, stampIt != keyframeTimestamps_.end() ? stampIt->second : latestTimestamp_,
                             fromGtsam(pose), queryPoseCovariance(id)});
                    }
                }

                if (useInverseDepth_)
                {
                    // 逆深度模式下路标是 Vector3(u, v, rho)，必须经锚点重建；
                    // 注意 V(key) 也是 Vector3，靠 symbol 字符区分。
                    for (const auto &[key, param] : currentEstimate_.extract<::gtsam::Vector3>())
                    {
                        const ::gtsam::Symbol symbol(key);
                        if (symbol.chr() != 'l')
                        {
                            continue;
                        }
                        const auto id = static_cast<domain::LandmarkId>(symbol.index());
                        ::gtsam::Point3 ptW;
                        if (landmarkWorldPosition(id, ptW))
                        {
                            snapshot.landmarks.push_back({id, fromGtsam(ptW)});
                        }
                    }
                }
                else
                {
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

        /**
         * @brief 读取某个关键帧的 6x6 位姿边际协方差
         *
         * iSAM2 的 @c marginalCovariance 在变量已退出窗口、或内部分解失败时
         * 会抛异常。协方差是诊断信息，不应因它把一帧正常的估计判成失败，
         * 所以这里全部收口成 @c std::nullopt。
         */
        [[nodiscard]] domain::PoseCovariance queryPoseCovariance(domain::KeyframeId id) const
        {
            if (!config_.estimator.publishCovariance || !currentEstimate_.exists(X(id)))
            {
                return std::nullopt;
            }
            try
            {
                const Eigen::MatrixXd covariance = smoother_->marginalCovariance(X(id));
                if (covariance.rows() != 6 || covariance.cols() != 6 || !covariance.allFinite())
                {
                    return std::nullopt;
                }
                // 对称化：iSAM2 返回的块在数值上可能带微小的非对称，
                // 下游做特征值分解或求逆时会被当成病态。
                domain::Mat6 sym = 0.5 * (covariance + covariance.transpose());
                return sym;
            }
            catch (const std::exception &error)
            {
                mylog::debug("[协方差] KF #{} 边际协方差不可用: {}", id, error.what());
                return std::nullopt;
            }
        }

        void resetInternal()
        {
            createSmoother();
            windowKeyframes_.clear();
            triangulatedLandmarks_.clear();
            landmarkAnchors_.clear();
            outlierLandmarks_.clear();
            outlierStreak_.clear();
            landmarkObsCount_.clear();
            keyframeTimestamps_.clear();
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

                ::gtsam::Point3 ptW;
                if (!landmarkWorldPosition(observation.id, ptW))
                {
                    continue;
                }
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

        /**
         * @brief 把一个路标还原成世界系 3D 点
         *
         * 门控、导出、健康诊断都只认世界点，不应该各自知道路标是哪种参数化。
         * 逆深度模式下走「参数 + 锚点」重建，与因子内部的公式完全一致。
         *
         * @return false 表示该路标不在当前估计里（已边缘化或从未入图）
         */
        [[nodiscard]] bool landmarkWorldPosition(domain::LandmarkId id, ::gtsam::Point3 &out) const
        {
            if (!currentEstimate_.exists(L(id)))
            {
                return false;
            }
            if (!useInverseDepth_)
            {
                out = currentEstimate_.at<::gtsam::Point3>(L(id));
                return true;
            }
            const auto anchorIt = landmarkAnchors_.find(id);
            if (anchorIt == landmarkAnchors_.end())
            {
                return false;
            }
            const ::gtsam::Vector3 param = currentEstimate_.at<::gtsam::Vector3>(L(id));
            return landmarkWorldPositionFromParam(id, Eigen::Vector3d(param(0), param(1), param(2)), out);
        }

        /**
         * @brief 从原始参数块重建世界系位置（两种参数化统一）
         *
         * 单独拆出来是为了对参数块求数值雅可比：协方差活在参数空间
         * （全局点或锚定逆深度），而 χ² 需要的是世界系位置的协方差，
         * 两者之间差一个非线性映射。
         */
        [[nodiscard]] bool landmarkWorldPositionFromParam(domain::LandmarkId id, const Eigen::Vector3d &param,
                                                          ::gtsam::Point3 &out) const
        {
            if (!useInverseDepth_)
            {
                out = ::gtsam::Point3(param(0), param(1), param(2));
                return true;
            }
            const auto anchorIt = landmarkAnchors_.find(id);
            if (anchorIt == landmarkAnchors_.end())
            {
                return false;
            }
            const double rho = std::max(param(2), InverseDepthProjectionFactor::kMinInverseDepth);
            const ::gtsam::Point3 inAnchor(param(0) / rho, param(1) / rho, 1.0 / rho);
            out = anchorIt->second.transformFrom(inAnchor);
            return true;
        }

        /**
         * @brief 路标世界系位置的边际协方差（3x3）
         *
         * 先把参数空间的 3x3 协方差经非线性映射推到世界系。对两种参数化
         * 都用同一条数值链式：@c P_world = J·P_param·Jᵀ。
         *
         * 拿不到时返回 @c nullopt，调用方退回「只用位姿协方差」的旧行为：
         * 门控宁可宽松，也不应该因为一次分解失败就静默失效。
         */
        [[nodiscard]] std::optional<domain::Mat3> landmarkWorldCovariance(domain::LandmarkId id) const
        {
            try
            {
                const Eigen::MatrixXd cov = smoother_->marginalCovariance(L(id));
                if (cov.rows() != 3 || cov.cols() != 3 || !cov.allFinite())
                {
                    return std::nullopt;
                }

                Eigen::Vector3d param;
                if (useInverseDepth_)
                {
                    const ::gtsam::Vector3 v = currentEstimate_.at<::gtsam::Vector3>(L(id));
                    param = Eigen::Vector3d(v(0), v(1), v(2));
                }
                else
                {
                    const ::gtsam::Point3 p = currentEstimate_.at<::gtsam::Point3>(L(id));
                    param = Eigen::Vector3d(p.x(), p.y(), p.z());
                }

                constexpr double kStep = 1e-6;
                Eigen::Matrix3d J;
                for (int i = 0; i < 3; ++i)
                {
                    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
                    delta(i) = kStep;
                    ::gtsam::Point3 plus;
                    ::gtsam::Point3 minus;
                    if (!landmarkWorldPositionFromParam(id, param + delta, plus) ||
                        !landmarkWorldPositionFromParam(id, param - delta, minus))
                    {
                        return std::nullopt;
                    }
                    J.col(i) = (plus - minus) / (2.0 * kStep);
                }

                domain::Mat3 world = J * cov * J.transpose();
                world = 0.5 * (world + world.transpose());
                if (!world.allFinite())
                {
                    return std::nullopt;
                }
                return world;
            }
            catch (const std::exception &)
            {
                return std::nullopt;
            }
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
            int triangulationPostponed = 0;
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

                // 基线张角门控：纯旋转时两射线平行，深度不可观。
                // （深度/基线比在三角化之后再判——那时才有 Z。）
                if (config_.vision.minTriangulationBaselineDeg > 0.0)
                {
                    const double baselineDeg = baselineAngleDeg(tWc1, tWc2, p1, p2);
                    if (std::isfinite(baselineDeg) && baselineDeg < config_.vision.minTriangulationBaselineDeg)
                    {
                        ++triangulationPostponed;
                        mylog::debug("[三角化推迟] 特征点 #{}: 基线角 {:.2f}° < {:.2f}°，深度不可观", observation.id,
                                     baselineDeg, config_.vision.minTriangulationBaselineDeg);
                        continue;
                    }
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
                        // 深度/基线比：深度远大于基线时，三角化对观测噪声
                        // 极敏感，解出的 Z 可以落在合理区间却完全错误，而
                        // 重投影校验拦不住它（残差已被求解吸收）。
                        // 只推迟，不标外点。
                        const double baseline = (tWc2.translation() - tWc1.translation()).norm();
                        if (config_.vision.maxDepthBaselineRatio > 0.0 && baseline > 1e-9)
                        {
                            const double ratio = std::max(ptC1.z(), ptC2.z()) / baseline;
                            if (ratio > config_.vision.maxDepthBaselineRatio)
                            {
                                ++triangulationPostponed;
                                mylog::debug("[三角化推迟] 特征点 #{}: 深度/基线 {:.1f} > {:.1f}（基线 {:.3f} "
                                             "m），保留至更长基线",
                                             observation.id, ratio, config_.vision.maxDepthBaselineRatio, baseline);
                                continue;
                            }
                        }
                        // 两种参数化都需要这个标记：「已三角化且已入图」。逆深度
                        // 模式下它的坐标仅作缓存，实时值一律通过
                        // @c landmarkWorldPosition 从参数与锚点重建。
                        triangulatedLandmarks_[observation.id] = ptW;
                        landmarkObsCount_[observation.id] = 2;

                        if (useInverseDepth_)
                        {
                            const ::gtsam::Point3 ptAnchor = tWc1.transformTo(ptW);
                            const double depth = ptAnchor.z();
                            const double rho = 1.0 / depth;
                            const double u = ptAnchor.x() / depth;
                            const double v = ptAnchor.y() / depth;
                            const ::gtsam::Vector3 param(u, v, rho);

                            landmarkAnchors_[observation.id] = tWc1;
                            newValues_.insert(L(observation.id), param);

                            // (u, v) 是锚点帧的观测方向，本质上已经由那次观测确定，
                            // 因此紧约束；rho 才是真正要被多帧观测解的量，保持宽松。
                            // 各向同性会把 rho 一起压死，所以必须对角。
                            //
                            // (u, v) 的 sigma 取「一个像素在归一化平面的尺寸」：
                            // 1/focal。实测把这个值放大到 1.0 会让横向不确定性
                            // 被高估，NEES 从 3 附近掉到 0.46（过度保守）。
                            const double angularSigma = std::max(1e-6, config_.estimator.inverseDepthAngularSigma);
                            auto inverseDepthPrior = ::gtsam::noiseModel::Diagonal::Sigmas(
                                (::gtsam::Vector(3) << angularSigma, angularSigma, 1.0).finished());
                            newFactors_.addPrior(L(observation.id), param, inverseDepthPrior);

                            newFactors_.emplace_shared<InverseDepthProjectionFactor>(
                                X(prevKfId), L(observation.id), p1, robustPixelNoise_,
                                toGtsam(config_.camera.bodyPSensor), tWc1, K_);
                        }
                        else
                        {
                            newValues_.insert(L(observation.id), ptW);

                            auto landmarkPriorNoise =
                                ::gtsam::noiseModel::Isotropic::Sigma(3, config_.estimator.landmarkPriorSigma);
                            newFactors_.addPrior(L(observation.id), ptW, landmarkPriorNoise);

                            newFactors_.emplace_shared<
                                ::gtsam::GenericProjectionFactor<::gtsam::Pose3, ::gtsam::Point3, ::gtsam::Cal3_S2>>(
                                p1, robustPixelNoise_, X(prevKfId), L(observation.id), K_,
                                toGtsam(config_.camera.bodyPSensor));
                        }
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
            if (triangulationPostponed > 0)
            {
                mylog::debug("[视觉前端] {} 个点因几何退化被推迟三角化（非外点，保留待更长基线）",
                             triangulationPostponed);
            }
        }

        /**
         * @brief 两视图观测射线的世界系张角 (deg)
         *
         * 纯旋转时两射线平行，深度不可观。这个量只依赖旋转与像素，
         * 与焦距无关。
         */
        [[nodiscard]] double baselineAngleDeg(const ::gtsam::Pose3 &tWc1, const ::gtsam::Pose3 &tWc2,
                                              const ::gtsam::Point2 &p1, const ::gtsam::Point2 &p2) const
        {
            if (!K_)
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
            const Eigen::Matrix3d K = K_->K();
            const auto worldRay = [&K](const ::gtsam::Pose3 &tWc, const ::gtsam::Point2 &uv)
            {
                const Eigen::Vector3d xn((uv.x() - K(0, 2)) / K(0, 0), (uv.y() - K(1, 2)) / K(1, 1), 1.0);
                return (tWc.rotation().matrix().transpose() * xn).normalized();
            };
            const Eigen::Vector3d r1 = worldRay(tWc1, p1);
            const Eigen::Vector3d r2 = worldRay(tWc2, p2);
            return std::acos(std::clamp(r1.dot(r2), -1.0, 1.0)) * 180.0 / std::numbers::pi;
        }

        std::size_t addVisualFactors(domain::KeyframeId currentKfId, std::span<const domain::Observation> observations,
                                     const PreOptGate &gate, const ::gtsam::NavState &propState)
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

                if (useInverseDepth_)
                {
                    const auto anchorIt = landmarkAnchors_.find(observation.id);
                    if (anchorIt == landmarkAnchors_.end())
                    {
                        // 路标在逆深度模式下必须有锚点；没有说明登记已损坏，
                        // 跳过这条观测而不是拿一个无效锚点建因子。
                        mylog::warn("[gtsam后端] 路标 #{} 缺少逆深度锚点，跳过本帧观测", observation.id);
                        continue;
                    }
                    newFactors_.emplace_shared<InverseDepthProjectionFactor>(
                        X(currentKfId), L(observation.id), ::gtsam::Point2(observation.uv.x(), observation.uv.y()),
                        pixelNoise, toGtsam(config_.camera.bodyPSensor), anchorIt->second, K_);
                }
                else
                {
                    newFactors_.emplace_shared<
                        ::gtsam::GenericProjectionFactor<::gtsam::Pose3, ::gtsam::Point3, ::gtsam::Cal3_S2>>(
                        ::gtsam::Point2(observation.uv.x(), observation.uv.y()), pixelNoise, X(currentKfId),
                        L(observation.id), K_, toGtsam(config_.camera.bodyPSensor));
                }
                landmarkObsCount_[observation.id]++;
                visualFactorCount++;
            }

            if (visualFactorCount < static_cast<std::size_t>(std::max(0, config_.vision.minVisualFactorsForHealth)) &&
                zuptAccepted(observations, propState.velocity()))
            {
                // 零速先验必须刻意保守：判错一次就把整段运动拉停，
                // 比漏检一次静止的代价大得多。
                const double sigma = config_.estimator.zuptVelocitySigma * config_.estimator.zuptNoiseMultiplier;
                auto zuptNoise = ::gtsam::noiseModel::Isotropic::Sigma(3, sigma);
                const ::gtsam::Vector3 zeroVelocity = ::gtsam::Vector3::Zero();
                newFactors_.addPrior(V(currentKfId), zeroVelocity, zuptNoise);
                ++zuptCount_;
                if (zuptCount_ % 50 == 1)
                {
                    mylog::warn("[零速更新] KF #{}: 视觉仅剩 {} 个观测且四路判据均通过静止，已注入速度为零先验 "
                                "(sigma={:.3f}, 累计 {} 次)",
                                currentKfId, visualFactorCount, sigma, zuptCount_);
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
            // 重建后窗口里只剩锚点，其余关键帧的时间戳已经无法对应到任何变量。
            // 锚点的那一条要留着，否则恢复后导出的轨迹缺一根。
            {
                const auto stampIt = keyframeTimestamps_.find(anchor);
                const double anchorStamp = stampIt != keyframeTimestamps_.end() ? stampIt->second : timestamp;
                keyframeTimestamps_.clear();
                keyframeTimestamps_[anchor] = anchorStamp;
            }
            triangulatedLandmarks_.clear();
            landmarkAnchors_.clear();
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
                                          const ::gtsam::Pose3 &optPose, const domain::PoseCovariance &poseCovariance)
        {
            const ::gtsam::Pose3 optCamPose = optPose.compose(toGtsam(config_.camera.bodyPSensor));
            int postOptOutliers = 0;

            // 统计门控需要协方差；拿不到时退回像素阈值。两条路径的硬门限
            // 完全一致：无论统计检验怎么说，灾难性误匹配都必须立即剔除。
            const bool useChi2 = config_.vision.chi2Gating && poseCovariance.has_value();
            const double chi2Quantile = chi2Quantile2dof(config_.vision.chi2Confidence);
            const double chi2LimitTracked = config_.vision.chi2Multiplier * chi2Quantile;
            // 新观测只被一次观测约束过，用成熟倍数卡它会误杀仍在收敛的正确点。
            const double chi2LimitNew = config_.vision.chi2MultiplierNew * chi2Quantile;
            int chi2Fallbacks = 0;
            int chi2Rejections = 0;

            for (const domain::Observation &observation : observations)
            {
                if (!triangulatedLandmarks_.contains(observation.id) || outlierLandmarks_.contains(observation.id) ||
                    !currentEstimate_.exists(L(observation.id)) ||
                    landmarkObsCount_[observation.id] < config_.vision.landmarkGateWarmupObs)
                {
                    continue;
                }

                ::gtsam::Point3 ptW;
                if (!landmarkWorldPosition(observation.id, ptW))
                {
                    continue;
                }
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

                // 硬门限优先：统计门控也拦不住的东西（错误匹配到完全
                // 别的物体）必须先被几何拿掉。
                if (residualPx > config_.vision.maxReprojectionErrorHard)
                {
                    outlierLandmarks_.insert(observation.id);
                    outlierStreak_.erase(observation.id);
                    postOptOutliers++;
                    mylog::warn("[外点剔除] 特征点 #{} 优化后重投影残差 {:.2f} px 超硬门限 {:.2f} px，已标记为外点",
                                observation.id, residualPx, config_.vision.maxReprojectionErrorHard);
                    continue;
                }

                bool exceeded = residualPx > config_.vision.maxReprojectionError;
                if (useChi2)
                {
                    // 短路：残差明显低时无论协方差如何都会接受。统计门控
                    // 的目的是随协方差缩放门限，只在「像素门限正在做决定」
                    // 的区间才有意义。每个观测算一次 3x3 边际协方差需要一次
                    // 分解，无条件付出会把后端耗时甩到两倍以上。
                    const bool decisionInPlay = residualPx > 0.25 * config_.vision.maxReprojectionError;
                    if (!decisionInPlay)
                    {
                        outlierStreak_.erase(observation.id);
                        continue;
                    }

                    std::optional<domain::Mat3> landmarkCov;
                    if (config_.vision.chi2UseLandmarkCovariance)
                    {
                        landmarkCov = landmarkWorldCovariance(observation.id);
                    }
                    const double chi2Limit =
                        observation.kind == domain::ObservationKind::NEW ? chi2LimitNew : chi2LimitTracked;
                    if (const auto chi2 =
                            chi2ResidualSquared(optPose, ptW, observation.uv, *poseCovariance, landmarkCov);
                        chi2.has_value())
                    {
                        exceeded = *chi2 > chi2Limit;
                        if (exceeded)
                        {
                            ++chi2Rejections;
                            mylog::debug("[χ²门控] 特征点 #{}: 残差 {:.2f} px, χ²={:.2f} > 门限 {:.2f} (路标协方差{})",
                                         observation.id, residualPx, *chi2, chi2Limit,
                                         landmarkCov.has_value() ? "已计入" : "缺席");
                        }
                    }
                    else
                    {
                        ++chi2Fallbacks;
                    }
                }

                if (!exceeded)
                {
                    outlierStreak_.erase(observation.id);
                    continue;
                }

                int &streak = outlierStreak_[observation.id];
                ++streak;
                mylog::debug("[外点门控] 特征点 #{} 优化后残差 {:.2f} px 超限 (连续 {} 帧, χ²={})", observation.id,
                             residualPx, streak, useChi2 ? "已启用" : "未启用");
                if (streak >= config_.vision.outlierStreakThreshold)
                {
                    outlierLandmarks_.insert(observation.id);
                    outlierStreak_.erase(observation.id);
                    postOptOutliers++;
                    mylog::warn("[外点剔除] 特征点 #{} 连续 {} 帧超限，已标记为外点", observation.id,
                                config_.vision.outlierStreakThreshold);
                }
            }

            if (useChi2)
            {
                mylog::debug("[χ²门控] KF #{}: 门限={:.2f} (95% 分位 × {:.1f}), 判超限 {} 个, 退回像素阈值 {} 个",
                             currentKfId, chi2LimitTracked, config_.vision.chi2Multiplier, chi2Rejections,
                             chi2Fallbacks);
            }

            if (postOptOutliers > 0)
            {
                mylog::warn("[外点处理] KF #{} 优化后共标记 {} 个外点观测", currentKfId, postOptOutliers);
            }
        }

        /**
         * @brief 自由度 2 的 χ² 分位数（精确闭式解）
         *
         * 二维卡方分布的 CDF 是 @c 1-exp(-x/2)，所以分位数就是
         * @c -2ln(1-p)。不需要查表也不需要 boost 分布对象，且对任意置信
         * 水平都直接给出结果。
         */
        [[nodiscard]] static double chi2Quantile2dof(double confidence) noexcept
        {
            const double tail = std::clamp(1.0 - confidence, 1e-12, 1.0);
            return -2.0 * std::log(tail);
        }

        /**
         * @brief 单个观测的平方马氏距离残差 rᵀ(H P Hᵀ + R)⁻¹ r
         *
         * H 是像素残差对 body 位姿（6 维，与 @c Mat6 同序）的雅可比，P 是
         * 优化后位姿的边际协方差，R = σ²I 是像素噪声。
         *
         * 雅可比用中心差分：解析式要穿过 bodyPSensor 外参复合与四元数
         * 切线，符号写错不会报错，只会让门控安静地在错误方向上工作。
         *
         * @param landmarkCovariance 路标世界系位置协方差。提供时计入 S：
         *        缺它会让 S 偏小、χ² 偏大，把「深度尚未收敛」的正确观测
         *        当成外点剔除。为 @c nullopt 时退回只用位姿协方差。
         */
        [[nodiscard]] std::optional<double>
        chi2ResidualSquared(const ::gtsam::Pose3 &tWb, const ::gtsam::Point3 &ptW, const domain::Vec2 &measuredUv,
                            const domain::Mat6 &bodyCovariance,
                            const std::optional<domain::Mat3> &landmarkCovariance) const
        {
            const ::gtsam::Pose3 bodyPSensor = toGtsam(config_.camera.bodyPSensor);
            const auto predict = [&](const ::gtsam::Pose3 &pose) -> std::optional<domain::Vec2>
            { return camera_->project(fromGtsam(pose.compose(bodyPSensor).transformTo(ptW))); };

            const auto center = predict(tWb);
            if (!center)
            {
                return std::nullopt;
            }
            const domain::Vec2 residual = *center - measuredUv;

            constexpr double kStep = 1e-6;
            Eigen::Matrix<double, 2, 6> H;
            for (int i = 0; i < 6; ++i)
            {
                ::gtsam::Vector6 delta = ::gtsam::Vector6::Zero();
                delta(i) = kStep;
                const auto plus = predict(tWb.retract(delta));
                const auto minus = predict(tWb.retract(-delta));
                if (!plus || !minus)
                {
                    return std::nullopt;
                }
                H.col(i) = (*plus - *minus) / (2.0 * kStep);
            }

            const double sigma = config_.vision.pixelNoiseSigma;
            Eigen::Matrix2d S = H * bodyCovariance * H.transpose();
            if (landmarkCovariance.has_value())
            {
                // 像素对世界系路标位置的雅可比，2x3 中心差分
                const ::gtsam::Pose3 tWc = tWb.compose(bodyPSensor);
                Eigen::Matrix<double, 2, 3> Hlm;
                for (int i = 0; i < 3; ++i)
                {
                    ::gtsam::Point3 delta = ::gtsam::Point3::Zero();
                    delta(i) = kStep;
                    const ::gtsam::Point3 plusPoint(ptW.x() + delta.x(), ptW.y() + delta.y(), ptW.z() + delta.z());
                    const ::gtsam::Point3 minusPoint(ptW.x() - delta.x(), ptW.y() - delta.y(), ptW.z() - delta.z());
                    const auto plus = camera_->project(fromGtsam(tWc.transformTo(plusPoint)));
                    const auto minus = camera_->project(fromGtsam(tWc.transformTo(minusPoint)));
                    if (!plus || !minus)
                    {
                        return std::nullopt;
                    }
                    Hlm.col(i) = (*plus - *minus) / (2.0 * kStep);
                }
                S += Hlm * (*landmarkCovariance) * Hlm.transpose();
            }
            S += Eigen::Matrix2d::Identity() * sigma * sigma;
            const Eigen::LDLT<Eigen::Matrix2d> ldlt(S);
            if (ldlt.info() != Eigen::Success || !ldlt.isPositive())
            {
                return std::nullopt;
            }
            const double chi2 = residual.dot(ldlt.solve(residual));
            if (!std::isfinite(chi2) || chi2 < 0.0)
            {
                return std::nullopt;
            }
            return chi2;
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
                        landmarkAnchors_.erase(id);
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
                        keyframeTimestamps_.erase(id);
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

        /**
         * @brief 零速判定的四路与门
         *
         * 1) IMU 方差与比力模长（@c isStationary）；
         * 2) 速度估计上限。纯 IMU 判据在**匀速直线**运动时全部通过（加速度
         *    与角速度都是零），于是 ZUPT 会把真实运动钉死。速度是唯一能
         *    区分「静止」与「匀速」的状态量；
         * 3) 图像平均视差。IMU 与速度估计可能同时错在同一方向上，
         *    但图像在这时仍然会动，这是最后一个独立信号；
         * 4) 共视特征数下限——视差统计本身要有样本。
         */
        [[nodiscard]] bool zuptAccepted(std::span<const domain::Observation> observations,
                                        const ::gtsam::Vector3 &velocity) const
        {
            if (!isStationary())
            {
                return false;
            }

            const double speed = velocity.norm();
            if (speed > config_.estimator.zuptMaxVelocity)
            {
                mylog::debug("[零速更新] 拒绝: 速度 {:.3f} > {:.3f} m/s（匀速运动不是静止）", speed,
                             config_.estimator.zuptMaxVelocity);
                return false;
            }

            std::unordered_map<domain::FeatureId, domain::Vec2> previousUv;
            previousUv.reserve(previousObservations_.size());
            for (const domain::Observation &previous : previousObservations_)
            {
                previousUv[previous.id] = previous.uv;
            }

            double disparitySum = 0.0;
            std::size_t common = 0;
            for (const domain::Observation &observation : observations)
            {
                const auto it = previousUv.find(observation.id);
                if (it == previousUv.end())
                {
                    continue;
                }
                disparitySum += (observation.uv - it->second).norm();
                ++common;
            }

            if (common < static_cast<std::size_t>(config_.estimator.zuptMinFeatures))
            {
                mylog::debug("[零速更新] 拒绝: 共视特征 {} < {}，视差统计无样本", common,
                             config_.estimator.zuptMinFeatures);
                return false;
            }

            const double meanDisparity = disparitySum / static_cast<double>(common);
            if (meanDisparity > config_.estimator.zuptMaxDisparity)
            {
                mylog::debug("[零速更新] 拒绝: 平均视差 {:.3f} > {:.3f} px（图像证明在动）", meanDisparity,
                             config_.estimator.zuptMaxDisparity);
                return false;
            }
            return true;
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
        /// 路标参数化方式；全局 3D 点与锚定逆深度共用其余流程
        bool useInverseDepth_{false};

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
        /// 关键帧时间戳表：Values 不携带时间，快照与 NEES 配对需要它
        std::unordered_map<domain::KeyframeId, double> keyframeTimestamps_;
        std::vector<domain::Observation> previousObservations_;
        std::unordered_map<domain::LandmarkId, ::gtsam::Point3> triangulatedLandmarks_;
        /// 逆深度模式下每个路标的锚点相机位姿（World ← Camera），固定不入优化
        std::unordered_map<domain::LandmarkId, ::gtsam::Pose3> landmarkAnchors_;
        std::unordered_set<domain::LandmarkId> outlierLandmarks_;
        std::unordered_map<domain::LandmarkId, int> outlierStreak_;
        std::unordered_map<domain::LandmarkId, int> landmarkObsCount_;

        domain::HealthMonitor healthMonitor_;
        domain::HealthReport lastHealth_{};

        std::size_t recoveredFrames_{0};
        std::deque<domain::ImuSample> recentImu_;
        std::size_t zuptCount_{0};
    };

} // namespace myslam::adapters
