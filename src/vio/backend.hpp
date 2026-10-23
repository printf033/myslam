#pragma once

/**
 * @file backend.hpp
 * @brief 紧耦合固定滞后后端：预积分、三角化、分层外点门控与真边缘化。
 */

#include "logger.hpp"
#include "vio/health_monitor.hpp"
#include "vio/types.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
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
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace myslam::vio
{

    using gtsam::symbol_shorthand::B; // imuBias::ConstantBias
    using gtsam::symbol_shorthand::L; // Point3 (Landmark)
    using gtsam::symbol_shorthand::V; // Vector3 (Velocity)
    using gtsam::symbol_shorthand::X; // Pose3 (R, p)

    // =========================================================================
    // 7. 紧耦合滑动窗口优化核心后端 (VioBackend)
    // =========================================================================

    class VioBackend
    {
    public:
        explicit VioBackend(const VioConfig &config)
            : config_(config),
              K_(std::make_shared<gtsam::Cal3_S2>(config.fx, config.fy, 0.0, config.cx, config.cy))
        {
            // 初始化 IMU 预积分参数
            auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(config.gravityMagnitude);
            p->accelerometerCovariance = gtsam::I_3x3 * std::pow(config.accelNoiseSigma, 2);
            p->gyroscopeCovariance = gtsam::I_3x3 * std::pow(config.gyroNoiseSigma, 2);
            p->integrationCovariance = gtsam::I_3x3 * config.integrationNoise;
            p->biasAccCovariance = gtsam::I_3x3 * std::pow(config.accelBiasRw, 2);
            p->biasOmegaCovariance = gtsam::I_3x3 * std::pow(config.gyroBiasRw, 2);

            imuParams_ = p;
            pim_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imuParams_, currentBias_);

            // 鲁棒核模型 (Huber Loss 结合像素噪声)
            auto isotropicPixel = gtsam::noiseModel::Isotropic::Sigma(2, config_.pixelNoiseSigma);
            robustPixelNoise_ = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(config_.huberLossDelta), isotropicPixel);

            // 配置固定滞后平滑器 (gtsam 官方 iSAM2 + 滑动窗口边缘化实现)：
            // 内部自动完成 ordering constraints -> extraReelim -> marginalizeLeaves，
            // 并正确处理 pending value / 无因子变量等边界情况
            gtsam::ISAM2Params isamParams;
            isamParams.relinearizeThreshold = 0.05;  // 增量重线性化阈值 (delta 变化超过该比例才重线性化)
            isamParams.relinearizeSkip = 1;          // 每次 update 都检查重线性化，保证收敛质量
            isamParams.findUnusedFactorSlots = true; // 复用已删除因子的槽位，抑制图规模无谓增长
            // Cholesky：图规模较大时比 QR 快一个量级；噪声参数取真实量级后
            // Hessian 条件数正常，无需 QR 的额外鲁棒性代价。
            isamParams.factorization = gtsam::ISAM2Params::CHOLESKY;
            smoother_ = gtsam::IncrementalFixedLagSmoother(config_.slidingWindowLag, isamParams);
        }

        /**
         * @brief 传感器前端：累积关键帧之间的 IMU 读数
         */
        void integrateImu(const ImuMeasurement &imu, double dt) { pim_->integrateMeasurement(imu.accel, imu.gyro, dt); }

        /**
         * @brief 系统初始化完成时注入先验因子
         */
        void initializeSystem(int keyframeId, double timestamp, const gtsam::Pose3 &priorPose,
                              const gtsam::Vector3 &priorVelocity, const gtsam::imuBias::ConstantBias &priorBias)
        {
            currentBias_ = priorBias;
            pim_->resetIntegrationAndSetBias(currentBias_);

            // 注入初始先验因子 (带有工业合理的不确定度)
            auto poseNoise =
                gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
            auto velNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
            auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.02, 0.02, 0.02).finished());

            newFactors_.addPrior(X(keyframeId), priorPose, poseNoise);
            newFactors_.addPrior(V(keyframeId), priorVelocity, velNoise);
            newFactors_.addPrior(B(keyframeId), priorBias, biasNoise);

            newValues_.insert(X(keyframeId), priorPose);
            newValues_.insert(V(keyframeId), priorVelocity);
            newValues_.insert(B(keyframeId), priorBias);

            gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
            timestamps[X(keyframeId)] = timestamp;
            timestamps[V(keyframeId)] = timestamp;
            timestamps[B(keyframeId)] = timestamp;
            smoother_.update(newFactors_, newValues_, timestamps);
            currentEstimate_ = smoother_.calculateEstimate();
            newFactors_.resize(0);
            newValues_.clear();

            windowKeyframes_.push_back(keyframeId);

            latestNavState_ = gtsam::NavState(priorPose, priorVelocity);

            mylog::info(
                "[后端初始化] iSAM2 注入初始帧先验: KF_ID={}, P=[{:.3f}, {:.3f}, {:.3f}], V=[{:.3f}, {:.3f}, {:.3f}]",
                keyframeId, priorPose.x(), priorPose.y(), priorPose.z(), priorVelocity(0), priorVelocity(1),
                priorVelocity(2));
        }

        /**
         * @brief 尝试三角化新路标点 (利用前后两帧视差，并为历史帧补齐投影因子)
         */
        void triangulateNewLandmarks(int prevKfId, const CameraFrame &kf1, const CameraFrame &kf2,
                                     const gtsam::Pose3 &tWb1, const gtsam::Pose3 &tWb2)
        {
            gtsam::Pose3 tWc1 = tWb1.compose(config_.bodyPSensor);
            gtsam::Pose3 tWc2 = tWb2.compose(config_.bodyPSensor);

            std::unordered_map<int, gtsam::Point2> obs1;
            for (const auto &f : kf1.features)
            {
                obs1[f.featureId] = f.uv;
            }

            int triangulatedCount = 0;
            for (const auto &f2 : kf2.features)
            {
                int id = f2.featureId;
                if (triangulatedLandmarks_.contains(id))
                {
                    continue; // 已经三角化过
                }

                auto it = obs1.find(id);
                if (it != obs1.end())
                {
                    gtsam::Point2 p1 = it->second;
                    gtsam::Point2 p2 = f2.uv;

                    // 检查像素视差：小视差三角化深度误差大，会引入系统性坏路标
                    if ((p1 - p2).norm() < config_.minTriangulationParallaxPx)
                    {
                        continue;
                    }

                    try
                    {
                        std::vector<gtsam::Pose3> cameraPoses = {tWc1, tWc2};
                        gtsam::Point2Vector measurements = {p1, p2};
                        gtsam::Point3 ptW = gtsam::triangulatePoint3(cameraPoses, K_, measurements, 1e-4, true);

                        // 深度正定与合理性校验 (保证点在两个相机前方且距离合理)
                        gtsam::Point3 ptC1 = tWc1.transformTo(ptW);
                        gtsam::Point3 ptC2 = tWc2.transformTo(ptW);

                        // 工业校验：三角化结果必须在两帧上重投影一致，剔除受误匹配污染
                        // 的三角化 (外点观测可能在此之前尚未进入重投影门控)
                        double reprojErr1 = 0.0;
                        double reprojErr2 = 0.0;
                        try
                        {
                            gtsam::Point2 uv1Check = gtsam::PinholeCamera<gtsam::Cal3_S2>(tWc1, *K_).project(ptW);
                            gtsam::Point2 uv2Check = gtsam::PinholeCamera<gtsam::Cal3_S2>(tWc2, *K_).project(ptW);
                            reprojErr1 = (uv1Check - p1).norm();
                            reprojErr2 = (uv2Check - p2).norm();
                        }
                        catch (const gtsam::CheiralityException &)
                        {
                            continue;
                        }

                        if (reprojErr1 > config_.maxReprojectionErrorHard ||
                            reprojErr2 > config_.maxReprojectionErrorHard)
                        {
                            mylog::warn("[三角化剔除] 特征点 #{} 三角化重投影不一致 (帧1 {:.2f} px, 帧2 {:.2f} px "
                                        "> {:.2f} px)，疑似误匹配，已丢弃",
                                        id, reprojErr1, reprojErr2, config_.maxReprojectionErrorHard);
                            continue;
                        }

                        if (ptC1.z() > 0.3 && ptC1.z() < 20.0 && ptC2.z() > 0.3 && ptC2.z() < 20.0)
                        {
                            triangulatedLandmarks_[id] = ptW;
                            landmarkObsCount_[id] = 2; // 三角化本身用掉 2 个视角观测
                            newValues_.insert(L(id), ptW);

                            // 软正则化先验 (1.0 m 标准差)：仅用于消除两视图三角化的
                            // 欠约束/视角退化奇异性，强度远弱于投影因子，不会压制观测。
                            // 注：真实工业系统通常靠多帧观测与鲁棒核即可，此先验属教学稳定化技巧。
                            auto landmarkPriorNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
                            newFactors_.addPrior(L(id), ptW, landmarkPriorNoise);

                            // 补齐上一关键帧的投影因子，确保多视角立体重建良态约束 (至少 2 个视角约束该 3D 点)
                            newFactors_.emplace_shared<
                                gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                                p1, robustPixelNoise_, X(prevKfId), L(id), K_, config_.bodyPSensor);

                            triangulatedCount++;
                        }
                    }
                    catch (const std::exception &)
                    {
                        // 几何退化忽略
                    }
                }
            }

            if (triangulatedCount > 0)
            {
                mylog::info("[视觉前端] 跨帧三角化成功生成 {} 个新路标点 (当前总活跃路标: {})", triangulatedCount,
                            triangulatedLandmarks_.size());
            }
        }

        /**
         * @brief 处理新的关键帧：预优化外点门控、构建因子图、执行优化、
         *        滑动窗口边缘化与零偏反馈
         */
        void processKeyframe(int currentKfId, double timestamp, const CameraFrame &frame,
                             const CameraFrame &lastKfFrame)
        {
            int prevKfId = windowKeyframes_.back();

            // 1. 状态初值推导 (通过 IMU 预积分外推当前关键帧初值)
            gtsam::NavState propState = pim_->predict(latestNavState_, currentBias_);
            newValues_.insert(X(currentKfId), propState.pose());
            newValues_.insert(V(currentKfId), propState.v());
            newValues_.insert(B(currentKfId), currentBias_);

            // 2. 添加 IMU 预积分因子 (CombinedImuFactor)
            newFactors_.emplace_shared<gtsam::CombinedImuFactor>(X(prevKfId), V(prevKfId), X(currentKfId),
                                                                 V(currentKfId), B(prevKfId), B(currentKfId), *pim_);

            mylog::debug("[IMU预积分] 插入 CombinedImuFactor: KF #{} -> #{}, 积分时长={:.3f}s", prevKfId, currentKfId,
                         pim_->deltaTij());

            // 3. 预优化外点门控 (Pre-optimization Outlier Gating)：
            //    在 iSAM2 update 前用预积分预测位姿与当前路标估计做重投影检验，
            //    避免外点观测进入本次优化污染后验估计。
            //    工业实践：配合连续两帧超限滞回 (hysteresis)，避免误伤噪声边缘的真观测。
            gtsam::Pose3 predCamPose = propState.pose().compose(config_.bodyPSensor);
            gtsam::PinholeCamera<gtsam::Cal3_S2> predCamera(predCamPose, *K_);

            int outliersDetected = 0;
            std::unordered_set<int> gatedOutThisFrame; // 本帧被硬门限拒绝的观测
            for (const auto &feat : frame.features)
            {
                if (triangulatedLandmarks_.contains(feat.featureId) && !outlierLandmarks_.contains(feat.featureId) &&
                    currentEstimate_.exists(L(feat.featureId)) &&
                    landmarkObsCount_[feat.featureId] >= config_.landmarkGateWarmupObs)
                {
                    gtsam::Point3 ptW = currentEstimate_.at<gtsam::Point3>(L(feat.featureId));
                    bool exceedsGate = false;
                    double residualPx = 0.0;
                    try
                    {
                        gtsam::Point2 projUv = predCamera.project(ptW);
                        residualPx = (projUv - feat.uv).norm();
                        exceedsGate = residualPx > config_.maxReprojectionError;
                    }
                    catch (const gtsam::CheiralityException &)
                    {
                        exceedsGate = true;
                    }

                    if (exceedsGate)
                    {
                        // 预优化只有预测位姿可用：残差大的观测本帧不加入优化 (保护优化质量)，
                        // 但不在此永久标记 —— 预测位姿误差会随时间收敛，
                        // 永久标记统一交给优化后门控 (用优化位姿判定)。
                        // 注意：被拒绝的只是本帧因子，路标时间戳仍会刷新 (见步骤 6)，
                        // 因此门控不会缩短任何真实路标的寿命。
                        gatedOutThisFrame.insert(feat.featureId);
                        outliersDetected++;

                        if (residualPx > config_.maxReprojectionErrorPreopt)
                        {
                            // 灾难级残差：远超预测不确定性，判定为严重误匹配并报告
                            mylog::warn(
                                "[外点剔除] 特征点 #{} 预优化残差 {:.2f} px 超灾难门限 {:.2f} px，"
                                "判定严重误匹配，已拒绝观测 (路标=[{:.2f},{:.2f},{:.2f}] 相机=[{:.2f},{:.2f},{:.2f}])",
                                feat.featureId, residualPx, config_.maxReprojectionErrorPreopt, ptW.x(), ptW.y(),
                                ptW.z(), predCamPose.translation().x(), predCamPose.translation().y(),
                                predCamPose.translation().z());
                        }
                        else
                        {
                            mylog::debug("[外点门控] 特征点 #{} 预优化残差 {:.2f} px 超软门限 {:.2f} px，"
                                         "本帧暂不参与优化",
                                         feat.featureId, residualPx, config_.maxReprojectionError);
                        }
                    }
                }
            }

            if (outliersDetected > 0)
            {
                mylog::debug("[外点门控] KF #{}: 本帧 {} 个观测预优化残差超限，暂不参与优化", currentKfId,
                             outliersDetected);
            }

            // 4. 跨帧三角化新路标 (使用优化后的历史位姿与预积分预测的当前位姿，抑制深度系统偏差)
            triangulateNewLandmarks(prevKfId, lastKfFrame, frame, currentEstimate_.at<gtsam::Pose3>(X(prevKfId)),
                                    propState.pose());

            // 5. 添加视觉重投影误差因子 (GenericProjectionFactor)
            size_t visualFactorCount = 0;
            for (const auto &feat : frame.features)
            {
                // 路标必须仍在图中：既可能是已优化变量 (currentEstimate_)，
                // 也可能是本帧刚三角化的新变量 (newValues_，尚未进入估计)
                const bool landmarkInGraph =
                    currentEstimate_.exists(L(feat.featureId)) || newValues_.exists(L(feat.featureId));
                if (triangulatedLandmarks_.contains(feat.featureId) && !outlierLandmarks_.contains(feat.featureId) &&
                    !gatedOutThisFrame.contains(feat.featureId) && landmarkInGraph)
                {
                    newFactors_
                        .emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                            feat.uv, robustPixelNoise_, X(currentKfId), L(feat.featureId), K_, config_.bodyPSensor);
                    landmarkObsCount_[feat.featureId]++;
                    visualFactorCount++;
                }
            }

            mylog::debug("[因子图构建] KF #{}: 新增 IMU 因子 1 个, 视觉投影因子 {} 个", currentKfId, visualFactorCount);

            // 6. 增量贝叶斯平滑优化 (固定滞后平滑器：iSAM2 + 真边缘化)
            //    时间戳驱动窗口管理：关键帧变量取当前时间；路标取本次观测时间，
            //    从而「仍被观测的路标」永不退出，「失去视野的路标」在超出滞后
            //    窗口后随其最后观测关键帧一起被边缘化 (VINS-Mono 同思路)。
            gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
            timestamps[X(currentKfId)] = timestamp;
            timestamps[V(currentKfId)] = timestamp;
            timestamps[B(currentKfId)] = timestamp;
            for (const auto &feat : frame.features)
            {
                // 路标寿命只由「是否仍被观测」决定，不由本帧是否被门控决定：
                // 被临时拒绝的观测同样刷新时间戳，避免真实路标被门控「饿死」后过期删除。
                // 仅对仍在图中的路标刷新 (已被边缘化删除的变量不能再提交时间戳)。
                const bool landmarkInGraph =
                    currentEstimate_.exists(L(feat.featureId)) || newValues_.exists(L(feat.featureId));
                if (triangulatedLandmarks_.contains(feat.featureId) && !outlierLandmarks_.contains(feat.featureId) &&
                    landmarkInGraph)
                {
                    timestamps[L(feat.featureId)] = timestamp;
                }
            }

            auto tStart = std::chrono::steady_clock::now();
            auto smootherResult = smoother_.update(newFactors_, newValues_, timestamps);
            // 额外迭代：同一批因子继续收敛 (空 update 复用当前线性化点做 Gauss-Newton 迭代)
            for (int iter = 0; iter < config_.extraSmootherIterations; ++iter)
            {
                const auto extra = smoother_.update();
                smootherResult.keysOfDeletedNodes.insert(extra.keysOfDeletedNodes.begin(),
                                                         extra.keysOfDeletedNodes.end());
                smootherResult.expiredPendingKeys.insert(extra.expiredPendingKeys.begin(),
                                                         extra.expiredPendingKeys.end());
            }
            currentEstimate_ = smoother_.calculateEstimate();
            auto tEnd = std::chrono::steady_clock::now();
            double elapsedMs = std::chrono::duration<double, std::milli>(tEnd - tStart).count();

            newFactors_.resize(0);
            newValues_.clear();

            // 7. 提取最新优化状态
            gtsam::Pose3 optPose = currentEstimate_.at<gtsam::Pose3>(X(currentKfId));
            gtsam::Vector3 optVel = currentEstimate_.at<gtsam::Vector3>(V(currentKfId));
            gtsam::imuBias::ConstantBias optBias = currentEstimate_.at<gtsam::imuBias::ConstantBias>(B(currentKfId));

            latestNavState_ = gtsam::NavState(optPose, optVel);

            mylog::info("[后端iSAM2优化] KF #{}: 增量优化耗时={:.2f}ms, 活跃变量数={}", currentKfId, elapsedMs,
                        currentEstimate_.size());

            // 7.5 优化后精确外点门控 (VINS-Mono 标准顺序)：
            //     优化后位姿与路标均已收敛，残差可信。单帧超硬门限立即标记；
            //     连续超软门限 (滞回) 标记持续性异常。被标记路标从下一帧起不再参与优化。
            gtsam::Pose3 optCamPose = optPose.compose(config_.bodyPSensor);
            gtsam::PinholeCamera<gtsam::Cal3_S2> optCamera(optCamPose, *K_);
            int postOptOutliers = 0;
            for (const auto &feat : frame.features)
            {
                // 注意：不跳过 gatedOutThisFrame —— 预优化门控只是本帧不加因子，
                // 优化后位姿才是可信判据；坏路标必须在此被识别并永久标记，
                // 否则会永远反复触发门控告警且占用图资源。
                if (!triangulatedLandmarks_.contains(feat.featureId) || outlierLandmarks_.contains(feat.featureId) ||
                    !currentEstimate_.exists(L(feat.featureId)) ||
                    landmarkObsCount_[feat.featureId] < config_.landmarkGateWarmupObs)
                {
                    continue;
                }

                double residualPx = 0.0;
                try
                {
                    gtsam::Point3 ptW = currentEstimate_.at<gtsam::Point3>(L(feat.featureId));
                    residualPx = (optCamera.project(ptW) - feat.uv).norm();
                }
                catch (const gtsam::CheiralityException &)
                {
                    residualPx = std::numeric_limits<double>::infinity();
                }

                if (!config_.postOptOutlierMarking)
                {
                    continue;
                }
                if (residualPx <= config_.maxReprojectionError)
                {
                    outlierStreak_.erase(feat.featureId);
                    continue;
                }

                if (residualPx > config_.maxReprojectionErrorHard)
                {
                    // 优化后仍大幅超限：确认为误匹配，单帧即标记
                    outlierLandmarks_.insert(feat.featureId);
                    outlierStreak_.erase(feat.featureId);
                    postOptOutliers++;
                    mylog::warn("[外点剔除] 特征点 #{} 优化后重投影残差 {:.2f} px 超硬门限 {:.2f} px，已标记为外点！",
                                feat.featureId, residualPx, config_.maxReprojectionErrorHard);
                    continue;
                }

                int &streak = outlierStreak_[feat.featureId];
                ++streak;
                mylog::debug("[外点门控] 特征点 #{} 优化后残差 {:.2f} px 超软门限 {:.2f} px (连续 {} 帧)",
                             feat.featureId, residualPx, config_.maxReprojectionError, streak);
                if (streak >= kOutlierStreakThreshold)
                {
                    outlierLandmarks_.insert(feat.featureId);
                    outlierStreak_.erase(feat.featureId);
                    postOptOutliers++;
                    mylog::warn("[外点剔除] 特征点 #{} 连续 {} 帧优化后重投影残差超过软门限 {:.2f} px (当前 {:.2f} "
                                "px)，已标记为外点！",
                                feat.featureId, kOutlierStreakThreshold, config_.maxReprojectionError, residualPx);
                }
            }
            if (postOptOutliers > 0)
            {
                mylog::warn("[外点处理] KF #{} 优化后共标记 {} 个外点观测", currentKfId, postOptOutliers);
            }

            // 8. 在线零偏闭环反馈与预积分器重置
            currentBias_ = optBias;
            pim_->resetIntegrationAndSetBias(currentBias_);

            mylog::info("[零偏更新反馈] KF #{}: 后验加速度偏置=[{:.4f}, {:.4f}, {:.4f}] m/s^2, 陀螺偏置=[{:.5f}, "
                        "{:.5f}, {:.5f}] rad/s",
                        currentKfId, currentBias_.accelerometer()(0), currentBias_.accelerometer()(1),
                        currentBias_.accelerometer()(2), currentBias_.gyroscope()(0), currentBias_.gyroscope()(1),
                        currentBias_.gyroscope()(2));

            // 9. 滑动窗口边缘化：由固定滞后平滑器按时间戳自动完成。
            //    需要同步清理本地登记的两类被删除变量：
            //    - keysOfDeletedNodes：已完成边缘化的变量；
            //    - expiredPendingKeys：超期但尚无因子引用的变量 (gtsam 直接删除，未走边缘化)。
            windowKeyframes_.push_back(currentKfId);
            gtsam::KeySet removedKeys = smootherResult.keysOfDeletedNodes;
            removedKeys.insert(smootherResult.expiredPendingKeys.begin(), smootherResult.expiredPendingKeys.end());
            if (!removedKeys.empty())
            {
                size_t removedKeyframes = 0;
                size_t removedLandmarks = 0;
                for (gtsam::Key k : removedKeys)
                {
                    const gtsam::Symbol sym(k);
                    if (sym.chr() == 'l')
                    {
                        ++removedLandmarks;
                        const int lmId = static_cast<int>(sym.index());
                        triangulatedLandmarks_.erase(lmId);
                        outlierLandmarks_.erase(lmId);
                        outlierStreak_.erase(lmId);
                        landmarkObsCount_.erase(lmId);
                    }
                    else if (sym.chr() == 'x')
                    {
                        ++removedKeyframes;
                        windowKeyframes_.erase(std::remove(windowKeyframes_.begin(), windowKeyframes_.end(),
                                                           static_cast<int>(sym.index())),
                                               windowKeyframes_.end());
                    }
                }
                mylog::info("[滑动窗口边缘化] 边缘化退出变量: 关键帧 {} 个, 路标 {} 个 (活跃关键帧 {}, 活跃变量 {})",
                            removedKeyframes, removedLandmarks, windowKeyframes_.size(), currentEstimate_.size());
            }

            // 10. 健康监控
            auto health = HealthMonitor::inspect(latestNavState_, currentBias_, visualFactorCount);
            if (!health.isHealthy)
            {
                mylog::warn("[健康报警] {}", health.alertMessage);
            }
        }

        const gtsam::NavState &latestNavState() const { return latestNavState_; }
        const gtsam::imuBias::ConstantBias &latestBias() const { return currentBias_; }

        /**
         * @brief 当前优化估计（含活跃路标 L 变量），用于点云导出与可视化
         */
        const gtsam::Values &currentEstimate() const { return currentEstimate_; }

        /**
         * @brief 已三角化路标（未随窗口边缘化的原始结果）
         */
        const std::unordered_map<int, gtsam::Point3> &triangulatedLandmarks() const { return triangulatedLandmarks_; }

    private:
        VioConfig config_;
        std::shared_ptr<gtsam::Cal3_S2> K_;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams_;
        std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
        gtsam::SharedNoiseModel robustPixelNoise_;

        gtsam::IncrementalFixedLagSmoother smoother_; ///< 固定滞后平滑器 (滑动窗口 + 真边缘化)
        gtsam::NonlinearFactorGraph newFactors_;
        gtsam::Values newValues_;
        gtsam::Values currentEstimate_;

        gtsam::NavState latestNavState_;
        gtsam::imuBias::ConstantBias currentBias_;

        std::deque<int> windowKeyframes_;
        std::unordered_map<int, gtsam::Point3> triangulatedLandmarks_;
        std::unordered_set<int> outlierLandmarks_;
        std::unordered_map<int, int> outlierStreak_; ///< 外点门控滞回：连续超限帧计数
        /// 路标累计观测次数：新三角化路标位置尚粗糙，先给优化若干轮收敛机会，
        /// 达到阈值后才纳入外点门控 (工业 VIO 对新特征的保护期)。
        std::unordered_map<int, int> landmarkObsCount_;

        /// 外点判定滞回阈值：连续超限达到该帧数才标记，防止单帧噪声误杀
        static constexpr int kOutlierStreakThreshold = 3;
    };

} // namespace myslam::vio
