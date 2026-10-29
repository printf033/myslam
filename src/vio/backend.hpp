#pragma once

/**
 * @file backend.hpp
 * @brief 紧耦合固定滞后后端：预积分、三角化、分层外点门控与真边缘化。
 */

#include "logger.hpp"
#include "vio/configHolder.hpp"
#include "vio/health_monitor.hpp"
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
            isamParams_.relinearizeThreshold = 0.05;  // 增量重线性化阈值 (delta 变化超过该比例才重线性化)
            isamParams_.relinearizeSkip = 1;          // 每次 update 都检查重线性化，保证收敛质量
            isamParams_.findUnusedFactorSlots = true; // 复用已删除因子的槽位，抑制图规模无谓增长
            // Cholesky：图规模较大时比 QR 快一个量级；噪声参数取真实量级后
            // Hessian 条件数正常，无需 QR 的额外鲁棒性代价。
            isamParams_.factorization = gtsam::ISAM2Params::CHOLESKY;
            // 用 unique_ptr 而非值成员：update() 在求解阶段抛不定系统异常后，
            // iSAM2 内部线性缓存已经处于不可信状态，无法原地复原。分级恢复需要
            // 能够丢弃旧对象重新构造，值成员无法做到。
            smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.slidingWindowLag, isamParams_);
        }

        /**
         * @brief 传感器前端：累积关键帧之间的 IMU 读数
         */
        void integrateImu(const ImuMeasurement &imu, double dt)
        {
            pim_->integrateMeasurement(imu.accel, imu.gyro, dt);

            // 维护 ZUPT 用的近期样本窗口。必须按时间裁剪而不是按个数：采样率
            // 一旦与配置不符，按个数会悄悄把窗口拉长到几秒，静止判据随之失效。
            recentImu_.push_back(imu);
            while (!recentImu_.empty() && imu.timestamp - recentImu_.front().timestamp > config_.zuptWindowSeconds)
            {
                recentImu_.pop_front();
            }
        }

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
            smoother_->update(newFactors_, newValues_, timestamps);
            currentEstimate_ = smoother_->calculateEstimate();
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

                        // 校验：三角化结果必须在两帧上重投影一致，剔除受误匹配污染
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

                        if (ptC1.z() > config_.minLandmarkDepth && ptC1.z() < config_.maxLandmarkDepth &&
                            ptC2.z() > config_.minLandmarkDepth && ptC2.z() < config_.maxLandmarkDepth)
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
        /**
         * @brief 处理新的关键帧：预优化外点门控、构建因子图、执行优化、
         *        滑动窗口边缘化与零偏反馈
         *
         * @return 是否产出了新的位姿估计。@c false 表示本帧未能优化（窗口
         *         为空，或求解异常但已通过重建平滑器局部恢复）。调用方应保持
         *         上一次成功的位姿与外推状态继续运行，不应重建整条流水线
         *         —— 重建会让坐标系归零，轨迹表现为“跳回原点”。
         */
        bool processKeyframe(int currentKfId, double timestamp, const CameraFrame &frame,
                             const CameraFrame &lastKfFrame)
        {
            // 窗口为空时没有可承接 IMU 因子的前一关键帧。正常流程由
            // initializeSystem 注入首帧，这里防御的是“首帧已被边缘化”或
            // 调用方顺序错误——直接 .back() 是未定义行为。
            if (windowKeyframes_.empty())
            {
                mylog::error("[后端] 关键帧 #{} 到达时滑动窗口为空（缺少 initializeSystem 或首帧已被边缘化），跳过",
                             currentKfId);
                return false;
            }
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
            std::unordered_set<int> gatedOutThisFrame;    // 本帧被宽门限拒绝的观测
            std::unordered_set<int> geometricallyInvalid; // 本帧几何无效、完全不给因子的路标
            for (const auto &feat : frame.features)
            {
                if (!triangulatedLandmarks_.contains(feat.featureId) || outlierLandmarks_.contains(feat.featureId) ||
                    !currentEstimate_.exists(L(feat.featureId)))
                {
                    continue;
                }

                const gtsam::Point3 ptW = currentEstimate_.at<gtsam::Point3>(L(feat.featureId));

                // 3a. 几何有效性：无条件检查，不受 landmarkGateWarmupObs 豁免。
                //     保护期的本意是「新路标位置未收敛，残差大属正常」，但「投影
                //     是否有意义」并不依赖收敛：落在相机后方或贴着镜头的路标，
                //     其投影因子对坐标的雅可比随 1/z 发散，单条这样的观测就足以
                //     把 Hessian 推到 Cholesky 失败。旧实现把几何检查也放在保护期
                //     之后，导致新路标在前 landmarkGateWarmupObs 次观测期间完全
                //     没有任何几何校验，正是 IndeterminateSystemException 的来源。
                double residualPx = 0.0;
                bool geometricallyValid = true;
                try
                {
                    const gtsam::Point3 ptC = predCamPose.transformTo(ptW);
                    if (ptC.z() < config_.minLandmarkDepth || ptC.z() > config_.maxLandmarkDepth)
                    {
                        geometricallyValid = false;
                    }
                    else
                    {
                        residualPx = (predCamera.project(ptW) - feat.uv).norm();
                    }
                }
                catch (const gtsam::CheiralityException &)
                {
                    geometricallyValid = false;
                }

                if (!geometricallyValid)
                {
                    geometricallyInvalid.insert(feat.featureId);
                    continue;
                }

                // 3b. 残差门控：保护期内的新路标不做外点判定（位置尚未收敛）
                if (landmarkObsCount_[feat.featureId] < config_.landmarkGateWarmupObs)
                {
                    continue;
                }

                // 预优化阶段只有预积分预测位姿可用，早期零偏未收敛时预测误差
                // 可达 20+ px。因此这里必须用“宽门限”只拦截灾难性误匹配；
                // 若误用优化后的 8 px 窄门限，视觉约束会在最需要的阶段被抽空，
                // 三级门控退化为“优化前 8px / 优化后 8px”。
                if (residualPx > config_.maxReprojectionErrorPreopt)
                {
                    // 预优化只有预测位姿可用：残差大的观测本帧不加入优化 (保护优化质量)，
                    // 但不在此永久标记 —— 预测位姿误差会随时间收敛，
                    // 永久标记统一交给优化后门控 (用优化位姿判定)。
                    // 注意：被拒绝的只是本帧因子，路标时间戳仍会刷新 (见步骤 6)，
                    // 因此门控不会缩短任何真实路标的寿命。
                    outliersDetected++;

                    // 灾难门限按宽门限的倍数定义：预优化阶段的不确定性本身
                    // 就可能达 20~50 px，所以不能用优化后的窄门限做告警阈值
                    // （那样在该分支内恒为真，debug 分支成死代码，所有被门控
                    // 的观测都会刷 warn）。超过宽门限两倍才归为灾难级。
                    const double catastrophicPx = config_.maxReprojectionErrorPreopt * 2.0;
                    const char *action = config_.preoptTemporaryRejection ? "已拒绝本帧观测" : "仍送入优化";
                    if (residualPx > catastrophicPx)
                    {
                        // 即使考虑到预测位姿的不确定性，残差依旧远超合理范围，
                        // 判为严重误匹配并上报（无论临时拒绝开关如何都告警）。
                        mylog::warn("[外点剔除] 特征点 #{} 预优化残差 {:.2f} px 超灾难门限 {:.2f} px"
                                    "（预测位姿下仍严重超限），{} "
                                    "(路标=[{:.2f},{:.2f},{:.2f}] 相机=[{:.2f},{:.2f},{:.2f}])",
                                    feat.featureId, residualPx, catastrophicPx, action, ptW.x(), ptW.y(), ptW.z(),
                                    predCamPose.translation().x(), predCamPose.translation().y(),
                                    predCamPose.translation().z());
                    }
                    else
                    {
                        // 措辞随开关变化：关闭预优化拒绝时观测实际会进优化，
                        // 若仍写“暂不参与优化”就是日志与行为相反。
                        mylog::debug("[外点门控] 特征点 #{} 预优化残差 {:.2f} px 超宽门限 {:.2f} px，{}",
                                     feat.featureId, residualPx, config_.maxReprojectionErrorPreopt, action);
                    }

                    // 诊断开关：关闭后仅告警、仍把观测送入优化，用于离线对比
                    // 门控对轨迹的影响（原先该字段被解析但从未生效）。
                    if (config_.preoptTemporaryRejection)
                    {
                        gatedOutThisFrame.insert(feat.featureId);
                    }
                }
            }

            if (!geometricallyInvalid.empty())
            {
                mylog::debug("[外点门控] KF #{}: 本帧 {} 个路标几何无效（相机后方或深度超出 "
                             "[{:.2f}, {:.2f}] m），已剔除其观测",
                             currentKfId, geometricallyInvalid.size(), config_.minLandmarkDepth,
                             config_.maxLandmarkDepth);
            }

            if (outliersDetected > 0)
            {
                mylog::debug("[外点门控] KF #{}: 本帧 {} 个观测预优化残差超限，{}", currentKfId, outliersDetected,
                             config_.preoptTemporaryRejection ? "暂不参与优化" : "仍送入优化（诊断开关已关闭）");
            }

            // 4. 跨帧三角化新路标 (使用优化后的历史位姿与预积分预测的当前位姿，抑制深度系统偏差)
            //    上一关键帧的位姿可能已随滞后窗口边缘化而离开估计：此时 at() 会抛
            //    std::out_of_range。三角化依赖历史位姿，因此这种情况下直接跳过。
            if (currentEstimate_.exists(X(prevKfId)))
            {
                triangulateNewLandmarks(prevKfId, lastKfFrame, frame, currentEstimate_.at<gtsam::Pose3>(X(prevKfId)),
                                        propState.pose());
            }
            else
            {
                mylog::warn("[后端] 上一关键帧 #{} 已不在估计中（已边缘化），跳过本次三角化", prevKfId);
            }

            // 5. 添加视觉重投影误差因子 (GenericProjectionFactor)
            size_t visualFactorCount = 0;
            for (const auto &feat : frame.features)
            {
                // 路标必须仍在图中：既可能是已优化变量 (currentEstimate_)，
                // 也可能是本帧刚三角化的新变量 (newValues_，尚未进入估计)
                const bool landmarkInGraph =
                    currentEstimate_.exists(L(feat.featureId)) || newValues_.exists(L(feat.featureId));
                if (triangulatedLandmarks_.contains(feat.featureId) && !outlierLandmarks_.contains(feat.featureId) &&
                    !gatedOutThisFrame.contains(feat.featureId) && !geometricallyInvalid.contains(feat.featureId) &&
                    landmarkInGraph)
                {
                    newFactors_
                        .emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                            feat.uv, robustPixelNoise_, X(currentKfId), L(feat.featureId), K_, config_.bodyPSensor);
                    landmarkObsCount_[feat.featureId]++;
                    visualFactorCount++;
                }
            }

            // 5.5 视觉失明期间的速度约束 (ZUPT)
            //
            // 视觉因子几乎为零时，因子图里只剩 CombinedImuFactor 链：它约束的是
            // 速度的**变化**，没有任何绝对观测约束速度本身。此时积分误差与零偏
            // 残差会让速度单调累积，位置随时间的平方增长 —— 实测该段轨迹速度在
            // 7 s 内从 0.2 m/s 涨到 18 m/s，而对照原始 IMU 数据，设备当时其实
            // 静止（线性加速度幅值 < 0.03 m/s^2）。
            //
            // 处置：视觉已失去约束能力、且 IMU 判定设备静止时，注入一个速度为零
            // 的先验把速度钉住。两个条件缺一不可——只有“静止”时不加（正常跟踪
            // 时视觉本来就在约束速度，多加先验反而会污染运动中的估计），
            // 只有“视觉失明”时也不加（设备可能真的在动，强行归零就是错的）。
            if (visualFactorCount < config_.minVisualFactorsForHealth && isStationary())
            {
                auto zuptNoise = gtsam::noiseModel::Isotropic::Sigma(3, config_.zuptVelocitySigma);
                // Eigen 的 Zero() 是表达式模板，traits 里需要具体类型，必须先落成变量
                const gtsam::Vector3 zeroVelocity = gtsam::Vector3::Zero();
                newFactors_.addPrior(V(currentKfId), zeroVelocity, zuptNoise);
                ++zuptCount_;
                if (zuptCount_ % 50 == 1)
                {
                    mylog::warn("[零速更新] KF #{}: 视觉仅剩 {} 个观测且 IMU 判定静止，已注入速度为零先验 "
                                "(累计 {} 次)，避免纯 IMU 外推把速度积分到发散",
                                currentKfId, visualFactorCount, zuptCount_);
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

            // 6a. 求解与分级恢复。
            //
            // 求解阶段抛出的 IndeterminateSystemException 意味着 iSAM2 的内部
            // 线性缓存已不可信，原地重试没有意义。旧实现把异常直接抛给上层，
            // 而上层唯一的手段是重建整条流水线 —— 坐标系随之作废，轨迹表现为
            // “跳回原点”。这里改为先做局部恢复：丢掉旧平滑器、以最近一次成功
            // 位姿为锚点重建，保持坐标系连续；本帧不产出新位姿，等下一关键帧
            // 重新三角化即可。只有局部恢复本身失败时才把异常逻辑上交。
            auto tStart = std::chrono::steady_clock::now();
            lastSmootherRebuilt_ = false;
            gtsam::FixedLagSmoother::Result smootherResult;
            try
            {
                smootherResult = smoother_->update(newFactors_, newValues_, timestamps);
                // 额外迭代：同一批因子继续收敛 (空 update 复用当前线性化点做 Gauss-Newton 迭代)
                for (int iter = 0; iter < config_.extraSmootherIterations; ++iter)
                {
                    const auto extra = smoother_->update();
                    smootherResult.keysOfDeletedNodes.insert(extra.keysOfDeletedNodes.begin(),
                                                             extra.keysOfDeletedNodes.end());
                    smootherResult.expiredPendingKeys.insert(extra.expiredPendingKeys.begin(),
                                                             extra.expiredPendingKeys.end());
                }
                currentEstimate_ = smoother_->calculateEstimate();
            }
            catch (const std::exception &error)
            {
                mylog::warn("[后端] KF #{} 平滑器求解失败 ({}), 尝试局部恢复", currentKfId, error.what());

                // 丢弃本帧新增的全部因子与初值：导致奇异的新三角化路标正在其中。
                newFactors_.resize(0);
                newValues_.clear();

                if (!recoverSmoother(timestamp))
                {
                    mylog::error("[后端] KF #{} 局部恢复失败（窗口无可锚定关键帧）", currentKfId);
                    return false;
                }
                lastSmootherRebuilt_ = true;
                ++recoveredFrames_;
                // pim_ 刻意不重置：它从锚点起的积分仍然有效，下一关键帧的
                // CombinedImuFactor 会继续覆盖这段区间，时间轴保持连续。
                return false;
            }
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
                    // 深度越界同样按「硬门限超限」处理：project() 在 z>0 时仍可能
                    // 给出很小的残差，但雅可比已随 1/z 发散，留着它会持续把线性
                    // 系统推向病态。置为 inf 使其在本帧即被永久标记。
                    const double depth = optCamPose.transformTo(ptW).z();
                    residualPx = (depth < config_.minLandmarkDepth || depth > config_.maxLandmarkDepth)
                                     ? std::numeric_limits<double>::infinity()
                                     : (optCamera.project(ptW) - feat.uv).norm();
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

            // 窗口规模诊断：真正的窗口边界由 slidingWindowLag 的时间戳控制，
            // maxSlidingWindowSize 是期望的关键帧数上限。两者脱节时靠边化仍然
            // 正确，但说明配置的时间跨度与预期规模不一致，值得排查。
            if (windowKeyframes_.size() > config_.maxSlidingWindowSize)
            {
                mylog::debug("[滑动窗口] 活跃关键帧 {} 个，超过 maxSlidingWindowSize={}（实际窗口由 "
                             "slidingWindowLag={:.2f}s 决定）",
                             windowKeyframes_.size(), config_.maxSlidingWindowSize, config_.slidingWindowLag);
            }

            // 10. 健康监控（带连续计数滞回，避免单帧噪声触发降级）
            lastHealth_ = healthMonitor_.inspect(latestNavState_, currentBias_, visualFactorCount);
            if (!lastHealth_.isHealthy)
            {
                mylog::warn("[健康报警] {}", lastHealth_.alertMessage);
            }

            return true;
        }

        /**
         * @brief 基于近期 IMU 样本判定设备是否静止（供 ZUPT 使用）
         *
         * 判据与初始化器的静态检测保持一致，只是作用在滑动窗口而非初始化样本集上：
         *   - 加速度与角速度的样本标准差都足够小；
         *   - 加速度模长接近重力（排除有线性加速度的匀速运动）。
         *
         * 门限随噪声配置缩放，而不是硬编码：单样本标准差为
         * sigma / sqrt(dt) = sigma * sqrt(imuRateHz)，取 5 倍并保留工业下限。
         *
         * @return 窗口内样本一致且无线性加速时返回 @c true
         */
        bool isStationary() const
        {
            // 样本太少时统计量不可信，宁可判为“非静止”而不施加错误约束
            if (recentImu_.size() < 10)
            {
                return false;
            }

            gtsam::Vector3 meanAcc = gtsam::Vector3::Zero();
            gtsam::Vector3 meanGyro = gtsam::Vector3::Zero();
            for (const auto &sample : recentImu_)
            {
                meanAcc += sample.accel;
                meanGyro += sample.gyro;
            }
            const double invCount = 1.0 / static_cast<double>(recentImu_.size());
            meanAcc *= invCount;
            meanGyro *= invCount;

            gtsam::Vector3 varAcc = gtsam::Vector3::Zero();
            gtsam::Vector3 varGyro = gtsam::Vector3::Zero();
            for (const auto &sample : recentImu_)
            {
                varAcc += (sample.accel - meanAcc).cwiseAbs2();
                varGyro += (sample.gyro - meanGyro).cwiseAbs2();
            }
            varAcc *= invCount;
            varGyro *= invCount;

            const double accStd = std::sqrt(varAcc.sum() / 3.0);
            const double gyroStd = std::sqrt(varGyro.sum() / 3.0);

            const double rateScale = std::sqrt(std::max(1.0, config_.imuRateHz));
            const double accThreshold = std::max(0.3, 5.0 * config_.accelNoiseSigma * rateScale);
            const double gyroThreshold = std::max(0.05, 5.0 * config_.gyroNoiseSigma * rateScale);

            if (accStd > accThreshold || gyroStd > gyroThreshold)
            {
                return false;
            }

            // 有线性加速度的匀速直线运动同样“方差小”，但加速度模长会偏离重力。
            // 只靠方差判定会把这类运动误钉成静止，因此再加一道模长检验。
            // 容差同时随噪声缩放：3 倍单样本标准差，至少 0.2 m/s^2。
            const double accNormTolerance = std::max(0.2, 3.0 * config_.accelNoiseSigma * rateScale);
            return std::abs(meanAcc.norm() - config_.gravityMagnitude) < accNormTolerance;
        }

        /**
         * @brief 求解失败后重建平滑器，保持坐标系连续
         *
         * 以滑动窗口内最后一个成功关键帧为锚点，用它已收敛的位姿/速度/偏置
         * 重新注入先验。锚点的坐标系与失败前完全一致，因此轨迹不会跳回原点。
         * 代价是丢弃窗口内的历史关键帧与全部路标：它们需要后续观测重新建立，
         * 但换来的是一条连续的轨迹而非一个作废的坐标系。
         *
         * @param timestamp 锚点时间戳，参与固定滞后计时
         * @return 是否恢复成功；窗口为空时无法锚定，返回 @c false
         */
        bool recoverSmoother(double timestamp)
        {
            if (windowKeyframes_.empty())
            {
                return false;
            }

            const int anchor = windowKeyframes_.back();
            gtsam::Pose3 anchorPose = latestNavState_.pose();
            gtsam::Vector3 anchorVel = latestNavState_.velocity();
            if (currentEstimate_.exists(X(anchor)) && currentEstimate_.exists(V(anchor)))
            {
                anchorPose = currentEstimate_.at<gtsam::Pose3>(X(anchor));
                anchorVel = currentEstimate_.at<gtsam::Vector3>(V(anchor));
            }

            // 旧平滑器在求解异常后内部线性缓存已不可信，直接丢弃重建
            smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.slidingWindowLag, isamParams_);

            // 恢复先验比初始化先验宽：这是“已知大致位姿”的弱约束而不是真值，
            // 收紧会让后续观测难以纠正累积误差。
            auto poseNoise =
                gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2).finished());
            auto velNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.2);
            auto biasNoise =
                gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());

            gtsam::NonlinearFactorGraph anchorFactors;
            gtsam::Values anchorValues;
            gtsam::FixedLagSmoother::KeyTimestampMap anchorTimestamps;

            anchorFactors.addPrior(X(anchor), anchorPose, poseNoise);
            anchorFactors.addPrior(V(anchor), anchorVel, velNoise);
            anchorFactors.addPrior(B(anchor), currentBias_, biasNoise);
            anchorValues.insert(X(anchor), anchorPose);
            anchorValues.insert(V(anchor), anchorVel);
            anchorValues.insert(B(anchor), currentBias_);
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
                mylog::error("[后端] 重建平滑器失败: {}", error.what());
                return false;
            }

            // 窗口只剩锚点：历史关键帧与路标已随旧平滑器一并丢弃。
            // 消掉这些登记，避免后续帧引用已不存在的变量。
            windowKeyframes_.clear();
            windowKeyframes_.push_back(anchor);
            triangulatedLandmarks_.clear();
            outlierLandmarks_.clear();
            outlierStreak_.clear();
            landmarkObsCount_.clear();
            // 健康滞回计数属于已丢弃的那段图，清零后重新评估
            healthMonitor_.reset();

            mylog::info("[后端恢复] 平滑器已重建: 锚点 KF #{} P=[{:.3f}, {:.3f}, {:.3f}]，坐标系保持连续", anchor,
                        anchorPose.x(), anchorPose.y(), anchorPose.z());
            return true;
        }

        /// 因不定系统而重建平滑器的累计次数（供上层诊断）
        std::size_t recoveredFrames() const { return recoveredFrames_; }

        /// 零速更新（ZUPT）累计生效次数
        std::size_t zuptCount() const { return zuptCount_; }

        /// 平滑器是否因不定系统故障被重建过（供上层诊断与日志）
        bool lastSmootherRebuilt() const { return lastSmootherRebuilt_; }

        const gtsam::NavState &latestNavState() const { return latestNavState_; }
        const gtsam::imuBias::ConstantBias &latestBias() const { return currentBias_; }

        /**
         * @brief 最近一次关键帧之后的健康结论
         *
         * 上层据此把 @c SystemState 推进到 @c LOST_DEGRADED 并触发流水线重建。
         */
        const HealthMonitor::HealthReport &health() const { return lastHealth_; }

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

        gtsam::ISAM2Params isamParams_; ///< 平滑器参数：重建 smoother_ 时必须复用同一套设置
        std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_; ///< 固定滞后平滑器 (滑动窗口 + 真边缘化)
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

        HealthMonitor healthMonitor_;            ///< 带滞回的健康监控
        HealthMonitor::HealthReport lastHealth_; ///< 最近一次关键帧后的健康结论

        /// 外点判定滞回阈值：连续超限达到该帧数才标记，防止单帧噪声误杀
        static constexpr int kOutlierStreakThreshold = 3;

        /// 最近一次 processKeyframe 是否因不定系统而重建了平滑器
        bool lastSmootherRebuilt_{false};
        /// 累计重建次数：上层据此判断该段轨迹是否经历过局部恢复
        std::size_t recoveredFrames_{0};
        /// ZUPT 使用的近期 IMU 样本窗口（按时间裁剪）
        std::deque<ImuMeasurement> recentImu_;
        /// 零速更新累计生效次数（供诊断）
        std::size_t zuptCount_{0};
    };

} // namespace myslam::vio
