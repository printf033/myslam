#pragma once

/**
 * @file system.hpp
 * @brief VIO 系统运行编排（双速率主循环）与平移精度评估。
 */

#include "logger.hpp"
#include "myslam.hpp"
#include "vio/backend.hpp"
#include "vio/health_monitor.hpp"
#include "vio/initializer.hpp"
#include "vio/keyframe_selector.hpp"
#include "vio/params.hpp"
#include "vio/propagator.hpp"
#include "vio/simulator.hpp"
#include <cmath>
#include <cstddef>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <optional>
#include <utility>
#include <vector>

namespace myslam::vio
{

    // =========================================================================
    // 9. VIO 系统运行编排 (Sensor Loop, Evaluation)
    // =========================================================================

    /**
     * @brief 系统运行配置：仿真时序与验收门限
     *
     * 不保存 IMU 采样步长：步长与 IMU 采样率是同一物理量的两种表述，
     * 采样率的唯一来源是 @c VioConfig::imuRateHz（噪声模型离散化同样依赖它）。
     * 因此这里只提供派生规则，保证两者永不脱节。
     */
    struct SystemConfig
    {
        double cameraDt{0.050};     ///< 相机采样步长 (s)，对应 20 Hz
        double totalDuration{5.0};  ///< 仿真/运行时长 (s)
        double rmseThreshold{0.20}; ///< 平移 RMSE 验收门限 (m)

        /**
         * @brief 由 IMU 采样率派生采样步长
         *
         * @param imuRateHz IMU 采样率 (Hz)，取自 @c VioConfig::imuRateHz
         * @return double 采样步长 (s)；采样率非正时返回 0，由调用方判定非法
         */
        [[nodiscard]] static constexpr double imuDtFrom(double imuRateHz) noexcept
        {
            return imuRateHz > 0.0 ? 1.0 / imuRateHz : 0.0;
        }
    };

    /**
     * @brief 一次完整运行的精度报告
     */
    struct RunReport
    {
        size_t keyframeCount{0}; ///< 参与评估的关键帧样本数
        double rmse{0.0};        ///< 整体平移 RMSE (m)
        double maxError{0.0};    ///< 最大平移偏差 (m)
    };

    /**
     * @brief VIO 最小系统的运行编排器
     *
     * 组合初始化器、关键帧决策器、紧耦合后端、高频外推器与仿真器，
     * 驱动 IMU (200 Hz) / 相机 (20 Hz) 双速率主循环，并在结束时输出
     * 与真值对齐的平移精度评估。使用者只需构造配置并调用 run()。
     */
    class VioSystem
    {
    public:
        explicit VioSystem(const VioConfig &config = {}, const SystemConfig &systemConfig = {})
            : config_(config),
              systemConfig_(systemConfig),
              initializer_(config),
              keyframeSelector_(config),
              backend_(config),
              propagator_(config.gravityMagnitude),
              simulator_(config)
        {
        }

        /**
         * @brief 执行完整传感循环并返回精度报告
         */
        RunReport run()
        {
            mylog::info("================================================================================");
            mylog::info("🚀 启动 {} v{} - Visual-Inertial Odometry (VIO) 紧耦合最小系统", kProjectName, kVersion);
            mylog::info("================================================================================");
            mylog::info("[配置参数] 相机内参: fx={:.1f}, fy={:.1f}, cx={:.1f}, cy={:.1f}, 噪声={:.1f}px", config_.fx,
                        config_.fy, config_.cx, config_.cy, config_.pixelNoiseSigma);
            mylog::info("[配置参数] IMU 特性: 加速度白噪声={:.4f}, 陀螺白噪声={:.5f}, 滑动窗口规模={}",
                        config_.accelNoiseSigma, config_.gyroNoiseSigma, config_.maxSlidingWindowSize);

            const double imuDt = SystemConfig::imuDtFrom(config_.imuRateHz);
            const double camDt = systemConfig_.cameraDt;

            double currentTime = 0.0;
            double nextCamTime = 0.0;
            int camFrameId = 0;
            int keyframeId = 0;

            std::optional<CameraFrame> lastKeyframeFrame;
            std::vector<std::pair<double, gtsam::Point3>> estimatedKfTrajectory;
            std::vector<std::pair<double, gtsam::Point3>> groundTruthTrajectory;

            mylog::info("⏳ 进入多传感器融合主循环 (IMU: 200Hz, Camera: 20Hz)...");

            while (currentTime <= systemConfig_.totalDuration)
            {
                // 步骤 A: 接收 IMU 读数并预积分 / 高频外推
                ImuMeasurement imuSample = simulator_.generateImu(currentTime);

                if (initializer_.state() != SystemState::TRACKING_OK)
                {
                    initializer_.feedImu(imuSample);
                }
                else
                {
                    backend_.integrateImu(imuSample, imuDt);
                    gtsam::NavState highRateNav = propagator_.propagate(imuSample);

                    if (static_cast<int>(currentTime * 200) % 80 == 0)
                    {
                        mylog::debug(
                            "[200Hz实时推算] t={:.3f}s: P=[{:.3f}, {:.3f}, {:.3f}], V=[{:.3f}, {:.3f}, {:.3f}]",
                            currentTime, highRateNav.pose().x(), highRateNav.pose().y(), highRateNav.pose().z(),
                            highRateNav.v()(0), highRateNav.v()(1), highRateNav.v()(2));
                    }
                }

                // 步骤 B: 图像帧到达
                if (currentTime >= nextCamTime - 1e-6)
                {
                    CameraFrame camFrame = simulator_.generateCameraFrame(camFrameId++, currentTime);
                    auto gtState = simulator_.evaluateGroundTruth(currentTime);

                    if (initializer_.state() == SystemState::STATIONARY_COLLECTING)
                    {
                        // 阶段 1: 初始化完成则注入首帧先验
                        gtsam::Rot3 initRWb;
                        gtsam::imuBias::ConstantBias initBias;
                        if (initializer_.tryGravityAlignment(initRWb, initBias))
                        {
                            gtsam::Pose3 initPose(initRWb, gtsam::Point3(0.0, 0.0, 1.0));
                            gtsam::Vector3 initVel = gtsam::Vector3::Zero();

                            backend_.initializeSystem(keyframeId, currentTime, initPose, initVel, initBias);
                            propagator_.resetWithOptimizedState(currentTime, gtsam::NavState(initPose, initVel),
                                                                initBias);

                            initializer_.setState(SystemState::TRACKING_OK);
                            lastKeyframeFrame = camFrame;

                            estimatedKfTrajectory.emplace_back(currentTime, initPose.translation());
                            groundTruthTrajectory.emplace_back(currentTime, gtState.pose.translation());
                            keyframeId++;
                        }
                    }
                    else if (initializer_.state() == SystemState::TRACKING_OK)
                    {
                        // 阶段 2: 紧耦合追踪与关键帧决策
                        auto decision = keyframeSelector_.evaluate(camFrame, lastKeyframeFrame);

                        if (decision.isKeyframe)
                        {
                            mylog::info(
                                "--------------------------------------------------------------------------------");
                            mylog::info("[关键帧判定] 图像帧 #{} 触发关键帧 #{} -> 判定原因: {}", camFrame.frameId,
                                        keyframeId, decision.reason);

                            backend_.processKeyframe(keyframeId, currentTime, camFrame, *lastKeyframeFrame);
                            propagator_.resetWithOptimizedState(currentTime, backend_.latestNavState(),
                                                                backend_.latestBias());

                            lastKeyframeFrame = camFrame;

                            gtsam::Point3 estPos = backend_.latestNavState().pose().translation();
                            gtsam::Point3 gtPos = gtState.pose.translation();
                            double transErr = (estPos - gtPos).norm();

                            mylog::info("[位姿对比] KF #{}: 估计=[{:.3f}, {:.3f}, {:.3f}], 真值=[{:.3f}, {:.3f}, "
                                        "{:.3f}], 绝对误差={:.3f}m",
                                        keyframeId, estPos.x(), estPos.y(), estPos.z(), gtPos.x(), gtPos.y(), gtPos.z(),
                                        transErr);

                            estimatedKfTrajectory.emplace_back(currentTime, estPos);
                            groundTruthTrajectory.emplace_back(currentTime, gtPos);

                            keyframeId++;
                        }
                        else
                        {
                            mylog::debug("[跳过非关键帧] 图像帧 #{}: 视差={:.2f}px, 共视率={:.1f}%", camFrame.frameId,
                                         decision.avgParallax, decision.trackingRatio * 100.0);
                        }
                    }

                    nextCamTime += camDt;
                }

                currentTime += imuDt;
            }

            return evaluate(estimatedKfTrajectory, groundTruthTrajectory);
        }

    private:
        /**
         * @brief 以真值为基准评估平移精度 (RMSE / 最大偏差)
         */
        RunReport evaluate(const std::vector<std::pair<double, gtsam::Point3>> &estimated,
                           const std::vector<std::pair<double, gtsam::Point3>> &groundTruth) const
        {
            mylog::info("================================================================================");
            mylog::info("📊 VIO 最小系统工业精度全面评测报告");
            mylog::info("================================================================================");

            RunReport report;
            if (estimated.size() != groundTruth.size() || estimated.empty())
            {
                mylog::warn("⚠️ 评估样本缺失或不匹配: 估计 {} 帧, 真值 {} 帧", estimated.size(), groundTruth.size());
                return report;
            }

            double sumSqError = 0.0;
            for (size_t i = 0; i < estimated.size(); ++i)
            {
                double err = (estimated[i].second - groundTruth[i].second).norm();
                sumSqError += err * err;
                if (err > report.maxError)
                {
                    report.maxError = err;
                }
            }

            report.keyframeCount = estimated.size();
            report.rmse = std::sqrt(sumSqError / static_cast<double>(estimated.size()));

            mylog::info("评估关键帧样本数 : {}", report.keyframeCount);
            mylog::info("整体平移 RMSE 误差: {:.4f} 米", report.rmse);
            mylog::info("最大轨迹偏差       : {:.4f} 米", report.maxError);

            if (report.rmse < systemConfig_.rmseThreshold)
            {
                mylog::info("🎉 系统性能表现卓越: 精度指标完全达到工业级紧耦合 VIO 验收标准！");
            }
            else
            {
                mylog::warn("⚠️ 轨迹误差略微偏高，可适当调优噪声协方差与 Huber 阈值。");
            }

            return report;
        }

        VioConfig config_;
        SystemConfig systemConfig_;
        VioInitializer initializer_;
        KeyframeSelector keyframeSelector_;
        VioBackend backend_;
        HighRatePropagator propagator_;
        VioSimulator simulator_;
    };

} // namespace myslam::vio
