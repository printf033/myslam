#pragma once

/**
 * @file system.hpp
 * @brief VIO 系统运行编排（双速率主循环）与平移精度评估。
 */

#include "logger.hpp"
#include "myslam.hpp"
#include "vio/backend.hpp"
#include "vio/configHolder.hpp"
#include "vio/health_monitor.hpp"
#include "vio/initializer.hpp"
#include "vio/keyframe_selector.hpp"
#include "vio/propagator.hpp"
#include "vio/simulator.hpp"
#include <cmath>
#include <cstddef>
#include <exception>
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
        /// 初始化超时 (s)：超过该时刻仍未通过静止检测则放宽为“非静止对齐”。
        /// 与数据集回放的 initDeadline 语义一致——设备可能从运动状态开始，
        /// 死等静止会让整个运行停在未初始化状态。
        /// 取 0（或负值）表示不超时：严格等待静止条件，永不放宽。
        double initTimeout{2.0};

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
        /// 是否真正完成过初始化并产出关键帧。为 false 时 @c rmse 无意义
        /// （保持 0），调用方不得据此判定“精度达标”。
        bool initialized{false};
        /// 因健康异常或优化异常导致的流水线重建次数
        size_t resets{0};
    };

    /**
     * @brief 验收判定：本次运行是否达到精度门限
     *
     * 必须同时检查 @c initialized：未初始化时 @c rmse 保持 0，
     * 只看 @c rmse < threshold 会把“零关键帧”误判为“完美精度”。
     * 该逻辑抽成自由函数以保证调用方与被测行为是同一份代码。
     *
     * 同时还要求至少两个位姿样本：初始化成功时会无条件压入一条与真值几乎
     * 重合的首帧位姿，只有它一条时 @c rmse 也会接近 0，但此时根本没有轨迹
     * 可言（零次优化）。
     *
     * @param report 运行报告
     * @param rmseThreshold 平移 RMSE 门限 (m)
     * @return true 已完成初始化且 RMSE 在门限内
     */
    [[nodiscard]] inline bool isAcceptable(const RunReport &report, double rmseThreshold) noexcept
    {
        // 至少要有两个时刻不同的位姿才能谈“轨迹精度”：初始化时无条件压入的
        // 首帧位姿是一个与真值几乎重合的锚点，若只要求 keyframeCount > 0，
        // 那么“初始化完成、一次优化都没做”的运行会以 rmse≈0 冒充成功。
        constexpr size_t kMinEvaluableSamples = 2;
        return report.initialized && report.keyframeCount >= kMinEvaluableSamples && report.rmse < rmseThreshold;
    }

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
              propagator_(config.gravityMagnitude, config.imuRateHz),
              simulator_(config)
        {
            // 后端持有 unique_ptr 与 gtsam 平滑器，用 optional 延迟构造以便
            // 运行期整体重建（见 resetPipeline）。
            backend_.emplace(config);
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
            if (!(imuDt > 0.0))
            {
                mylog::error("[配置错误] vio.imuRateHz = {} 非正，无法派生 IMU 采样步长", config_.imuRateHz);
                return {};
            }
            const double camDt = systemConfig_.cameraDt;

            double currentTime = 0.0;
            double nextCamTime = 0.0;
            int camFrameId = 0;
            int keyframeId = 0;
            /// 超过该时刻仍未完成初始化时放宽静止检测；-1 表示尚未开始计时
            double initDeadline = -1.0;
            /// 高频推算日志的节流计数（按采样率派生，避免浮点取模漏打印）
            const long long logStride = std::max<long long>(1, std::llround(0.4 * std::max(1.0, config_.imuRateHz)));
            long long imuSampleIndex = 0;

            bool initialized = false;
            size_t resets = 0;

            std::optional<CameraFrame> lastKeyframeFrame;
            std::vector<std::pair<double, gtsam::Point3>> estimatedKfTrajectory;
            std::vector<std::pair<double, gtsam::Point3>> groundTruthTrajectory;

            /**
             * @brief 丢弃当前流水线的全部状态并重新开始初始化
             *
             * 各组件之间持有互相引用的状态（窗口、先验、预积分基线），
             * 局部修复无法保证一致，因此整体重建。关键帧编号继续递增，
             * 避免日志与外部记录出现重号。
             */
            auto resetPipeline = [&](const char *reason, bool degraded)
            {
                // LOST_DEGRADED 必须先显式置位并记录，否则它只是一个枚举值：
                // 下面的 initializer_ 重建会直接回到 UNINITIALIZED，状态转移
                // 变成“从没发生过”，日志里也看不到降级事实。
                if (degraded)
                {
                    initializer_.setState(SystemState::LOST_DEGRADED);
                    mylog::error("[状态转换] {} -> {} -> {}: {}，重建流水线",
                                 systemStateToString(SystemState::TRACKING_OK),
                                 systemStateToString(SystemState::LOST_DEGRADED),
                                 systemStateToString(SystemState::UNINITIALIZED), reason);
                }
                else
                {
                    mylog::warn("[流水线重置] {}，丢弃当前状态并重新初始化", reason);
                }
                initializer_ = VioInitializer(config_);
                backend_.emplace(config_);
                propagator_ = HighRatePropagator(config_.gravityMagnitude, config_.imuRateHz);
                lastKeyframeFrame.reset();
                initDeadline = -1.0;
                ++resets;
            };

            mylog::info("⏳ 进入多传感器融合主循环 (IMU: {:.0f}Hz, Camera: {:.0f}Hz)...", config_.imuRateHz,
                        1.0 / camDt);

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
                    backend_->integrateImu(imuSample, imuDt);
                    gtsam::NavState highRateNav = propagator_.propagate(imuSample);

                    if (imuSampleIndex % logStride == 0)
                    {
                        mylog::debug(
                            "[200Hz实时推算] t={:.3f}s: P=[{:.3f}, {:.3f}, {:.3f}], V=[{:.3f}, {:.3f}, {:.3f}]",
                            currentTime, highRateNav.pose().x(), highRateNav.pose().y(), highRateNav.pose().z(),
                            highRateNav.v()(0), highRateNav.v()(1), highRateNav.v()(2));
                    }
                }
                ++imuSampleIndex;

                // 步骤 B: 图像帧到达
                if (currentTime >= nextCamTime - 1e-6)
                {
                    CameraFrame camFrame = simulator_.generateCameraFrame(camFrameId++, currentTime);
                    auto gtState = simulator_.evaluateGroundTruth(currentTime);

                    if (initializer_.state() == SystemState::STATIONARY_COLLECTING)
                    {
                        // 阶段 1: 静止检测与重力对齐。超过 initTimeout 后放宽静止
                        // 判据，避免设备从运动状态开始时永久卡在初始化阶段。
                        // initTimeout <= 0 表示不超时（严格等静止），此时 relaxed
                        // 永远为 false——不能直接算 currentTime + 0，否则第一帧就会
                        // 放宽，与“不超时”的意图完全相反。
                        bool relaxed = false;
                        if (systemConfig_.initTimeout > 0.0)
                        {
                            if (initDeadline < 0.0)
                            {
                                initDeadline = currentTime + systemConfig_.initTimeout;
                            }
                            relaxed = currentTime >= initDeadline;
                        }
                        gtsam::Rot3 initRWb;
                        gtsam::imuBias::ConstantBias initBias;
                        if (initializer_.tryGravityAlignment(initRWb, initBias, !relaxed))
                        {
                            mylog::info("[初始化] 重力对齐完成{}，等待注入首帧先验",
                                        relaxed ? "（已超出超时，采用非静止对齐）" : "");
                            // 状态已推进到 GRAVITY_ALIGNED，由下面的分支统一消费
                        }
                    }

                    if (initializer_.state() == SystemState::GRAVITY_ALIGNED)
                    {
                        // 阶段 2: 注入首帧先验并进入追踪。
                        // 世界系原点取初始 IMU 位置，z 偏移 1.0 m 是仿真器真值
                        // 轨迹的初始高度（@c VioSimulator::evaluateGroundTruth），
                        // 使估计轨迹可直接与真值比较。
                        gtsam::Pose3 initPose(initializer_.alignedRotation(), gtsam::Point3(0.0, 0.0, 1.0));
                        gtsam::Vector3 initVel = gtsam::Vector3::Zero();

                        backend_->initializeSystem(keyframeId, currentTime, initPose, initVel,
                                                   initializer_.alignedBias());
                        propagator_.resetWithOptimizedState(currentTime, gtsam::NavState(initPose, initVel),
                                                            initializer_.alignedBias());

                        initializer_.setState(SystemState::TRACKING_OK);
                        lastKeyframeFrame = camFrame;
                        initialized = true;

                        estimatedKfTrajectory.emplace_back(currentTime, initPose.translation());
                        groundTruthTrajectory.emplace_back(currentTime, gtState.pose.translation());
                        keyframeId++;
                    }
                    else if (initializer_.state() == SystemState::TRACKING_OK)
                    {
                        // 阶段 3: 紧耦合追踪与关键帧决策
                        auto decision = keyframeSelector_.evaluate(camFrame, lastKeyframeFrame);

                        if (decision.isKeyframe && lastKeyframeFrame.has_value())
                        {
                            mylog::info(
                                "--------------------------------------------------------------------------------");
                            mylog::info("[关键帧判定] 图像帧 #{} 触发关键帧 #{} -> 判定原因: {}", camFrame.frameId,
                                        keyframeId, decision.reason);

                            // 后端内部涉及 gtsam 求解，任何异常（不定系统、数值失败、
                            // 变量缺失）都必须在这里截住：VioSystem::run() 没有上层
                            // 调用者可以兜住它，异常穿出会直接 terminate 进程。
                            //
                            // processKeyframe 返回 false 表示求解失败但后端已完成局部恢复
                            // （平滑器重建、坐标系未变），此时不应重建流水线，也不应
                            // 把旧位姿当成本帧估计做误差统计。
                            bool optimized = false;
                            try
                            {
                                optimized =
                                    backend_->processKeyframe(keyframeId, currentTime, camFrame, *lastKeyframeFrame);
                            }
                            catch (const std::exception &error)
                            {
                                mylog::error("[关键帧优化失败] KF #{}: {}，重置流水线", keyframeId, error.what());
                                resetPipeline("后端优化抛出异常", /*degraded=*/false);
                            }

                            if (optimized)
                            {
                                propagator_.resetWithOptimizedState(currentTime, backend_->latestNavState(),
                                                                    backend_->latestBias());

                                gtsam::Point3 estPos = backend_->latestNavState().pose().translation();
                                gtsam::Point3 gtPos = gtState.pose.translation();
                                double transErr = (estPos - gtPos).norm();

                                mylog::info("[位姿对比] KF #{}: 估计=[{:.3f}, {:.3f}, {:.3f}], 真值=[{:.3f}, "
                                            "{:.3f}, {:.3f}], 绝对误差={:.3f}m",
                                            keyframeId, estPos.x(), estPos.y(), estPos.z(), gtPos.x(), gtPos.y(),
                                            gtPos.z(), transErr);

                                estimatedKfTrajectory.emplace_back(currentTime, estPos);
                                groundTruthTrajectory.emplace_back(currentTime, gtPos);
                                lastKeyframeFrame = camFrame;
                                keyframeId++;

                                // 健康监控：连续异常才降级，避免单帧抖动触发重建
                                if (!backend_->health().isHealthy)
                                {
                                    mylog::error("[健康降级] {}", backend_->health().alertMessage);
                                    resetPipeline("健康监控判定系统退化", /*degraded=*/true);
                                }
                            }
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

            if (!initialized)
            {
                mylog::error("[运行结束] 仿真时长 {:.2f}s 内始终未完成初始化（IMU 样本不足或静止判据未满足），"
                             "本次运行没有可评估的轨迹",
                             systemConfig_.totalDuration);
            }

            return evaluate(estimatedKfTrajectory, groundTruthTrajectory, initialized, resets);
        }

    private:
        /**
         * @brief 以真值为基准评估平移精度 (RMSE / 最大偏差)
         *
         * 样本为空时保留 @c rmse = 0，但把 @c initialized 置为 false，
         * 因此调用方不能用“rmse 小于门限”来判定成功——必须同时检查
         * @c RunReport::initialized。
         */
        RunReport evaluate(const std::vector<std::pair<double, gtsam::Point3>> &estimated,
                           const std::vector<std::pair<double, gtsam::Point3>> &groundTruth, bool initialized,
                           size_t resets) const
        {
            mylog::info("================================================================================");
            mylog::info("📊 VIO 最小系统工业精度全面评测报告");
            mylog::info("================================================================================");

            RunReport report;
            report.initialized = initialized;
            report.resets = resets;

            if (estimated.size() != groundTruth.size() || estimated.empty())
            {
                mylog::warn("⚠️ 评估样本缺失或不匹配: 估计 {} 帧, 真值 {} 帧", estimated.size(), groundTruth.size());
                return report;
            }
            // 只有初始化锚点一条样时，rmse 会接近 0 但根本没有轨迹（零次优化），
            // 必须明确说明“不可评估”，而不是让它被当成高精度结果。
            if (estimated.size() < 2)
            {
                mylog::warn("⚠️ 仅 {} 个位姿样本，轨迹不可评估（至少需要 2 个）", estimated.size());
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
            if (report.resets > 0)
            {
                mylog::warn("运行期间发生 {} 次流水线重建，RMSE 包含各段独立坐标系下的误差", report.resets);
            }

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
        std::optional<VioBackend> backend_;
        HighRatePropagator propagator_;
        VioSimulator simulator_;
    };

} // namespace myslam::vio
