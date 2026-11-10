#pragma once

/**
 * @file benchmark.hpp
 * @brief 带真值的批处理运行与精度评估。
 *
 * 消费任意 @c ISensorSource，按 IMU / 相机双速率推进 @c VioPipeline，把每个
 * 关键帧位姿与真值配对，产出平移 RMSE 与最大偏差。
 *
 * 数据源不提供真值时只运行不评估。真实记录回放见 @c app/replay.hpp。
 */

#include "adapters/register_builtin.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include "myslam.hpp"
#include "pipeline/pipeline.hpp"
#include "sensor/simulator.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace myslam::vio
{

    /**
     * @brief 跑完整条流水线并与真值比对
     */
    class VioBenchmark
    {
    public:
        /**
         * @brief 用内置仿真器运行
         *
         * 仿真器的观测模型取自流水线（估计器声明的那一个），而不是按内参
         * 自行推断。MEI 内参下两者不同：估计器（gtsam）用折算后的虚拟针孔，
         * 若仿真器仍按原始 MEI 投影产生像素，全部重投影都会超出硬门限，
         * 视觉约束被静默丢光。
         */
        explicit VioBenchmark(const domain::PipelineConfig &config = {}, const domain::SystemConfig &system = {})
            : VioBenchmark(config, system, nullptr)
        {
        }

        VioBenchmark(const domain::PipelineConfig &config, const domain::SystemConfig &system,
                     std::unique_ptr<domain::ISensorSource> source, domain::EstimatorFactory factory = {})
            : config_(config),
              system_(system),
              pipeline_(config, prepareFactory(std::move(factory))),
              source_(source ? std::move(source) : makeDefaultSource(config_, pipeline_))
        {
        }

        /**
         * @brief 执行传感循环并返回精度报告
         */
        domain::RunReport run()
        {
            mylog::info("================================================================================");
            mylog::info("🚀 启动 {} v{} - Visual-Inertial Odometry (VIO) 紧耦合最小系统", kProjectName, kVersion);
            mylog::info("================================================================================");

            if (!pipeline_.valid())
            {
                mylog::error("[装配错误] 流水线无效: {}", pipeline_.error());
                return {};
            }

            mylog::info("[配置参数] 相机内参({}) 观测模型={}, IMU 采样率={:.0f}Hz, 估计器={}",
                        camera::cameraModelName(config_.camera.intrinsics.model), pipeline_.observationModel()->name(),
                        config_.imu.rateHz, pipeline_.estimator()->name());

            const double imuDt = domain::SystemConfig::imuDtFrom(config_.imu.rateHz);
            if (!(imuDt > 0.0))
            {
                mylog::error("[配置错误] imu.rateHz = {} 非正，无法派生 IMU 采样步长", config_.imu.rateHz);
                return {};
            }
            const double camDt = system_.cameraDt;
            if (!(camDt > 0.0))
            {
                mylog::error("[配置错误] system.cameraDt = {} 非正，无法推进相机时间轴", camDt);
                return {};
            }
            if (imuDt > camDt)
            {
                mylog::error("[配置错误] IMU 步长 {:.6f}s 大于相机步长 {:.6f}s，相机帧时间戳会重复", imuDt, camDt);
                return {};
            }
            if (!source_)
            {
                mylog::error("[配置错误] 传感器源为空，无法运行");
                return {};
            }

            double currentTime = 0.0;
            double nextCamTime = 0.0;
            int camFrameId = 0;
            domain::KeyframeId keyframeId = 0;
            const long long logStride = std::max<long long>(1, std::llround(0.4 * std::max(1.0, config_.imu.rateHz)));
            long long imuSampleIndex = 0;

            bool initialized = false;
            bool sawAnyGroundTruth = false;
            std::size_t skippedNoTruth = 0;
            std::vector<TrajectorySample> samples;

            mylog::info("⏳ 进入多传感器融合主循环 (IMU: {:.0f}Hz, Camera: {:.0f}Hz)...", config_.imu.rateHz,
                        1.0 / camDt);

            while (currentTime <= system_.totalDuration)
            {
                const domain::ImuSample imuSample = source_->imuAt(currentTime);
                if (const auto highRate = pipeline_.feedImu(domain::ImuSlice{imuSample, imuDt});
                    highRate.has_value() && imuSampleIndex % logStride == 0)
                {
                    mylog::debug("[高频实时推算] t={:.3f}s: P=[{:.3f}, {:.3f}, {:.3f}], V=[{:.3f}, {:.3f}, {:.3f}]",
                                 currentTime, highRate->pose.t.x(), highRate->pose.t.y(), highRate->pose.t.z(),
                                 highRate->velocity.x(), highRate->velocity.y(), highRate->velocity.z());
                }
                ++imuSampleIndex;

                if (currentTime >= nextCamTime - 1e-6)
                {
                    nextCamTime += camDt;

                    domain::Frame frame = source_->frameAt(camFrameId++, currentTime);
                    const std::optional<domain::GroundTruth> truth = source_->groundTruthAt(currentTime);
                    sawAnyGroundTruth = sawAnyGroundTruth || truth.has_value();

                    if (!pipeline_.tracking())
                    {
                        const InitOutcome init = pipeline_.stepInitialization(keyframeId, frame, currentTime);
                        if (init.aligned)
                        {
                            mylog::info("[初始化] 重力对齐完成{}，等待注入首帧先验",
                                        init.relaxed ? "（已超出超时，采用非静止对齐）" : "");
                        }
                        if (init.tracking && init.initialPose)
                        {
                            initialized = true;
                            ++keyframeId;
                            recordSample(samples, currentTime, init.initialPose->t, truth, skippedNoTruth);
                        }
                    }
                    else
                    {
                        const FrameOutcome outcome = pipeline_.processFrame(keyframeId, frame, currentTime);

                        if (outcome.kind == FrameOutcome::Kind::KEYFRAME_FAILED)
                        {
                            mylog::error(
                                "[关键帧优化失败] KF #{}: {}，按建议处置 ({})", keyframeId, outcome.failureReason,
                                outcome.recovery == domain::RecoveryAction::RESET_PIPELINE ? "重建流水线"
                                                                                           : "估计器建议的恢复动作");
                            pipeline_.reset(outcome.recovery);
                        }
                        else if (outcome.decision.isKeyframe)
                        {
                            mylog::info(
                                "--------------------------------------------------------------------------------");
                            mylog::info("[关键帧判定] 图像帧 #{} 触发关键帧 #{} -> 判定原因: {}", frame.index,
                                        keyframeId, outcome.decision.reason);

                            if (outcome.kind == FrameOutcome::Kind::KEYFRAME_RECOVERED)
                            {
                                mylog::warn("[关键帧局部恢复] KF #{} 不出位姿，坐标系保持连续", keyframeId);
                            }
                            else if (outcome.producedPose())
                            {
                                const domain::Vec3 estimated = outcome.pose->t;
                                if (truth)
                                {
                                    const domain::Vec3 gtPosition = truth->state.pose.t;
                                    mylog::info(
                                        "[位姿对比] KF #{}: 估计=[{:.3f}, {:.3f}, {:.3f}], 真值=[{:.3f}, {:.3f}, "
                                        "{:.3f}], 绝对误差={:.3f}m",
                                        keyframeId, estimated.x(), estimated.y(), estimated.z(), gtPosition.x(),
                                        gtPosition.y(), gtPosition.z(), (estimated - gtPosition).norm());
                                }
                                else
                                {
                                    mylog::info("[位姿估计] KF #{}: P=[{:.3f}, {:.3f}, {:.3f}]（数据源无真值）",
                                                keyframeId, estimated.x(), estimated.y(), estimated.z());
                                }
                                ++keyframeId;
                                recordSample(samples, currentTime, estimated, truth, skippedNoTruth);

                                if (!outcome.health.isHealthy)
                                {
                                    mylog::error("[健康降级] {}", outcome.health.alertMessage);
                                    if (outcome.health.degradation == domain::Degradation::VISUAL)
                                    {
                                        // 视觉中断期间重建坐标系毫无作用（下一帧图像依旧如此），
                                        // 与回放器保持同一策略。即便某个估计器误报
                                        // RESET_PIPELINE，也不在这里执行。
                                        mylog::warn("[健康降级] 视觉观测中断，保持当前坐标系并等待恢复");
                                    }
                                    else
                                    {
                                        pipeline_.reset(outcome.recovery);
                                    }
                                }
                            }
                        }
                        else
                        {
                            mylog::debug("[跳过非关键帧] 图像帧 #{}: 视差={:.2f}px, 共视率={:.1f}%", frame.index,
                                         outcome.decision.avgParallaxPx, outcome.decision.trackingRatio * 100.0);
                        }
                    }
                }

                currentTime += imuDt;
            }

            if (!initialized)
            {
                mylog::error("[运行结束] 仿真时长 {:.2f}s 内始终未完成初始化，本次运行没有可评估的轨迹",
                             system_.totalDuration);
            }

            return evaluate(samples, initialized, sawAnyGroundTruth, skippedNoTruth, pipeline_.resets());
        }

    private:
        struct TrajectorySample
        {
            double timestamp{0.0};
            domain::Vec3 estimated;
            domain::Vec3 groundTruth;
        };

        /// 默认真值仿真器：观测模型必须与估计器声明的一致
        [[nodiscard]] static std::unique_ptr<domain::ISensorSource>
        makeDefaultSource(const domain::PipelineConfig &config, const VioPipeline &pipeline)
        {
            return std::make_unique<VioSimulator>(makeSimulatorOptions(config), pipeline.observationModel());
        }

        /// 构造流水线前确保注册表已就绪（参数求值先于 pipeline_ 构造）
        [[nodiscard]] static domain::EstimatorFactory prepareFactory(domain::EstimatorFactory factory)
        {
            adapters::registerBuiltinEstimators();
            return factory;
        }

        static void recordSample(std::vector<TrajectorySample> &out, double timestamp, const domain::Vec3 &estimated,
                                 const std::optional<domain::GroundTruth> &truth, std::size_t &skipped)
        {
            if (!truth)
            {
                ++skipped;
                return;
            }
            out.push_back(TrajectorySample{timestamp, estimated, truth->state.pose.t});
        }

        domain::RunReport evaluate(const std::vector<TrajectorySample> &samples, bool initialized,
                                   bool sawAnyGroundTruth, std::size_t skippedNoTruth, std::size_t resets) const
        {
            mylog::info("================================================================================");
            mylog::info("📊 VIO 最小系统工业精度全面评测报告");
            mylog::info("================================================================================");

            domain::RunReport report;
            report.initialized = initialized;
            report.resets = resets;

            if (skippedNoTruth > 0)
            {
                mylog::warn("{} 个关键帧无真值，未参与评估", skippedNoTruth);
            }

            if (samples.empty())
            {
                if (!sawAnyGroundTruth)
                {
                    mylog::info("数据源不提供真值（groundTruthAt 恒为空），跳过精度评估");
                }
                else
                {
                    mylog::warn("没有同时具备估计与真值的样本，无法评估");
                }
                return report;
            }

            if (samples.size() < 2)
            {
                mylog::warn("⚠️ 仅 {} 个位姿样本，轨迹不可评估（至少需要 2 个）", samples.size());
                return report;
            }

            double sumSqError = 0.0;
            for (const TrajectorySample &sample : samples)
            {
                const double error = (sample.estimated - sample.groundTruth).norm();
                sumSqError += error * error;
                if (error > report.maxError)
                {
                    report.maxError = error;
                }
            }

            report.keyframeCount = samples.size();
            report.rmse = std::sqrt(sumSqError / static_cast<double>(samples.size()));

            mylog::info("评估关键帧样本数 : {}", report.keyframeCount);
            mylog::info("整体平移 RMSE 误差: {:.4f} 米", report.rmse);
            mylog::info("最大轨迹偏差       : {:.4f} 米", report.maxError);
            if (report.resets > 0)
            {
                // resets 同时计入完整重置与就地重建：只有前者会换坐标系。
                // 这里不宣称"各段独立坐标系"，否则就地重建（坐标连续）也被
                // 说成坐标系跳变，与事实相反。
                mylog::warn("运行期间发生 {} 次流水线重建（含就地重建；仅完整重置会更换坐标系）", report.resets);
            }

            if (domain::isAcceptable(report, system_.rmseThreshold))
            {
                mylog::info("🎉 系统性能表现卓越: 精度指标完全达到工业级紧耦合 VIO 验收标准！");
            }
            else
            {
                mylog::warn("⚠️ 轨迹误差略微偏高，可适当调优噪声协方差与 Huber 阈值。");
            }

            return report;
        }

        domain::PipelineConfig config_;
        domain::SystemConfig system_;
        VioPipeline pipeline_;
        std::unique_ptr<domain::ISensorSource> source_;
    };

} // namespace myslam::vio
