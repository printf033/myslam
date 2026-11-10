#pragma once

/**
 * @file replay.hpp
 * @brief 真实记录回放：IMU 时间同步、断档检测、特征提取与轨迹导出。
 *
 * @par 与 benchmark 的分界
 *
 * 真实记录没有真值，本类不做 RMSE 评估，只统计处理规模与状态。
 *
 * @par 时间同步已经迁出
 *
 * 重构前零阶保持、断档检测与积分截断手写在主循环里，无法单测。现在它们
 * 属于 @c domain::TimeSynchronizer；本类只负责把样本推给它、按结果驱动
 * 流水线。断档时丢弃本批切片并重建段——绝不把跨断层的积分送进估计器。
 */

#include "adapters/register_builtin.hpp"
#include "app/run_observer.hpp"
#include "common/dataset_config.hpp"
#include "common/ply_export.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/status.hpp"
#include "domain/time_synchronizer.hpp"
#include "domain/types.hpp"
#include "frontend/mei_tracker.hpp"
#include "logger.hpp"
#include "pipeline/pipeline.hpp"
#include "sensor/subt_dataset.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 数据集回放器
     */
    class ReplayRunner
    {
    public:
        /**
         * 不接收 @c domain::SystemConfig：相机步长、运行时长与验收门限都描述
         * 仿真与打分，回放由序列时间戳驱动，一个都不读。此前它被存成成员却
         * 从未使用，属于"看起来依赖"而非真实依赖。
         */
        ReplayRunner(const domain::PipelineConfig &config, const MeiIntrinsics &intrinsics,
                     const DatasetRunConfig &runConfig, std::unique_ptr<vio::IRunObserver> observer = nullptr,
                     domain::EstimatorFactory factory = {})
            : config_(config),
              intrinsics_(intrinsics),
              runConfig_(runConfig),
              observer_(std::move(observer)),
              factory_(std::move(factory))
        {
            adapters::registerBuiltinEstimators();
        }

        DatasetRunReport run(const SubtSequence &sequence)
        {
            const auto wallStart = std::chrono::steady_clock::now();

            /**
             * @brief 一段连续可积分的回放会话
             */
            struct Segment
            {
                Segment(const domain::PipelineConfig &config, const MeiIntrinsics &intrinsics, int maxFeatures,
                        const domain::EstimatorFactory &factory)
                    : pipeline(config, factory),
                      sync(config.imu),
                      tracker(intrinsics, domain::makeCameraModel(intrinsics), pipeline.observationModel(), maxFeatures,
                              config.vision.minFeatures)
                {
                    // 相机时间戳与 IMU 时间戳的固定偏移：由同步器统一扣除，
                    // 而不是在每个调用点手工加减（那样必然会漏掉某一条路径）。
                    sync.setTimeOffset(config.camera.timeOffsetSeconds);
                }

                vio::VioPipeline pipeline;
                domain::TimeSynchronizer sync;
                vio::MeiFeatureTracker tracker;

                /// 观测模型由估计器声明，可视化也必须用它，否则右半画面
                /// 与特征点会处于两套像素语义
                [[nodiscard]] std::shared_ptr<const domain::ICameraModel> observationModel() const
                {
                    return pipeline.observationModel();
                }
            };

            domain::KeyframeId keyframeId = 0;

            std::optional<Segment> segment;
            segment.emplace(config_, intrinsics_, runConfig_.maxFeatures, factory_);

            std::ofstream tum;
            if (!runConfig_.outputTum.empty())
            {
                tum.open(runConfig_.outputTum);
                if (!tum)
                {
                    mylog::warn("无法写入轨迹文件: {}", runConfig_.outputTum.string());
                }
            }

            DatasetRunReport report;

            if (!segment->pipeline.valid())
            {
                // 装配失败（未知估计器名、模型不支持）时套用整套序列毫无意义：
                // 每帧只会重复一条「观测模型为空」然后静默推进时间轴。
                mylog::error("[回放] 流水线装配失败，终止回放: {}", segment->pipeline.error());
                return report;
            }

            report.segments = 1;
            const bool rendering = observer_ != nullptr && observer_->active();
            std::vector<domain::Vec3> exportedLandmarks;
            std::vector<domain::Vec3> exportedTrajectory;

            auto accumulatePoints =
                [](const Segment &current, std::vector<domain::Vec3> &landmarks, std::vector<domain::Vec3> &trajectory)
            {
                const domain::EstimateSnapshot snapshot = current.pipeline.snapshot();
                for (const domain::LandmarkEstimate &landmark : snapshot.landmarks)
                {
                    landmarks.push_back(landmark.position);
                }
                for (const domain::KeyframeState &keyframe : snapshot.keyframes)
                {
                    trajectory.push_back(keyframe.pose.t);
                }
            };

            /// @return false 表示重建失败，调用方应终止回放
            auto restartSegment = [&](double cameraTime) -> bool
            {
                accumulatePoints(*segment, exportedLandmarks, exportedTrajectory);
                segment.emplace(config_, intrinsics_, runConfig_.maxFeatures, factory_);
                if (!segment->pipeline.valid())
                {
                    mylog::error("[回放] 重建后的流水线无效，终止回放: {}", segment->pipeline.error());
                    return false;
                }
                segment->sync.reset(cameraTime);
                ++report.resets;
                ++report.segments;
                writeTumSegment(tum, report.segments, cameraTime);
                return true;
            };

            const auto &stamps = sequence.stamps();
            const auto &imu = sequence.imu();
            const std::size_t firstFrame =
                runConfig_.startFrame > 0 ? static_cast<std::size_t>(runConfig_.startFrame) : 0;

            std::size_t imuCursor = 0;
            if (firstFrame < stamps.size())
            {
                const double startTime = stamps[firstFrame];
                auto it = std::upper_bound(imu.cbegin(), imu.cend(), startTime,
                                           [](double t, const ImuRecord &record) { return t < record.timestamp; });
                if (it != imu.cbegin())
                {
                    --it;
                }
                imuCursor = static_cast<std::size_t>(it - imu.cbegin());
            }

            double lastProcessedTime = 0.0;

            for (std::size_t frameIndex = firstFrame; frameIndex < stamps.size(); frameIndex += stride())
            {
                if (runConfig_.maxFrames > 0 &&
                    report.framesProcessed >= static_cast<std::size_t>(runConfig_.maxFrames))
                {
                    break;
                }
                const double cameraTime = stamps[frameIndex];
                if (runConfig_.maxDuration > 0.0 && cameraTime - stamps.front() > runConfig_.maxDuration)
                {
                    break;
                }

                // 把不晚于相机时刻的 IMU 样本推入同步器，再截断到相机时刻
                while (imuCursor < imu.size() && imu[imuCursor].timestamp <= cameraTime)
                {
                    segment->sync.push(
                        domain::ImuSample{imu[imuCursor].timestamp, imu[imuCursor].accel, imu[imuCursor].gyro});
                    ++imuCursor;
                }

                const domain::SyncResult synced = segment->sync.advanceTo(cameraTime);
                if (synced.gapDetected)
                {
                    mylog::warn("[回放] IMU 断档 {:.3f}s (t={:.4f}s)，重建流水线并重新初始化（第 {} 段）",
                                synced.gapSeconds, cameraTime, report.segments + 1);
                    if (!restartSegment(cameraTime))
                    {
                        break;
                    }
                }
                else
                {
                    for (const domain::ImuSlice &slice : synced.slices)
                    {
                        segment->pipeline.feedImu(slice);
                        ++report.imuSamples;
                    }
                }

                cv::Mat gray = cv::imread(sequence.imagePath(frameIndex), cv::IMREAD_GRAYSCALE);
                if (gray.empty())
                {
                    mylog::warn("图像读取失败，跳过第 {} 帧: {}", frameIndex, sequence.imagePath(frameIndex));
                    continue;
                }
                if (gray.cols != intrinsics_.imageWidth || gray.rows != intrinsics_.imageHeight)
                {
                    mylog::error("[回放] 图像尺寸 {}x{} 与配置的 {}x{} 不一致，终止回放", gray.cols, gray.rows,
                                 intrinsics_.imageWidth, intrinsics_.imageHeight);
                    report.durationSeconds = cameraTime - stamps.front();
                    report.elapsedSeconds =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
                    return report;
                }

                domain::ImageView imageView;
                imageView.data = gray.data;
                imageView.width = gray.cols;
                imageView.height = gray.rows;
                imageView.stride = static_cast<int>(gray.step);
                imageView.format = domain::PixelFormat::GRAY8;

                domain::Frame frame;
                frame.index = static_cast<int>(frameIndex);
                frame.timestamp = cameraTime;
                frame.image = imageView;
                if (!segment->tracker.extract(imageView, static_cast<int>(frameIndex), cameraTime, frame.observations))
                {
                    mylog::warn("[回放] 第 {} 帧特征提取失败，跳过", frameIndex);
                    continue;
                }

                ++report.framesProcessed;
                lastProcessedTime = cameraTime;

                if (rendering)
                {
                    vio::FrameObservation observation;
                    observation.frameIndex = frameIndex;
                    observation.cameraTime = cameraTime;
                    observation.gray = &gray;
                    observation.frame = &frame;
                    observation.stateName = domain::systemStateName(segment->pipeline.state());
                    observation.imuSamples = report.imuSamples;
                    observation.resets = report.resets;
                    observation.keyframeId = keyframeId;
                    observer_->onFrame(observation);
                    if (observer_->quitRequested())
                    {
                        break;
                    }
                }

                if (!segment->pipeline.tracking())
                {
                    const vio::InitOutcome init = segment->pipeline.stepInitialization(keyframeId, frame, cameraTime);
                    if (init.aligned)
                    {
                        mylog::info("[回放] 重力对齐完成{}，注入首帧先验",
                                    init.relaxed ? "（超时放宽为非静止对齐）" : "");
                    }
                    if (init.tracking && init.initialPose)
                    {
                        report.initialized = true;
                        writeTum(tum, cameraTime, *init.initialPose);
                        ++report.trajectorySamples;
                        if (observer_)
                        {
                            observer_->onPose(*init.initialPose);
                        }
                        ++keyframeId;
                    }
                    continue;
                }

                const vio::FrameOutcome outcome =
                    segment->pipeline.processFrame(keyframeId, frame, cameraTime, /*requireImuSinceLastKeyframe=*/true);

                if (outcome.kind == vio::FrameOutcome::Kind::KEYFRAME_FAILED)
                {
                    mylog::warn("[回放] 关键帧 #{} 优化失败 ({})，处置: {}", keyframeId, outcome.failureReason,
                                outcome.recovery == domain::RecoveryAction::RESET_PIPELINE ? "重建段"
                                                                                           : "估计器建议动作");
                    if (outcome.recovery == domain::RecoveryAction::RESET_PIPELINE)
                    {
                        if (!restartSegment(cameraTime))
                        {
                            break;
                        }
                    }
                    else
                    {
                        segment->pipeline.reset(outcome.recovery);
                    }
                    continue;
                }

                if (outcome.kind == vio::FrameOutcome::Kind::KEYFRAME_SKIPPED)
                {
                    mylog::debug("[回放] 跳过关键帧 #{} t={:.4f}s: 无可用 IMU 区间", keyframeId, cameraTime);
                    continue;
                }

                if (outcome.kind == vio::FrameOutcome::Kind::KEYFRAME_RECOVERED)
                {
                    mylog::warn("[回放] 关键帧 #{} 已由估计器局部恢复，本帧不出位姿（坐标保持连续）", keyframeId);
                    ++report.recoveredFrames;
                    continue;
                }

                if (outcome.producedPose())
                {
                    const domain::Pose pose = *outcome.pose;
                    writeTum(tum, cameraTime, pose);
                    ++report.trajectorySamples;
                    if (observer_)
                    {
                        observer_->onPose(pose);
                    }
                    ++keyframeId;

                    // 视觉中断（变暗/遮挡）不应重建坐标系：下一帧图像依旧是黑的，
                    // 重建只会让坐标系反复归零。估计器用 recovery=NONE 表达这一点。
                    if (!outcome.health.isHealthy && outcome.health.degradation == domain::Degradation::VISUAL)
                    {
                        if (!report.visualOutageReported)
                        {
                            mylog::warn("[回放] 视觉观测中断 ({})，保持当前坐标系并等待恢复",
                                        outcome.health.alertMessage);
                            report.visualOutageReported = true;
                        }
                        ++report.visualOutageFrames;
                    }
                    else if (!outcome.health.isHealthy && outcome.recovery == domain::RecoveryAction::RESET_PIPELINE)
                    {
                        mylog::error("[回放] 惯性状态退化 ({}), 重置 VIO 系统", outcome.health.alertMessage);
                        if (!restartSegment(cameraTime))
                        {
                            break;
                        }
                    }
                }
                else if (!outcome.decision.isKeyframe)
                {
                    mylog::debug("[回放] 跳过非关键帧 #{}: {}", keyframeId, outcome.decision.reason);
                }
            }

            report.keyframes = static_cast<std::size_t>(keyframeId);
            const domain::EstimatorDiagnostics diagnostics = segment->pipeline.diagnostics();
            if (!runConfig_.plyPath.empty() && segment)
            {
                accumulatePoints(*segment, exportedLandmarks, exportedTrajectory);
                if (const auto written = io::writePly(runConfig_.plyPath, exportedLandmarks, exportedTrajectory);
                    written)
                {
                    mylog::info("点云已导出: {} ({} 个路标, {} 个轨迹点)", runConfig_.plyPath.string(),
                                exportedLandmarks.size(), exportedTrajectory.size());
                }
                else
                {
                    mylog::warn("点云导出失败: {}", written.error().message);
                }
            }

            report.durationSeconds =
                (stamps.empty() || lastProcessedTime <= stamps.front()) ? 0.0 : lastProcessedTime - stamps.front();
            report.elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();

            mylog::info("================================================================================");
            mylog::info("📊 数据集回放完成: 处理 {} 帧, 关键帧 {}, IMU {} 条, 初始化 {}", report.framesProcessed,
                        report.keyframes, report.imuSamples, report.initialized ? "成功" : "未完成");
            mylog::info("   回放时长 {:.2f} s (序列总长 {:.2f} s), 墙钟耗时 {:.2f} s", report.durationSeconds,
                        stamps.empty() ? 0.0 : stamps.back() - stamps.front(), report.elapsedSeconds);
            if (report.visualOutageFrames > 0)
            {
                mylog::warn("   其中 {} 个关键帧处于视觉观测中断，坐标系未重建；该段轨迹由 IMU 维持",
                            report.visualOutageFrames);
                if (diagnostics.zuptCount > 0)
                {
                    mylog::warn("   零速更新生效 {} 次（视觉失明 + IMU 判静止时把速度约束为 0）",
                                diagnostics.zuptCount);
                }
            }
            if (report.recoveredFrames > 0)
            {
                mylog::warn("   其中 {} 个关键帧因线性系统不定而做了局部恢复：坐标系保持连续，但这些帧不产出新位姿",
                            report.recoveredFrames);
            }
            if (report.segments > 1)
            {
                mylog::warn("   流水线被重建 {} 次，各段独立重新初始化，轨迹与点云坐标互不连续", report.resets);
                mylog::warn("   跨段输出共 {} 条轨迹记录，直接拼接会在段边界出现位置跳变", report.trajectorySamples);
            }
            mylog::info("================================================================================");
            return report;
        }

    private:
        [[nodiscard]] int stride() const { return runConfig_.stride > 0 ? runConfig_.stride : 1; }

        static void writeTumSegment(std::ofstream &stream, std::size_t segment, double timestamp)
        {
            if (!stream.is_open())
            {
                return;
            }
            stream << "# segment " << segment << " begins at t=" << timestamp << " (独立初始化，坐标不与上一段连续)\n";
        }

        static void writeTum(std::ofstream &stream, double timestamp, const domain::Pose &pose)
        {
            if (!stream.is_open())
            {
                return;
            }
            const domain::Vec3 t = pose.t;
            const Eigen::Quaterniond &q = pose.q;
            stream << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " "
                   << q.z() << " " << q.w() << "\n";
        }

        domain::PipelineConfig config_;
        MeiIntrinsics intrinsics_;
        DatasetRunConfig runConfig_;
        std::unique_ptr<vio::IRunObserver> observer_;
        domain::EstimatorFactory factory_;
    };

} // namespace myslam::dataset
