#pragma once

/**
 * @file dataset_runner.hpp
 * @brief 真实数据集回放：按相机帧驱动、以 IMU 样本积分，复用 myslam 的 VIO 组件。
 *
 * `VioSystem::run()` 由内置仿真器驱动；本类把同一套初始化器、关键帧决策、
 * 紧耦合后端与高频外推器接到真实的 SubT 序列上，并输出 TUM 位姿轨迹。
 *
 * SubT 原始数据存在 IMU 时间戳跳变（采集分段/丢包）。回放器检测到超过
 * `kMaxImuGapSeconds` 的间隔时会重置整条 VIO 流水线并重新初始化，避免把
 * 跨越断层的预积分送进因子图。
 *
 * 数据没有真值轨迹，因此报告只统计处理规模与初始化状态，不做 RMSE 评估。
 */

#include "dataset/mei_tracker.hpp"
#include "dataset/ply_export.hpp"
#include "dataset/subt_dataset.hpp"
#include "dataset/types.hpp"
#include "dataset/visualizer.hpp"
#include "logger.hpp"
#include "vio/backend.hpp"
#include "vio/initializer.hpp"
#include "vio/keyframe_selector.hpp"
#include "vio/params.hpp"
#include "vio/propagator.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <gtsam/geometry/Quaternion.h>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>

namespace myslam::dataset
{

    /**
     * @brief 数据集回放器
     */
    class DatasetRunner
    {
    public:
        DatasetRunner(const vio::VioConfig &config, const MeiIntrinsics &intrinsics, const DatasetRunConfig &runConfig)
            : config_(config),
              intrinsics_(intrinsics),
              runConfig_(runConfig)
        {
        }

        DatasetRunReport run(const SubtSequence &sequence)
        {
            const auto wallStart = std::chrono::steady_clock::now();

            // 一条完整的 VIO 流水线；IMU 时间跳变后整体重建以重新初始化
            struct Pipeline
            {
                Pipeline(const vio::VioConfig &config, const MeiIntrinsics &intrinsics, int maxFeatures)
                    : initializer(config),
                      selector(config),
                      backend(config),
                      propagator(config.gravityMagnitude),
                      tracker(intrinsics, maxFeatures)
                {
                }

                vio::VioInitializer initializer;
                vio::KeyframeSelector selector;
                vio::VioBackend backend;
                vio::HighRatePropagator propagator;
                MeiFeatureTracker tracker;
                std::optional<vio::CameraFrame> lastKeyframe;
                int keyframeId{0};
                std::size_t imuSinceLastKeyframe{0};
                double initDeadline{-1.0}; ///< 超过该时刻仍未初始化则放宽静止检测
                /// 已积分的物理时刻。预积分必须精确停在这里，使关键帧时间戳与
                /// CombinedImuFactor 覆盖的区间末端一致。-1 表示尚未开始。
                double integratedUntil{-1.0};
            };

            std::optional<Pipeline> pipeline;
            pipeline.emplace(config_, intrinsics_, runConfig_.maxFeatures);

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
            report.segments = 1; // 首个流水线构成第 1 段
            DatasetVisualizer visualizer(runConfig_.viz);
            std::vector<gtsam::Point2> trail;              // 关键帧 XY 轨迹（用于小地图）
            std::vector<gtsam::Point3> exportedLandmarks;  // 跨重置累积的路标点
            std::vector<gtsam::Point3> exportedTrajectory; // 跨重置累积的轨迹点
            // 收集当前流水线的路标与轨迹点（优先取优化后的估计）
            auto accumulatePoints =
                [](const auto &current, std::vector<gtsam::Point3> &landmarks, std::vector<gtsam::Point3> &trajectory)
            {
                const auto &estimate = current.backend.currentEstimate();
                if (!estimate.empty())
                {
                    for (const auto &kv : estimate.template extract<gtsam::Point3>())
                    {
                        landmarks.push_back(kv.second);
                    }
                    for (const auto &kv : estimate.template extract<gtsam::Pose3>())
                    {
                        trajectory.push_back(kv.second.translation());
                    }
                }
                else
                {
                    for (const auto &kv : current.backend.triangulatedLandmarks())
                    {
                        landmarks.push_back(kv.second);
                    }
                }
            };

            std::size_t imuCursor = 0;
            std::size_t countedImuSamples = 0;
            const auto &stamps = sequence.stamps();
            const auto &imu = sequence.imu();
            const double nominalDt = 1.0 / std::max(1.0, config_.imuRateHz);
            // 断档阈值按采样率倍数给出：固定 0.1 s 会漏掉 200 Hz 下缺 5~19 个样本
            // 的短断档，而那些断档同样不能跨越积分。
            const double maxGapSeconds = std::max(nominalDt, runConfig_.maxImuGapFactor * nominalDt);

            const std::size_t firstFrame =
                runConfig_.startFrame > 0 ? static_cast<std::size_t>(runConfig_.startFrame) : 0;

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

                // 把 IMU 精确积分到 cameraTime。
                //
                // 关键点：相机时刻通常落在两条 IMU 样本之间。若只积分到「不晚于
                // cameraTime 的最后一条样本」，预积分末端会比关键帧时间戳早最多一个
                // 采样周期，导致 CombinedImuFactor 覆盖的区间与状态时间戳错位，
                // 固定滞后窗口的边缘化边界也随之偏移。
                //
                // 这里采用零阶保持：样本 k 的测量代表 [t_k, t_{k+1})。积分在
                // cameraTime 处截断，跨越边界的样本只消费其前半段，剩余部分留给
                // 下一帧，由 integratedUntil 记录截断位置。
                std::size_t integratedThisFrame = 0;
                while (imuCursor < imu.size())
                {
                    const ImuRecord &record = imu[imuCursor];
                    if (record.timestamp > cameraTime)
                    {
                        break; // 该样本完全位于相机时刻之后
                    }

                    // 每样本只做一次断档检测与计数；跨边界的样本会被后续帧再次访问
                    if (imuCursor >= countedImuSamples)
                    {
                        countedImuSamples = imuCursor + 1;
                        ++report.imuSamples;

                        if (imuCursor > 0)
                        {
                            const double gap = record.timestamp - imu[imuCursor - 1].timestamp;
                            if (gap > maxGapSeconds)
                            {
                                mylog::warn("[回放] IMU 断档 {:.3f}s (t={:.4f}s) 超过阈值 {:.3f}s，"
                                            "重建流水线并重新初始化（第 {} 段）",
                                            gap, record.timestamp, maxGapSeconds, report.segments + 1);
                                report.keyframes += static_cast<std::size_t>(pipeline->keyframeId);
                                accumulatePoints(*pipeline, exportedLandmarks, exportedTrajectory);
                                pipeline.emplace(config_, intrinsics_, runConfig_.maxFeatures);
                                ++report.resets;
                                ++report.segments;
                                writeTumSegment(tum, report.segments, cameraTime);
                                // 新段从零开始积分，绝不沿用跨断层的 dt
                                pipeline->integratedUntil = -1.0;
                            }
                        }
                    }

                    // 样本代表的区间 [record.timestamp, segEnd)。零阶保持只能维持一个
                    // 正常采样周期：若下一条样本距离过远（断档），绝不能把该测量外推
                    // 整段缺失时间，否则预积分会虚拟出不存在的匀速运动。
                    double segEnd =
                        (imuCursor + 1 < imu.size()) ? imu[imuCursor + 1].timestamp : record.timestamp + nominalDt;
                    if (segEnd - record.timestamp > maxGapSeconds)
                    {
                        segEnd = record.timestamp + nominalDt;
                    }
                    const double from = pipeline->integratedUntil < 0.0
                                            ? record.timestamp
                                            : std::max(pipeline->integratedUntil, record.timestamp);
                    const double to = std::min(segEnd, cameraTime);

                    if (to > from)
                    {
                        vio::ImuMeasurement measurement;
                        measurement.timestamp = record.timestamp;
                        measurement.accel = record.accel;
                        measurement.gyro = record.gyro;

                        if (pipeline->initializer.state() != vio::SystemState::TRACKING_OK)
                        {
                            pipeline->initializer.feedImu(measurement);
                        }
                        else
                        {
                            pipeline->backend.integrateImu(measurement, to - from);
                            pipeline->propagator.propagate(measurement);
                            ++integratedThisFrame;
                            ++pipeline->imuSinceLastKeyframe;
                        }
                        pipeline->integratedUntil = to;
                    }

                    if (segEnd > cameraTime)
                    {
                        break; // 已截断到相机时刻，样本残余留给下一帧
                    }
                    ++imuCursor;
                }
                mylog::debug(
                    "[回放] 帧 #{} t={:.4f}s: 积分 IMU {} 条, 累计 {} 条, 状态={}, 预积分终点={:.6f}s (偏差 {:.6f}s)",
                    frameIndex, cameraTime, integratedThisFrame, report.imuSamples,
                    vio::systemStateToString(pipeline->initializer.state()), pipeline->integratedUntil,
                    pipeline->integratedUntil < 0.0 ? 0.0 : cameraTime - pipeline->integratedUntil);

                cv::Mat gray = cv::imread(sequence.imagePath(frameIndex), cv::IMREAD_GRAYSCALE);
                if (gray.empty())
                {
                    mylog::warn("图像读取失败，跳过第 {} 帧: {}", frameIndex, sequence.imagePath(frameIndex));
                    continue;
                }

                const vio::CameraFrame frame = pipeline->tracker.track(gray, static_cast<int>(frameIndex), cameraTime);
                ++report.framesProcessed;

                if (visualizer.active())
                {
                    char line1[160];
                    std::snprintf(line1, sizeof(line1), "frame %zu  t=%.2fs  feat=%zu  kf=%d", frameIndex, cameraTime,
                                  frame.features.size(), pipeline->keyframeId);
                    char line2[160];
                    std::snprintf(line2, sizeof(line2), "state=%s  imu=%zu  resets=%zu",
                                  std::string(vio::systemStateToString(pipeline->initializer.state())).c_str(),
                                  report.imuSamples, report.resets);
                    std::string line3 = "ESC/q to quit";
                    visualizer.update(gray, frame, trail, {line1, line2, line3}, cameraTime);
                    if (visualizer.shouldQuit())
                    {
                        break;
                    }
                }

                if (pipeline->initializer.state() == vio::SystemState::STATIONARY_COLLECTING)
                {
                    gtsam::Rot3 rWb;
                    gtsam::imuBias::ConstantBias bias;
                    if (pipeline->initDeadline < 0.0)
                    {
                        pipeline->initDeadline = cameraTime + 2.0;
                    }
                    const bool relaxed = cameraTime >= pipeline->initDeadline;
                    if (pipeline->initializer.tryGravityAlignment(rWb, bias, !relaxed))
                    {
                        const gtsam::Pose3 initPose(rWb, gtsam::Point3(0.0, 0.0, 1.0));
                        const gtsam::Vector3 initVel = gtsam::Vector3::Zero();
                        pipeline->backend.initializeSystem(pipeline->keyframeId, cameraTime, initPose, initVel, bias);
                        pipeline->propagator.resetWithOptimizedState(cameraTime, gtsam::NavState(initPose, initVel),
                                                                     bias);
                        pipeline->initializer.setState(vio::SystemState::TRACKING_OK);
                        pipeline->lastKeyframe = frame;
                        pipeline->imuSinceLastKeyframe = 0;
                        // initializeSystem 内部重置了预积分，积分基线同步移到相机时刻
                        pipeline->integratedUntil = cameraTime;
                        report.initialized = true;
                        writeTum(tum, cameraTime, initPose);
                        ++report.trajectorySamples;
                        trail.emplace_back(initPose.x(), initPose.y());
                        ++pipeline->keyframeId;
                    }
                }
                else if (pipeline->initializer.state() == vio::SystemState::TRACKING_OK)
                {
                    const auto decision = pipeline->selector.evaluate(frame, pipeline->lastKeyframe);
                    if (decision.isKeyframe)
                    {
                        // 预积分必需：自上个关键帧以来没有任何 IMU 数据时（采集空洞），
                        // 无法构建 CombinedImuFactor，跳过本次关键帧以免因子图欠约束。
                        if (pipeline->imuSinceLastKeyframe == 0)
                        {
                            mylog::debug("[回放] 跳过关键帧 #{} t={:.4f}s: 自上次关键帧以来无 IMU 数据",
                                         pipeline->keyframeId, cameraTime);
                            continue;
                        }
                        mylog::debug("[回放] 关键帧 #{} t={:.4f}s: 特征 {} 个, 原因={}", pipeline->keyframeId,
                                     cameraTime, frame.features.size(), decision.reason);
                        try
                        {
                            pipeline->backend.processKeyframe(pipeline->keyframeId, cameraTime, frame,
                                                              *pipeline->lastKeyframe);
                        }
                        catch (const std::exception &error)
                        {
                            mylog::warn("[回放] 关键帧 #{} 优化失败 ({}), 重置 VIO 系统", pipeline->keyframeId,
                                        error.what());
                            report.keyframes += static_cast<std::size_t>(pipeline->keyframeId);
                            accumulatePoints(*pipeline, exportedLandmarks, exportedTrajectory);
                            pipeline.emplace(config_, intrinsics_, runConfig_.maxFeatures);
                            ++report.resets;
                            ++report.segments;
                            writeTumSegment(tum, report.segments, cameraTime);
                            continue;
                        }
                        pipeline->propagator.resetWithOptimizedState(cameraTime, pipeline->backend.latestNavState(),
                                                                     pipeline->backend.latestBias());
                        pipeline->lastKeyframe = frame;
                        pipeline->imuSinceLastKeyframe = 0;

                        const gtsam::Pose3 pose = pipeline->backend.latestNavState().pose();
                        writeTum(tum, cameraTime, pose);
                        ++report.trajectorySamples;
                        trail.emplace_back(pose.x(), pose.y());
                        ++pipeline->keyframeId;
                    }
                }
            }

            report.keyframes += pipeline ? static_cast<std::size_t>(pipeline->keyframeId) : 0;
            if (!runConfig_.plyPath.empty() && pipeline)
            {
                accumulatePoints(*pipeline, exportedLandmarks, exportedTrajectory);
                if (const auto written = writePly(runConfig_.plyPath, exportedLandmarks, exportedTrajectory); written)
                {
                    mylog::info("点云已导出: {} ({} 个路标, {} 个轨迹点)", runConfig_.plyPath.string(),
                                exportedLandmarks.size(), exportedTrajectory.size());
                }
                else
                {
                    mylog::warn("点云导出失败: {}", written.error().message);
                }
            }
            report.durationSeconds = stamps.empty() ? 0.0 : stamps.back() - stamps.front();
            report.elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();

            mylog::info("================================================================================");
            mylog::info("📊 数据集回放完成: 处理 {} 帧, 关键帧 {}, IMU {} 条, 初始化 {}", report.framesProcessed,
                        report.keyframes, report.imuSamples, report.initialized ? "成功" : "未完成");
            mylog::info("   回放时长 {:.2f} s (序列总长 {:.2f} s), 墙钟耗时 {:.2f} s", report.durationSeconds,
                        stamps.empty() ? 0.0 : stamps.back() - stamps.front(), report.elapsedSeconds);
            if (report.segments > 1)
            {
                mylog::warn("   序列被 IMU 断档切成 {} 段，各段独立重新初始化，"
                            "轨迹与点云坐标互不连续，统计量只能按段解读",
                            report.segments);
                mylog::warn("   跨段输出共 {} 条轨迹记录，直接拼接会在段边界出现位置跳变", report.trajectorySamples);
            }
            mylog::info("================================================================================");
            return report;
        }

    private:
        int stride() const { return runConfig_.stride > 0 ? runConfig_.stride : 1; }

        /**
         * @brief 在 TUM 轨迹中标记段边界
         *
         * 各段独立初始化，坐标系不同，轨迹不可跨段直接拼接。TUM 格式本身没有
         * 分段概念，因此用注释行标注；plot_tum.py 会跳过 `#` 开头的行。
         *
         * @param stream 输出流
         * @param segment 即将开始的段号（从 2 起）
         * @param timestamp 段起点的相机时刻
         */
        static void writeTumSegment(std::ofstream &stream, std::size_t segment, double timestamp)
        {
            if (!stream.is_open())
            {
                return;
            }
            stream << "# segment " << segment << " begins at t=" << timestamp << " (独立初始化，坐标不与上一段连续)\n";
        }

        static void writeTum(std::ofstream &stream, double timestamp, const gtsam::Pose3 &pose)
        {
            if (!stream.is_open())
            {
                return;
            }
            const gtsam::Vector3 t = pose.translation();
            const gtsam::Quaternion q = pose.rotation().toQuaternion();
            stream << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " "
                   << q.z() << " " << q.w() << "\n";
        }

        vio::VioConfig config_;
        MeiIntrinsics intrinsics_;
        DatasetRunConfig runConfig_;
    };

} // namespace myslam::dataset
