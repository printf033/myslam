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

#include "dataset/configHolder.hpp"
#include "dataset/mei_tracker.hpp"
#include "dataset/mei_undistort.hpp"
#include "dataset/ply_export.hpp"
#include "dataset/subt_dataset.hpp"
#include "dataset/visualizer.hpp"
#include "logger.hpp"
#include "vio/backend.hpp"
#include "vio/configHolder.hpp"
#include "vio/initializer.hpp"
#include "vio/keyframe_selector.hpp"
#include "vio/propagator.hpp"
#include "vio/system.hpp"
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
        DatasetRunner(const vio::VioConfig &config, const MeiIntrinsics &intrinsics, const DatasetRunConfig &runConfig,
                      const vio::SystemConfig &systemConfig = {})
            : config_(config),
              intrinsics_(intrinsics),
              runConfig_(runConfig),
              systemConfig_(systemConfig)
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
                      // 采样率必须传入：外推器的异常 dt 兜底值按它派生，
                      // 否则会硬编码成 200 Hz 的 0.005 s。
                      propagator(config.gravityMagnitude, config.imuRateHz),
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

            // keyframeId 跨段单调递增：重建后从上一段末尾继续，避免日志与 TUM
            // 注释出现重号（每段都从 0 开始会让排查无法定位）。
            // 因此它同时就是“已处理关键帧总数”，不需另外累加。
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
            // 去畸变图只在真要渲染时生成：没开窗口/录像时每帧 remap 纯属浪费。
            // 跟踪器自己的有效视场掩码直接拿来描边，不再重复算一遍映射。
            std::optional<MeiUndistorter> undistorter;
            if (visualizer.active() && runConfig_.viz.showUndistorted)
            {
                undistorter.emplace(intrinsics_);
                mylog::info("[可视化] MEI 去畸变图: 虚拟针孔映射覆盖率 {:.1f}%", 100.0 * undistorter->validRatio());
            }
            std::vector<cv::Point2f> rawPixels;            // 与 frame.features 对齐的原始鱼眼像素
            std::vector<gtsam::Point2> trail;              // 关键帧 XY 轨迹（用于小地图）
            std::vector<gtsam::Point3> exportedLandmarks;  // 跨重置累积的路标点
            std::vector<gtsam::Point3> exportedTrajectory; // 跨重置累积的轨迹点

            // trailMaxPoints 上限：小地图只关心近期轨迹，长序列下无界增长会让
            // 内存与每帧重绘成本线性上升。原先该配置项被解析后从未被读取。
            const std::size_t trailLimit = static_cast<std::size_t>(std::max(1, runConfig_.viz.trailMaxPoints));
            auto pushTrail = [&trail, trailLimit](double x, double y)
            {
                if (trail.size() >= trailLimit)
                {
                    // 丢掉最旧的四分之一点，避免每帧都做一次 O(n) 搬移
                    const std::size_t drop = std::max<std::size_t>(1, trailLimit / 4);
                    trail.erase(trail.begin(), trail.begin() + static_cast<std::ptrdiff_t>(drop));
                }
                trail.emplace_back(x, y);
            };
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
            // 实际回放到的时刻。收尾时用它算 durationSeconds —— 提前退出（含用户
            // 按 ESC/q）时报的是已处理进度，而不是整段序列长度。
            double lastProcessedTime = 0.0;
            const auto &stamps = sequence.stamps();
            const auto &imu = sequence.imu();
            const double nominalDt = 1.0 / std::max(1.0, config_.imuRateHz);
            // 断档阈值按采样率倍数给出：固定 0.1 s 会漏掉 200 Hz 下缺 5~19 个样本
            // 的短断档，而那些断档同样不能跨越积分。
            const double maxGapSeconds = std::max(nominalDt, runConfig_.maxImuGapFactor * nominalDt);

            const std::size_t firstFrame =
                runConfig_.startFrame > 0 ? static_cast<std::size_t>(runConfig_.startFrame) : 0;

            // 断档判定的起点基线。回放从前一帧的历史里起跑时，起点之前的间隔
            // 发生在回放窗口之外，不该触发重建。
            std::size_t imuBaseline = 0;

            // 从中间帧起跑时把 IMU 游标直接定位到该时刻，而不是从序列开头逐帧
            // 遍历过去。否则起点之前的历史断档会在处理任何帧之前就先触发一次
            // 流水线重建：TUM 首行凭空多出 "# segment 2"，而段 1 是空的。
            if (firstFrame < stamps.size())
            {
                const double startTime = stamps[firstFrame];
                // 取最后一个不晚于起始时刻的样本：积分需要从它覆盖到 cameraTime，
                // 若取第一个不早于起始时刻的样本，首帧会因 record.timestamp >
                // cameraTime 而完全不积分。
                auto it = std::upper_bound(imu.cbegin(), imu.cend(), startTime,
                                           [](double t, const ImuRecord &record) { return t < record.timestamp; });
                if (it != imu.cbegin())
                {
                    --it;
                }
                imuCursor = static_cast<std::size_t>(it - imu.cbegin());
                countedImuSamples = imuCursor;
                imuBaseline = imuCursor;
            }

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

                        if (imuCursor > imuBaseline)
                        {
                            const double gap = record.timestamp - imu[imuCursor - 1].timestamp;
                            if (gap > maxGapSeconds)
                            {
                                mylog::warn("[回放] IMU 断档 {:.3f}s (t={:.4f}s) 超过阈值 {:.3f}s，"
                                            "重建流水线并重新初始化（第 {} 段）",
                                            gap, record.timestamp, maxGapSeconds, report.segments + 1);
                                const int continueFrom = pipeline->keyframeId;
                                accumulatePoints(*pipeline, exportedLandmarks, exportedTrajectory);
                                pipeline.emplace(config_, intrinsics_, runConfig_.maxFeatures);
                                pipeline->keyframeId = continueFrom;
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

                // 配置里的画幅必须与实际图像一致：特征跟踪器会用它做视场掩码
                // 与越界过滤，不一致时 cv::bitwise_and 会因尺寸不匹配直接抛异常，
                // 而该异常在本层无人捕获。首帧就拦住并明确报错。
                if (gray.cols != intrinsics_.imageWidth || gray.rows != intrinsics_.imageHeight)
                {
                    mylog::error("[回放] 图像尺寸 {}x{} 与配置的 {}x{} 不一致，终止回放", gray.cols, gray.rows,
                                 intrinsics_.imageWidth, intrinsics_.imageHeight);
                    report.durationSeconds = cameraTime - stamps.front();
                    report.elapsedSeconds =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
                    return report;
                }

                const vio::CameraFrame frame = pipeline->tracker.track(gray, static_cast<int>(frameIndex), cameraTime,
                                                                       visualizer.active() ? &rawPixels : nullptr);
                ++report.framesProcessed;
                lastProcessedTime = cameraTime;

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
                    cv::Mat undistortedImage;
                    if (undistorter)
                    {
                        undistortedImage = undistorter->apply(gray);
                    }
                    visualizer.update(gray, frame, trail, {line1, line2, line3}, cameraTime,
                                      undistorter ? &undistortedImage : nullptr, undistorter ? &rawPixels : nullptr,
                                      undistorter ? &pipeline->tracker.validMask() : nullptr);
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
                        // 超时值取自 system.initTimeout（原先硬编码 2.0，配置改不动）
                        pipeline->initDeadline = cameraTime + systemConfig_.initTimeout;
                    }
                    // initTimeout <= 0 表示不超时：不能直接比 deadline，
                    // 否则 deadline == cameraTime 会让第一帧就放宽静止判据。
                    const bool relaxed = systemConfig_.initTimeout > 0.0 && cameraTime >= pipeline->initDeadline;
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
                        pushTrail(initPose.x(), initPose.y());
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
                        bool optimized = false;
                        try
                        {
                            optimized = pipeline->backend.processKeyframe(pipeline->keyframeId, cameraTime, frame,
                                                                          *pipeline->lastKeyframe);
                        }
                        catch (const std::exception &error)
                        {
                            // 后端内部已做局部恢复（重建平滑器、保留坐标系）。能走到
                            // 这里说明连恢复都失败（窗口内没有可锚定的关键帧），
                            // 此时才不得不放弃当前坐标系。
                            mylog::warn("[回放] 关键帧 #{} 优化失败且无法局部恢复 ({}), 重建 VIO 系统",
                                        pipeline->keyframeId, error.what());
                            const int continueFrom = pipeline->keyframeId;
                            accumulatePoints(*pipeline, exportedLandmarks, exportedTrajectory);
                            pipeline.emplace(config_, intrinsics_, runConfig_.maxFeatures);
                            pipeline->keyframeId = continueFrom;
                            ++report.resets;
                            ++report.segments;
                            writeTumSegment(tum, report.segments, cameraTime);
                            continue;
                        }

                        if (!optimized)
                        {
                            // 后端已完成局部恢复：坐标系连续，只是本帧不产出位姿。
                            // 这里刻意不更新 lastKeyframe —— 下一帧的三角化基线因此
                            // 更长，视差更大，更容易把丢掉的路标重新建立起来。
                            // 也不清零 imuSinceLastKeyframe：IMU 积分要跨过本帧继续累积。
                            mylog::warn("[回放] 关键帧 #{} 已由后端局部恢复，本帧不出位姿（坐标保持连续）",
                                        pipeline->keyframeId);
                            ++report.recoveredFrames;
                            continue;
                        }
                        pipeline->propagator.resetWithOptimizedState(cameraTime, pipeline->backend.latestNavState(),
                                                                     pipeline->backend.latestBias());
                        pipeline->lastKeyframe = frame;
                        pipeline->imuSinceLastKeyframe = 0;

                        const gtsam::Pose3 pose = pipeline->backend.latestNavState().pose();
                        writeTum(tum, cameraTime, pose);
                        ++report.trajectorySamples;
                        pushTrail(pose.x(), pose.y());
                        ++pipeline->keyframeId;

                        // 健康降级：连续异常说明当前段已不可信，重建流水线重新初始化。
                        // 与优化异常使用同一套重建流程，保证段边界都会被写入 TUM 注释，
                        // 不留下隐性坐标系跳变。
                        //
                        // 但必须区分异常来源：图像变暗/遮挡/无纹理导致的“视觉观测中断”
                        // 重建流水线完全无效 —— 下一帧图像依旧是黑的，重建只会让坐标系
                        // 反复归零，把一次暂短的视觉中断放大成几十次轨迹跳变（实测
                        // 732 s 之后的黑暗通道因此产生了 52 次重建、52 次跳回原点）。
                        // 视觉中断期间坐标系仍然有效：纯 IMU 因子继续约束姿态，等特征
                        // 恢复后视觉观测会自动接上。只有惯性状态本身失稳才重建。
                        const auto &health = pipeline->backend.health();
                        if (!health.isHealthy && health.degradation == vio::HealthMonitor::Degradation::VISUAL)
                        {
                            if (!report.visualOutageReported)
                            {
                                mylog::warn("[回放] 视觉观测中断 ({})，保持当前坐标系并等待恢复"
                                            "（重建流水线无法改善图像质量，只会让坐标跳变）",
                                            health.alertMessage);
                                report.visualOutageReported = true;
                            }
                            ++report.visualOutageFrames;
                        }
                        else if (!health.isHealthy)
                        {
                            mylog::error("[回放] 惯性状态退化 ({}), 重置 VIO 系统", health.alertMessage);
                            // keyframeId 带着已产出的计数一起继承，因此降级不会让
                            // 该段的关键帧从统计里消失（原有实现会丢掉整段计数）。
                            const int continueFrom = pipeline->keyframeId;
                            accumulatePoints(*pipeline, exportedLandmarks, exportedTrajectory);
                            pipeline.emplace(config_, intrinsics_, runConfig_.maxFeatures);
                            pipeline->keyframeId = continueFrom;
                            ++report.resets;
                            ++report.segments;
                            writeTumSegment(tum, report.segments, cameraTime);
                        }
                    }
                }
            }

            // keyframeId 跨段继承，本身就是累计总数，直接取用即可。
            report.keyframes = pipeline ? static_cast<std::size_t>(pipeline->keyframeId) : 0;
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
            // 进度口径：以实际处理到的时刻为准。旧实现在这里无条件写入
            // stamps.back() - stamps.front()，于是中途退出时报告仍显示整段序列
            // 长度，进度与真实处理量脱节。
            // 一帧都没处理时 lastProcessedTime 仍是 0，差值会变成负数，需夹住。
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
                mylog::warn("   其中 {} 个关键帧处于视觉观测中断（图像变暗/遮挡），坐标系未重建；"
                            "该段轨迹由 IMU 维持，静止时已由零速更新钉住速度",
                            report.visualOutageFrames);
                if (const std::size_t zupt = pipeline ? pipeline->backend.zuptCount() : 0; zupt > 0)
                {
                    mylog::warn("   零速更新生效 {} 次（视觉失明 + IMU 判静止时把速度约束为 0）", zupt);
                }
            }
            if (report.recoveredFrames > 0)
            {
                mylog::warn("   其中 {} 个关键帧因线性系统不定而做了局部恢复：坐标系保持连续，"
                            "但这些帧本身不产出新位姿",
                            report.recoveredFrames);
            }
            if (report.segments > 1)
            {
                mylog::warn("   流水线被重建 {} 次（IMU 断档或无法局部恢复的求解失败），各段独立"
                            "重新初始化，轨迹与点云坐标互不连续，统计量只能按段解读",
                            report.resets);
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
        vio::SystemConfig systemConfig_;
    };

} // namespace myslam::dataset
