#pragma once

/**
 * @file replay_observer.hpp
 * @brief 回放可视化观察者：把 @c DatasetVisualizer 与去畸变图接到回放器上。
 *
 * 回放器不认识任何 viz 类型；窗口、视频与并排去畸变图的全部细节都收敛在
 * 本文件。想关掉可视化时不构造本对象即可。
 */

#include "app/run_observer.hpp"
#include "domain/camera_model.hpp"
#include "domain/types.hpp"
#include "viz/mei_undistort.hpp"
#include "viz/visualizer.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace myslam::viz
{

    /**
     * @brief 实时窗口 / 视频输出 / 去畸变对照图的观察者实现
     */
    class ReplayObserver final : public vio::IRunObserver
    {
    public:
        /**
         * @param targetModel 流水线实际消费的观测模型。可视化必须与算法使用
         *        同一个模型，否则去畸变面板与特征点会落在两套像素语义上
         *        （例如估计器选了虚拟针孔、可视化却按原始 MEI 画）。
         */
        ReplayObserver(const dataset::VisualizationConfig &config, const camera::CameraIntrinsics &intrinsics,
                       std::shared_ptr<const domain::ICameraModel> targetModel)
            : config_(config),
              visualizer_(config),
              intrinsics_(intrinsics),
              sourceModel_(domain::makeCameraModel(intrinsics)),
              targetModel_(targetModel ? std::move(targetModel) : domain::makeVirtualPinholeModel(intrinsics)),
              trailLimit_(static_cast<std::size_t>(std::max(1, config.trailMaxPoints)))
        {
            if (visualizer_.active() && config_.showUndistorted)
            {
                undistorter_ = std::make_unique<MeiUndistorter>(sourceModel_, targetModel_);
                mylog::info("[可视化] 去畸变图: 虚拟针孔映射覆盖率 {:.1f}%", 100.0 * undistorter_->validRatio());
            }
        }

        [[nodiscard]] bool active() const override { return visualizer_.active(); }

        void onFrame(const vio::FrameObservation &observation) override
        {
            if (!visualizer_.active() || observation.gray == nullptr || observation.frame == nullptr)
            {
                return;
            }

            char line1[160];
            std::snprintf(line1, sizeof(line1), "frame %zu  t=%.2fs  feat=%zu  kf=%d", observation.frameIndex,
                          observation.cameraTime, observation.frame->observations.size(), observation.keyframeId);
            char line2[160];
            std::snprintf(line2, sizeof(line2), "state=%.*s  imu=%zu  resets=%zu",
                          static_cast<int>(observation.stateName.size()), observation.stateName.data(),
                          observation.imuSamples, observation.resets);
            const std::string line3 = "ESC/q to quit";

            cv::Mat undistortedImage;
            if (undistorter_)
            {
                undistortedImage = undistorter_->apply(*observation.gray);
            }
            visualizer_.update(*observation.gray, *observation.frame, trail_, {line1, line2, line3},
                               observation.cameraTime, undistorter_ ? &undistortedImage : nullptr,
                               undistorter_ ? &undistorter_->sourceValidMask() : nullptr);
        }

        void onPose(const domain::Pose &pose) override
        {
            if (!visualizer_.active())
            {
                return;
            }
            if (trail_.size() >= trailLimit_)
            {
                const std::size_t drop = std::max<std::size_t>(1, trailLimit_ / 4);
                trail_.erase(trail_.begin(), trail_.begin() + static_cast<std::ptrdiff_t>(drop));
            }
            trail_.emplace_back(pose.position.x(), pose.position.y());
        }

        [[nodiscard]] bool quitRequested() const override { return visualizer_.shouldQuit(); }

    private:
        dataset::VisualizationConfig config_;
        DatasetVisualizer visualizer_;
        camera::CameraIntrinsics intrinsics_;
        std::shared_ptr<const domain::ICameraModel> sourceModel_;
        std::shared_ptr<const domain::ICameraModel> targetModel_;
        std::unique_ptr<MeiUndistorter> undistorter_;
        std::vector<domain::Vec2> trail_;
        std::size_t trailLimit_;
    };

} // namespace myslam::viz
