#pragma once

/**
 * @file keyframe_selector.hpp
 * @brief 自适应关键帧决策（视差 / 共视率 / 时间保底）。
 */

#include "vio/configHolder.hpp"
#include <format>
#include <gtsam/geometry/Point2.h>
#include <optional>
#include <string>
#include <unordered_map>

namespace myslam::vio
{

    // =========================================================================
    // 4. 自适应关键帧决策器
    // =========================================================================

    class KeyframeSelector
    {
    public:
        explicit KeyframeSelector(const VioConfig &config)
            : config_(config)
        {
        }

        struct DecisionResult
        {
            bool isKeyframe{false};
            std::string reason;
            double avgParallax{0.0};
            double trackingRatio{1.0};
        };

        /**
         * @brief 判断当前图像帧是否应当作为关键帧插入后端
         */
        DecisionResult evaluate(const CameraFrame &currentFrame, const std::optional<CameraFrame> &lastKeyframe)
        {
            if (!lastKeyframe.has_value())
            {
                return {true, "第一帧初始关键帧", 0.0, 1.0};
            }

            double dt = currentFrame.timestamp - lastKeyframe->timestamp;
            if (dt < config_.minKeyframeInterval)
            {
                return {false, "时间间隔未达最小视差基线要求 (保持高频推算)", 0.0, 1.0};
            }

            // 建立上一关键帧的特征点映射表
            std::unordered_map<int, gtsam::Point2> prevFeatures;
            for (const auto &feat : lastKeyframe->features)
            {
                prevFeatures[feat.featureId] = feat.uv;
            }

            int coVisibleCount = 0;
            double totalParallax = 0.0;

            for (const auto &feat : currentFrame.features)
            {
                auto it = prevFeatures.find(feat.featureId);
                if (it != prevFeatures.end())
                {
                    coVisibleCount++;
                    double dist = (feat.uv - it->second).norm();
                    totalParallax += dist;
                }
            }

            double avgParallax = (coVisibleCount > 0) ? (totalParallax / coVisibleCount) : 0.0;
            double trackingRatio =
                prevFeatures.empty() ? 1.0 : (static_cast<double>(coVisibleCount) / prevFeatures.size());

            // 判据 1: 视差超出门限 (满足足够的立体几何基线进行三角化)
            if (avgParallax >= config_.minParallaxPx)
            {
                return {true, std::format("视差充分 ({:.2f} px >= {:.2f} px)", avgParallax, config_.minParallaxPx),
                        avgParallax, trackingRatio};
            }

            // 判据 2: 特征共视留存率过低 (防止旧特征丢失导致脱靶)
            if (trackingRatio < config_.minTrackingRatio)
            {
                return {true,
                        std::format("共视特征比例低 ({:.1f}% < {:.1f}%)", trackingRatio * 100.0,
                                    config_.minTrackingRatio * 100.0),
                        avgParallax, trackingRatio};
            }

            // 判据 3: 超时保底 (防止低速匀速下长时间不插关键帧导致 IMU 预积分线性化发散)。
            // 但静止/悬停时视差≈0，此时插入关键帧无法三角化任何新路标、只会让图欠约束，
            // 因此保底判据同时要求达到三角化最小视差。
            if (dt >= config_.maxKeyframeInterval && avgParallax >= config_.minTriangulationParallaxPx)
            {
                return {true,
                        std::format("时间跨度达到保底上限 ({:.2f}s >= {:.2f}s) 且视差可用 ({:.2f} px)", dt,
                                    config_.maxKeyframeInterval, avgParallax),
                        avgParallax, trackingRatio};
            }

            return {false, "非关键帧 (保持高频推算)", avgParallax, trackingRatio};
        }

    private:
        VioConfig config_;
    };

} // namespace myslam::vio
