#pragma once

/**
 * @file keyframe_selector.hpp
 * @brief 自适应关键帧决策（视差 / 共视率 / 时间保底）。
 *
 * @par 为什么视差不再直接用像素差
 *
 * 重构前视差是「虚拟针孔像素」的欧氏距离，而虚拟针孔焦距是
 * @c fx * pinholeScale。于是 @c pinholeScale 一改，同一个 @c minParallaxPx
 * 对应的物理视差就整体漂移——阈值被隐式绑定在某个特定相机模型上。
 *
 * 视差取像素欧氏距离。**不能**改用两条射线的夹角：它们分属两帧各自的
 * 相机坐标系，夹角不是视差。像素阈值的语义是"在该模型下的像素"，而观测
 * 本身就产生在同一個模型下（流水线把估计器声明的模型交给前端），因此
 * 阈值与观测同源，不与某个具体的 @c pinholeScale 暗中绑定。
 *
 * 本类无状态：上一关键帧的时间戳由编排层持有并显式传入，避免「决策器自己
 * 记一份时间、流水线再记一份」这类无法对账的重复状态。
 *
 * 约束：只依赖领域类型。
 */

#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/types.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <span>
#include <string>
#include <unordered_map>

namespace myslam::domain
{

    /**
     * @brief 关键帧决策结果
     */
    struct KeyframeDecision
    {
        bool isKeyframe{false};
        std::string reason;
        double avgParallaxPx{0.0};
        double trackingRatio{1.0};
    };

    /**
     * @brief 自适应关键帧决策器
     */
    class KeyframeSelector
    {
    public:
        explicit KeyframeSelector(const VisionConfig &config)
            : config_(config)
        {
        }

        /**
         * @brief 判断当前帧是否应作为关键帧
         *
         * @param current 当前帧
         * @param lastKeyframe 上一成功关键帧的观测（为空且时间戳 < 0 表示首帧）
         * @param lastKeyframeTimestamp 上一成功关键帧的时刻 (s)
         *
         * @note 观测像素必须与阈值采用同一模型。由流水线保证：它把估计器声明的
         *       观测模型交给前端使用，因此"配置里的像素阈值"与"观测像素"同源。
         */
        [[nodiscard]] KeyframeDecision evaluate(const Frame &current, std::span<const Observation> lastKeyframe,
                                                double lastKeyframeTimestamp) const
        {
            if (lastKeyframe.empty() && lastKeyframeTimestamp < 0.0)
            {
                return {true, "第一帧初始关键帧", 0.0, 1.0};
            }

            std::unordered_map<FeatureId, const Observation *> previous;
            previous.reserve(lastKeyframe.size());
            for (const Observation &observation : lastKeyframe)
            {
                previous[observation.id] = &observation;
            }

            // 上一关键帧没有任何观测（视觉中断期间插入的帧）时，共视率与视差
            // 都无从计算。若此时仍按"共视率 100%"处理，三条判据会全部失效，
            // 视觉恢复后永远不再插入关键帧——轨迹停在旧基线直到 IMU 断档重建。
            // 因此这里把它当成"必须重建基线"：本帧有观测就成为关键帧。
            if (previous.empty())
            {
                if (current.observations.empty())
                {
                    return {false, "上一关键帧与本帧均无观测，无可建立基线", 0.0, 1.0};
                }
                return {true, "上一关键帧无观测，重建共视基线", 0.0, 1.0};
            }

            int coVisible = 0;
            double totalParallax = 0.0;
            for (const Observation &observation : current.observations)
            {
                const auto it = previous.find(observation.id);
                if (it == previous.end())
                {
                    continue;
                }
                ++coVisible;
                totalParallax += pixelParallax(*it->second, observation);
            }

            const double avgParallax = coVisible > 0 ? totalParallax / static_cast<double>(coVisible) : 0.0;
            const double trackingRatio = static_cast<double>(coVisible) / static_cast<double>(previous.size());

            const double dt = current.timestamp - lastKeyframeTimestamp;
            if (dt < config_.minKeyframeInterval)
            {
                return {false, "时间间隔未达最小视差基线要求 (保持高频推算)", 0.0, 1.0};
            }

            if (avgParallax >= config_.minParallaxPx)
            {
                return {true, std::format("视差充分 ({:.2f} px >= {:.2f} px)", avgParallax, config_.minParallaxPx),
                        avgParallax, trackingRatio};
            }

            if (trackingRatio < config_.minTrackingRatio)
            {
                return {true,
                        std::format("共视特征比例低 ({:.1f}% < {:.1f}%)", trackingRatio * 100.0,
                                    config_.minTrackingRatio * 100.0),
                        avgParallax, trackingRatio};
            }

            // 超时保底：静止/悬停时视差 ≈ 0，插关键帧只会让图欠约束，
            // 因此还要求达到三角化最小视差
            if (dt >= config_.maxKeyframeInterval && avgParallax >= config_.minTriangulationParallaxPx)
            {
                return {true,
                        std::format("时间跨度达到保底上限 ({:.2f}s >= {:.2f}s) 且视差可用 ({:.2f} px)", dt,
                                    config_.maxKeyframeInterval, avgParallax),
                        avgParallax, trackingRatio};
            }

            return {false, "非关键帧 (保持高频推算)", avgParallax, trackingRatio};
        }

        /**
         * @brief 两条观测的等效像素视差
         *
         * 直接取像素欧氏距离。**不能**用两条 @c normalized 射线的夹角代替：
         * 它们分属两帧各自的相机坐标系，夹角只反映相对姿态变化的一部分，
         * 不是视差（实测会把 20 px 的判据压到 6 px，在阈值附近改变关键帧
         * 节奏并让精度退化）。跨帧几何量必须在同一参考系里比较。
         *
         * 像素阈值的物理含义取决于观测模型：同一个 @c minParallaxPx 在 MEI 与
         * 虚拟针孔下对应不同的角度。这是配置语义——阈值必须与观测像素同源。
         */
        [[nodiscard]] static double pixelParallax(const Observation &lhs, const Observation &rhs)
        {
            return (lhs.uv - rhs.uv).norm();
        }

    private:
        VisionConfig config_;
    };

} // namespace myslam::domain
