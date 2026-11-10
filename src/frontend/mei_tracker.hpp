#pragma once

/**
 * @file mei_tracker.hpp
 * @brief GFTT + 金字塔 LK 光流前端，实现 @c IObservationExtractor。
 *
 * @par 相机数学不再散落在这里
 *
 * 重构前本文件自己实现了 MEI 反投影、虚拟针孔缩放与视场掩码，与
 * @c camera_model.hpp、@c projection_model.hpp 各存一份公式。现在它只做
 * 两件事：用 OpenCV 检测/跟踪原始像素，用**源模型**把像素变成射线、
 * 用**目标模型**把射线变成观测像素。模型是什么由装配层决定。
 *
 * 观测里同时带三样东西：
 *   - @c uv：目标模型（后端消费）的像素；
 *   - @c normalized：相机系射线，供与模型无关的几何判断；
 *   - @c sourcePixel：原始鱼眼像素，供可视化叠加。
 *
 * 这些以前分散在旁路数组里，现在随观测一起流动，可视化不再需要
 * @c rawPixels / @c validMask 这类实现细节。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/camera_model.hpp"
#include "domain/ports.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace myslam::vio
{

    /**
     * @brief GFTT + LK 光流特征跟踪器
     */
    class MeiFeatureTracker final : public domain::IObservationExtractor
    {
    public:
        /**
         * @param intrinsics 原始相机内参
         * @param sourceModel 原始像素所属模型（MEI 或针孔）
         * @param targetModel 观测输出模型（后端消费的模型）
         */
        MeiFeatureTracker(const camera::CameraIntrinsics &intrinsics,
                          std::shared_ptr<const domain::ICameraModel> sourceModel,
                          std::shared_ptr<const domain::ICameraModel> targetModel, int maxFeatures = 400,
                          int minFeatures = 90)
            : intrinsics_(intrinsics),
              source_(std::move(sourceModel)),
              target_(std::move(targetModel)),
              maxFeatures_(std::max(1, maxFeatures)),
              minFeatures_(std::clamp(minFeatures, 0, std::max(1, maxFeatures))),
              K_((cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0,
                  0.0, 1.0)),
              D_((cv::Mat_<double>(1, 5) << intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2, 0.0))
        {
            if (!source_ || !target_)
            {
                mylog::error("[特征跟踪] 源/目标相机模型为空，观测提取将全部失败");
            }
            buildValidMask();
        }

        void reset() override
        {
            prevGray_.release();
            prevPoints_.clear();
            prevIds_.clear();
            nextFeatureId_ = 0;
        }

        bool extract(const domain::ImageView &image, int frameIndex, double timestamp,
                     std::vector<domain::Observation> &out) override
        {
            out.clear();
            (void)timestamp;

            if (!image.valid() || image.format != domain::PixelFormat::GRAY8 || !source_ || !target_)
            {
                return false;
            }
            if (image.width != intrinsics_.imageWidth || image.height != intrinsics_.imageHeight)
            {
                mylog::warn("[特征跟踪] 图像尺寸 {}x{} 与标定 {}x{} 不一致，跳过本帧", image.width, image.height,
                            intrinsics_.imageWidth, intrinsics_.imageHeight);
                return false;
            }

            // 只读视图：OpenCV 的 Mat 构造需要非 const 指针，本函数不写入任何像素
            const cv::Mat gray(image.height, image.width, CV_8UC1, const_cast<std::uint8_t *>(image.data),
                               static_cast<std::size_t>(image.stride));

            std::vector<cv::Point2f> candidates;
            std::vector<int> candidateIds;
            std::size_t trackedCount = 0;
            const std::size_t prevCount = prevPoints_.size();

            if (!prevGray_.empty() && !prevPoints_.empty())
            {
                std::vector<cv::Point2f> nextPoints;
                std::vector<uchar> status;
                std::vector<float> err;
                cv::calcOpticalFlowPyrLK(prevGray_, gray, prevPoints_, nextPoints, status, err, cv::Size(31, 31), 4);

                for (std::size_t i = 0; i < nextPoints.size(); ++i)
                {
                    if (status[i] == 0 || err[i] > 50.0F || !inBounds(nextPoints[i]))
                    {
                        continue;
                    }
                    candidates.push_back(nextPoints[i]);
                    candidateIds.push_back(prevIds_[i]);
                    ++trackedCount;
                }
            }

            std::vector<cv::Point2f> validRaw;
            std::vector<int> validIds;
            std::vector<domain::Observation> validObservations;
            std::size_t rejected = 0;

            auto filterCandidates =
                [&](const std::vector<cv::Point2f> &raw, const std::vector<int> &ids, domain::ObservationKind kind)
            {
                for (std::size_t i = 0; i < raw.size(); ++i)
                {
                    domain::Observation observation;
                    observation.id = ids[i];
                    observation.kind = kind;
                    if (!mapPixel(domain::Vec2(raw[i].x, raw[i].y), observation))
                    {
                        ++rejected;
                        continue;
                    }
                    validRaw.push_back(raw[i]);
                    validIds.push_back(ids[i]);
                    validObservations.push_back(observation);
                }
            };

            filterCandidates(candidates, candidateIds, domain::ObservationKind::TRACKED);
            const std::size_t trackedValid = validRaw.size();

            if (static_cast<int>(validRaw.size()) < minFeatures_)
            {
                cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar(255));
                cv::bitwise_and(mask, validMask_, mask);
                for (const auto &point : validRaw)
                {
                    cv::circle(mask, point, 8, cv::Scalar(0), cv::FILLED);
                }

                std::vector<cv::Point2f> fresh;
                const int budget = maxFeatures_ - static_cast<int>(validRaw.size());
                if (budget > 0)
                {
                    cv::goodFeaturesToTrack(gray, fresh, budget, 0.01, 10.0, mask);
                }
                std::vector<int> freshIds;
                freshIds.reserve(fresh.size());
                for (std::size_t i = 0; i < fresh.size(); ++i)
                {
                    freshIds.push_back(nextFeatureId_++);
                }
                filterCandidates(fresh, freshIds, domain::ObservationKind::NEW);
            }

            prevGray_ = gray.clone();
            prevPoints_ = validRaw;
            prevIds_ = validIds;
            out = std::move(validObservations);

            mylog::debug("[特征跟踪] 帧 #{}: 跟踪 {}/{} 点, 新增 {}, 模型拒绝 {}, 保留 {}", frameIndex, trackedCount,
                         prevCount, out.size() - trackedValid, rejected, out.size());
            return true;
        }

        /// 有效视场掩码（CV_8U，255 = 该原始像素能映射进目标画幅）
        [[nodiscard]] const cv::Mat &validMask() const noexcept { return validMask_; }

    private:
        /**
         * @brief 原始像素 -> 射线 -> 目标模型像素
         */
        [[nodiscard]] bool mapPixel(const domain::Vec2 &raw, domain::Observation &observation) const
        {
            const auto ray = source_->unproject(raw);
            if (!ray)
            {
                return false;
            }
            const auto uv = target_->project(*ray);
            if (!uv)
            {
                return false; // 超出目标模型可见范围（例如鱼眼视场边缘）
            }
            if (uv->x() < 0.0 || uv->x() >= static_cast<double>(target_->imageWidth()) || uv->y() < 0.0 ||
                uv->y() >= static_cast<double>(target_->imageHeight()))
            {
                return false;
            }
            observation.uv = *uv;
            observation.normalized = *ray;
            observation.sourcePixel = raw;
            return true;
        }

        /**
         * @brief 预计算目标画幅内的有效原始像素掩码
         *
         * 鱼眼视场远大于虚拟针孔画幅，直接在整幅图上检测角点会浪费大量被
         * 过滤的特征。构造时逐像素判定一次，之后 GFTT 只在该掩码内补点。
         */
        void buildValidMask()
        {
            validMask_ = cv::Mat::zeros(intrinsics_.imageHeight, intrinsics_.imageWidth, CV_8UC1);
            if (!source_ || !target_)
            {
                return;
            }
            domain::Observation probe;
            for (int y = 0; y < intrinsics_.imageHeight; ++y)
            {
                for (int x = 0; x < intrinsics_.imageWidth; ++x)
                {
                    if (mapPixel(domain::Vec2(x, y), probe))
                    {
                        validMask_.at<uchar>(y, x) = 255;
                    }
                }
            }
            const auto total = static_cast<double>(intrinsics_.imageWidth) * intrinsics_.imageHeight;
            const auto available = static_cast<double>(cv::countNonZero(validMask_));
            mylog::info("[特征跟踪] 有效视场掩码: {:.0f}/{:.0f} 像素可用 ({:.1f}%)", available, total,
                        total > 0.0 ? 100.0 * available / total : 0.0);
        }

        [[nodiscard]] bool inBounds(const cv::Point2f &point) const
        {
            return point.x >= 0.0F && point.x < static_cast<float>(intrinsics_.imageWidth) && point.y >= 0.0F &&
                   point.y < static_cast<float>(intrinsics_.imageHeight);
        }

        camera::CameraIntrinsics intrinsics_;
        std::shared_ptr<const domain::ICameraModel> source_;
        std::shared_ptr<const domain::ICameraModel> target_;
        int maxFeatures_;
        int minFeatures_;
        cv::Mat K_;
        cv::Mat D_;
        cv::Mat validMask_;

        cv::Mat prevGray_;
        std::vector<cv::Point2f> prevPoints_;
        std::vector<int> prevIds_;
        int nextFeatureId_{0};
    };

} // namespace myslam::vio
