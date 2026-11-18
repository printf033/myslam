#pragma once

/**
 * @file mei_undistort.hpp
 * @brief 把原始鱼眼图重映射成目标画幅，供可视化对照。
 *
 * @par 映射链路由相机模型定义
 *
 * 对每个目标像素执行 @c target->unproject 得到射线，再用 @c source->project
 * 投回原始像素。这条链路与跟踪器完全一致（同一个模型对），因此叠加在
 * 去畸变图上的特征点不会错位。
 *
 * 重构前本文件自己重写了一遍 MEI 投影公式，与跟踪器各自维护——任何一处
 * 公式修正都要在两处同步。现在只有模型知道公式，本文件只负责查表。
 */

#include "domain/camera_model.hpp"
#include <cmath>
#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace myslam::viz
{

    /**
     * @brief 原始图像 -> 目标画幅的重映射器
     */
    class MeiUndistorter
    {
    public:
        MeiUndistorter(std::shared_ptr<const domain::ICameraModel> source,
                       std::shared_ptr<const domain::ICameraModel> target)
            : source_(std::move(source)),
              target_(std::move(target)),
              mapX_(target_->imageHeight(), target_->imageWidth(), CV_32FC1),
              mapY_(target_->imageHeight(), target_->imageWidth(), CV_32FC1),
              sourceMask_(source_->imageHeight(), source_->imageWidth(), CV_8UC1, cv::Scalar(0))
        {
            buildMaps();
        }

        /**
         * @brief 把一帧原始图重映射为目标画幅
         *
         * @return 同尺寸图像；输入为空或尺寸不符时返回空 Mat
         */
        [[nodiscard]] cv::Mat apply(const cv::Mat &raw) const
        {
            if (raw.empty() || raw.cols != source_->imageWidth() || raw.rows != source_->imageHeight())
            {
                return {};
            }
            cv::Mat out;
            cv::remap(raw, out, mapX_, mapY_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
            return out;
        }

        /// 目标画幅内可映射回原图的像素比例
        [[nodiscard]] double validRatio() const noexcept { return validRatio_; }

        /**
         * @brief 原始图像上的有效视场掩码（CV_8UC1，255 = 该像素能映射进目标画幅）
         *
         * 由同一对模型算出，供可视化描出"哪些原图像素真正参与后端约束"。
         * 与跟踪器内部的补点掩码同源但独立：可视化不应读取前端的私有状态。
         */
        [[nodiscard]] const cv::Mat &sourceValidMask() const noexcept { return sourceMask_; }

    private:
        void buildMaps()
        {
            const int width = target_->imageWidth();
            const int height = target_->imageHeight();
            std::size_t valid = 0;

            for (int y = 0; y < height; ++y)
            {
                for (int x = 0; x < width; ++x)
                {
                    float sourceX = -1.0F;
                    float sourceY = -1.0F;
                    if (const auto ray = target_->unproject(domain::Vec2(x, y)); ray.has_value())
                    {
                        if (const auto raw = source_->project(*ray); raw.has_value())
                        {
                            // 必须同时判结果落在源画幅内，否则 validRatio_ 会把
                            // 「投影成功但落在原图之外」也计为可用；而下面
                            // sourceMask_ 的构造是判边界的。同一文件两套口径。
                            if (raw->x() >= 0.0 && raw->x() < static_cast<double>(source_->imageWidth()) &&
                                raw->y() >= 0.0 && raw->y() < static_cast<double>(source_->imageHeight()))
                            {
                                sourceX = static_cast<float>(raw->x());
                                sourceY = static_cast<float>(raw->y());
                                ++valid;
                            }
                        }
                    }
                    mapX_.at<float>(y, x) = sourceX;
                    mapY_.at<float>(y, x) = sourceY;
                }
            }

            const double total = static_cast<double>(width) * static_cast<double>(height);
            validRatio_ = total > 0.0 ? static_cast<double>(valid) / total : 0.0;

            // 原图侧掩码：逐像素问"它能落到目标画幅里吗"。与 buildMaps 的
            // 映射方向相反，但用的是同一对模型与同一套边界判定。
            for (int y = 0; y < source_->imageHeight(); ++y)
            {
                for (int x = 0; x < source_->imageWidth(); ++x)
                {
                    const auto ray = source_->unproject(domain::Vec2(x, y));
                    if (!ray)
                    {
                        continue;
                    }
                    const auto projected = target_->project(*ray);
                    if (!projected || projected->x() < 0.0 || projected->y() < 0.0 ||
                        projected->x() >= static_cast<double>(width) || projected->y() >= static_cast<double>(height))
                    {
                        continue;
                    }
                    sourceMask_.at<uchar>(y, x) = 255;
                }
            }
        }

        std::shared_ptr<const domain::ICameraModel> source_;
        std::shared_ptr<const domain::ICameraModel> target_;
        cv::Mat mapX_;
        cv::Mat mapY_;
        cv::Mat sourceMask_;
        double validRatio_{0.0};
    };

} // namespace myslam::viz
