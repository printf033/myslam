#pragma once

/**
 * @file mei_tracker.hpp
 * @brief MEI 鱼眼模型的像素校正与光流特征跟踪。
 *
 * myslam 的后端使用 `gtsam::Cal3_S2`（无畸变针孔）做重投影，因此鱼眼图像上
 * 的像素必须转换为「去畸变针孔像素」。本文件实现两级换算：
 *   1. 用 OpenCV 的 radtan 模型反解 MEI 的径向/切向畸变，得到归一化平面坐标；
 *   2. 按 MEI (Mei & Rives) 模型把归一化坐标反投影为归一化射线，再用虚拟针孔
 *      内参投影回像素平面。
 *
 * 虚拟针孔内参沿用 MASt3R-Fusion 的做法：焦距乘以 `pinholeScale`（默认 0.4），
 * 主点不变。MEI 相机的 xi 可达 1.5+（视场 > 180°），若直接用原始焦距投影，
 * 视场边缘会被推到图像之外；缩放焦距后可在 640x480 画幅内保留前向视场，
 * 且与 MASt3R-Fusion 的图像预处理保持一致。
 *
 * 特征跟踪采用 GFTT 角点 + 金字塔 LK 光流：跨帧保留 featureId，供后端按 ID
 * 建立不同关键帧之间的共视关系并三角化。
 */

#include "dataset/types.hpp"
#include "logger.hpp"
#include "vio/types.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief MEI 像素 -> 去畸变针孔像素
     *
     * @param normalized 已去除 radtan 畸变的归一化平面坐标 (x, y)
     * @param[out] pinhole 虚拟针孔像素坐标；射线不可见（Z <= 0）时返回 false
     */
    inline bool meiToPinhole(const MeiIntrinsics &intr, cv::Point2d normalized, gtsam::Point2 &pinhole)
    {
        const double x = normalized.x;
        const double y = normalized.y;
        if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > 20.0 || std::abs(y) > 20.0)
        {
            return false; // 畸变反解失败或数值溢出
        }
        const double r2 = x * x + y * y;

        // Mei & Rives 反投影：单位球面点 mx, my, mz
        const double d = std::sqrt(1.0 + (1.0 - intr.xi * intr.xi) * r2);
        const double denom = 1.0 + r2;
        const double mx = (intr.xi + d) / denom * x;
        const double my = (intr.xi + d) / denom * y;
        const double mz = (intr.xi + d) / denom - intr.xi;

        if (mz <= 1e-6)
        {
            return false; // 超出针孔可见范围（视场边缘）
        }
        pinhole =
            gtsam::Point2(intr.pinholeFx() * mx / mz + intr.pinholeCx(), intr.pinholeFy() * my / mz + intr.pinholeCy());
        return std::isfinite(pinhole.x()) && std::isfinite(pinhole.y());
    }

    /**
     * @brief GFTT + LK 光流特征跟踪器，输出符合后端约定的 CameraFrame
     */
    class MeiFeatureTracker
    {
    public:
        explicit MeiFeatureTracker(const MeiIntrinsics &intr, int maxFeatures = 400, int minFeatures = 90)
            : intr_(intr),
              maxFeatures_(maxFeatures),
              minFeatures_(minFeatures),
              K_((cv::Mat_<double>(3, 3) << intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0)),
              D_((cv::Mat_<double>(1, 5) << intr.k1, intr.k2, intr.p1, intr.p2, 0.0))
        {
            buildValidMask();
        }

        /**
         * @brief 处理一帧灰度图，返回带跨帧 featureId 的特征集合
         */
        vio::CameraFrame track(const cv::Mat &gray, int frameId, double timestamp)
        {
            vio::CameraFrame frame;
            frame.frameId = frameId;
            frame.timestamp = timestamp;

            std::vector<cv::Point2f> candidates; // 原始像素坐标
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

            // 只保留 MEI 视场内的点参与下一帧跟踪：越界点若留在跟踪列表里会
            // 持续漂移并占位，最终把有效特征挤到零。
            std::vector<cv::Point2f> validRaw;
            std::vector<int> validIds;
            std::vector<gtsam::Point2> validUv;
            std::size_t rejectedByMei = 0;
            std::size_t rejectedByBounds = 0;

            auto filterCandidates = [&](const std::vector<cv::Point2f> &raw, const std::vector<int> &ids)
            {
                if (raw.empty())
                {
                    return;
                }
                std::vector<cv::Point2f> normalized;
                cv::undistortPoints(raw, normalized, K_, D_);
                for (std::size_t i = 0; i < raw.size(); ++i)
                {
                    gtsam::Point2 pinhole;
                    if (!meiToPinhole(intr_, cv::Point2d(normalized[i].x, normalized[i].y), pinhole))
                    {
                        ++rejectedByMei;
                        continue;
                    }
                    if (pinhole.x() < 0.0 || pinhole.x() >= intr_.imageWidth || pinhole.y() < 0.0 ||
                        pinhole.y() >= intr_.imageHeight)
                    {
                        ++rejectedByBounds;
                        continue;
                    }
                    validRaw.push_back(raw[i]);
                    validIds.push_back(ids[i]);
                    validUv.push_back(pinhole);
                }
            };

            filterCandidates(candidates, candidateIds);
            const std::size_t trackedValid = validRaw.size();

            // 有效点不足时补测新角点，新点获得新的全局 featureId
            if (static_cast<int>(validRaw.size()) < minFeatures_)
            {
                cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar(255));
                cv::bitwise_and(mask, validMask_, mask);
                for (const auto &pt : validRaw)
                {
                    cv::circle(mask, pt, 8, cv::Scalar(0), cv::FILLED);
                }
                std::vector<cv::Point2f> fresh;
                cv::goodFeaturesToTrack(gray, fresh, maxFeatures_ - static_cast<int>(validRaw.size()), 0.01, 10.0,
                                        mask);
                if (!fresh.empty())
                {
                    std::vector<int> freshIds;
                    freshIds.reserve(fresh.size());
                    for (std::size_t i = 0; i < fresh.size(); ++i)
                    {
                        freshIds.push_back(nextFeatureId_++);
                    }
                    filterCandidates(fresh, freshIds);
                }
            }

            const std::size_t newCount = validRaw.size() - trackedValid;
            const std::size_t firstNewIndex = trackedValid;

            prevGray_ = gray.clone();
            prevPoints_ = validRaw;
            prevIds_ = validIds;

            frame.features.reserve(validRaw.size());
            for (std::size_t i = 0; i < validRaw.size(); ++i)
            {
                vio::FeatureObservation obs;
                obs.featureId = validIds[i];
                obs.uv = validUv[i];
                obs.uvNorm = gtsam::Point2((validUv[i].x() - intr_.pinholeCx()) / intr_.pinholeFx(),
                                           (validUv[i].y() - intr_.pinholeCy()) / intr_.pinholeFy());
                obs.isNew = i >= firstNewIndex;
                frame.features.push_back(obs);
            }
            mylog::debug("[特征跟踪] 帧 #{}: 跟踪 {}/{} 点, 新增 {}, MEI 拒绝 {}, 越界 {}, 保留 {}", frameId,
                         trackedCount, prevCount, newCount, rejectedByMei, rejectedByBounds, frame.features.size());
            return frame;
        }

    private:
        /**
         * @brief 预计算 MEI 视场落在虚拟针孔画幅内的像素掩码
         *
         * 鱼眼视场远大于 640x480 的虚拟针孔画幅，直接在整幅图上检测角点会
         * 浪费大量被越界过滤的特征。构造时用批量 undistortPoints 逐像素判定，
         * 之后 GFTT 只在该掩码内补充新点。
         */
        void buildValidMask()
        {
            validMask_ = cv::Mat::zeros(intr_.imageHeight, intr_.imageWidth, CV_8UC1);
            std::vector<cv::Point2f> pixels;
            pixels.reserve(static_cast<std::size_t>(intr_.imageWidth) * intr_.imageHeight);
            for (int y = 0; y < intr_.imageHeight; ++y)
            {
                for (int x = 0; x < intr_.imageWidth; ++x)
                {
                    pixels.emplace_back(static_cast<float>(x), static_cast<float>(y));
                }
            }

            std::vector<cv::Point2f> normalized;
            cv::undistortPoints(pixels, normalized, K_, D_);
            for (std::size_t i = 0; i < pixels.size(); ++i)
            {
                gtsam::Point2 pinhole;
                if (!meiToPinhole(intr_, cv::Point2d(normalized[i].x, normalized[i].y), pinhole))
                {
                    continue;
                }
                if (pinhole.x() < 0.0 || pinhole.x() >= intr_.imageWidth || pinhole.y() < 0.0 ||
                    pinhole.y() >= intr_.imageHeight)
                {
                    continue;
                }
                const auto col = static_cast<int>(pixels[i].x);
                const auto row = static_cast<int>(pixels[i].y);
                validMask_.at<uchar>(row, col) = 255;
            }
            mylog::info("[特征跟踪] MEI 有效视场掩码: {}/{} 像素可用 ({:.1f}%)", cv::countNonZero(validMask_),
                        intr_.imageWidth * intr_.imageHeight,
                        100.0 * cv::countNonZero(validMask_) / (intr_.imageWidth * intr_.imageHeight));
        }

        bool inBounds(cv::Point2f pt) const
        {
            return pt.x >= 0.0F && pt.x < static_cast<float>(intr_.imageWidth) && pt.y >= 0.0F &&
                   pt.y < static_cast<float>(intr_.imageHeight);
        }

        MeiIntrinsics intr_;
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

} // namespace myslam::dataset
