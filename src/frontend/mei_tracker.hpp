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
#include <limits>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <unordered_map>
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
         * @param vision 视觉配置：特征数下限、RANSAC 与网格均匀化参数
         * @param maxFeatures 每帧特征数上限（来自数据集回放配置）
         */
        MeiFeatureTracker(const camera::CameraIntrinsics &intrinsics,
                          std::shared_ptr<const domain::ICameraModel> sourceModel,
                          std::shared_ptr<const domain::ICameraModel> targetModel, const domain::VisionConfig &vision,
                          int maxFeatures)
            : intrinsics_(intrinsics),
              source_(std::move(sourceModel)),
              target_(std::move(targetModel)),
              vision_(vision),
              maxFeatures_(std::max(1, maxFeatures)),
              minFeatures_(std::clamp(vision.minFeatures, 0, std::max(1, maxFeatures)))
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
            prevNormalized_.clear();
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

            std::vector<cv::Point2f> validRaw;
            std::vector<int> validIds;
            std::vector<domain::Observation> validObservations;
            // RANSAC 用的归一化平面对（与 validRaw 同序，只管 tracked）
            std::vector<cv::Point2f> validPrevNormalized;
            std::vector<cv::Point2f> validCurNormalized;
            std::size_t rejected = 0;
            std::size_t trackedCount = 0;
            const std::size_t prevCount = prevPoints_.size();
            const bool hadPreviousFrame = !prevGray_.empty();

            // 0. 先补点（在上一帧）——默认路径
            //
            // 顺序不是风格问题：在**上一帧**补点再跟踪到当前帧，新点当帧就
            // 获得一次可被光流与几何校验验证的观测。反过来（跟踪完再在当前
            // 帧补点）则让新点未经任何校验直接进入后端，一个误检的角点会
            // 以一个「看起来正常」的因子进图。
            std::size_t preAdded = 0;
            if (vision_.detectBeforeTrack && hadPreviousFrame && static_cast<int>(prevPoints_.size()) < minFeatures_ &&
                prevNormalized_.size() == prevPoints_.size())
            {
                const int budget = maxFeatures_ - static_cast<int>(prevPoints_.size());
                if (budget > 0)
                {
                    preAdded = detectOnPreviousFrame(prevGray_, budget);
                }
            }

            // 1. 跟踪 + 前后向校验 + 模型映射
            //
            // prevNormalized_ 必须与 prevPoints_ 等长，否则上一帧的归一化
            // 坐标对不上号，几何校验会拿错点作比较。下面显式检查而不是
            // 假定不变量成立。
            if (!prevGray_.empty() && !prevPoints_.empty() && prevNormalized_.size() == prevPoints_.size())
            {
                std::vector<cv::Point2f> nextPoints;
                std::vector<uchar> status;
                std::vector<float> err;
                cv::calcOpticalFlowPyrLK(prevGray_, gray, prevPoints_, nextPoints, status, err, cv::Size(31, 31), 4);

                // 前向-后向一致性校验：OpenCV 的 err 输出是末层金字塔 SAD 代价（31x31 窗口），
                // 单位不是像素，完美匹配也可能达几十到上百，不能当残差门限用。标准做法是反向
                // 再跑一次 LK，往返残差超过 ~2x 像素噪声(1px) 即判为坏跟踪（ORB-SLAM 同款）。
                std::vector<cv::Point2f> backPoints;
                std::vector<uchar> backStatus;
                std::vector<float> backErr;
                cv::calcOpticalFlowPyrLK(gray, prevGray_, nextPoints, backPoints, backStatus, backErr, cv::Size(31, 31),
                                         4);

                for (std::size_t i = 0; i < nextPoints.size(); ++i)
                {
                    if (status[i] == 0 || !inBounds(nextPoints[i]))
                    {
                        continue;
                    }
                    const float dx = backPoints[i].x - prevPoints_[i].x;
                    const float dy = backPoints[i].y - prevPoints_[i].y;
                    if (backStatus[i] == 0 || std::hypot(dx, dy) > 2.0F)
                    {
                        continue;
                    }

                    domain::Observation observation;
                    observation.id = prevIds_[i];
                    observation.kind = domain::ObservationKind::TRACKED;
                    if (!mapPixel(domain::Vec2(nextPoints[i].x, nextPoints[i].y), observation))
                    {
                        ++rejected;
                        continue;
                    }

                    cv::Point2f curNormalized;
                    if (!observation.normalized.has_value() ||
                        !toNormalizedPlane(*observation.normalized, curNormalized))
                    {
                        ++rejected;
                        continue;
                    }

                    validRaw.push_back(nextPoints[i]);
                    validIds.push_back(prevIds_[i]);
                    validObservations.push_back(observation);
                    validPrevNormalized.push_back(prevNormalized_[i]);
                    validCurNormalized.push_back(curNormalized);
                    ++trackedCount;
                }
            }

            // 2. 几何一致性（基础矩阵 RANSAC）
            //
            // 前后向校验只能抓「光流自己不确定」的失败，抓不住「前后向一致地
            // 跟到了错误纹理上」。后者只有几何约束能发现。
            std::size_t ransacRemoved = 0;
            if (vision_.enableRansac && validRaw.size() >= 8)
            {
                ransacRemoved = rejectGeometricOutliers(validPrevNormalized, validCurNormalized, validRaw, validIds,
                                                        validObservations);
            }

            // 2.5 活动轨迹去重
            //
            // 检测掩码只能排除「新点与已有点重合」，管不住「两条已存在的
            // 轨迹逐渐收敛到同一片纹理」。同一物理点被两条轨迹各报一次，
            // 就是同一约束在因子图里被计两次信息，估计器会过度自信。
            const std::size_t deduped =
                dedupeActiveTracks(validRaw, validIds, validObservations, validPrevNormalized, validCurNormalized);
            const std::size_t trackedValid = validRaw.size();

            // 3. 补点
            //
            // 两条路径：
            //  - @c detectBeforeTrack（已在步骤 0 在上一帧补过）：这里只处理
            //    首帧冷启动，不再往当前帧塞未经验证的新点；
            //  - 旧的「先跟踪后补点」：保留以做对照实验。
            std::size_t addedFresh = 0;
            const bool needCurrentFrameDetection = !vision_.detectBeforeTrack || !hadPreviousFrame;
            if (needCurrentFrameDetection && static_cast<int>(validRaw.size()) < minFeatures_)
            {
                const int budget = maxFeatures_ - static_cast<int>(validRaw.size());
                if (budget > 0)
                {
                    addedFresh = detectUniform(gray, validRaw, validIds, validObservations, budget);
                }
            }

            // 4. 保存上一帧状态（点、ID、归一化坐标三者严格同序）
            prevGray_ = gray.clone();
            prevPoints_ = validRaw;
            prevIds_ = validIds;
            prevNormalized_.clear();
            prevNormalized_.reserve(validObservations.size());
            for (const domain::Observation &observation : validObservations)
            {
                cv::Point2f normalized{0.0F, 0.0F};
                if (!observation.normalized.has_value() || !toNormalizedPlane(*observation.normalized, normalized))
                {
                    // 退化射线的点不参与下一帧的几何校验，但仍是有效观测。
                    normalized =
                        cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
                }
                prevNormalized_.push_back(normalized);
            }
            out = std::move(validObservations);

            mylog::debug(
                "[特征跟踪] 帧 #{}: 跟踪 {}/{} 点, RANSAC 剔除 {}, 去重 {}, 预补 {}, 新增 {}, 模型拒绝 {}, 保留 {}",
                frameIndex, trackedCount, prevCount, ransacRemoved, deduped, preAdded, addedFresh, rejected,
                out.size());
            (void)trackedValid;
            return true;
        }

        /// 有效视场掩码（CV_8U，255 = 该原始像素能映射进目标画幅）
        [[nodiscard]] const cv::Mat &validMask() const noexcept { return validMask_; }

    private:
        /**
         * @brief 相机系射线 -> 归一化平面坐标 (x/z, y/z)
         *
         * 几何校验必须在一个与像素模型无关的坐标系里做：MEI 原图像素与
         * 虚拟针孔像素的尺度差一个 pinholeScale，拿像素做阈值会让阈值
         * 在两种模型下含义不同。z 近于零的射线（视场边缘）直接判失败。
         */
        [[nodiscard]] static bool toNormalizedPlane(const domain::Vec3 &ray, cv::Point2f &out) noexcept
        {
            if (std::abs(ray.z()) < 1e-9)
            {
                return false;
            }
            const auto x = static_cast<float>(ray.x() / ray.z());
            const auto y = static_cast<float>(ray.y() / ray.z());
            if (!std::isfinite(x) || !std::isfinite(y))
            {
                return false;
            }
            out = cv::Point2f(x, y);
            return true;
        }

        /**
         * @brief 用基础矩阵 RANSAC 剔除几何不一致的跟踪
         *
         * 阈值以归一化单位表达 = @c ransacThresholdSigma * 像素噪声 / 焦距，
         * 因此与分辨率、焦距、以及 MEI 去畸变缩放都无关。
         *
         * 关键保护：内点占比过低时**放弃本次剔除**。纯旋转、低视差、平面
         * 场景下基础矩阵病态，RANSAC 会把大片正确跟踪判成外点；一次误杀
         * 比漏掉几个外点代价大得多。
         *
         * @return 被剔除的点数（0 表示未剔除或已放弃）
         */
        std::size_t rejectGeometricOutliers(std::vector<cv::Point2f> &prevNormalized,
                                            std::vector<cv::Point2f> &curNormalized,
                                            std::vector<cv::Point2f> &rawPoints, std::vector<int> &ids,
                                            std::vector<domain::Observation> &observations)
        {
            const double focal = std::max(1e-6, target_->focalLengthPx());
            const double threshold = std::max(1e-9, vision_.ransacThresholdSigma * vision_.pixelNoiseSigma / focal);

            std::vector<uchar> inlierMask;
            try
            {
                cv::findFundamentalMat(prevNormalized, curNormalized, cv::FM_RANSAC, threshold,
                                       vision_.ransacConfidence, inlierMask);
            }
            catch (const cv::Exception &error)
            {
                mylog::warn("[RANSAC] 基础矩阵求解异常，本帧跳过几何校验: {}", error.what());
                return 0;
            }

            if (inlierMask.size() != prevNormalized.size())
            {
                return 0;
            }

            std::size_t inliers = 0;
            for (uchar value : inlierMask)
            {
                if (value != 0)
                {
                    ++inliers;
                }
            }
            const std::size_t total = inlierMask.size();
            const std::size_t minimumInliers = std::max<std::size_t>(8, total / 4);
            if (inliers < minimumInliers)
            {
                mylog::warn("[RANSAC] 内点仅 {}/{}, 低于安全下限 {}，本帧放弃几何剔除（疑似纯旋转或低视差退化）",
                            inliers, total, minimumInliers);
                return 0;
            }

            std::vector<cv::Point2f> keptRaw;
            std::vector<int> keptIds;
            std::vector<domain::Observation> keptObservations;
            std::vector<cv::Point2f> keptPrev;
            std::vector<cv::Point2f> keptCur;
            keptRaw.reserve(inliers);
            keptIds.reserve(inliers);
            keptObservations.reserve(inliers);
            keptPrev.reserve(inliers);
            keptCur.reserve(inliers);
            for (std::size_t i = 0; i < total; ++i)
            {
                if (inlierMask[i] == 0)
                {
                    continue;
                }
                keptRaw.push_back(rawPoints[i]);
                keptIds.push_back(ids[i]);
                keptObservations.push_back(observations[i]);
                keptPrev.push_back(prevNormalized[i]);
                keptCur.push_back(curNormalized[i]);
            }

            const std::size_t removed = total - keptRaw.size();
            rawPoints.swap(keptRaw);
            ids.swap(keptIds);
            observations.swap(keptObservations);
            prevNormalized.swap(keptPrev);
            curNormalized.swap(keptCur);
            return removed;
        }

        /**
         * @brief 在上一帧图像上补点，供本次跟踪消费
         *
         * 这些点带 @c NEW 类型检出，但会与既有轨迹一起被跟踪到当前帧：
         * 跟踪成功后在主循环里被重新标为 @c TRACKED，于是新点当帧就经过
         * 一次前后向校验与几何校验。
         *
         * @return 实际新增的点数
         */
        std::size_t detectOnPreviousFrame(const cv::Mat &gray, int budget)
        {
            const std::size_t pointsBefore = prevPoints_.size();
            const std::size_t idsBefore = prevIds_.size();
            const std::size_t normalizedBefore = prevNormalized_.size();

            std::vector<domain::Observation> fresh;
            fresh.reserve(static_cast<std::size_t>(std::max(0, budget)));
            const std::size_t added = detectUniform(gray, prevPoints_, prevIds_, fresh, budget);

            // prevNormalized_ 必须与 prevPoints_ 严格同序：detectUniform 只会
            // 往尾部追加，所以这里按同样的顺序补归一化坐标即可。
            prevNormalized_.reserve(prevPoints_.size());
            for (const domain::Observation &observation : fresh)
            {
                cv::Point2f normalized{0.0F, 0.0F};
                if (!observation.normalized.has_value() || !toNormalizedPlane(*observation.normalized, normalized))
                {
                    normalized =
                        cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
                }
                prevNormalized_.push_back(normalized);
            }

            if (prevNormalized_.size() != prevPoints_.size())
            {
                // 不变量被破坏时回滚到调用前，宁可少补点，也不能让几何
                // 校验拿错点配对（那会把正确跟踪系统性地判成外点）。
                mylog::warn("[特征跟踪] 预补点后归一化坐标数 {} 与点数 {} 不一致，回滚本次补点", prevNormalized_.size(),
                            prevPoints_.size());
                prevPoints_.resize(pointsBefore);
                prevIds_.resize(idsBefore);
                prevNormalized_.resize(normalizedBefore);
                return 0;
            }
            return added;
        }

        /**
         * @brief 剔除互相重合的活动轨迹（同一格只保留最早的一条）
         *
         * 以 @c minTrackSeparationPx 为边长建占据栅格。栅格内按输入顺序保留
         * 第一条（跟踪顺序即上帧顺序，最早者位置最可信），其余全部剔除。
         *
         * @return 被剔除的轨迹数
         */
        std::size_t dedupeActiveTracks(std::vector<cv::Point2f> &raw, std::vector<int> &ids,
                                       std::vector<domain::Observation> &observations,
                                       std::vector<cv::Point2f> &prevNormalized,
                                       std::vector<cv::Point2f> &curNormalized)
        {
            const double separation = vision_.minTrackSeparationPx;
            if (separation <= 0.0 || raw.size() < 2)
            {
                return 0;
            }

            // key = (col, row) 打包成一个 64 位整数，避免 pair 的哈希开销
            std::unordered_map<long long, std::size_t> occupied;
            occupied.reserve(raw.size());
            std::vector<char> keep(raw.size(), 1);
            std::size_t removed = 0;

            for (std::size_t i = 0; i < raw.size(); ++i)
            {
                const long long column = static_cast<long long>(std::floor(raw[i].x / separation));
                const long long row = static_cast<long long>(std::floor(raw[i].y / separation));
                const long long key = (column << 32) ^ (row & 0xffffffffLL);
                if (occupied.emplace(key, i).second)
                {
                    continue;
                }
                keep[i] = 0;
                ++removed;
            }

            if (removed == 0)
            {
                return 0;
            }

            std::size_t write = 0;
            for (std::size_t read = 0; read < raw.size(); ++read)
            {
                if (keep[read] == 0)
                {
                    continue;
                }
                raw[write] = raw[read];
                ids[write] = ids[read];
                observations[write] = observations[read];
                prevNormalized[write] = prevNormalized[read];
                curNormalized[write] = curNormalized[read];
                ++write;
            }
            raw.resize(write);
            ids.resize(write);
            observations.resize(write);
            prevNormalized.resize(write);
            curNormalized.resize(write);
            return removed;
        }

        /**
         * @brief 按网格均匀化补检特征点
         *
         * 单次全局 GFTT 会把特征堆在纹理强的一角：另一侧没有观测，位姿在这
         * 个方向上的约束就是空的。按网格分预算能让特征覆盖整幅图，而不仅
         * 是「总数够」。
         *
         * 已有特征多的格子会少分预算（除以 2），否则一个本来就很密的角落
         * 会继续吸走配额。
         *
         * @return 实际新增的点数
         */
        std::size_t detectUniform(const cv::Mat &gray, std::vector<cv::Point2f> &points, std::vector<int> &ids,
                                  std::vector<domain::Observation> &observations, int budget)
        {
            const int columns = std::max(1, vision_.gridColumns);
            const int rows = std::max(1, vision_.gridRows);
            const int cells = columns * rows;

            // 基础掩码：有效视场 + 排除已有点，避免新点与既有轨迹重叠
            cv::Mat baseMask(gray.size(), CV_8UC1, cv::Scalar(255));
            cv::bitwise_and(baseMask, validMask_, baseMask);
            for (const auto &point : points)
            {
                cv::circle(baseMask, point, 8, cv::Scalar(0), cv::FILLED);
            }

            std::vector<int> existingPerCell(static_cast<std::size_t>(cells), 0);
            for (const auto &point : points)
            {
                const int column =
                    std::clamp(static_cast<int>(point.x) * columns / std::max(1, gray.cols), 0, columns - 1);
                const int row = std::clamp(static_cast<int>(point.y) * rows / std::max(1, gray.rows), 0, rows - 1);
                ++existingPerCell[static_cast<std::size_t>(row * columns + column)];
            }

            const int baseQuota = budget / cells;
            int remainder = budget % cells;
            cv::Mat cellMask = cv::Mat::zeros(gray.size(), CV_8UC1);
            std::size_t added = 0;

            for (int row = 0; row < rows; ++row)
            {
                for (int column = 0; column < columns; ++column)
                {
                    const auto cellIndex = static_cast<std::size_t>(row * columns + column);
                    int quota = baseQuota + (remainder > 0 ? 1 : 0);
                    if (remainder > 0)
                    {
                        --remainder;
                    }
                    quota = std::max(0, quota - existingPerCell[cellIndex] / 2);
                    if (quota <= 0)
                    {
                        continue;
                    }

                    const int x0 = column * gray.cols / columns;
                    const int x1 = (column + 1) * gray.cols / columns;
                    const int y0 = row * gray.rows / rows;
                    const int y1 = (row + 1) * gray.rows / rows;
                    if (x1 <= x0 || y1 <= y0)
                    {
                        continue;
                    }

                    cellMask.setTo(0);
                    cellMask(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(255);
                    cv::bitwise_and(cellMask, baseMask, cellMask);

                    std::vector<cv::Point2f> fresh;
                    try
                    {
                        cv::goodFeaturesToTrack(gray, fresh, quota, 0.01, 10.0, cellMask);
                    }
                    catch (const cv::Exception &error)
                    {
                        mylog::debug("[特征跟踪] 网格 ({},{}) 检测异常: {}", row, column, error.what());
                        continue;
                    }

                    for (const auto &point : fresh)
                    {
                        domain::Observation observation;
                        observation.id = nextFeatureId_++;
                        observation.kind = domain::ObservationKind::NEW;
                        if (!mapPixel(domain::Vec2(point.x, point.y), observation))
                        {
                            continue;
                        }
                        points.push_back(point);
                        ids.push_back(observation.id);
                        observations.push_back(observation);
                        // 同一帧内不重复检测同一点
                        cv::circle(baseMask, point, 8, cv::Scalar(0), cv::FILLED);
                        ++added;
                    }
                }
            }
            return added;
        }

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
        domain::VisionConfig vision_;
        int maxFeatures_;
        int minFeatures_;
        cv::Mat validMask_;

        cv::Mat prevGray_;
        std::vector<cv::Point2f> prevPoints_;
        std::vector<int> prevIds_;
        /// 与 prevPoints_ 严格同序的归一化平面坐标；几何校验靠它配对
        std::vector<cv::Point2f> prevNormalized_;
        int nextFeatureId_{0};
    };

} // namespace myslam::vio
