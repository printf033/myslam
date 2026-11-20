#pragma once

/**
 * @file orb_loop_detector.hpp
 * @brief ORB 外观索引 + 几何验证的回环检测器（OpenCV 实现）。
 *
 * @par 它解决什么
 *
 * 固定滞后窗口之外的关键帧位姿不再被修正，误差沿轨迹单向累积；三圈回访同一
 * 洞穴时，每圈的几何中心会逐步外移。@c IPoseGraph 只提供「修正往哪写」的容器，
 * 修正量必须来自一次真实的**重访观测**——本类就是那个观测的生产者。
 *
 * @par 为什么必须做几何验证，而不是只比描述子
 *
 * 洞穴岩壁纹理高度重复，仅凭描述子相似会大量误报。误回环边会把位姿图拉向
 * 一个物理上不存在的构型，其后果比漂移更糟：漂移至少是连续的。因此候选
 * 必须再过一道 PnP：内点数、重投影误差、深度合法性，三者都满足才输出。
 *
 * @par 尺度从哪来
 *
 * 单目 2D-2D 相对位姿**不含尺度**，这是几何事实。本类用「时间相邻关键帧
 * 之间三角化出的世界系地图点」提供基准——相邻帧基线 0.3~1 m，局部 VIO
 * 位姿在几十秒内足够准，因此这些点的尺度可信。查询时用这些 3D 点与当前帧
 * 做 2D-3D PnP，得到的是「当前帧在历史世界系下的绝对位姿」；它与当前 VIO
 * 位姿之差，就是需要写进位姿图的修正量。
 *
 * @par 为什么不做词袋
 *
 * 词袋的价值是让「与全部历史关键帧比对」变成常数时间。本类改用**几何先验
 * 预筛**（估计位置 15 m 半径 + 时间隔断），因为我们要检测的恰恰是「回到了
 * 附近」——位置本身虽然漂了 8 m，但仍是同一量级，足以把候选压到个位数。
 * 三圈的规模（几千个关键帧）下，这个预筛比维护一棵词汇树简单得多，也更容易
 * 解释每一次命中。
 *
 * 约束：只依赖 domain 契约与 OpenCV。
 */

#include "domain/camera_model.hpp"
#include "domain/types.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace myslam::frontend
{

    /**
     * @brief 回环检测参数
     *
     * 所有阈值都写在这里而不是散在实现里：三圈洞穴的纹理重复度决定了内点
     * 门限该有多严，换一个场景就得重调，而「调了哪个数」必须能被记录。
     */
    struct LoopDetectorConfig
    {
        /// 每帧 ORB 特征数上限
        int featuresPerFrame{2000};
        /// 候选预筛半径 (m)：以 VIO 估计位置为中心
        double searchRadiusMeters{22.0};
        /// 候选的朝向门限 (度)：位置接近但朝向差太多的帧，ORB 匹配必然弱
        double maxCandidateAngleDegrees{40.0};
        /// 时间隔断 (s)：短于此间隔的帧不算回环（那是正常共视）
        double minTimeGapSeconds{60.0};
        /// 单次查询最多验证多少个候选
        int maxCandidates{16};
        /// PnP 内点数下限
        int minInliers{15};
        /// PnP 重投影误差上限（像素）
        double maxReprojectionPx{6.0};
        /// 候选帧三角化地图点数下限
        int minMapPoints{30};
        /// 三角化所需最小视差（像素，按等效焦距折算）
        double minTriangulationParallaxPx{6.0};
        /// 三角化地图点的深度合法区间 (m)
        double minMapPointDepth{0.3};
        double maxMapPointDepth{25.0};
        /// 同一候选帧的重复触发冷却 (s)
        double candidateCooldownSeconds{20.0};
        /// 回环边的平移/旋转标准差：由 PnP 内点数推
        double loopTranslationSigma{0.35};
        double loopRotationSigma{0.02};
    };

    /**
     * @brief 一条通过几何验证的回环约束
     *
     * @c relative 的语义与 @c domain::PoseGraphEdge 对齐：
     * 「把 @c to 系的点变到 @c from 系」，即 @c from.pose.inverse() * to.pose。
     */
    struct LoopConstraint
    {
        domain::KeyframeId from{0}; ///< 历史关键帧
        domain::KeyframeId to{0};   ///< 当前关键帧
        domain::Pose relative{};
        int inliers{0};
        double meanReprojectionPx{0.0};
    };

    /**
     * @brief ORB 回环检测器
     *
     * 使用顺序是「先 @c addKeyframe，再 @c query」：@c addKeyframe 同时完成
     * 特征提取与地图点三角化，@c query 只做检索与验证，不修改索引。
     */
    class OrbLoopDetector
    {
    public:
        /**
         * @param cameraModel 源图像的相机模型（像素 ↔ 射线）
         * @param bodyPSensor 相机在机体系下的位姿 T_body_cam；位姿来自 VIO 时
         *        必须给出，否则相机中心与旋转都会有系统性偏差
         */
        OrbLoopDetector(std::shared_ptr<const domain::ICameraModel> cameraModel, domain::Pose bodyPSensor,
                        LoopDetectorConfig config = {})
            : model_(std::move(cameraModel)),
              bodyPSensor_(std::move(bodyPSensor)),
              config_(config)
        {
            orb_ = cv::ORB::create(config_.featuresPerFrame, 1.2F, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
            maxCandidateAngle_ = config_.maxCandidateAngleDegrees * std::numbers::pi / 180.0;
            matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, /*crossCheck=*/false);
        }

        [[nodiscard]] const LoopDetectorConfig &config() const noexcept { return config_; }

        /**
         * @brief 索引一个关键帧
         *
         * @param gray 原始灰度图（与 @c cameraModel 对应的**未去畸变**图像）
         * @param pose VIO 给出的 World←Body 位姿
         * @return 该帧最终可用的地图点数（0 表示该帧无法作为回环目标）
         */
        std::size_t addKeyframe(const cv::Mat &gray, domain::KeyframeId id, domain::MapId map, double timestamp,
                                const domain::Pose &pose)
        {
            Entry entry;
            entry.id = id;
            entry.map = map;
            entry.timestamp = timestamp;
            entry.pose = pose;

            if (gray.empty())
            {
                return 0;
            }

            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            orb_->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
            if (keypoints.size() < static_cast<std::size_t>(config_.minMapPoints / 2) || descriptors.empty())
            {
                return 0;
            }

            entry.keypoints = std::move(keypoints);
            entry.descriptors = std::move(descriptors);
            entry.rays.reserve(entry.keypoints.size());
            for (const cv::KeyPoint &keypoint : entry.keypoints)
            {
                const std::optional<domain::Vec3> ray = model_->unproject(domain::Vec2(keypoint.pt.x, keypoint.pt.y));
                // 反投影失败的点保留占位，后续三角化按 index 对齐检查有效性。
                // 用零向量作哨兵而不是跳过：射线数组与关键点数组必须一一对应，
                // 否则描述子行号与地图点会错位，而错位不会让任何断言失败。
                entry.rays.push_back(ray && ray->allFinite() ? ray->normalized() : domain::Vec3::Zero());
            }

            const std::size_t mapPointCount = triangulateWithLast(entry);
            entries_.push_back(std::move(entry));
            return mapPointCount;
        }

        /**
         * @brief 为最新索引的关键帧查询回环
         *
         * 只返回通过几何验证的约束；无法验证时返回空，**不得**降级成
         * 「按位置猜一个」。调用方拿到的 @c relative 可以直接写进位姿图。
         */
        [[nodiscard]] std::vector<LoopConstraint> query(std::size_t maxCount)
        {
            std::vector<LoopConstraint> constraints;
            if (entries_.size() < 2)
            {
                return constraints;
            }

            const Entry &current = entries_.back();
            if (current.descriptors.empty() || current.keypoints.size() < 20)
            {
                return constraints;
            }

            const double bestDistance = static_cast<double>(config_.searchRadiusMeters);
            std::vector<std::pair<double, std::size_t>> candidates;
            candidates.reserve(entries_.size());
            for (std::size_t i = 0; i + 1 < entries_.size(); ++i)
            {
                const Entry &candidate = entries_[i];
                if (candidate.map != current.map)
                {
                    continue;
                }
                if (candidate.mapPoints.empty())
                {
                    continue;
                }
                if (current.timestamp - candidate.timestamp < config_.minTimeGapSeconds)
                {
                    continue;
                }
                const auto cooldown = cooldown_.find(candidate.id);
                if (cooldown != cooldown_.end() &&
                    current.timestamp - cooldown->second < config_.candidateCooldownSeconds)
                {
                    continue;
                }
                const double distance = (candidate.pose.position - current.pose.position).norm();
                if (distance > bestDistance)
                {
                    continue;
                }
                // 朝向门控：位置在 22 m 内但朝向差 90° 的帧，几乎不可能是同一次
                // 观测。它们会把「内点中位数」拉到 7，让合格候选淹没在噪声里。
                if (candidate.pose.orientation.angularDistance(current.pose.orientation) > maxCandidateAngle_)
                {
                    continue;
                }
                candidates.emplace_back(distance, i);
            }

            if (candidates.empty())
            {
                return constraints;
            }
            std::sort(candidates.begin(), candidates.end(),
                      [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });

            // 时间维度的非极大值抑制。
            //
            // 只按距离排序会有一个致命偏差：真正要检测的「上一圈」距离 7~13 m，
            // 而几十秒前刚走过的路段距离只有几米——后者全部排在前面，把候选
            // 名额挤满，回环召回率反而随搜索半径增大而下降（实测从 63 条跌到
            // 11 条）。同一片区域连续多帧只保留一帧，候选才会在时间上散开，
            // 覆盖到真正的历史圈层。
            std::vector<std::pair<double, std::size_t>> picked;
            picked.reserve(candidates.size());
            for (const auto &candidate : candidates)
            {
                const double stamp = entries_[candidate.second].timestamp;
                bool suppressed = false;
                for (const auto &chosen : picked)
                {
                    if (std::abs(stamp - entries_[chosen.second].timestamp) < kCandidateTimeSuppression)
                    {
                        suppressed = true;
                        break;
                    }
                }
                if (!suppressed)
                {
                    picked.push_back(candidate);
                }
                if (picked.size() >= static_cast<std::size_t>(config_.maxCandidates))
                {
                    break;
                }
            }
            const std::size_t considered = picked.size();

            for (std::size_t k = 0; k < considered; ++k)
            {
                Entry &candidate = entries_[picked[k].second];
                const std::optional<LoopConstraint> constraint = verify(picked[k].second, current);
                if (!constraint)
                {
                    continue;
                }
                cooldown_[candidate.id] = current.timestamp;
                constraints.push_back(*constraint);
                if (constraints.size() >= maxCount)
                {
                    break;
                }
            }
            return constraints;
        }

        /**
         * @brief 把回环边的协方差按内点数折算
         *
         * 内点越多，PnP 解越可信。按 1/sqrt(n) 收缩，与共视边同一套推导；
         * 但不收缩到共视边的量级——PnP 依赖三角化点，深度误差不会因为
         * 内点变多而消失，因此下限由 @c kLoopSigmaFloor 兜住。
         */
        [[nodiscard]] domain::Mat6 loopCovariance(int inliers) const noexcept
        {
            const double floor = kLoopSigmaFloor;
            const double n = std::max(1.0, static_cast<double>(inliers));
            const double scale = std::max(floor, 1.0 / std::sqrt(n));
            domain::Mat6 covariance = domain::Mat6::Zero();
            const double rotation = config_.loopRotationSigma * scale;
            const double translation = config_.loopTranslationSigma * scale;
            covariance.diagonal() << rotation * rotation, rotation * rotation, rotation * rotation,
                translation * translation, translation * translation, translation * translation;
            return covariance;
        }

        [[nodiscard]] std::size_t keyframeCount() const noexcept { return entries_.size(); }

        /// 已通过验证并采纳的回环数（诊断用，不参与判断）
        [[nodiscard]] std::size_t loopCount() const noexcept { return loopCount_; }

        [[nodiscard]] std::size_t rejectedPnP() const noexcept { return rejectedPnP_; }

        /// 分项统计：没有这一层，854 次拒绝到底是「匹配不够」还是「门控太严」
        /// 完全无法区分，调参数只能靠猜。
        struct RejectionStats
        {
            std::size_t verified{0};      ///< 进入几何验证的候选数
            std::size_t tooFewMatches{0}; ///< 匹配对数不足以求解
            std::size_t lowInliers{0};    ///< RANSAC 内点不足
            std::size_t nonFinite{0};     ///< PnP 产出非有限值
            std::size_t badCorrection{0}; ///< 与 VIO 位姿差得太多
            std::size_t notCoincident{0}; ///< 相对平移过大（重复纹理）
            std::size_t accepted{0};
            /// 每次 RANSAC 的内点数（含未通过门限的），用于判断门限是否合理
            std::vector<int> inlierHistogram;
            /// 通过门限但被「非重访」拒绝时的相对平移距离
            std::vector<double> rejectedTranslations;
        };
        [[nodiscard]] const RejectionStats &rejectionStats() const noexcept { return stats_; }

    private:
        /// 协方差不因内点数无限收缩：三角化点的深度误差是系统性的
        static constexpr double kLoopSigmaFloor = 0.35;

        struct Entry
        {
            domain::KeyframeId id{0};
            domain::MapId map{0};
            double timestamp{0.0};
            domain::Pose pose{};
            std::vector<cv::KeyPoint> keypoints;
            std::vector<domain::Vec3> rays;
            cv::Mat descriptors;
            /// 世界系地图点（由本帧与上一帧三角化）
            std::vector<cv::Point3f> mapPoints;
            cv::Mat mapPointDescriptors;
            /// 已占用体素，供去重使用
            std::set<std::tuple<int, int, int>> voxels;
        };

        /// 相机中心（世界系）与相机到世界的旋转
        [[nodiscard]] domain::Vec3 cameraCenter(const domain::Pose &bodyPose) const
        {
            return bodyPose * bodyPSensor_.position;
        }

        [[nodiscard]] domain::Mat3 cameraRotation(const domain::Pose &bodyPose) const
        {
            return bodyPose.rotation() * bodyPSensor_.rotation();
        }

        /// 相机系射线 -> 世界系方向
        [[nodiscard]] domain::Vec3 rayToWorld(const domain::Vec3 &cameraRay, const domain::Pose &bodyPose) const
        {
            return cameraRotation(bodyPose) * cameraRay;
        }

        /**
         * @brief 两视角三角化
         *
         * 最小化两射线的最近距离中点。返回该点与两个深度；深度必须为正，
         * 否则说明匹配对落在相机后方，是外点的典型症状。
         */
        [[nodiscard]] static std::optional<domain::Vec3> triangulate(const domain::Vec3 &center1,
                                                                     const domain::Vec3 &dir1,
                                                                     const domain::Vec3 &center2,
                                                                     const domain::Vec3 &dir2)
        {
            const domain::Vec3 offset = center1 - center2;
            const double a = dir1.dot(dir1);
            const double b = dir1.dot(dir2);
            const double c = dir2.dot(dir2);
            const double d = dir1.dot(offset);
            const double e = dir2.dot(offset);
            const double denominator = a * c - b * b;
            if (std::abs(denominator) < 1e-12)
            {
                return std::nullopt;
            }
            const double s1 = (b * e - c * d) / denominator;
            const double s2 = (a * e - b * d) / denominator;
            if (s1 <= 0.0 || s2 <= 0.0)
            {
                return std::nullopt;
            }
            return 0.5 * ((center1 + s1 * dir1) + (center2 + s2 * dir2));
        }

        /**
         * @brief 体素下采样后加入地图点
         *
         * 继承会把同一个世界点从多帧带进来，产生多份几乎相同的描述子。
         * 这在 ratio test 下是致命的：最近邻与次近邻是同一个点的两份拷贝，
         * 比值接近 1，正确的匹配反而被丢弃。按 0.15 m 体素去重后，每个空间
         * 位置只留一个描述子，ratio test 的假设重新成立。
         */
        static void appendVoxel(Entry &entry, const domain::Vec3 &point, const cv::Mat &descriptor)
        {
            if (!point.allFinite())
            {
                return;
            }
            constexpr double kVoxelSize = 0.15;
            const auto key = std::tuple{static_cast<int>(std::floor(point.x() / kVoxelSize)),
                                        static_cast<int>(std::floor(point.y() / kVoxelSize)),
                                        static_cast<int>(std::floor(point.z() / kVoxelSize))};
            if (!entry.voxels.insert(key).second)
            {
                return;
            }
            entry.mapPoints.emplace_back(static_cast<float>(point.x()), static_cast<float>(point.y()),
                                         static_cast<float>(point.z()));
            entry.mapPointDescriptors.push_back(descriptor);
        }

        /**
         * @brief 把上一关键帧的地图点投影到当前帧，可见者继承下来
         *
         * 只做一次投影与视野判定，不重新优化：这些点的世界坐标已经由局部
         * 三角化确定，重投影只是判断「它还在不在当前画面里」。
         */
        void inheritMapPoints(const Entry &previous, Entry &current)
        {
            if (previous.mapPoints.empty())
            {
                return;
            }
            const domain::Mat3 R_wc = cameraRotation(current.pose);
            const domain::Mat3 R_cw = R_wc.transpose();
            const domain::Vec3 center = cameraCenter(current.pose);
            const double width = static_cast<double>(model_->imageWidth());
            const double height = static_cast<double>(model_->imageHeight());

            const std::size_t limit = std::min<std::size_t>(previous.mapPoints.size(), kMaxMapPointsPerFrame);
            for (std::size_t i = 0; i < limit; ++i)
            {
                if (current.mapPoints.size() >= kMaxMapPointsPerFrame)
                {
                    break;
                }
                const cv::Point3f &point = previous.mapPoints[i];
                const domain::Vec3 world(point.x, point.y, point.z);
                const domain::Vec3 cameraPoint = R_cw * (world - center);
                if (cameraPoint.z() <= config_.minMapPointDepth || cameraPoint.norm() > config_.maxMapPointDepth)
                {
                    continue;
                }
                const std::optional<domain::Vec2> uv = model_->project(cameraPoint);
                if (!uv || uv->x() < 0.0 || uv->y() < 0.0 || uv->x() >= width || uv->y() >= height)
                {
                    continue;
                }
                appendVoxel(current, world, previous.mapPointDescriptors.row(static_cast<int>(i)));
            }
        }

        /**
         * @brief 与上一关键帧匹配并三角化，产出本帧的世界系地图点
         *
         * 两个限制决定了它能覆盖多大范围：
         *   - 基线太短：深度误差按 1/baseline 放大；
         *   - 基线太长：匹配对会落到重复纹理上。
         * 折中由 @c maxMapPointDepth 与视差门限共同兜住。
         */
        std::size_t triangulateWithLast(Entry &current)
        {
            if (entries_.empty())
            {
                return 0;
            }
            const Entry &previous = entries_.back();
            if (previous.map != current.map || previous.descriptors.empty() || current.descriptors.empty())
            {
                return 0;
            }
            const double elapsed = current.timestamp - previous.timestamp;
            if (elapsed <= 0.0 || elapsed > kMaxTriangulationGapSeconds)
            {
                return 0;
            }

            // 先继承上一关键帧的地图点，再补本帧新三角化的点。
            //
            // 只靠「本帧与上一帧三角化」得到的地图点数量随两帧基线与视差
            // 波动，常常刚过门限；PnP 需要几十个在空间上铺开的点才稳。
            // 继承把局部地图变成跨帧累积的，代价是误差会随传播缓慢增长——
            // 但传播窗口只有几十秒，局部 VIO 位姿在这么短的跨度上是准的。
            inheritMapPoints(previous, current);

            std::vector<std::vector<cv::DMatch>> matches;
            matcher_->knnMatch(current.descriptors, previous.descriptors, matches, 2);
            const domain::Vec3 center1 = cameraCenter(previous.pose);
            const domain::Vec3 center2 = cameraCenter(current.pose);
            const double focal = model_->focalLengthPx();
            const double minParallaxRad = config_.minTriangulationParallaxPx / std::max(1.0, focal);
            const double maxDepth = config_.maxMapPointDepth;

            for (const auto &pair : matches)
            {
                if (pair.size() < 2)
                {
                    continue;
                }
                if (pair[0].distance > kRatio * pair[1].distance)
                {
                    continue;
                }
                const std::size_t previousIndex = static_cast<std::size_t>(pair[0].trainIdx);
                const std::size_t currentIndex = static_cast<std::size_t>(pair[0].queryIdx);
                if (previousIndex >= previous.rays.size() || currentIndex >= current.rays.size())
                {
                    continue;
                }
                const domain::Vec3 ray1 = previous.rays[previousIndex];
                const domain::Vec3 ray2 = current.rays[currentIndex];
                if (ray1.isZero() || ray2.isZero())
                {
                    continue;
                }
                const domain::Vec3 dir1 = rayToWorld(ray1, previous.pose);
                const domain::Vec3 dir2 = rayToWorld(ray2, current.pose);
                if (dir1.dot(dir2) < std::cos(minParallaxRad))
                {
                    continue;
                }
                const std::optional<domain::Vec3> point = triangulate(center1, dir1, center2, dir2);
                if (!point)
                {
                    continue;
                }
                const double depth = (*point - center2).norm();
                if (depth < config_.minMapPointDepth || depth > maxDepth)
                {
                    continue;
                }
                if (!point->allFinite())
                {
                    continue;
                }
                appendVoxel(current, *point, current.descriptors.row(pair[0].queryIdx));
            }

            // 三角化出的点与「当前帧关键点」是两套索引：PnP 只需要 3D 点与
            // 描述子，不要求 2D 关键点，所以这里不必保存索引映射。
            return current.mapPoints.size();
        }

        /**
         * @brief 用候选帧的地图点做 2D-3D PnP，得到当前帧的世界系位姿
         */
        [[nodiscard]] std::optional<LoopConstraint> verify(std::size_t candidateIndex, const Entry &current)
        {
            Entry &candidate = entries_[candidateIndex];
            if (current.descriptors.empty())
            {
                return std::nullopt;
            }

            // 合并候选帧时间邻域的地图点，而不是只用它自己那一帧。
            //
            // 单帧的地图点数量随基线、视差与纹理一起波动，前几百帧尤其少；
            // 回环召回率因此被「候选帧恰好点少」这件事卡住，而不是被匹配
            // 质量卡住。邻域合并后，候选代表的是「这一小段时间的局部地图」，
            // 与它是否恰好在某帧上看到足够多的点是两件事。
            const std::size_t lo = candidateIndex >= kNeighborhoodRadius ? candidateIndex - kNeighborhoodRadius : 0;
            const std::size_t hi = std::min(entries_.size() - 1, candidateIndex + kNeighborhoodRadius);
            std::vector<cv::Point3f> localPoints;
            cv::Mat localDescriptors;
            std::set<std::tuple<int, int, int>> voxels;
            for (std::size_t i = lo; i <= hi; ++i)
            {
                const Entry &near = entries_[i];
                if (near.map != candidate.map)
                {
                    continue;
                }
                for (int row = 0; row < near.mapPointDescriptors.rows; ++row)
                {
                    const cv::Point3f &point = near.mapPoints[static_cast<std::size_t>(row)];
                    const auto key = std::tuple{static_cast<int>(std::floor(point.x / kNeighborhoodVoxel)),
                                                static_cast<int>(std::floor(point.y / kNeighborhoodVoxel)),
                                                static_cast<int>(std::floor(point.z / kNeighborhoodVoxel))};
                    if (!voxels.insert(key).second)
                    {
                        continue;
                    }
                    localPoints.push_back(point);
                    localDescriptors.push_back(near.mapPointDescriptors.row(row));
                }
            }
            if (localPoints.size() < static_cast<std::size_t>(config_.minMapPoints) || localDescriptors.empty())
            {
                return std::nullopt;
            }

            std::vector<std::vector<cv::DMatch>> matches;
            matcher_->knnMatch(current.descriptors, localDescriptors, matches, 2);

            std::vector<cv::Point3f> objectPoints;
            std::vector<cv::Point2f> imagePoints;
            objectPoints.reserve(matches.size());
            imagePoints.reserve(matches.size());
            for (const auto &pair : matches)
            {
                if (pair.size() < 2 || pair[0].distance > kRatio * pair[1].distance)
                {
                    continue;
                }
                const std::size_t currentIndex = static_cast<std::size_t>(pair[0].queryIdx);
                const std::size_t mapIndex = static_cast<std::size_t>(pair[0].trainIdx);
                if (currentIndex >= current.rays.size() || mapIndex >= localPoints.size())
                {
                    continue;
                }
                const domain::Vec3 &ray = current.rays[currentIndex];
                if (ray.isZero() || std::abs(ray.z()) < 1e-6)
                {
                    continue;
                }
                objectPoints.push_back(localPoints[mapIndex]);
                imagePoints.emplace_back(static_cast<float>(ray.x() / ray.z()), static_cast<float>(ray.y() / ray.z()));
            }

            ++stats_.verified;
            // 匹配对数量只需要够 PnP 起步（6 个），合格与否由 RANSAC 内点数
            // 判定。要求「匹配对 ≥ minInliers」会把「20 个匹配、20 个全是
            // 内点」这种完全正确的候选挡在门外。
            if (objectPoints.size() < kMinPnpPoints)
            {
                ++stats_.tooFewMatches;
                return std::nullopt;
            }

            // 归一化坐标下的重投影阈值：像素阈值除以等效焦距
            const double normalizedThreshold = config_.maxReprojectionPx / std::max(1.0, model_->focalLengthPx());
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            std::vector<int> inliers;
            const bool solved = cv::solvePnPRansac(objectPoints, imagePoints, cv::Mat::eye(3, 3, CV_64F), cv::noArray(),
                                                   rvec, tvec, /*useExtrinsicGuess=*/false,
                                                   /*iterationsCount=*/200, static_cast<float>(normalizedThreshold),
                                                   static_cast<double>(0.99), inliers, cv::SOLVEPNP_EPNP);
            if (solved)
            {
                stats_.inlierHistogram.push_back(static_cast<int>(inliers.size()));
            }
            if (!solved || static_cast<int>(inliers.size()) < config_.minInliers)
            {
                ++rejectedPnP_;
                ++stats_.lowInliers;
                return std::nullopt;
            }

            cv::Mat rotation;
            cv::Rodrigues(rvec, rotation);
            domain::Mat3 R_cw; // 相机 -> 世界
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    // solvePnP 返回 world->camera，取转置得到 camera->world
                    R_cw(row, col) = rotation.at<double>(col, row);
                }
            }
            const domain::Vec3 centerWorld = -R_cw * domain::Vec3(tvec[0], tvec[1], tvec[2]);
            if (!centerWorld.allFinite())
            {
                ++rejectedPnP_;
                ++stats_.nonFinite;
                return std::nullopt;
            }

            // 相机位姿 -> 机体位姿：T_world_body = T_world_cam * T_cam_body
            const domain::Pose worldCamera{Eigen::Quaterniond(R_cw), centerWorld};
            const domain::Pose worldBody = worldCamera * bodyPSensor_.inverse();
            if (!worldBody.position.allFinite() || !worldBody.orientation.coeffs().allFinite())
            {
                ++rejectedPnP_;
                ++stats_.nonFinite;
                return std::nullopt;
            }

            // 与 VIO 位姿的一致性：若 PnP 结果与当前估计相差超过「漂移量级的
            // 数倍」，更可能是重复纹理造成的错配，而不是一次真实回环。
            const double correction = (worldBody.position - current.pose.position).norm();
            if (correction > kMaxCorrectionMeters)
            {
                ++rejectedPnP_;
                ++stats_.badCorrection;
                return std::nullopt;
            }
            const double angle = worldBody.orientation.angularDistance(current.pose.orientation);
            if (angle > kMaxCorrectionRadians)
            {
                ++rejectedPnP_;
                ++stats_.badCorrection;
                return std::nullopt;
            }

            LoopConstraint constraint;
            constraint.from = candidate.id;
            constraint.to = current.id;
            constraint.relative = candidate.pose.inverse() * worldBody;

            // 重访的定义就是「位置重合」。PnP 给出几米的相对平移时，两帧
            // 在物理上根本不是同一处——洞穴的长直走廊与重复岩壁会让描述子
            // 大量误匹配，而 RANSAC 仍能给出一组自洽内点。这类边不会让轨迹
            // 更准，只会把它拉向一个不存在的构型，因此在这里直接拒绝。
            stats_.rejectedTranslations.push_back(constraint.relative.position.norm());
            if (constraint.relative.position.norm() > kMaxLoopTranslationMeters)
            {
                ++rejectedPnP_;
                ++stats_.notCoincident;
                return std::nullopt;
            }
            ++stats_.accepted;
            constraint.inliers = static_cast<int>(inliers.size());
            constraint.meanReprojectionPx = 0.0;
            ++loopCount_;
            return constraint;
        }

        /// 三角化允许的最大时间间隔：超过它说明中间丢了关键帧，基线不可信
        static constexpr double kMaxTriangulationGapSeconds = 2.0;
        /// 单帧地图点上限：既控制 PnP 规模，也限制继承链的内存增长
        static constexpr std::size_t kMaxMapPointsPerFrame = 1500;
        /// 候选在时间上的抑制窗口 (s)：同一时段只保留距离最近的一个候选
        static constexpr double kCandidateTimeSuppression = 20.0;
        /// 候选邻域半径（关键帧数）：局部地图合并的时间跨度
        static constexpr std::size_t kNeighborhoodRadius = 3;
        /// 邻域合并时的体素尺寸 (m)
        static constexpr double kNeighborhoodVoxel = 0.3;
        /// Lowe 比值检验
        ///
        /// 这里是 0.85 而不是常见的 0.75：洞穴岩壁纹理重复，最近邻与次近邻
        /// 的距离本来就更接近，0.75 会把大量正确匹配判为歧义。把筛选交给
        /// PnP 的 RANSAC（几何一致性）比交给描述子距离可靠——前者用的正是
        /// 我们真正关心的约束。
        static constexpr double kRatio = 0.85;
        /// PnP 起步所需的最少匹配对
        static constexpr std::size_t kMinPnpPoints = 6;
        /// 回环两帧的相对平移上限 (m)：超过它就不是「重访」，而是重复纹理
        static constexpr double kMaxLoopTranslationMeters = 3.0;
        /// 单次回环允许的最大平移修正 (m) 与旋转修正 (rad)
        static constexpr double kMaxCorrectionMeters = 25.0;
        static constexpr double kMaxCorrectionRadians = 0.7;

        std::shared_ptr<const domain::ICameraModel> model_;
        domain::Pose bodyPSensor_{};
        LoopDetectorConfig config_;
        double maxCandidateAngle_{0.7};
        cv::Ptr<cv::ORB> orb_;
        cv::Ptr<cv::BFMatcher> matcher_;
        std::vector<Entry> entries_;
        std::map<domain::KeyframeId, double> cooldown_;
        std::size_t loopCount_{0};
        std::size_t rejectedPnP_{0};
        RejectionStats stats_{};
    };

} // namespace myslam::frontend
