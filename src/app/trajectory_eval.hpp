#pragma once

/**
 * @file trajectory_eval.hpp
 * @brief 轨迹读取与绝对轨迹误差（ATE）评估。
 *
 * 回放器只把轨迹写进 TUM 文件；本文件负责把它读回来、与真值按时间关联、
 * 做 SE(3) 对齐（Umeyama，不估尺度）并给出误差统计。TUM RGB-D 自带
 * 运动捕捉真值，因此这条路径能给出量化结论；rosbag 系列没有真值，用不上。
 */

#include "domain/types.hpp"
#include "sensor/tum_dataset.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

namespace myslam::app
{

    /// 轨迹中的一个采样点
    struct TrackSample
    {
        double timestamp{0.0};
        domain::Vec3 position = domain::Vec3::Zero();
    };

    /// ATE 统计（单位 m）
    struct AteMetrics
    {
        std::size_t matched{0}; ///< 成功关联的点对数量
        double rmse{0.0};
        double mean{0.0};
        double median{0.0};
        double max{0.0};
    };

    /**
     * @brief 读取 TUM 格式轨迹（timestamp tx ty tz [qx qy qz qw]）
     * @return 成功返回采样点；文件缺失或格式错误返回说明
     */
    inline std::expected<std::vector<TrackSample>, std::string> loadTumTrajectory(const std::filesystem::path &path)
    {
        std::ifstream in(path);
        if (!in)
        {
            return std::unexpected(std::format("无法打开轨迹文件: {}", path.string()));
        }
        std::vector<TrackSample> out;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }
            double values[4] = {};
            std::size_t begin = 0;
            int count = 0;
            while (count < 4)
            {
                const std::size_t end = line.find_first_of(" \t", begin);
                const std::size_t length = (end == std::string::npos ? line.size() : end) - begin;
                const char *first = line.data() + begin;
                const char *last = first + length;
                double value = 0.0;
                const auto [parsed, ec] = std::from_chars(first, last, value);
                if (ec != std::errc{} || parsed != last || !std::isfinite(value))
                {
                    break;
                }
                values[count++] = value;
                if (end == std::string::npos)
                {
                    break;
                }
                begin = end + 1;
            }
            if (count < 4)
            {
                continue;
            }
            out.push_back(TrackSample{values[0], domain::Vec3(values[1], values[2], values[3])});
        }
        if (out.empty())
        {
            return std::unexpected(std::format("轨迹文件无有效行: {}", path.string()));
        }
        return out;
    }

    /**
     * @brief 计算绝对轨迹误差（SE(3) 对齐，不估尺度）
     *
     * 关联规则：对每个估计点找时间上最近的参考点，|Δt| ≤ @p maxDiffSeconds
     * 才算配对。对齐用 Umeyama（R,t 最小二乘），因此估计轨迹的初始坐标系
     * 与真值不一致不会污染误差。
     *
     * @param estimated 估计轨迹（按时间升序）
     * @param reference 真值轨迹（按时间升序）
     * @param maxDiffSeconds 时间关联阈值
     */
    inline AteMetrics evaluateAte(const std::vector<TrackSample> &estimated,
                                  const std::vector<dataset::GroundTruthPose> &reference, double maxDiffSeconds)
    {
        AteMetrics metrics;
        if (estimated.empty() || reference.empty())
        {
            return metrics;
        }

        // 时间关联：reference 已按时间升序，二分找最近点
        std::vector<domain::Vec3> estPoints;
        std::vector<domain::Vec3> refPoints;
        estPoints.reserve(estimated.size());
        refPoints.reserve(estimated.size());
        for (const TrackSample &sample : estimated)
        {
            const auto it =
                std::lower_bound(reference.begin(), reference.end(), sample.timestamp,
                                 [](const dataset::GroundTruthPose &pose, double t) { return pose.timestamp < t; });
            const dataset::GroundTruthPose *best = nullptr;
            double bestDiff = maxDiffSeconds;
            if (it != reference.end() && std::abs(it->timestamp - sample.timestamp) <= bestDiff)
            {
                best = &*it;
                bestDiff = std::abs(it->timestamp - sample.timestamp);
            }
            if (it != reference.begin())
            {
                const auto prev = std::prev(it);
                const double diff = std::abs(prev->timestamp - sample.timestamp);
                if (diff <= bestDiff)
                {
                    best = &*prev;
                    bestDiff = diff;
                }
            }
            if (best != nullptr)
            {
                estPoints.push_back(sample.position);
                refPoints.push_back(best->position);
            }
        }
        metrics.matched = estPoints.size();
        if (metrics.matched < 3)
        {
            return metrics;
        }

        // Umeyama：求 R,t 使 Σ||ref - (R·est + t)||² 最小
        const std::size_t n = estPoints.size();
        domain::Vec3 muEst = domain::Vec3::Zero();
        domain::Vec3 muRef = domain::Vec3::Zero();
        for (std::size_t i = 0; i < n; ++i)
        {
            muEst += estPoints[i];
            muRef += refPoints[i];
        }
        muEst /= static_cast<double>(n);
        muRef /= static_cast<double>(n);

        Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < n; ++i)
        {
            sigma += (refPoints[i] - muRef) * (estPoints[i] - muEst).transpose();
        }
        sigma /= static_cast<double>(n);

        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
        if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0)
        {
            d(2, 2) = -1.0;
        }
        const Eigen::Matrix3d rotation = svd.matrixU() * d * svd.matrixV().transpose();
        const domain::Vec3 translation = muRef - rotation * muEst;

        std::vector<double> errors;
        errors.reserve(n);
        double sumSquared = 0.0;
        double sum = 0.0;
        double worst = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const domain::Vec3 aligned = rotation * estPoints[i] + translation;
            const double error = (refPoints[i] - aligned).norm();
            errors.push_back(error);
            sumSquared += error * error;
            sum += error;
            worst = std::max(worst, error);
        }
        const std::size_t mid = errors.size() / 2;
        std::nth_element(errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>(mid), errors.end());

        metrics.rmse = std::sqrt(sumSquared / static_cast<double>(n));
        metrics.mean = sum / static_cast<double>(n);
        metrics.median = errors[mid];
        metrics.max = worst;
        return metrics;
    }

} // namespace myslam::app
