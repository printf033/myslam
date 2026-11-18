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
#include <format>
#include <fstream>
#include <numbers>
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
        /**
         * @brief 姿态（World <- Body）
         *
         * 缺失时不参与旋转误差统计。位置对了但朝向错了是完全可能的——
         * 只量平移的评估对此完全盲。
         */
        std::optional<Eigen::Quaterniond> orientation{};
    };

    /**
     * @brief ATE 与旋转误差统计
     *
     * @par 为什么没有 NEES
     *
     * NEES（@c eᵀP⁻¹e）需要每个采样点携带位姿协方差。本评估器的输入是
     * TUM 轨迹文件（@c t x y z qx qy qz qw 八列），而 @c ReplayRunner 写出
     * 的轨迹也只有这八列——协方差从未落盘，也无法从轨迹还原。
     *
     * 于是「真实数据上的 NEES」在这条链路上不可得。曾在这里保留了一整套
     * 计算与表格列，但那一段永远读到 @c std::nullopt，输出的恒为 @c "-"：
     * 一个看起来在度量、实际上从不度量的指标比没有它更危险。因此删掉，
     * 而不是留一个永远为空的分支。
     *
     * 一致性结论请走仿真路径：@c VioBenchmark::run() 返回的
     * @c RunReport::neesMean / @c neesSamples 直接来自估计器的
     * @c StepResult::poseCovariance，协方差在那条链路上是真实存在的
     * （实测均值 1.23，理论期望 3）。
     *
     * 若将来轨迹格式附带协方差列（TUM 的 @c cov 扩展），应同时增加解析
     * 与本节计算，而不是只加一半。
     */
    struct AteMetrics
    {
        std::size_t matched{0}; ///< 成功关联的点对数量
        double rmse{0.0};
        double mean{0.0};
        double median{0.0};
        double max{0.0};

        /// 参与旋转误差统计的点数（两边都带姿态时）
        std::size_t rotationSamples{0};
        /// 旋转误差的 RMSE（度）
        double rotationRmse{0.0};
        double rotationMax{0.0};
    };

    /**
     * @brief 读取 TUM 格式轨迹（timestamp tx ty tz [qx qy qz qw]）
     *
     * 姿态列可选：缺失时 @c TrackSample::orientation 为空，旋转误差统计
     * 会明确缺席而不是当成零误差。
     *
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
            constexpr int kMaxColumns = 8;
            double values[kMaxColumns] = {};
            std::size_t begin = 0;
            int count = 0;
            while (count < kMaxColumns)
            {
                // 分隔符必须包含 '\r'：std::getline 只吃掉 '\n'，CRLF 文件的每行
                // 末字段会带着 '\r' 进入 from_chars，parsed != last 导致该字段
                // 解析失败。四列文件因此整行被丢弃，八列文件则静默丢掉姿态列。
                const std::size_t end = line.find_first_of(" \t\r", begin);
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

            TrackSample sample;
            sample.timestamp = values[0];
            sample.position = domain::Vec3(values[1], values[2], values[3]);
            if (count >= 8)
            {
                Eigen::Quaterniond q(values[7], values[4], values[5], values[6]); // w, x, y, z
                if (q.norm() > 1e-12)
                {
                    sample.orientation = q.normalized();
                }
            }
            // 下游 @c evaluateAte/@c evaluateRpe 用 lower_bound 做时间关联，
            // 隐含要求升序；乱序输入会静默错配，而不是报错。
            if (!out.empty() && sample.timestamp < out.back().timestamp)
            {
                return std::unexpected(std::format("轨迹文件时间戳非升序（t={} 出现在 t={} 之后）: {}",
                                                   sample.timestamp, out.back().timestamp, path.string()));
            }
            out.push_back(std::move(sample));
        }
        if (out.empty())
        {
            return std::unexpected(std::format("轨迹文件无有效行: {}", path.string()));
        }
        return out;
    }

    /**
     * @brief 刚体对齐结果：ref ≈ R·est + t
     */
    struct RigidAlignment
    {
        domain::Mat3 rotation = domain::Mat3::Identity();
        domain::Vec3 translation = domain::Vec3::Zero();
    };

    /**
     * @brief Umeyama 刚体对齐（不估尺度）
     *
     * 求 @c R,t 使 @c Σ||ref - (R·est + t)||² 最小。关掉尺度自由度是刻意的：
     * VIO 输出的是度量轨迹，允许缩放会把尺度误差藏进对齐里，ATE 因此偏小。
     */
    [[nodiscard]] inline std::optional<RigidAlignment> estimateAlignment(const std::vector<domain::Vec3> &estimated,
                                                                         const std::vector<domain::Vec3> &reference)
    {
        const std::size_t n = std::min(estimated.size(), reference.size());
        if (n < 3)
        {
            return std::nullopt;
        }

        // 每个位姿一行（@c Eigen::umeyama 的输入约定）。原先手写的版本
        // （去均值 → Σ → SVD → det 修正）在 @c evaluateAte 里又抄了一遍，
        // 两处必须同步修改；改用 Eigen 的实现后只剩这一个入口。
        Eigen::MatrixXd from(static_cast<Eigen::Index>(n), 3);
        Eigen::MatrixXd to(static_cast<Eigen::Index>(n), 3);
        for (std::size_t i = 0; i < n; ++i)
        {
            const auto row = static_cast<Eigen::Index>(i);
            from.row(row) = estimated[i].transpose();
            to.row(row) = reference[i].transpose();
        }

        const Eigen::Matrix4d transform = Eigen::umeyama(from, to, /*with_scaling=*/false);
        if (!transform.allFinite())
        {
            return std::nullopt;
        }
        RigidAlignment alignment;
        alignment.rotation = transform.topLeftCorner<3, 3>();
        alignment.translation = transform.topRightCorner<3, 1>();
        return alignment;
    }

    /**
     * @brief 相对位姿误差（RPE）的平移分量统计
     *
     * RPE 看的是「走过一段固定距离后，估计轨迹的相对位移错了多少」，与 ATE
     * 的差别是：ATE 对全局偏移敏感，RPE 对局部漂移率敏感。滑动窗口系统用
     * 固定滞后边缘化，误差是逐段积累的，看 RPE 比看 ATE 更能说明问题。
     *
     * 本实现同时统计平移与旋转两个分量：只量平移时，「位置对了但朝向
     * 系统性偏转」这类错误完全隐形。
     */
    struct RpeMetrics
    {
        std::size_t pairs{0}; ///< 参与统计的轨迹段数
        double segmentLength{0.0};
        double rmse{0.0};
        double mean{0.0};
        double max{0.0};

        /// 参与旋转统计的段数（两端都带姿态时）
        std::size_t rotationPairs{0};
        /// 相对旋转误差的 RMSE（度）
        double rotationRmse{0.0};
        double rotationMax{0.0};
    };

    /**
     * @brief 按固定路径长度评估相对位移误差
     *
     * @param segmentLength 每段的路径长度 (m)；太短则噪声主导，太长则段数不够
     * @param toleranceRatio 端点匹配容差（相对 @p segmentLength）；找不到合理端点就跳过该段
     */
    inline RpeMetrics evaluateRpe(const std::vector<TrackSample> &estimated,
                                  const std::vector<dataset::GroundTruthPose> &reference, double maxDiffSeconds,
                                  double segmentLength, double toleranceRatio = 0.25)
    {
        RpeMetrics metrics;
        metrics.segmentLength = segmentLength;
        if (segmentLength <= 0.0 || estimated.empty() || reference.empty())
        {
            return metrics;
        }

        // 时间关联（与 ATE 同一口径：二分找最近点，|Δt| ≤ maxDiffSeconds）
        std::vector<domain::Vec3> estPoints;
        std::vector<domain::Vec3> refPoints;
        std::vector<std::optional<Eigen::Quaterniond>> estOrientations;
        std::vector<std::optional<Eigen::Quaterniond>> refOrientations;
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
                }
            }
            if (best != nullptr)
            {
                estPoints.push_back(sample.position);
                refPoints.push_back(best->position);
                estOrientations.push_back(sample.orientation);
                refOrientations.push_back(best->orientation);
            }
        }
        if (estPoints.size() < 3)
        {
            return metrics;
        }

        // 仍先对齐一次，让残差与 ATE 同源。注意对齐**会**改变 RPE 数值：
        // |R·Δest − Δref| ≠ |Δest − Δref|（除非 R = I）。保留对齐是为了让
        // 两条指标用同一个估计坐标系，而不是因为相对量对旋转不变。
        const auto alignment = estimateAlignment(estPoints, refPoints);
        if (!alignment.has_value())
        {
            return metrics;
        }
        for (std::size_t i = 0; i < estPoints.size(); ++i)
        {
            estPoints[i] = alignment->rotation * estPoints[i] + alignment->translation;
            if (estOrientations[i].has_value())
            {
                // 姿态随对齐旋转一起转：R' = R_align·R
                *estOrientations[i] = Eigen::Quaterniond(alignment->rotation) * (*estOrientations[i]);
            }
        }

        // 累积路径长度用真值算：真值不会因为估计漂移而改变「走过多远」的定义
        const std::size_t n = refPoints.size();
        std::vector<double> cumulative(n, 0.0);
        for (std::size_t i = 1; i < n; ++i)
        {
            cumulative[i] = cumulative[i - 1] + (refPoints[i] - refPoints[i - 1]).norm();
        }

        const double tolerance = toleranceRatio * segmentLength;
        double sumSquared = 0.0;
        double sum = 0.0;
        double worst = 0.0;
        double rotationSquared = 0.0;
        double rotationWorst = 0.0;
        for (std::size_t i = 0; i + 1 < n; ++i)
        {
            const double target = cumulative[i] + segmentLength;
            const auto it =
                std::lower_bound(cumulative.begin() + static_cast<std::ptrdiff_t>(i + 1), cumulative.end(), target);
            if (it == cumulative.end())
            {
                break;
            }
            auto j = static_cast<std::size_t>(std::distance(cumulative.begin(), it));
            // 端点落在两帧之间时取更接近目标的那一帧
            if (j > i + 1 && std::abs(cumulative[j - 1] - target) < std::abs(cumulative[j] - target))
            {
                --j;
            }
            if (std::abs(cumulative[j] - target) > tolerance)
            {
                continue;
            }

            const domain::Vec3 estDelta = estPoints[j] - estPoints[i];
            const domain::Vec3 refDelta = refPoints[j] - refPoints[i];
            const double error = (estDelta - refDelta).norm();
            sumSquared += error * error;
            sum += error;
            worst = std::max(worst, error);
            ++metrics.pairs;

            // 相对旋转：两端都带姿态才能算
            if (estOrientations[i].has_value() && estOrientations[j].has_value() && refOrientations[i].has_value() &&
                refOrientations[j].has_value())
            {
                const Eigen::Matrix3d relEst =
                    estOrientations[j]->toRotationMatrix() * estOrientations[i]->toRotationMatrix().transpose();
                const Eigen::Matrix3d relRef =
                    refOrientations[j]->toRotationMatrix() * refOrientations[i]->toRotationMatrix().transpose();
                const Eigen::Matrix3d errorR = relEst * relRef.transpose();
                const double angle = Eigen::AngleAxisd(errorR).angle();
                if (std::isfinite(angle))
                {
                    rotationSquared += angle * angle;
                    rotationWorst = std::max(rotationWorst, angle);
                    ++metrics.rotationPairs;
                }
            }
        }

        if (metrics.rotationPairs > 0)
        {
            metrics.rotationRmse =
                std::sqrt(rotationSquared / static_cast<double>(metrics.rotationPairs)) * 180.0 / std::numbers::pi;
            metrics.rotationMax = rotationWorst * 180.0 / std::numbers::pi;
        }

        if (metrics.pairs > 0)
        {
            metrics.rmse = std::sqrt(sumSquared / static_cast<double>(metrics.pairs));
            metrics.mean = sum / static_cast<double>(metrics.pairs);
            metrics.max = worst;
        }
        return metrics;
    }

    /**
     * @brief 计算绝对轨迹误差（SE(3) 对齐，不估尺度）
     *
     * 关联规则：对每个估计点找时间上最近的参考点，|Δt| ≤ @p maxDiffSeconds
     * 才算配对。对齐用 Umeyama（R,t 最小二乘），因此估计轨迹的初始坐标系
     * 与真值不一致不会污染误差。
     *
     * 只给出平移与旋转误差，不含 NEES：轨迹文件不携带协方差，理由见
     * @c AteMetrics 的说明。
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
        std::vector<std::optional<Eigen::Quaterniond>> estOrientations;
        std::vector<std::optional<Eigen::Quaterniond>> refOrientations;
        estPoints.reserve(estimated.size());
        refPoints.reserve(estimated.size());
        estOrientations.reserve(estimated.size());
        refOrientations.reserve(estimated.size());
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
                estOrientations.push_back(sample.orientation);
                refOrientations.push_back(best->orientation);
            }
        }
        metrics.matched = estPoints.size();
        if (metrics.matched < 3)
        {
            return metrics;
        }

        // Umeyama：求 R,t 使 Σ||ref - (R·est + t)||² 最小。与 @c estimateAlignment
        // 共用同一实现，不在这里再写一份。
        const std::size_t n = estPoints.size();
        const auto alignment = estimateAlignment(estPoints, refPoints);
        if (!alignment.has_value())
        {
            return metrics;
        }
        const domain::Mat3 rotation = alignment->rotation;
        const domain::Vec3 translation = alignment->translation;

        std::vector<double> errors;
        errors.reserve(n);
        double sumSquared = 0.0;
        double sum = 0.0;
        double worst = 0.0;
        double rotationSquared = 0.0;
        double rotationWorst = 0.0;
        std::size_t rotationCount = 0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const domain::Vec3 aligned = rotation * estPoints[i] + translation;
            const domain::Vec3 residual = refPoints[i] - aligned;
            const double error = residual.norm();
            errors.push_back(error);
            sumSquared += error * error;
            sum += error;
            worst = std::max(worst, error);
        }

        // 旋转误差：对齐旋转必须同样作用到姿态上（R' = R_align·R），否则
        // 比的是两个坐标系之间的姿态差，与估计器无关。
        for (std::size_t i = 0; i < n; ++i)
        {
            if (!estOrientations[i].has_value() || !refOrientations[i].has_value())
            {
                continue;
            }
            const Eigen::Matrix3d alignedR = rotation * estOrientations[i]->toRotationMatrix();
            const Eigen::Matrix3d errorR = alignedR * refOrientations[i]->toRotationMatrix().transpose();
            const double angle = Eigen::AngleAxisd(errorR).angle();
            if (!std::isfinite(angle))
            {
                continue;
            }
            rotationSquared += angle * angle;
            rotationWorst = std::max(rotationWorst, angle);
            ++rotationCount;
        }
        if (rotationCount > 0)
        {
            metrics.rotationSamples = rotationCount;
            metrics.rotationRmse =
                std::sqrt(rotationSquared / static_cast<double>(rotationCount)) * 180.0 / std::numbers::pi;
            metrics.rotationMax = rotationWorst * 180.0 / std::numbers::pi;
        }
        const std::size_t mid = errors.size() / 2;
        std::nth_element(errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>(mid), errors.end());

        metrics.rmse = std::sqrt(sumSquared / static_cast<double>(n));
        metrics.mean = sum / static_cast<double>(n);
        metrics.median = errors[mid];
        metrics.max = worst;
        return metrics;
    }

    /**
     * @brief 一行评估结果（Markdown 表行）
     *
     * 跨序列、跨参数对比目前只能靠人眼抄终端输出。结构化一行让多次运行
     * 能直接拼成表格：「改了哪个开关、精度与姿态误差各变了多少」。
     *
     * 缺失的分量写 @c "-" 而不是 0：无法度量的不等于零误差。
     */
    [[nodiscard]] inline std::string toMarkdownRow(std::string_view label, const AteMetrics &ate,
                                                   const RpeMetrics &rpe = {})
    {
        const auto num = [](double value, int precision) { return std::format("{:.{}f}", value, precision); };

        std::string row = std::format("| {} | {} | {} | ", label, num(ate.rmse, 4), num(ate.max, 4));
        row += ate.rotationSamples > 0 ? num(ate.rotationRmse, 3) : std::string("-");
        row += " | ";
        row += rpe.pairs > 0 ? num(rpe.rmse, 4) : std::string("-");
        row += " | ";
        row += rpe.rotationPairs > 0 ? num(rpe.rotationRmse, 3) : std::string("-");
        row += " |\n";
        return row;
    }

    /// Markdown 表头（与 @c toMarkdownRow 的列一一对应）
    [[nodiscard]] inline std::string_view markdownHeader() noexcept
    {
        return "| 序列 | ATE RMSE (m) | ATE max (m) | 旋转 RMSE (°) | RPE RMSE (m) | RPE 旋转 (°) |\n"
               "| :--- | ---: | ---: | ---: | ---: | ---: |\n";
    }

    /**
     * @brief 一行评估结果（LaTeX 表行）
     *
     * 下划线转义：序列名里通常带 @c _ （如 @c rgbd_dataset_freiburg1_desk），
     * 不转义会让 LaTeX 编译失败。
     */
    [[nodiscard]] inline std::string toLatexRow(std::string_view label, const AteMetrics &ate,
                                                const RpeMetrics &rpe = {})
    {
        std::string escaped;
        escaped.reserve(label.size());
        for (const char character : label)
        {
            if (character == '_' || character == '&' || character == '%' || character == '#')
            {
                escaped.push_back('\\');
            }
            escaped.push_back(character);
        }

        const auto num = [](double value, int precision) { return std::format("{:.{}f}", value, precision); };

        std::string row = std::format("{} & {} & {} & ", escaped, num(ate.rmse, 4), num(ate.max, 4));
        row += ate.rotationSamples > 0 ? num(ate.rotationRmse, 3) : std::string("-");
        row += " & ";
        row += rpe.pairs > 0 ? num(rpe.rmse, 4) : std::string("-");
        row += " & ";
        row += rpe.rotationPairs > 0 ? num(rpe.rotationRmse, 3) : std::string("-");
        row += " \\\\ \n";
        return row;
    }

} // namespace myslam::app
