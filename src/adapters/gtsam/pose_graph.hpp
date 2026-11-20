#pragma once

/**
 * @file pose_graph.hpp
 * @brief 位姿图的 gtsam 实现（LM 批量优化）。
 *
 * @par 为什么是批量 LM 而不是 iSAM2
 *
 * 位姿图的规模是「关键帧数」，而关键帧按秒级插入，一段几分钟的序列也就
 * 几百个节点。这个规模下批量 LM 的耗时是毫秒级，而 iSAM2 的增量结构要
 * 维护贝叶斯树、要处理变量重排序，复杂度花在了当前用不上的地方。
 * 什么时候这个判断会翻转：节点数到万级，或需要每帧优化的实时闭环。
 *
 * @par 噪声模型从哪来
 *
 * 共视边的协方差由编排层按共视观测数给出（见 @c VioPipeline 的登记逻辑），
 * 不从两个绝对位姿的边际协方差相加——相邻帧的漂移高度相关，相加会系统性
 * 高估相对误差，让位姿图几乎不修正任何东西。
 *
 * @par 线程安全
 *
 * 本类不做同步。编排层保证 @c addNode / @c addEdge / @c optimize 串行调用。
 */

#include "adapters/gtsam/types.hpp"
#include "domain/pose_graph.hpp"
#include "logger.hpp"
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace myslam::adapters
{

    /**
     * @brief 基于 gtsam 的位姿图
     *
     * 节点按加入顺序编号，每个 `(map, id)` 映射到一个唯一的 gtsam Symbol
     * 索引。不同地图的节点共享同一个 @c Values，但各自有独立的锚点先验，
     * 因此它们的 gauge 互不影响——这是「多地图」在不引入地图间边的条件下
     * 能共存于一张图里的原因。
     */
    class GtsamPoseGraph final : public domain::IPoseGraph
    {
    public:
        /// @param iterations LM 最大迭代次数
        explicit GtsamPoseGraph(int iterations = 20)
            : iterations_(iterations)
        {
        }

        [[nodiscard]] bool addNode(const domain::PoseGraphNode &node) override
        {
            const auto key = std::pair{node.map, node.id};
            if (index_.contains(key))
            {
                return false;
            }
            const auto index = nodes_.size();
            index_.emplace(key, index);
            nodes_.push_back(node);
            ++revision_;
            return true;
        }

        [[nodiscard]] bool addEdge(const domain::PoseGraphEdge &edge) override
        {
            if (!index_.contains({edge.map, edge.from}) || !index_.contains({edge.map, edge.to}))
            {
                return false;
            }
            const auto key = std::tuple{edge.map, edge.from, edge.to};
            if (!edgeKeys_.insert(key).second)
            {
                return false;
            }
            edges_.push_back(edge);
            ++revision_;
            return true;
        }

        [[nodiscard]] bool optimize() override
        {
            if (nodes_.size() < 2 || edges_.empty())
            {
                return false;
            }

            try
            {
                gtsam::NonlinearFactorGraph graph;
                gtsam::Values initial;

                for (std::size_t i = 0; i < nodes_.size(); ++i)
                {
                    initial.insert(symbolFor(i), toGtsam(nodes_[i].pose));
                }

                // 每张地图必须有一个 gauge 锚点。有 covariance 的节点由边
                // 约束、不应被先验钉死；若整张地图都没给出锚点（例如调用方
                // 忘了给首节点留空 covariance），用最早加入的那个顶住，
                // 否则 LM 在 6 自由度零空间里游走，结果依赖初值的绝对位置。
                for (const auto &[map, anchorIndex] : anchors())
                {
                    (void)map;
                    gtsam::Vector6 sigmas;
                    sigmas << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
                    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
                    graph.add(gtsam::PriorFactor<gtsam::Pose3>(symbolFor(anchorIndex),
                                                               toGtsam(nodes_[anchorIndex].pose), noise));
                }

                for (const auto &edge : edges_)
                {
                    const auto gaussian = gtsam::noiseModel::Gaussian::Covariance(edge.covariance);
                    // 回环边包一层 Huber 核。
                    //
                    // 共视边来自相邻帧，量级一致、不太可能整条错；回环边来自
                    // 外观检索 + PnP，个别误匹配无法从几何门控里完全排除。
                    // 用二次代价时，一条错边会硬把一大段轨迹拉过去——而且
                    // 拉错的量会顺着共视链传播，结果是「局部正确、全局扭曲」，
                    // 比不修更糟。Huber 让残差超过阈值后降为线性，错边只能是
                    // 干扰，不能是绑架。
                    gtsam::SharedNoiseModel noise = gaussian;
                    if (edge.loopClosure)
                    {
                        noise = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::Huber::Create(kLoopHuberDelta), gaussian);
                    }
                    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symbolFor(index_.at({edge.map, edge.from})),
                                                                 symbolFor(index_.at({edge.map, edge.to})),
                                                                 toGtsam(edge.relative), noise));
                }

                gtsam::LevenbergMarquardtParams params;
                params.setMaxIterations(iterations_);
                const gtsam::Values optimized = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();

                // 先整体校验、再统一写回。
                //
                // gtsam 在退化输入（例如非正定的边协方差）下**不抛异常**，而是
                // 返回一批 NaN。若一边算一边写，前面几个节点已经变成 NaN、后面
                // 才被发现，节点就处于一个既非旧值也非新值的混合状态，而
                // `longTermTrajectory()` 会照常把 NaN 交出去。调用方拿到 NaN
                // 轨迹时无法判断这是「优化失败」还是「估计器坏了」。
                std::vector<domain::Pose> updatedPoses;
                updatedPoses.reserve(nodes_.size());
                for (std::size_t i = 0; i < nodes_.size(); ++i)
                {
                    const std::optional<domain::Pose> updated = finitePose(optimized.at<gtsam::Pose3>(symbolFor(i)));
                    if (!updated)
                    {
                        ++failedOptimizations_;
                        mylog::warn("[位姿图] 优化产出了非有限位姿（节点 #{} map {}），整批丢弃并保留上一份结果",
                                    nodes_[i].id, nodes_[i].map);
                        return false;
                    }
                    updatedPoses.push_back(*updated);
                }

                bool corrected = false;
                for (std::size_t i = 0; i < nodes_.size(); ++i)
                {
                    const double delta = (updatedPoses[i].position - nodes_[i].pose.position).norm();
                    const double angle = updatedPoses[i].orientation.angularDistance(nodes_[i].pose.orientation);
                    if (delta > kCorrectionEpsilon || angle > kCorrectionEpsilon)
                    {
                        nodes_[i].pose = updatedPoses[i];
                        ++nodes_[i].corrections;
                        corrected = true;
                    }
                }
                if (corrected)
                {
                    ++revision_;
                }
                return corrected;
            }
            catch (const std::exception &error)
            {
                // 契约：适配器不抛。协方差非正定、gtsam 内部断言都可能抛，
                // 两者都不该让位姿图把整条流水线拖垮——上一份节点位姿仍然
                // 是自洽的，它们本来就是这个图最好的可用结果。
                //
                // 但**必须**留下痕迹：静默吞掉会让位姿图持续失败而外部只看到
                // 「每次优化都没有修正」。这与「本来就没有漂移」在日志上完全
                // 无法区分，而两者的处置方式正好相反。
                ++failedOptimizations_;
                mylog::warn("[位姿图] 优化失败（累计 {} 次），保留上一份节点位姿: {}", failedOptimizations_,
                            error.what());
                return false;
            }
        }

        [[nodiscard]] std::vector<domain::PoseGraphNode> nodes() const override { return nodes_; }

        [[nodiscard]] std::vector<domain::PoseGraphNode> nodesOf(domain::MapId map) const override
        {
            std::vector<domain::PoseGraphNode> out;
            for (const auto &node : nodes_)
            {
                if (node.map == map)
                {
                    out.push_back(node);
                }
            }
            return out;
        }

        [[nodiscard]] std::vector<domain::PoseGraphEdge> edgesOf(domain::MapId map) const override
        {
            std::vector<domain::PoseGraphEdge> out;
            for (const auto &edge : edges_)
            {
                if (edge.map == map)
                {
                    out.push_back(edge);
                }
            }
            return out;
        }

        [[nodiscard]] std::size_t edgeCount() const override { return edges_.size(); }

        [[nodiscard]] std::size_t mapCount() const override { return anchors().size(); }

        [[nodiscard]] std::uint64_t revision() const override { return revision_; }

        /// 优化抛异常或产出非有限值的累计次数（静默失败会让问题无法归因）
        [[nodiscard]] std::size_t failedOptimizations() const noexcept { return failedOptimizations_; }

        /**
         * @brief 把 gtsam 位姿转成领域位姿，非有限值返回 @c std::nullopt
         *
         * 独立成公开静态函数而不是内联在循环里，是为了**能被测到**：gtsam 在
         * 退化输入下是否真的产出 NaN 依赖版本与内部实现，无法靠构造输入稳定
         * 触发。把判定本身变成一个纯函数，测试就能直接喂 NaN 进来验证它确实
         * 拒绝。优化循环只负责「遇到 nullopt 就整批放弃」，那部分是三行直白
         * 代码，不需要再单独构造。
         */
        [[nodiscard]] static std::optional<domain::Pose> finitePose(const gtsam::Pose3 &pose)
        {
            domain::Pose converted = fromGtsam(pose);
            if (!converted.position.allFinite() || !converted.orientation.coeffs().allFinite())
            {
                return std::nullopt;
            }
            return converted;
        }

    private:
        /// 修正判定阈值：位置 0.1 mm 或姿态 0.1 mrad 以上才算"改写过历史"
        static constexpr double kCorrectionEpsilon = 1e-4;

        /// 回环边的 Huber 阈值。取 1.345 而不是更小：真实回环的残差应当远小于
        /// 它（正确边残差是厘米级），而错边通常是米级——阈值落在两者之间，
        /// 既能放过正确边，又能按住错边。
        static constexpr double kLoopHuberDelta = 1.345;

        [[nodiscard]] static gtsam::Key symbolFor(std::size_t index) noexcept { return gtsam::Symbol('x', index); }

        /// 每张地图的锚点索引：优先取 covariance 为空的节点，没有则取最早者
        [[nodiscard]] std::map<domain::MapId, std::size_t> anchors() const
        {
            std::map<domain::MapId, std::size_t> anchors;
            for (std::size_t i = 0; i < nodes_.size(); ++i)
            {
                const auto it = anchors.find(nodes_[i].map);
                if (it == anchors.end())
                {
                    anchors.emplace(nodes_[i].map, i);
                    continue;
                }
                if (!nodes_[i].covariance.has_value() && nodes_[it->second].covariance.has_value())
                {
                    it->second = i;
                }
            }
            return anchors;
        }

        int iterations_{20};
        std::vector<domain::PoseGraphNode> nodes_;
        std::vector<domain::PoseGraphEdge> edges_;
        std::map<std::pair<domain::MapId, domain::KeyframeId>, std::size_t> index_;
        std::set<std::tuple<domain::MapId, domain::KeyframeId, domain::KeyframeId>> edgeKeys_;
        std::uint64_t revision_{0};
        std::size_t failedOptimizations_{0};
    };

} // namespace myslam::adapters
