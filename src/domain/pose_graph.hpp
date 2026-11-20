#pragma once

/**
 * @file pose_graph.hpp
 * @brief 位姿图端口：跨窗口的长期一致性容器。
 *
 * @par 为什么固定滞后窗口之外还需要一层
 *
 * @c IEstimator 的参考实现是固定滞后平滑：超出窗口的关键帧被边缘化，它的
 * 位姿自此不再被修正，误差沿轨迹单向累积。窗口内的精度与窗口外的漂移是
 * 两件不同的事，靠调窗口长度无法同时满足——拉长窗口解决漂移，代价是每帧
 * 求解规模与延迟一起增长。
 *
 * 位姿图把"长期"交给另一层：只存关键帧位姿与它们之间的相对观测（共视
 * 边），不存路标。优化规模与路标数无关，因此可以覆盖整段历史；一旦有位姿
 * 被修正，历史轨迹的一致性就恢复了，而估计器不必知道这件事。
 *
 * @par 为什么它不负责回环
 *
 * 共视边只连接时间上相邻、观测上重叠的关键帧。它抑制漂移，但**不能**
 * 消除漂移——闭环需要外观识别，那是 @c IPlaceRecognizer 的职责。两层分开
 * 的理由：位姿图没有回环也有收益（长期轨迹自洽、可导出），回环没有位姿图
 * 则没有落点（修正结果无处表达）。
 *
 * 约束：只依赖领域类型与标准库。
 */

#include "domain/types.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace myslam::domain
{

    /**
     * @brief 相对位姿的默认协方差
     *
     * 共视边从像素观测推出，而不是从两个绝对位姿的边际协方差相加推出——
     * 后者会把两个位姿的绝对误差当成相对误差，系统性高估（同一个漂移在
     * 相邻两帧里高度相关）。默认值取「共视边通常比绝对位姿准一个量级」的
     * 保守估计：旋转 0.02 rad，平移 0.05 m。
     */
    [[nodiscard]] inline Mat6 defaultRelativeCovariance() noexcept
    {
        Mat6 covariance = Mat6::Zero();
        covariance.diagonal() << 4e-4, 4e-4, 4e-4, 2.5e-3, 2.5e-3, 2.5e-3;
        return covariance;
    }

    /**
     * @brief 位姿图中的一个节点
     *
     * @c pose 是首次加入时的位姿，优化后被修正。@c covariance 为空的节点
     * 作为固定先验（通常只有每张地图的第一个节点），不会被优化移动。
     */
    struct PoseGraphNode
    {
        MapId map{0};
        KeyframeId id{0};
        double timestamp{0.0};
        Pose pose{};
        /// 为空表示该节点固定（地图锚点）
        PoseCovariance covariance{};
        /// 已经历位姿图修正的次数，供调用方判断历史是否被改写过
        std::uint32_t corrections{0};
    };

    /**
     * @brief 位姿图中的一条相对观测（共视边或回环边）
     *
     * @c relative 的语义必须与 @c Pose::operator* 对齐：
     * 它是「把 @c to 系的点变到 @c from 系」的变换，因此
     * @c node(to).pose == node(from).pose * relative。
     * 写成 @c from.pose.inverse() * to.pose 得到它。
     */
    struct PoseGraphEdge
    {
        MapId map{0};
        KeyframeId from{0};
        KeyframeId to{0};
        Pose relative{};
        /// 相对位姿的切线协方差（顺序同 @c Mat6：前旋转后平移）
        Mat6 covariance{defaultRelativeCovariance()};
        /// 支撑这条边的共视观测数；也是信息权重的原始依据
        int sharedObservations{0};
        /// 回环边（来自位置识别）与共视边分开记账，便于诊断
        bool loopClosure{false};
    };

    /**
     * @brief 位姿图端口
     *
     * 实现必须做到：
     *   - @c addNode 幂等：同一 `(map, id)` 重复加入时保留首次位姿，返回 false；
     *   - @c addEdge 幂等：同一 `(map, from, to)` 重复加入时保留首次观测；
     *   - @c optimize 不改地图锚点（无协方差的节点），不做线程间同步——
     *     调用方（编排层）保证串行；
     *   - 任何查询返回值 Copy，不承诺引用生命周期。
     */
    class IPoseGraph
    {
    public:
        virtual ~IPoseGraph() = default;

        /// 加入节点；已存在时返回 false 且不覆盖
        virtual bool addNode(const PoseGraphNode &node) = 0;

        /// 加入边；两端节点不存在、或边已存在时返回 false
        virtual bool addEdge(const PoseGraphEdge &edge) = 0;

        /**
         * @brief 优化整图
         *
         * @return 至少一个历史位姿被修正时返回 true
         *
         * 返回 @c false 不等于失败：没有可优化的自由度（例如只有一条链、
         * 或全部节点都被钉住）时同样没有位移，调用方不该把它当错误。
         */
        [[nodiscard]] virtual bool optimize() = 0;

        /// 全部节点（按加入顺序）
        [[nodiscard]] virtual std::vector<PoseGraphNode> nodes() const = 0;

        /// 指定地图的节点，用于分段导出轨迹
        [[nodiscard]] virtual std::vector<PoseGraphNode> nodesOf(MapId map) const = 0;

        /// 指定地图的边
        [[nodiscard]] virtual std::vector<PoseGraphEdge> edgesOf(MapId map) const = 0;

        [[nodiscard]] virtual std::size_t edgeCount() const = 0;

        /// 出现过的地图数（节点集合里的不同 @c mapId 数）
        [[nodiscard]] virtual std::size_t mapCount() const = 0;

        /**
         * @brief 图内容版本号
         *
         * 节点或边增加、或任意位姿被修正后递增。外部消费者用它判断"手上
         * 那份长期轨迹是否已过期"，而不是每帧重新拉全量。
         */
        [[nodiscard]] virtual std::uint64_t revision() const = 0;
    };

} // namespace myslam::domain
