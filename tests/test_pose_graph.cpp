/**
 * @file test_pose_graph.cpp
 * @brief 位姿图契约与 gtsam 实现的回归测试。
 *
 * 与 @c test_vio 分开的理由有两个：
 *   - 它必须 include gtsam 适配器，而 @c test_vio 刻意只依赖领域层；
 *   - 位姿图是「长期一致性」的唯一凭证，它的失败模式（静默不修正、
 *     跨地图互相污染、锚点被优化拖走）都不会让主流水线失败，只能在这里
 *     单独盯住。
 */

#include "adapters/gtsam/pose_graph.hpp"
#include "domain/pose_graph.hpp"
#include "domain/types.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

namespace
{

    int g_checks = 0;
    int g_failures = 0;

    void expectTrue(bool condition, const char *what)
    {
        ++g_checks;
        if (!condition)
        {
            ++g_failures;
            std::fprintf(stderr, "FAIL: %s\n", what);
        }
    }

    myslam::domain::Pose makePose(double x, double y, double z)
    {
        myslam::domain::Pose pose;
        pose.position = myslam::domain::Vec3(x, y, z);
        return pose;
    }

    /**
     * @brief 构造一条沿 x 轴前进的链，节点位姿带有指定的漂移
     *
     * @param driftPerStep 每个节点相对真值额外偏移的 y 分量（模拟累积漂移）
     */
    void buildChain(myslam::domain::IPoseGraph &graph, myslam::domain::MapId map, int count, double driftPerStep)
    {
        for (int i = 0; i < count; ++i)
        {
            myslam::domain::PoseGraphNode node;
            node.map = map;
            node.id = i;
            node.timestamp = static_cast<double>(i);
            node.pose = makePose(static_cast<double>(i), driftPerStep * static_cast<double>(i), 0.0);
            // 首个节点是锚点：不带协方差，位姿图不得移动它
            node.covariance = i == 0 ? myslam::domain::PoseCovariance{} : myslam::domain::Mat6::Identity();
            graph.addNode(node);

            if (i > 0)
            {
                myslam::domain::PoseGraphEdge edge;
                edge.map = map;
                edge.from = i - 1;
                edge.to = i;
                // 相对观测始终是「前进 1 m、无横向偏移」，因此优化后节点的
                // y 分量必须被压回 0——这正是漂移被修正的判据。
                edge.relative = makePose(1.0, 0.0, 0.0);
                edge.covariance = myslam::domain::defaultRelativeCovariance();
                edge.sharedObservations = 40;
                graph.addEdge(edge);
            }
        }
    }

} // namespace

int main()
{
    using namespace myslam;

    // ------------------------------------------------------------------
    // 幂等：重复加入同一节点/边不得改变图内容
    //
    // 流水线在每个关键帧都调用 addNode/addEdge，而重建会让同一关键帧
    // 被再次登记。不幂等的话，同一条约束会被计两次信息，优化结果偏向
    // 那条边——这是一个不会让任何断言失败的静默错误。
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        domain::PoseGraphNode node;
        node.map = 0;
        node.id = 7;
        node.pose = makePose(1.0, 2.0, 3.0);
        expectTrue(graph.addNode(node), "first addNode succeeds");
        expectTrue(!graph.addNode(node), "duplicate addNode is rejected");
        expectTrue(graph.nodes().size() == 1, "duplicate addNode does not duplicate the node");

        domain::PoseGraphNode second;
        second.map = 0;
        second.id = 8;
        second.pose = makePose(2.0, 2.0, 3.0);
        graph.addNode(second);

        domain::PoseGraphEdge edge;
        edge.map = 0;
        edge.from = 7;
        edge.to = 8;
        edge.relative = makePose(1.0, 0.0, 0.0);
        expectTrue(graph.addEdge(edge), "first addEdge succeeds");
        expectTrue(!graph.addEdge(edge), "duplicate addEdge is rejected");
        expectTrue(graph.edgeCount() == 1, "duplicate addEdge does not duplicate the edge");
    }

    // ------------------------------------------------------------------
    // 端点为未知节点的边必须被拒绝，而不是悄悄创建悬空引用
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        domain::PoseGraphNode node;
        node.map = 0;
        node.id = 1;
        graph.addNode(node);

        domain::PoseGraphEdge dangling;
        dangling.map = 0;
        dangling.from = 1;
        dangling.to = 99; // 不存在
        expectTrue(!graph.addEdge(dangling), "an edge to a missing node is rejected");
        expectTrue(graph.edgeCount() == 0, "rejected edge leaves the graph untouched");
    }

    // ------------------------------------------------------------------
    // 优化真的会修正漂移，且锚点不被移动
    //
    // 这是整个位姿图唯一的核心断言：如果它不成立，这一层就只是记账。
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        buildChain(graph, 0, 6, 0.05); // 每步 y 漂 5 cm，共 25 cm

        const auto before = graph.nodes();
        expectTrue(before.size() == 6, "chain has six nodes");
        expectTrue(std::abs(before.back().pose.position.y() - 0.25) < 1e-9, "chain starts with 25 cm of y drift");

        const bool corrected = graph.optimize();
        expectTrue(corrected, "optimize reports that history was corrected");

        const auto after = graph.nodes();
        expectTrue(after.size() == 6, "optimize does not add or drop nodes");
        expectTrue((after.front().pose.position - before.front().pose.position).norm() < 1e-9,
                   "the anchor node is not moved by optimization");
        expectTrue(std::abs(after.back().pose.position.y()) < 1e-3, "y drift is removed by the between constraints");
        expectTrue(std::abs(after.back().pose.position.x() - 5.0) < 1e-3,
                   "x advance survives optimization (the chain is not collapsed)");
        expectTrue(after.back().corrections > 0, "corrected nodes record a correction count");
        expectTrue(after.front().corrections == 0, "the anchor records no correction");
    }

    // ------------------------------------------------------------------
    // 无自由度可优化时返回 false，且不算失败
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph single;
        domain::PoseGraphNode node;
        node.map = 0;
        node.id = 0;
        single.addNode(node);
        expectTrue(!single.optimize(), "a graph with no edges does not claim a correction");

        adapters::GtsamPoseGraph empty;
        expectTrue(!empty.optimize(), "an empty graph does not claim a correction");
    }

    // ------------------------------------------------------------------
    // 多地图：两张地图各自有锚点，互不污染
    //
    // 若两张地图共用同一个 gauge，优化会把第二张地图整体拖向第一张，
    // 而它们的坐标系本来就不该有关系。断言第二张图的锚点位置不变。
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        buildChain(graph, 0, 4, 0.02);
        buildChain(graph, 1, 4, 0.02);

        expectTrue(graph.mapCount() == 2, "two maps are counted separately");

        const auto map1Before = graph.nodesOf(1);
        expectTrue(map1Before.size() == 4, "map 1 has its own four nodes");
        // 两张地图的节点 id 相同但坐标不同：这是真实情况（每段都从 0 开始
        // 编号），也是「必须按 map 分组」的原因。
        expectTrue(graph.nodesOf(0).size() == 4, "map 0 keeps its four nodes");

        graph.optimize();

        const auto map0After = graph.nodesOf(0);
        const auto map1After = graph.nodesOf(1);
        expectTrue(map0After.size() == 4 && map1After.size() == 4, "optimization keeps both maps intact");
        // 两张地图的锚点都必须保持原位：各自的 gauge 由各自的先验钉住。
        expectTrue(map0After.front().pose.position.norm() < 1e-9, "map 0 anchor stays at its origin");
        const double map1AnchorDistance = (map1After.front().pose.position - map1Before.front().pose.position).norm();
        expectTrue(map1AnchorDistance < 1e-9, "map 1 anchor is not dragged by map 0");
    }

    // ------------------------------------------------------------------
    // 版本号：内容变化才前进
    //
    // 消费者用它判断"手上那份长期轨迹是否过期"。若 optimize 无修正也前进，
    // 每 8 帧就会让可视化重拉全量，白白付出一次全量拷贝。
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        const auto empty = graph.revision();
        buildChain(graph, 0, 5, 0.03);
        const auto built = graph.revision();
        expectTrue(built > empty, "adding nodes and edges advances the revision");

        graph.optimize();
        const auto optimized = graph.revision();
        expectTrue(optimized > built, "a correction advances the revision");

        const auto stable = graph.revision();
        graph.optimize(); // 已经收敛：不应再产生修正
        expectTrue(graph.revision() == stable, "a no-op optimization does not advance the revision");
    }

    // ------------------------------------------------------------------
    // 优化抛异常必须留下痕迹，而不是安静地返回「没有修正」
    //
    // 「优化失败」与「本来就没有漂移」在返回值上都是 false。若前者也不打
    // 日志、不计数，位姿图持续失效时外部只能看到「每次都没有修正」，与
    // 正常工作完全无法区分——而两者的处置方式正好相反。
    //
    // 触发方式：给一条零协方差的边。**实测结果值得记下来**：gtsam 4.3 在这个
    // 输入下既不抛异常，也没有把 NaN 写进结果——它在内部打印了 NaN 诊断，
    // 最终优化值仍是有限但未收敛的。因此本用例断言的是「不谎报修正、节点不被
    // 破坏」，而不是「失败被计数」。后者由 `finitePose` 的直接用例覆盖：
    // 依赖 gtsam 的退化行为去稳定触发某条路径是不可靠的。
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        domain::PoseGraphNode first;
        first.map = 0;
        first.id = 0;
        graph.addNode(first);

        domain::PoseGraphNode second;
        second.map = 0;
        second.id = 1;
        second.pose = makePose(1.0, 0.0, 0.0);
        second.covariance = domain::Mat6::Identity();
        graph.addNode(second);

        domain::PoseGraphEdge degenerate;
        degenerate.map = 0;
        degenerate.from = 0;
        degenerate.to = 1;
        degenerate.relative = makePose(1.0, 0.0, 0.0);
        degenerate.covariance = domain::Mat6::Zero(); // 非正定
        graph.addEdge(degenerate);

        const auto beforePositions = [&graph]()
        {
            std::vector<domain::Vec3> out;
            for (const auto &node : graph.nodes())
            {
                out.push_back(node.pose.position);
            }
            return out;
        };
        const auto before = beforePositions();

        const bool corrected = graph.optimize();
        expectTrue(!corrected, "a degenerate covariance does not produce a reported correction");
        expectTrue(graph.nodes().size() == 2, "a failed optimization leaves the nodes intact");

        // 核心断言：退化输入不得把非有限值写进节点。
        //
        // 若实现一边算一边写回，一旦优化值里混进 NaN，它就会落到节点位姿里，
        // `longTermTrajectory()` 随后把 NaN 轨迹交给调用方——而调用方无法判断
        // 这是优化失败还是估计器出错。本输入下 gtsam 恰好没产出 NaN，所以这条
        // 断言此刻是防御性的；真正的判别逻辑由下面的 finitePose 用例直接验证。
        const auto after = graph.nodes();
        bool allFinite = true;
        for (const auto &node : after)
        {
            allFinite = allFinite && node.pose.position.allFinite() && node.pose.orientation.coeffs().allFinite();
        }
        expectTrue(allFinite, "a failed optimization never writes NaN into node poses");
        expectTrue((after.front().pose.position - before.front()).norm() < 1e-12 &&
                       (after.back().pose.position - before.back()).norm() < 1e-12,
                   "node poses are left exactly as they were before the failed optimization");
    }

    // ------------------------------------------------------------------
    // 非有限位姿必须被判定函数拒绝
    //
    // 这一条直接喂 NaN 给判定函数，而不是等 gtsam 自己产出 NaN：后者依赖
    // 版本与内部实现，无法靠构造输入稳定触发。判定被拒之后「整批放弃」是
    // 三行直白代码——真正的风险在判定本身写漏一个分量（例如只查 position
    // 不查四元数），那会让 NaN 顺着 `longTermTrajectory()` 流到调用方。
    // ------------------------------------------------------------------
    {
        const double nan = std::nan("");
        const double inf = std::numeric_limits<double>::infinity();

        const gtsam::Pose3 healthy(gtsam::Rot3(), gtsam::Point3(1.0, 2.0, 3.0));
        const auto ok = adapters::GtsamPoseGraph::finitePose(healthy);
        expectTrue(ok.has_value(), "a finite pose is accepted");
        expectTrue(ok.has_value() && (ok->position - myslam::domain::Vec3(1.0, 2.0, 3.0)).norm() < 1e-12,
                   "the accepted pose keeps its translation");

        const gtsam::Pose3 nanPosition(gtsam::Rot3(), gtsam::Point3(nan, 0.0, 0.0));
        expectTrue(!adapters::GtsamPoseGraph::finitePose(nanPosition).has_value(),
                   "NaN in the translation is rejected");

        const gtsam::Pose3 infPosition(gtsam::Rot3(), gtsam::Point3(0.0, inf, 0.0));
        expectTrue(!adapters::GtsamPoseGraph::finitePose(infPosition).has_value(),
                   "infinity in the translation is rejected");

        // 四元数里出现 NaN：只查 position 的实现会漏掉这一种
        Eigen::Quaterniond broken = Eigen::Quaterniond::Identity();
        broken.x() = nan;
        const gtsam::Pose3 nanRotation(gtsam::Rot3(broken.normalized().toRotationMatrix()),
                                       gtsam::Point3(0.0, 0.0, 0.0));
        // 归一化 NaN 四元数会得到非有限矩阵；这里只要求判定结果与矩阵是否
        // 有限一致，避免把 gtsam 的归一化策略钉死。
        const bool matrixFinite = nanRotation.rotation().matrix().allFinite();
        expectTrue(adapters::GtsamPoseGraph::finitePose(nanRotation).has_value() == matrixFinite,
                   "rotation finiteness is judged from the same data the optimizer wrote");
    }

    // ------------------------------------------------------------------
    // 旋转也会被修正（不只看平移）
    // ------------------------------------------------------------------
    {
        adapters::GtsamPoseGraph graph;
        for (int i = 0; i < 4; ++i)
        {
            domain::PoseGraphNode node;
            node.map = 0;
            node.id = i;
            // 累积航向误差：每步多转 1 度
            const double yaw = (i == 0 ? 0.0 : 1.0 * std::numbers::pi / 180.0 * static_cast<double>(i));
            node.pose = myslam::domain::Pose{};
            node.pose.position = domain::Vec3(static_cast<double>(i), 0.0, 0.0);
            node.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            node.covariance = i == 0 ? domain::PoseCovariance{} : domain::Mat6::Identity();
            graph.addNode(node);
            if (i > 0)
            {
                domain::PoseGraphEdge edge;
                edge.map = 0;
                edge.from = i - 1;
                edge.to = i;
                edge.relative.position = domain::Vec3(1.0, 0.0, 0.0); // 真值：只前进不转
                edge.covariance = domain::defaultRelativeCovariance();
                graph.addEdge(edge);
            }
        }
        graph.optimize();
        const double finalYaw = graph.nodes().back().pose.orientation.angularDistance(Eigen::Quaterniond::Identity());
        expectTrue(finalYaw < 1e-3, "accumulated heading error is corrected, not only translation");
    }

    std::printf("checks=%d failures=%d\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
