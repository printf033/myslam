/**
 * @file 5_loop_closure.cpp
 * @brief 离线回环闭环验证：用已有轨迹 + 原序列图像，量化三圈能否被位姿图拉齐。
 *
 * 为什么先做离线：回环检测的有效性不依赖在线时序。把「轨迹位姿」当成已知量，
 * 只检验两件事——外观检索能否找到真实重访、几何验证给出的相对位姿是否足够
 * 准——就能判断这条路值不值得接进流水线。在线接入多出来的是时序与写回，
 * 不会改变这两件事的结论。
 *
 * 用法：
 * @code
 * 5_loop_closure --config config/subt_handheld1.lua \
 *                --tum build-release/subt_handheld1.tum \
 *                --out build-release/subt_loop_closed.tum
 * @endcode
 *
 * 输出：优化前后分圈质心偏移、终点闭合误差、回环清单（CSV）。
 */

#include "adapters/gtsam/pose_graph.hpp"
#include "config/lua_config.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/types.hpp"
#include "frontend/orb_loop_detector.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "sensor/subt_dataset.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{

    struct Options
    {
        std::string config{"config/subt_handheld1.lua"};
        std::string tum{"build-release/subt_handheld1.tum"};
        std::string out{"build-release/subt_loop_closed.tum"};
        std::string loopsCsv{"build-release/subt_loop_edges.csv"};
        std::size_t maxKeyframes{0};
    };

    void printUsage()
    {
        std::fprintf(stderr, "用法: 5_loop_closure [选项]\n"
                             "  --config FILE      Lua 配置（默认 config/subt_handheld1.lua）\n"
                             "  --tum FILE         关键帧轨迹（默认 build-release/subt_handheld1.tum）\n"
                             "  --out FILE         修正后轨迹（默认 build-release/subt_loop_closed.tum）\n"
                             "  --loops FILE       回环清单 CSV（默认 build-release/subt_loop_edges.csv）\n"
                             "  --max-keyframes N  只处理前 N 个关键帧（快速试跑，0 = 全部）\n"
                             "  --help             显示本帮助\n");
    }

    struct KeyframeRecord
    {
        double timestamp{0.0};
        myslam::domain::Pose pose{};
    };

    std::vector<KeyframeRecord> loadTum(const std::string &path)
    {
        std::vector<KeyframeRecord> records;
        std::ifstream in(path);
        if (!in)
        {
            return records;
        }
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }
            std::istringstream stream(line);
            KeyframeRecord record;
            double qx = 0.0;
            double qy = 0.0;
            double qz = 0.0;
            double qw = 1.0;
            if (!(stream >> record.timestamp >> record.pose.position.x() >> record.pose.position.y() >>
                  record.pose.position.z() >> qx >> qy >> qz >> qw))
            {
                continue;
            }
            record.pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
            records.push_back(record);
        }
        return records;
    }

    /// 找与给定相对秒最接近的图像帧号
    std::optional<std::size_t> nearestFrame(const std::vector<double> &stamps, double target, double tolerance)
    {
        if (stamps.empty())
        {
            return std::nullopt;
        }
        const auto it = std::lower_bound(stamps.begin(), stamps.end(), target);
        std::size_t best = static_cast<std::size_t>(std::distance(stamps.begin(), it));
        if (best >= stamps.size())
        {
            best = stamps.size() - 1;
        }
        if (best > 0 && std::abs(stamps[best - 1] - target) < std::abs(stamps[best] - target))
        {
            best -= 1;
        }
        if (std::abs(stamps[best] - target) > tolerance)
        {
            return std::nullopt;
        }
        return best;
    }

    struct LapStats
    {
        myslam::domain::Vec3 centroid{};
        double pathLength{0.0};
        std::size_t count{0};
    };

    /// 按弧长均分，把轨迹切成 count 段
    std::vector<LapStats> splitByArcLength(const std::vector<myslam::domain::Pose> &poses, int count)
    {
        std::vector<LapStats> stats(static_cast<std::size_t>(count));
        if (poses.size() < 2 || count <= 0)
        {
            return stats;
        }
        std::vector<double> cumulative(poses.size(), 0.0);
        for (std::size_t i = 1; i < poses.size(); ++i)
        {
            cumulative[i] = cumulative[i - 1] + (poses[i].position - poses[i - 1].position).norm();
        }
        const double total = cumulative.back();
        int bucket = 0;
        for (std::size_t i = 0; i < poses.size(); ++i)
        {
            while (bucket + 1 < count &&
                   cumulative[i] >= total * static_cast<double>(bucket + 1) / static_cast<double>(count))
            {
                ++bucket;
            }
            LapStats &lap = stats[static_cast<std::size_t>(bucket)];
            lap.centroid += poses[i].position;
            ++lap.count;
            if (i > 0)
            {
                lap.pathLength += cumulative[i] - cumulative[i - 1];
            }
        }
        for (LapStats &lap : stats)
        {
            if (lap.count > 0)
            {
                lap.centroid /= static_cast<double>(lap.count);
            }
        }
        return stats;
    }

    void reportLaps(const std::string &label, const std::vector<myslam::domain::Pose> &poses)
    {
        const std::vector<LapStats> laps = splitByArcLength(poses, 3);
        mylog::info("[{}] 分段质心：", label);
        for (std::size_t i = 0; i < laps.size(); ++i)
        {
            const double offset = i == 0 ? 0.0 : (laps[i].centroid - laps[0].centroid).norm();
            mylog::info("  第 {} 圈: 质心 ({:.2f}, {:.2f}, {:.2f})，相对第 1 圈偏移 {:.2f} m，路径 {:.1f} m，{} 帧",
                        i + 1, laps[i].centroid.x(), laps[i].centroid.y(), laps[i].centroid.z(), offset,
                        laps[i].pathLength, laps[i].count);
        }
        if (!poses.empty())
        {
            const double closure = (poses.back().position - poses.front().position).norm();
            mylog::info("  终点闭合误差 {:.2f} m（起点 ({:.2f}, {:.2f}, {:.2f}) → 终点 ({:.2f}, {:.2f}, {:.2f})）",
                        closure, poses.front().position.x(), poses.front().position.y(), poses.front().position.z(),
                        poses.back().position.x(), poses.back().position.y(), poses.back().position.z());
        }
    }

} // namespace

int main(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next = [&](std::string &target)
        {
            if (i + 1 < argc)
            {
                target = argv[++i];
            }
        };
        if (arg == "--config")
        {
            next(options.config);
        }
        else if (arg == "--tum")
        {
            next(options.tum);
        }
        else if (arg == "--out")
        {
            next(options.out);
        }
        else if (arg == "--loops")
        {
            next(options.loopsCsv);
        }
        else if (arg == "--max-keyframes")
        {
            std::string value;
            next(value);
            options.maxKeyframes = static_cast<std::size_t>(std::max(0, std::atoi(value.c_str())));
        }
        else if (arg == "--help" || arg == "-h")
        {
            printUsage();
            return 0;
        }
        else
        {
            std::fprintf(stderr, "未知参数: %s\n", arg.c_str());
            printUsage();
            return 2;
        }
    }

    // 本工具是诊断程序：日志留在控制台而不是写文件，避免「跑完还要去翻日志」。
    // applyLogging = false 阻止 Lua 里的 fileManager 节接管输出后端。
    myslam::config::ProjectConfig project;
    if (const auto loaded = myslam::config::load(options.config, project, /*applyLogging=*/false); !loaded)
    {
        std::fprintf(stderr, "[error] 配置加载失败: %s\n", loaded.error().message.c_str());
        return 2;
    }

    const auto &sequenceConfig = project.input.sequence;
    if (!sequenceConfig.configured())
    {
        std::fprintf(stderr, "[error] 未配置 dataset.sequence.root\n");
        return 2;
    }
    myslam::dataset::SubtSequence sequence(sequenceConfig.resolveCameraDir(), sequenceConfig.resolveImuFile(),
                                           sequenceConfig.resolveStampFile());
    if (const auto loaded = sequence.load(); !loaded)
    {
        std::fprintf(stderr, "[error] %s\n", loaded.error().message.c_str());
        return 1;
    }

    const std::vector<KeyframeRecord> keyframes = loadTum(options.tum);
    if (keyframes.size() < 2)
    {
        std::fprintf(stderr, "[error] 轨迹文件为空或格式不符: %s\n", options.tum.c_str());
        return 1;
    }

    const myslam::domain::Pose bodyPSensor =
        project.input.hasExtrinsic ? project.input.extrinsic : project.vio.camera.bodyPSensor;
    const std::shared_ptr<const myslam::domain::ICameraModel> cameraModel =
        myslam::domain::makeCameraModel(project.intrinsics);

    mylog::info("[回环] 关键帧 {} 个，图像 {} 张，相机模型 {}，等效焦距 {:.1f} px", keyframes.size(), sequence.size(),
                cameraModel->name(), cameraModel->focalLengthPx());

    myslam::frontend::LoopDetectorConfig detectorConfig;
    detectorConfig.featuresPerFrame =
        project.replay.maxFeatures > 0 ? std::max(600, project.replay.maxFeatures * 6) : 1200;
    myslam::frontend::OrbLoopDetector detector(cameraModel, bodyPSensor, detectorConfig);

    std::vector<myslam::domain::Pose> poses;
    poses.reserve(keyframes.size());
    std::vector<bool> indexed;
    indexed.reserve(keyframes.size());

    std::vector<myslam::frontend::LoopConstraint> loops;
    const std::size_t limit =
        options.maxKeyframes > 0 ? std::min(options.maxKeyframes, keyframes.size()) : keyframes.size();

    const auto wallStart = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < limit; ++i)
    {
        const KeyframeRecord &record = keyframes[i];
        poses.push_back(record.pose);
        const std::optional<std::size_t> frame = nearestFrame(sequence.stamps(), record.timestamp, 0.05);
        bool ok = false;
        if (frame)
        {
            const cv::Mat gray = cv::imread(sequence.imagePath(*frame), cv::IMREAD_GRAYSCALE);
            if (!gray.empty() && gray.cols == project.intrinsics.imageWidth &&
                gray.rows == project.intrinsics.imageHeight)
            {
                const std::size_t mapPoints = detector.addKeyframe(gray, static_cast<myslam::domain::KeyframeId>(i), 0,
                                                                   record.timestamp, record.pose);
                if (mapPoints > 0)
                {
                    ok = true;
                    std::vector<myslam::frontend::LoopConstraint> found = detector.query(1);
                    for (auto &constraint : found)
                    {
                        loops.push_back(std::move(constraint));
                    }
                }
            }
        }
        indexed.push_back(ok);

        if ((i + 1) % 200 == 0 || i + 1 == limit)
        {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
            mylog::info("[回环] 进度 {}/{}，索引 {} 帧，命中回环 {} 条，拒绝 PnP {} 次，{:.1f}s", i + 1, limit,
                        detector.keyframeCount(), loops.size(), detector.rejectedPnP(), elapsed);
        }
    }

    const auto &st = detector.rejectionStats();
    {
        std::vector<int> hist = st.inlierHistogram;
        std::sort(hist.begin(), hist.end());
        if (!hist.empty())
        {
            const std::size_t n = hist.size();
            mylog::info("[回环] RANSAC 内点数分布：P10={} P50={} P75={} P90={} P99={} 最大={}（共 {} 次求解）",
                        hist[n / 10], hist[n / 2], hist[n * 3 / 4], hist[n * 9 / 10],
                        hist[std::min(n - 1, n * 99 / 100)], hist.back(), n);
        }
        std::vector<double> trans = st.rejectedTranslations;
        std::sort(trans.begin(), trans.end());
        if (!trans.empty())
        {
            const std::size_t n = trans.size();
            mylog::info("[回环] 通过内点门限后的相对平移：P10={:.2f} P50={:.2f} P90={:.2f} 最大={:.2f} m（共 {} 次）",
                        trans[n / 10], trans[n / 2], trans[n * 9 / 10], trans.back(), n);
        }
    }
    mylog::info("[回环] 检测结束：{} 条回环；验证 {} 次 → 匹配不足 {} / 内点不足 {} / 非有限 {} / "
                "偏离 VIO 过多 {} / 非重访 {} / 采纳 {}",
                loops.size(), st.verified, st.tooFewMatches, st.lowInliers, st.nonFinite, st.badCorrection,
                st.notCoincident, st.accepted);
    for (const auto &loop : loops)
    {
        mylog::info("  回环 #{} → #{}: 内点 {}，相对平移 ({:.2f}, {:.2f}, {:.2f}) m", loop.from, loop.to, loop.inliers,
                    loop.relative.position.x(), loop.relative.position.y(), loop.relative.position.z());
    }

    // ---------------------------------------------------------------------
    // 位姿图：共视边 + 回环边，一次性优化
    // ---------------------------------------------------------------------
    myslam::adapters::GtsamPoseGraph graph;
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
        myslam::domain::PoseGraphNode node;
        node.map = 0;
        node.id = static_cast<myslam::domain::KeyframeId>(i);
        node.timestamp = keyframes[i].timestamp;
        node.pose = poses[i];
        node.covariance = i == 0 ? myslam::domain::PoseCovariance{}
                                 : myslam::domain::PoseCovariance{myslam::domain::Mat6::Identity() * 1e-4};
        (void)graph.addNode(node);
    }
    for (std::size_t i = 1; i < poses.size(); ++i)
    {
        myslam::domain::PoseGraphEdge edge;
        edge.map = 0;
        edge.from = static_cast<myslam::domain::KeyframeId>(i - 1);
        edge.to = static_cast<myslam::domain::KeyframeId>(i);
        edge.relative = poses[i - 1].inverse() * poses[i];
        edge.covariance = myslam::domain::defaultRelativeCovariance();
        edge.sharedObservations = 100;
        (void)graph.addEdge(edge);
    }
    for (const auto &loop : loops)
    {
        myslam::domain::PoseGraphEdge edge;
        edge.map = 0;
        edge.from = loop.from;
        edge.to = loop.to;
        edge.relative = loop.relative;
        edge.covariance = detector.loopCovariance(loop.inliers);
        edge.sharedObservations = loop.inliers;
        edge.loopClosure = true;
        (void)graph.addEdge(edge);
    }

    // 连续优化直到不再修正：单次 LM 在三千节点上可能停得偏早，
    // 而「回环残差」正是判断它是否真的走到最优点的唯一直接证据。
    bool corrected = false;
    for (int pass = 0; pass < 5; ++pass)
    {
        if (!graph.optimize())
        {
            break;
        }
        corrected = true;
    }
    mylog::info("[回环] 位姿图优化{}（{} 节点，{} 边）", corrected ? "修正了历史位姿" : "无修正", graph.nodes().size(),
                graph.edgeCount());

    std::vector<myslam::domain::Pose> optimized;
    optimized.reserve(graph.nodes().size());
    for (const auto &node : graph.nodes())
    {
        optimized.push_back(node.pose);
    }

    // ---------------------------------------------------------------------
    // 结果对比
    // ---------------------------------------------------------------------
    reportLaps("优化前", poses);
    reportLaps("优化后", optimized);

    {
        std::ofstream out(options.out);
        if (out)
        {
            for (std::size_t i = 0; i < optimized.size(); ++i)
            {
                const myslam::domain::Pose &pose = optimized[i];
                const Eigen::Quaterniond &q = pose.orientation;
                out << std::format("{} {} {} {} {} {} {} {}\n", keyframes[i].timestamp, pose.position.x(),
                                   pose.position.y(), pose.position.z(), q.x(), q.y(), q.z(), q.w());
            }
            mylog::info("[回环] 修正后轨迹已写入 {}", options.out);
        }
    }
    {
        std::ofstream out(options.loopsCsv);
        if (out)
        {
            out << "from,to,inliers,tx,ty,tz\n";
            for (const auto &loop : loops)
            {
                out << std::format("{},{},{},{},{},{}\n", loop.from, loop.to, loop.inliers, loop.relative.position.x(),
                                   loop.relative.position.y(), loop.relative.position.z());
            }
        }
    }
    return 0;
}
