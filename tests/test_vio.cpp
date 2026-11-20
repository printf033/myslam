/**
 * @file test_vio.cpp
 * @brief VIO 核心回归测试：契约自持、可替换性、恢复语义与时间同步。
 *
 * 测试只用领域层与公开接口，不包含 gtsam 头：如果某条断言需要 gtsam 类型
 * 才能表达，说明契约又漏回了实现细节。
 */

#include "adapters/register_builtin.hpp"
#include "adapters/simple/imu_only_estimator.hpp"
#include "app/trajectory_eval.hpp"
#include "common/config_summary.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/health_monitor.hpp"
#include "domain/initializer.hpp"
#include "domain/place_recognition.hpp"
#include "domain/ports.hpp"
#include "domain/pose_graph.hpp"
#include "domain/propagator.hpp"
#include "domain/registry.hpp"
#include "domain/time_synchronizer.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "vio.hpp"
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <vector>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
/**
 * @brief LeakSanitizer 默认抑制规则
 *
 * gtsam 的全局 debug 标志表 (gtsam/base/debug.cpp) 在进程退出阶段受静态析构
 * 顺序影响，会被 LSan 误报为泄漏；该分配不属于本项目代码，按调用帧抑制。
 * 两个示例入口已有同一处理，测试漏了它，于是 ctest 在 Debug+ASan 下恒失败。
 */
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:gtsam::guardedIsDebug\n";
}
#endif

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

    /// 最小估计器替身：验证流水线只依赖 IEstimator
    class StubEstimator final : public myslam::domain::IEstimator
    {
    public:
        [[nodiscard]] std::string_view name() const noexcept override { return "stub"; }

        bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &,
                        const myslam::domain::PoseCovariance & = {}) override
        {
            return initializeResult;
        }

        myslam::domain::StepResult addKeyframe(const myslam::domain::Keyframe &,
                                               std::span<const myslam::domain::ImuSlice>) override
        {
            ++processCalls;
            myslam::domain::StepResult result;
            result.status = processResult ? myslam::domain::StepStatus::OK : myslam::domain::StepStatus::RECOVERED;
            if (processResult)
            {
                result.state = state;
            }
            result.health = report;
            return result;
        }

        [[nodiscard]] myslam::domain::KinematicState latestState() const override { return state; }
        [[nodiscard]] myslam::domain::HealthReport health() const override { return report; }
        [[nodiscard]] myslam::domain::EstimateSnapshot snapshot() const override { return {}; }

        int processCalls{0};
        bool initializeResult{true};
        bool processResult{true};
        myslam::domain::KinematicState state{};
        myslam::domain::HealthReport report{};
    };

    /// 计数装饰器：证明编排层确实使用调用方注入的数据源
    class CountingSource final : public myslam::domain::ISensorSource
    {
    public:
        explicit CountingSource(const myslam::domain::PipelineConfig &config)
            : inner_(myslam::vio::makeSimulatorOptions(config))
        {
        }

        [[nodiscard]] myslam::domain::ImuSample imuAt(double time) override
        {
            ++imuCalls;
            return inner_.imuAt(time);
        }

        [[nodiscard]] myslam::domain::Frame frameAt(int frameIndex, double time) override
        {
            ++cameraCalls;
            return inner_.frameAt(frameIndex, time);
        }

        [[nodiscard]] std::optional<myslam::domain::GroundTruth> groundTruthAt(double time) override
        {
            return inner_.groundTruthAt(time);
        }

        int imuCalls{0};
        int cameraCalls{0};

    private:
        myslam::vio::VioSimulator inner_;
    };

    /// 无真值源：沿用基类的 nullopt 默认实现
    class NoTruthSource final : public myslam::domain::ISensorSource
    {
    public:
        explicit NoTruthSource(const myslam::domain::PipelineConfig &config)
            : inner_(myslam::vio::makeSimulatorOptions(config))
        {
        }

        [[nodiscard]] myslam::domain::ImuSample imuAt(double time) override { return inner_.imuAt(time); }
        [[nodiscard]] myslam::domain::Frame frameAt(int frameIndex, double time) override
        {
            return inner_.frameAt(frameIndex, time);
        }

    private:
        myslam::vio::VioSimulator inner_;
    };

    /**
     * @brief 位置识别替身：只记录被索引的关键帧
     *
     * 参考实现缺席是刻意的（可靠的跨地图检索需要外观描述子），但端口本身
     * 必须可验证：它决定 @c RELOCALIZE 是否可执行。否则这个端口就是一段
     * 永远不会被调用的声明。
     */
    class StubPlaceRecognizer final : public myslam::domain::IPlaceRecognizer
    {
    public:
        [[nodiscard]] std::string_view name() const noexcept override { return "stub_recognizer"; }

        bool index(myslam::domain::MapId map, const myslam::domain::Keyframe &keyframe) override
        {
            indexed.push_back({map, keyframe.id});
            return true;
        }

        [[nodiscard]] std::vector<myslam::domain::PlaceCandidate> query(const myslam::domain::Keyframe &,
                                                                        std::size_t) override
        {
            ++queryCalls;
            return {};
        }

        void reset() override { indexed.clear(); }

        [[nodiscard]] std::size_t indexedKeyframes() const override { return indexed.size(); }

        std::vector<std::pair<myslam::domain::MapId, myslam::domain::KeyframeId>> indexed;
        int queryCalls{0};
    };

    /**
     * @brief 位姿图替身：只记录调用，不做优化
     *
     * 流水线的职责是「登记正确的节点与边」，gtsam 实现的职责是「优化对不对」。
     * 两者分开测，前者用本替身就能覆盖，不必把优化器拉进每一个断言。
     */
    class StubPoseGraph final : public myslam::domain::IPoseGraph
    {
    public:
        [[nodiscard]] bool addNode(const myslam::domain::PoseGraphNode &node) override
        {
            for (const auto &existing : nodes_)
            {
                if (existing.map == node.map && existing.id == node.id)
                {
                    return false;
                }
            }
            nodes_.push_back(node);
            ++revision_;
            return true;
        }

        [[nodiscard]] bool addEdge(const myslam::domain::PoseGraphEdge &edge) override
        {
            for (const auto &existing : edges_)
            {
                if (existing.map == edge.map && existing.from == edge.from && existing.to == edge.to)
                {
                    return false;
                }
            }
            edges_.push_back(edge);
            ++revision_;
            return true;
        }

        [[nodiscard]] bool optimize() override
        {
            ++optimizeCalls;
            // 默认不宣称修正：否则「登记节点推进版本号」这条断言会被这里的
            // 返回值污染，无法区分版本号到底是谁推进的。
            return reportCorrection;
        }

        [[nodiscard]] std::vector<myslam::domain::PoseGraphNode> nodes() const override { return nodes_; }

        [[nodiscard]] std::vector<myslam::domain::PoseGraphNode> nodesOf(myslam::domain::MapId map) const override
        {
            std::vector<myslam::domain::PoseGraphNode> out;
            for (const auto &node : nodes_)
            {
                if (node.map == map)
                {
                    out.push_back(node);
                }
            }
            return out;
        }

        [[nodiscard]] std::vector<myslam::domain::PoseGraphEdge> edgesOf(myslam::domain::MapId map) const override
        {
            std::vector<myslam::domain::PoseGraphEdge> out;
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

        [[nodiscard]] std::size_t mapCount() const override
        {
            std::set<myslam::domain::MapId> maps;
            for (const auto &node : nodes_)
            {
                maps.insert(node.map);
            }
            return maps.size();
        }

        [[nodiscard]] std::uint64_t revision() const override { return revision_; }

        std::vector<myslam::domain::PoseGraphNode> nodes_;
        std::vector<myslam::domain::PoseGraphEdge> edges_;
        int optimizeCalls{0};
        /// optimize() 的返回值：false 表示「没有历史位姿被修正」
        bool reportCorrection{false};
        std::uint64_t revision_{0};
    };

    /**
     * @brief 可配置失败语义的估计器替身
     *
     * 与 @c StubEstimator 分开：那个只区分「成功 / 局部恢复」，而恢复门控
     * 需要表达「失败 + 建议重建 + 降级类型」的三元组合。
     */
    class RecoveryStubEstimator final : public myslam::domain::IEstimator
    {
    public:
        [[nodiscard]] std::string_view name() const noexcept override { return "recovery_stub"; }

        bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &,
                        const myslam::domain::PoseCovariance & = {}) override
        {
            return true;
        }

        myslam::domain::StepResult addKeyframe(const myslam::domain::Keyframe &,
                                               std::span<const myslam::domain::ImuSlice>) override
        {
            ++processCalls;
            myslam::domain::StepResult result;
            result.status = fail ? myslam::domain::StepStatus::FAILED : myslam::domain::StepStatus::OK;
            result.recovery = fail ? recovery : myslam::domain::RecoveryAction::NONE;
            result.health = report;
            result.detail = "test: scripted failure";
            if (!fail)
            {
                result.state = state;
            }
            return result;
        }

        [[nodiscard]] myslam::domain::KinematicState latestState() const override { return state; }
        [[nodiscard]] myslam::domain::HealthReport health() const override { return report; }
        [[nodiscard]] myslam::domain::EstimateSnapshot snapshot() const override { return {}; }

        bool fail{true};
        myslam::domain::RecoveryAction recovery{myslam::domain::RecoveryAction::RESET_PIPELINE};
        myslam::domain::HealthReport report{};
        myslam::domain::KinematicState state{};
        int processCalls{0};
    };

    /// 直接驱动流水线（不经 VioBenchmark），返回是否初始化与产出位姿数
    struct DirectRun
    {
        bool initialized{false};
        int poses{0};
    };

    DirectRun drivePipeline(myslam::vio::VioPipeline &pipeline, myslam::vio::VioSimulator &simulator,
                            const myslam::domain::PipelineConfig &config, const myslam::domain::SystemConfig &system)
    {
        DirectRun run;
        const double imuDt = myslam::domain::SystemConfig::imuDtFrom(config.imu.rateHz);
        double time = 0.0;
        double nextCameraTime = 0.0;
        int frameId = 0;
        myslam::domain::KeyframeId keyframeId = 0;

        while (time <= system.totalDuration)
        {
            pipeline.feedImu(myslam::domain::ImuSlice{simulator.imuAt(time), imuDt});

            if (time >= nextCameraTime - 1e-6)
            {
                const myslam::domain::Frame frame = simulator.frameAt(frameId++, time);
                if (!pipeline.tracking())
                {
                    const myslam::vio::InitOutcome init = pipeline.stepInitialization(keyframeId, frame, time);
                    if (init.tracking)
                    {
                        run.initialized = true;
                        ++keyframeId;
                        ++run.poses;
                    }
                }
                else
                {
                    const myslam::vio::FrameOutcome outcome = pipeline.processFrame(keyframeId, frame, time);
                    if (outcome.producedPose())
                    {
                        ++keyframeId;
                        ++run.poses;
                    }
                }
                nextCameraTime += system.cameraDt;
            }

            time += imuDt;
        }
        return run;
    }

    /**
     * @brief 测试用 MEI 内参：代码内置默认值，仅把模型切换为 MEI
     *
     * 本文件只验证「模型按名字注册、能力查询正确、流水线能跑通 MEI 代码路径」，
     * 与标定数值无关，因此不携带任何数据源参数。注意默认 xi = 0 时 MEI 投影
     * 在数学上退化为针孔，鱼眼几何（视场 > 180°、畸变、视场收窄）由
     * tests/test_lua_config.cpp 的 checkShippedFisheyeGeometry 用真实 lua
     * 标定验证。
     */
    myslam::camera::CameraIntrinsics meiIntrinsicsFromDefaults()
    {
        myslam::camera::CameraIntrinsics intrinsics{};
        intrinsics.model = myslam::camera::CameraModel::MEI;
        return intrinsics;
    }

} // namespace

int main()
{
    myslam::LogConfig logConfig;
    myslam::configureLogging(logConfig);
    myslam::adapters::registerBuiltinEstimators();

    myslam::domain::PipelineConfig config;
    // 首关键帧退化门控针对真实前端；测试里的 stub 观测是人为构造的
    // （数量少、挤在一条直线上），显式关闭以免门控把桩路径拦住。
    config.initialization.minInitObservations = 0;
    config.initialization.minInitSpreadDeg = 0.0;
    myslam::domain::SystemConfig systemConfig;

    // ------------------------------------------------------------------
    // 默认配置与验收口径
    // ------------------------------------------------------------------
    {
        expectTrue(config.camera.intrinsics.fx > 0.0, "camera intrinsics are positive");
        expectTrue(myslam::domain::SystemConfig::imuDtFrom(config.imu.rateHz) > 0.0 && systemConfig.cameraDt > 0.0,
                   "sample steps are positive");
        expectTrue(myslam::domain::SystemConfig::imuDtFrom(0.0) == 0.0, "non-positive imu rate yields zero step");
        expectTrue(systemConfig.rmseThreshold > 0.0, "rmse threshold is positive");
    }

    myslam::vio::VioBenchmark benchmark(config, systemConfig);
    const myslam::domain::RunReport report = benchmark.run();

    expectTrue(report.initialized, "default run completes initialization");
    expectTrue(report.keyframeCount > 0, "keyframes were processed");
    expectTrue(report.rmse >= 0.0, "rmse is non-negative");
    expectTrue(report.maxError >= report.rmse, "max error bounds rmse");
    expectTrue(report.rmse < systemConfig.rmseThreshold, "rmse within acceptance threshold");
    expectTrue(myslam::domain::isAcceptable(report, systemConfig.rmseThreshold),
               "default run is accepted by isAcceptable");

    // ------------------------------------------------------------------
    // 可自证性：协方差必须被上报，且 NEES 落在一致性带上
    //
    // 这是「精度」之外的第二个结论。只断言 RMSE 无法区分「碰巧算对」与
    // 「知道自己有多不确定」；后者才能外推。上下界取得很宽（不是统计
    // 严格检验，而是回归门禁）：均值远大于 20 说明协方差严重偏小，
    // 均值远小于 0.2 说明根本不是在估计不确定性。
    // ------------------------------------------------------------------
    {
        expectTrue(report.neesSamples > 0, "backend publishes pose covariance for consistency checking");
        // 初始化锚点走 initialize() 而不是 addKeyframe()，不产出 StepResult，
        // 因此它没有边际协方差。这是契约形状决定的，不是缺陷：为它编造一个
        // 协方差只会让 NEES 的分子分母不再同源。除锚点外必须每个关键帧都有。
        expectTrue(report.neesSamples + 1 >= report.keyframeCount,
                   "every keyframe except the initial anchor carries a covariance sample");
        expectTrue(report.neesMean > 0.0, "nees mean is positive");
        expectTrue(myslam::domain::isConsistent(report, 0.2, 20.0),
                   "nees mean stays inside the consistency band (theory: 3)");

        // 反向用例：无法检验不得被当成检验通过
        myslam::domain::RunReport noCovariance = report;
        noCovariance.neesSamples = 0;
        expectTrue(!myslam::domain::isConsistent(noCovariance, 0.2, 20.0),
                   "missing covariance is not treated as consistent");
        myslam::domain::RunReport overconfident = report;
        overconfident.neesSamples = 10;
        overconfident.neesMean = 1000.0;
        expectTrue(!myslam::domain::isConsistent(overconfident, 0.2, 20.0),
                   "grossly overconfident covariance is rejected");
    }

    // ------------------------------------------------------------------
    // 逆深度路标：可切换、能跑通、仍然上报协方差
    //
    // 这里不断言逆深度的精度优于全局 3D 点——那是数据相关的结论，不该被
    // 钉死成回归门禁。要钉住的是「这条参数化路径确实是活的」：配置能切
    // 过去、流水线能初始化、能产出位姿、协方差照旧上报。仿真时长压到 2 s
    // 只为覆盖路径，不追求精度。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig inverseDepthConfig = config;
        inverseDepthConfig.estimator.landmarkRepresentation = "anchored_inverse_depth";
        myslam::domain::SystemConfig shortSystem = systemConfig;
        shortSystem.totalDuration = 2.0;

        myslam::vio::VioBenchmark inverseDepthBenchmark(inverseDepthConfig, shortSystem);
        const myslam::domain::RunReport inverseDepthReport = inverseDepthBenchmark.run();

        expectTrue(inverseDepthReport.initialized, "inverse-depth parameterization completes initialization");
        expectTrue(inverseDepthReport.keyframeCount >= 2, "inverse-depth parameterization produces keyframe poses");
        expectTrue(inverseDepthReport.neesSamples > 0, "inverse-depth parameterization still publishes covariance");
        expectTrue(inverseDepthReport.rmse < shortSystem.rmseThreshold,
                   "inverse-depth parameterization stays within the rmse threshold");
    }

    // ------------------------------------------------------------------
    // 验收判定不得把「没结果」当成「完美」
    // ------------------------------------------------------------------
    {
        myslam::domain::RunReport healthy = report;
        myslam::domain::RunReport noKeyframes = healthy;
        noKeyframes.keyframeCount = 0;
        expectTrue(!myslam::domain::isAcceptable(noKeyframes, 0.20), "zero keyframes is rejected");

        myslam::domain::RunReport onlyAnchor = healthy;
        onlyAnchor.keyframeCount = 1;
        onlyAnchor.rmse = 0.0;
        expectTrue(!myslam::domain::isAcceptable(onlyAnchor, 0.20),
                   "single initial-anchor sample must not be accepted as a trajectory");

        myslam::domain::RunReport inaccurate = healthy;
        inaccurate.rmse = 5.0;
        expectTrue(!myslam::domain::isAcceptable(inaccurate, 0.20), "rmse above threshold is rejected");
    }

    // ------------------------------------------------------------------
    // 默认几何不退化，标称运行不重建
    // ------------------------------------------------------------------
    {
        expectTrue(config.vision.minLandmarkDepth > 0.0, "landmark depth lower bound is positive");
        expectTrue(config.vision.maxLandmarkDepth > config.vision.minLandmarkDepth,
                   "landmark depth default interval is non-empty");
        expectTrue(report.resets == 0, "a nominal run performs no pipeline rebuild");
    }

    // ------------------------------------------------------------------
    // 健康监控滞回与降级分类
    // ------------------------------------------------------------------
    {
        myslam::domain::HealthConfig strict;
        strict.alertStreakThreshold = 1;
        strict.maxSpeed = 1.0;
        myslam::domain::HealthMonitor immediate(strict);

        myslam::domain::KinematicState fast;
        fast.velocity = myslam::domain::Vec3(2.0, 0.0, 0.0);
        expectTrue(!immediate.inspect(fast, 100).isHealthy,
                   "custom speed threshold makes 2 m/s unhealthy and degrades immediately");
        expectTrue(myslam::domain::HealthMonitor::diagnose(fast, 100, strict).degradation ==
                       myslam::domain::Degradation::INERTIAL,
                   "speed threshold violation is classified as INERTIAL");

        myslam::domain::HealthMonitor monitor;
        myslam::domain::KinematicState veryFast;
        veryFast.velocity = myslam::domain::Vec3(50.0, 0.0, 0.0);
        // 第一次异常：streak = 1，未达阈值
        expectTrue(monitor.inspect(veryFast, 100).isHealthy, "single-frame anomaly does not trigger degradation");
        for (int i = 1; i < monitor.config().alertStreakThreshold - 1; ++i)
        {
            expectTrue(monitor.inspect(veryFast, 100).isHealthy, "sustained anomaly still below threshold");
        }
        expectTrue(!monitor.inspect(veryFast, 100).isHealthy, "sustained anomaly triggers degradation");
        monitor.reset();
        expectTrue(monitor.streak() == 0, "reset clears the streak");

        const auto visual = myslam::domain::HealthMonitor::diagnose(myslam::domain::KinematicState{}, 0);
        expectTrue(!visual.isHealthy && visual.degradation == myslam::domain::Degradation::VISUAL,
                   "observation starvation is classified as VISUAL");
    }

    // ------------------------------------------------------------------
    // 相机模型：统一实现、MEI 往返一致、能力查询
    // ------------------------------------------------------------------
    {
        const auto pinhole = myslam::domain::makeCameraModel(config.camera.intrinsics);
        expectTrue(dynamic_cast<const myslam::domain::IPinholeCalibration *>(pinhole.get()) != nullptr,
                   "pinhole model exposes pinhole calibration");
        expectTrue(pinhole->name() == "pinhole", "pinhole model reports its name");

        const auto mei = myslam::domain::makeCameraModel(meiIntrinsicsFromDefaults());
        expectTrue(dynamic_cast<const myslam::domain::IPinholeCalibration *>(mei.get()) == nullptr,
                   "true MEI model does not pretend to be a pinhole calibration");
        expectTrue(mei->name() == "mei", "MEI model reports its name");

        // 往返：MEI 投影 -> 反投影 -> 再投影，误差应在亚像素（默认 xi=0，
        // 此处等价于针孔往返；真实鱼眼几何见 test_lua_config 的 shipped 配置用例）
        for (int i = 0; i < 50; ++i)
        {
            const double x = -1.0 + 0.04 * static_cast<double>(i);
            const myslam::domain::Vec3 point(x, 0.3 * x, 1.0 + 0.1 * std::abs(x));
            const auto uv = mei->project(point);
            expectTrue(uv.has_value(), "MEI projects a forward point");
            if (!uv)
            {
                continue;
            }
            const auto ray = mei->unproject(*uv);
            const auto again = ray ? mei->project(*ray) : std::nullopt;
            expectTrue(again.has_value() && (*again - *uv).norm() < 1e-3,
                       "MEI project/unproject round-trip stays sub-pixel");
        }

        const auto meiIntrinsics = meiIntrinsicsFromDefaults();
        const auto virtualPinhole = myslam::domain::makeVirtualPinholeModel(meiIntrinsics);
        expectTrue(std::abs(virtualPinhole->focalLengthPx() -
                            0.5 * (meiIntrinsics.pinholeFx() + meiIntrinsics.pinholeFy())) < 1e-9,
                   "virtual pinhole uses the scaled focal length");
        // 默认 pinholeScale = 1，折算退化为恒等：虚拟针孔焦距与 MEI 投影焦距相同。
        // 真实鱼眼下 pinholeScale < 1 会收窄视场，该性质在 test_lua_config 中验证。
        expectTrue(std::abs(virtualPinhole->focalLengthPx() - mei->focalLengthPx()) < 1e-9,
                   "默认 pinholeScale 下虚拟针孔焦距与 MEI 投影焦距一致");

        // xi=0 且无畸变时，MEI 必须与针孔在数值上等价。这是一条解析恒等式
        // 的检查：球面投影公式里的符号若写错，往返测试仍可能自洽通过
        // （project 与 unproject 用了同一处错误），但与针孔对照立刻暴露。
        {
            auto pinholeIntrinsics = meiIntrinsicsFromDefaults();
            pinholeIntrinsics.model = myslam::camera::CameraModel::PINHOLE;
            const auto pinholeZero = myslam::domain::makeCameraModel(pinholeIntrinsics);
            const auto meiZero = myslam::domain::makeCameraModel(meiIntrinsicsFromDefaults());

            double worstGap = 0.0;
            for (int i = -6; i <= 6; ++i)
            {
                for (int k = -6; k <= 6; ++k)
                {
                    const myslam::domain::Vec3 ray(0.18 * static_cast<double>(i), 0.18 * static_cast<double>(k), 1.0);
                    const auto uvPinhole = pinholeZero->project(ray);
                    const auto uvMei = meiZero->project(ray);
                    expectTrue(uvPinhole.has_value() && uvMei.has_value(), "both models project a forward ray");
                    if (!uvPinhole || !uvMei)
                    {
                        continue;
                    }
                    worstGap = std::max(worstGap, (*uvPinhole - *uvMei).norm());
                }
            }
            expectTrue(worstGap < 1e-6, "MEI with xi=0 matches the pinhole projection numerically");
        }
    }

    // ------------------------------------------------------------------
    // 分段耗时：CSV 结构与合并语义
    // ------------------------------------------------------------------
    {
        myslam::domain::StageTimings timings;
        expectTrue(timings.empty(), "fresh timings are empty");
        timings.add(myslam::domain::TimingStage::kEstimator, 1500);
        timings.add(myslam::domain::TimingStage::kEstimator, 500);
        timings.add(myslam::domain::TimingStage::kPropagation, 250);

        expectTrue(!timings.empty(), "timings become non-empty after recording");
        expectTrue(timings.stats(myslam::domain::TimingStage::kEstimator).calls == 2, "calls are counted per stage");
        expectTrue(std::abs(timings.stats(myslam::domain::TimingStage::kEstimator).meanMilliseconds() - 1.0) < 1e-9,
                   "mean milliseconds divide by call count");
        expectTrue(std::abs(timings.totalMilliseconds() - 2.25) < 1e-9, "total sums all stages");

        const std::string csv = timings.toCsv();
        expectTrue(csv.find("estimator,2.000,1.0000,2") != std::string::npos,
                   "csv row carries total, mean and call count");
        expectTrue(csv.find("initialization") == std::string::npos,
                   "stages that were never recorded are omitted from csv");

        myslam::domain::StageTimings other;
        other.add(myslam::domain::TimingStage::kEstimator, 1000);
        other.add(myslam::domain::TimingStage::kInitialization, 100);
        timings.merge(other);
        expectTrue(timings.stats(myslam::domain::TimingStage::kEstimator).calls == 3, "merge adds call counts");
        expectTrue(std::abs(timings.stats(myslam::domain::TimingStage::kInitialization).totalMilliseconds() - 0.1) <
                       1e-9,
                   "merge carries stages that the target did not have");
    }

    // ------------------------------------------------------------------
    // 注册表：按名字选择、未知名字明确失败
    // ------------------------------------------------------------------
    {
        const auto &registry = myslam::domain::EstimatorRegistry::instance();
        expectTrue(registry.contains("gtsam_fixed_lag"), "gtsam estimator is registered");
        expectTrue(registry.contains("imu_only"), "imu-only estimator is registered");

        const auto unknown = registry.create("does_not_exist", config);
        expectTrue(!unknown.has_value(), "unknown estimator name fails instead of falling back");
        expectTrue(unknown.error().find("does_not_exist") != std::string::npos,
                   "unknown-name error names the missing estimator");

        const auto bundle = registry.create("imu_only", config);
        expectTrue(bundle.has_value() && bundle->estimator != nullptr && bundle->observationModel != nullptr,
                   "registry returns a complete bundle");
    }

    // ------------------------------------------------------------------
    // 时间同步：零阶保持截断、断档、乱序
    // ------------------------------------------------------------------
    {
        myslam::domain::ImuConfig imu;
        imu.rateHz = 200.0;      // 5 ms
        imu.maxGapFactor = 10.0; // 50 ms
        myslam::domain::TimeSynchronizer sync(imu);

        expectTrue(sync.push({0.000, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()}),
                   "first sample accepted");
        expectTrue(sync.push({0.005, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()}),
                   "second sample accepted");
        expectTrue(sync.push({0.010, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()}),
                   "third sample accepted");
        expectTrue(!sync.push({0.008, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()}),
                   "out-of-order sample is rejected");
        expectTrue(sync.stats().droppedOutOfOrder == 1, "out-of-order drop is counted");

        const auto first = sync.advanceTo(0.012);
        expectTrue(!first.gapDetected, "no gap on a contiguous buffer");
        expectTrue(first.slices.size() == 3, "three samples cover the interval up to the camera time");
        expectTrue(std::abs(first.slices.back().dt - 0.002) < 1e-12,
                   "the sample crossing the camera time is truncated to the remainder");
        expectTrue(std::abs(first.integratedSeconds - 0.012) < 1e-12, "integrated duration matches the camera time");

        const auto second = sync.advanceTo(0.020);
        expectTrue(second.slices.size() == 1 && std::abs(second.slices.front().dt - 0.003) < 1e-12,
                   "the remainder of the crossing sample is consumed by the next camera frame");

        myslam::domain::TimeSynchronizer gapped(imu);
        gapped.push({0.000, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()});
        gapped.push({1.000, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()});
        const auto gap = gapped.advanceTo(1.000);
        expectTrue(gap.gapDetected, "a 1 s hole is reported as a gap");
        expectTrue(gap.gapSeconds > 0.9, "gap duration is reported");
    }

    // ------------------------------------------------------------------
    // 段重启：重置基线必须落在**对齐后**的时间域
    //
    // advanceTo() 工作在 cameraTime - timeOffsetSeconds 域。旧实现把未扣偏移的
    // cameraTime 存进基线：offset > 0 时 from > to、重启后整段时间窗的切片
    // 静默为空；offset < 0 时反向多积一段。这里用负偏移把「多积」量化出来。
    // ------------------------------------------------------------------
    {
        myslam::domain::ImuConfig imu;
        imu.rateHz = 200.0;      // 5 ms
        imu.maxGapFactor = 10.0; // 50 ms
        myslam::domain::TimeSynchronizer sync(imu);
        sync.setTimeOffset(-0.2); // aligned = cameraTime + 0.2

        sync.resetToCameraTime(1.0);
        expectTrue(std::abs(sync.integratedUntil() - 1.2) < 1e-12,
                   "resetToCameraTime stores the aligned timestamp, not the raw camera time");

        for (int i = 200; i < 400; ++i) // t = 1.000 .. 1.995
        {
            const double t = 0.005 * static_cast<double>(i);
            sync.push({t, myslam::domain::Vec3::Zero(), myslam::domain::Vec3::Zero()});
        }
        const auto afterRestart = sync.advanceTo(1.05); // target = 1.25
        expectTrue(std::abs(afterRestart.integratedSeconds - 0.05) < 1e-9,
                   "after a restart only the remaining span is integrated (not the whole offset)");
    }

    // ------------------------------------------------------------------
    // 高频外推：时间戳是积分区间的终点；非正步长不得凭空推进
    // ------------------------------------------------------------------
    {
        myslam::domain::ImuConfig imu;
        imu.rateHz = 200.0;
        myslam::domain::StatePropagator propagator(imu);
        propagator.reset(myslam::domain::KinematicState{});

        myslam::domain::ImuSample sample;
        sample.timestamp = 0.100;
        sample.accel = myslam::domain::Vec3(0.0, 0.0, imu.gravityMagnitude);
        const auto advanced = propagator.propagate(myslam::domain::ImuSlice{sample, 0.005});
        expectTrue(std::abs(advanced.timestamp - 0.105) < 1e-12,
                   "propagator timestamps the state at the end of the slice, not its start");
        expectTrue(advanced.velocity.norm() < 1e-9, "specific force cancels gravity exactly");

        const auto noProgress = propagator.propagate(myslam::domain::ImuSlice{sample, 0.0});
        expectTrue(std::abs(noProgress.timestamp - advanced.timestamp) < 1e-12,
                   "a non-positive dt does not advance the timestamp");
        expectTrue((noProgress.pose.position - advanced.pose.position).norm() < 1e-12,
                   "a non-positive dt does not invent motion");
    }

    // ------------------------------------------------------------------
    // 消融路径的倾角不确定度不能留 0
    //
    // 0 会被下游当成「倾角完全已知」，先验被拧到 1e-4 rad，估计器再没有
    // 自由度去纠回 initialTiltDeg 故意弄歪的角度。
    // ------------------------------------------------------------------
    {
        myslam::domain::InitializationConfig init;
        init.minSamples = 4;
        init.initialTiltDeg = 5.0;
        myslam::domain::Initializer initializer(init, myslam::domain::ImuConfig{});
        for (int i = 0; i < 8; ++i)
        {
            initializer.feedImu({0.005 * i, myslam::domain::Vec3(0.0, 0.0, 9.81), myslam::domain::Vec3::Zero()});
        }
        myslam::domain::Pose pose;
        myslam::domain::Bias bias;
        expectTrue(initializer.skipGravityAlignment(pose, bias), "skip-alignment path succeeds with enough samples");
        expectTrue(initializer.alignedTiltSigma() > 0.0,
                   "skip-alignment reports a non-zero tilt sigma instead of pinning the prior");
        const double injectedRad = init.initialTiltDeg * std::numbers::pi / 180.0;
        expectTrue(initializer.alignedTiltSigma() >= injectedRad - 1e-12,
                   "tilt sigma covers the deliberately injected tilt");

        initializer.resetSamples();
        expectTrue(initializer.alignedTiltSigma() == 0.0, "resetSamples clears the cached tilt sigma");
    }

    // ------------------------------------------------------------------
    // CRLF 轨迹文件：std::getline 只吃掉 '\n'，末字段会带 '\r'
    // ------------------------------------------------------------------
    {
        const auto path = std::filesystem::temp_directory_path() / "myslam_crlf_trajectory_test.tum";
        {
            std::ofstream out(path, std::ios::binary);
            out << "0.0 0 0 0 0 0 0 1\r\n"
                   "1.0 1 0 0 0 0 0 1\r\n"
                   "2.0 2 0 0 0 0 0 1\r\n";
        }
        const auto loaded = myslam::app::loadTumTrajectory(path);
        expectTrue(loaded.has_value() && loaded->size() == 3, "CRLF trajectory lines are not dropped");
        expectTrue(loaded.has_value() && !loaded->empty() && loaded->front().orientation.has_value(),
                   "CRLF trajectory keeps the optional orientation columns");
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }

    // ------------------------------------------------------------------
    // IMU 窗口压力：长时间无关键帧必须强制插入，且整段区间交给估计器
    //
    // 不能靠丢弃窗口前缀来限内存：估计器把窗口当作"自上个关键帧以来的
    // IMU"整体消费，丢前缀会让预积分时长与关键帧时间戳错位。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig pressureConfig = config;
        pressureConfig.estimator.slidingWindowLag = 0.1; // keepSeconds = 0.4 s

        auto factory =
            [](const myslam::domain::PipelineConfig &cfg) -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            return myslam::domain::EstimatorBundle{std::make_unique<myslam::adapters::ImuOnlyEstimator>(cfg),
                                                   myslam::domain::makeVirtualPinholeModel(cfg.camera.intrinsics)};
        };

        myslam::vio::VioPipeline pipeline(pressureConfig, factory);
        expectTrue(pipeline.valid(), "pressure pipeline assembles");

        myslam::domain::ImuSample sample;
        sample.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            sample.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{sample, 0.005});
        }
        // 初始化帧不带观测：之后所有帧都不带观测，决策器不会主动插关键帧
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "pressure pipeline initializes");

        // 再喂 1 s IMU，超过 0.4 s 保留时限
        for (int i = 0; i < 200; ++i)
        {
            sample.timestamp = 2.005 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{sample, 0.005});
        }
        expectTrue(pipeline.imuSamplesSinceLastKeyframe() > 100,
                   "IMU window accumulates while no keyframe is inserted");

        const auto outcome = pipeline.processFrame(2, myslam::domain::Frame{}, 3.0);
        expectTrue(outcome.kind == myslam::vio::FrameOutcome::Kind::KEYFRAME_OPTIMIZED,
                   "an overdue IMU window forces a keyframe even without observations");
        expectTrue(pipeline.imuSamplesSinceLastKeyframe() == 0,
                   "the forced keyframe consumes and clears the whole IMU window");
    }

    // ------------------------------------------------------------------
    // 关键帧判据：上一关键帧无观测时不得"永不再插关键帧"
    //
    // 视觉中断期间插进一个空观测关键帧后，若仍按"共视率 100%"处理，
    // 三条判据全部失效，视觉恢复后轨迹会停在旧基线直到 IMU 断档重建。
    // ------------------------------------------------------------------
    {
        myslam::domain::VisionConfig vision;
        myslam::domain::KeyframeSelector selector(vision);
        const auto model = myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics);
        (void)model;

        auto makeObservations = [](int count, double shift)
        {
            std::vector<myslam::domain::Observation> observations;
            for (int i = 0; i < count; ++i)
            {
                myslam::domain::Observation observation;
                observation.id = i;
                observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + shift, 100.0);
                observations.push_back(observation);
            }
            return observations;
        };

        // 空基线 + 有观测的当前帧 -> 必须成为关键帧（重建基线）
        myslam::domain::Frame currentFrame;
        currentFrame.timestamp = 1.0;
        currentFrame.observations = makeObservations(10, 0.0);
        expectTrue(selector.evaluate(currentFrame, {}, 0.5).isKeyframe,
                   "an empty previous keyframe forces a new keyframe (baseline rebuild)");

        // 空基线 + 空当前帧 -> 不必插入（没有可建立的信息）
        myslam::domain::Frame emptyFrame;
        emptyFrame.timestamp = 1.0;
        expectTrue(!selector.evaluate(emptyFrame, {}, 0.5).isKeyframe,
                   "two consecutive empty frames do not force a keyframe");
    }

    // ------------------------------------------------------------------
    // 工厂返回不完整 bundle：装配期失败，不得空解引用
    // ------------------------------------------------------------------
    {
        const auto model = myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics);
        myslam::domain::EstimatorFactory incomplete =
            [](const myslam::domain::PipelineConfig &) -> std::expected<myslam::domain::EstimatorBundle, std::string>
        { return myslam::domain::EstimatorBundle{nullptr, nullptr}; };

        const myslam::vio::VioPipeline broken(config, incomplete);
        expectTrue(!broken.valid(), "a factory returning an incomplete bundle yields an invalid pipeline");
        expectTrue(!broken.error().empty(), "the assembly error is reported instead of dereferencing null");
    }

    // ------------------------------------------------------------------
    // reset 必须复用注入的工厂，不得换回默认实现
    //
    // 契约保证的是"重建走调用方的工厂"，不是"同一个对象存活"：重建必然
    // 产生新实例。旧实现的实际缺陷是 reset 无条件 new 出默认 gtsam 后端，
    // 注入的实现被静默替换。
    // ------------------------------------------------------------------
    {
        const auto model = myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics);
        int factoryCalls = 0;
        myslam::domain::EstimatorFactory factory = [&factoryCalls, &config](const myslam::domain::PipelineConfig &)
            -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            ++factoryCalls;
            return myslam::domain::EstimatorBundle{std::make_unique<StubEstimator>(),
                                                   myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics)};
        };

        auto stub = std::make_unique<StubEstimator>();
        StubEstimator *const rawStub = stub.get();
        myslam::vio::VioPipeline pipeline(config, std::move(stub), model, factory);
        expectTrue(pipeline.estimator() == static_cast<const myslam::domain::IEstimator *>(rawStub),
                   "injected estimator is the one the pipeline uses");

        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }

        myslam::domain::Frame frame;
        expectTrue(pipeline.stepInitialization(1, frame, 2.0).tracking, "stub estimator completes initialization");

        // 注入构造只接收实例、不调用工厂；重建必须由 reset 触发
        expectTrue(factoryCalls == 0, "injected-constructor does not call the factory");
        pipeline.reset(myslam::domain::RecoveryAction::RESET_PIPELINE);
        expectTrue(factoryCalls == 1, "reset rebuilds the estimator through the injected factory");
        expectTrue(pipeline.valid() && pipeline.estimator()->name() == "stub",
                   "reset rebuilds through the injected factory, never the default backend");

        // 没有工厂时不得静默换回默认实现
        myslam::vio::VioPipeline orphan(config, std::make_unique<StubEstimator>(), model, {});
        orphan.reset(myslam::domain::RecoveryAction::RESET_PIPELINE);
        expectTrue(!orphan.valid(), "reset without a factory fails loudly rather than substituting another backend");
    }

    // ------------------------------------------------------------------
    // RESET_ESTIMATOR：就地重建，坐标系连续、观测模型不变、积分区间不丢
    // ------------------------------------------------------------------
    {
        class RecoveringEstimator final : public myslam::domain::IEstimator
        {
        public:
            [[nodiscard]] std::string_view name() const noexcept override { return "recovering"; }
            bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &,
                            const myslam::domain::PoseCovariance & = {}) override
            {
                return true;
            }
            myslam::domain::StepResult addKeyframe(const myslam::domain::Keyframe &,
                                                   std::span<const myslam::domain::ImuSlice> imu) override
            {
                // 记录收到的积分区间：就地重建后仍必须从"上一个成功关键帧"
                // 起算，否则预积分时长与关键帧时间间隔错位。
                receivedSlices = imu.size();
                if (!imu.empty())
                {
                    firstSliceTimestamp = imu.front().sample.timestamp;
                }
                if (failNext)
                {
                    failNext = false;
                    myslam::domain::StepResult failed;
                    failed.status = myslam::domain::StepStatus::FAILED;
                    failed.recovery = myslam::domain::RecoveryAction::RESET_ESTIMATOR;
                    failed.detail = "test: ask for local estimator rebuild";
                    return failed;
                }
                myslam::domain::StepResult result;
                result.status = myslam::domain::StepStatus::OK;
                result.state = state;
                return result;
            }
            [[nodiscard]] myslam::domain::KinematicState latestState() const override { return state; }
            [[nodiscard]] myslam::domain::HealthReport health() const override { return {}; }
            [[nodiscard]] myslam::domain::EstimateSnapshot snapshot() const override { return {}; }

            myslam::domain::KinematicState state{};
            bool failNext{true};
            std::size_t receivedSlices{0};
            double firstSliceTimestamp{-1.0};
        };

        const auto model = myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics);
        int factoryCalls = 0;
        std::vector<RecoveringEstimator *> created;
        myslam::domain::EstimatorFactory factory = [&factoryCalls, &created](const myslam::domain::PipelineConfig &cfg)
            -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            ++factoryCalls;
            // 触发 RESET_ESTIMATOR 的是注入的那个实例（failNext 默认 true）；
            // 工厂产出的重建实例必须正常工作，才能检查它收到的积分区间。
            auto estimator = std::make_unique<RecoveringEstimator>();
            estimator->failNext = false;
            created.push_back(estimator.get());
            return myslam::domain::EstimatorBundle{std::move(estimator),
                                                   myslam::domain::makeVirtualPinholeModel(cfg.camera.intrinsics)};
        };

        auto injected = std::make_unique<RecoveringEstimator>();
        RecoveringEstimator *const rawInjected = injected.get();
        myslam::vio::VioPipeline pipeline(config, std::move(injected), model, factory);

        // 关键帧判据要求视差或共视率触发，空帧永远到不了估计器：给一组
        // 同 ID、位移 30 px 的观测，让后续帧确定成为关键帧。
        auto makeFrame = [](double timestamp, double shift)
        {
            myslam::domain::Frame frame;
            frame.timestamp = timestamp;
            for (int i = 0; i < 10; ++i)
            {
                myslam::domain::Observation observation;
                observation.id = i;
                observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + shift, 100.0);
                frame.observations.push_back(observation);
            }
            return frame;
        };

        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);

        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, makeFrame(2.0, 0.0), 2.0).tracking, "recovering stub initializes");

        const auto modelBefore = pipeline.observationModel();

        constexpr double kIntegrationStart = 2.005;
        for (int i = 1; i <= 100; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        const auto outcome = pipeline.processFrame(2, makeFrame(2.5, 30.0), 2.5);
        expectTrue(outcome.kind == myslam::vio::FrameOutcome::Kind::KEYFRAME_FAILED &&
                       outcome.recovery == myslam::domain::RecoveryAction::RESET_ESTIMATOR,
                   "RESET_ESTIMATOR is surfaced to the orchestrator");
        expectTrue(rawInjected->receivedSlices == 100, "the failing call receives the full window");

        pipeline.reset(outcome.recovery);
        expectTrue(factoryCalls == 1, "RESET_ESTIMATOR rebuilds through the factory exactly once");
        expectTrue(pipeline.valid() && pipeline.tracking(),
                   "RESET_ESTIMATOR keeps the pipeline tracking (coordinate frame preserved)");
        expectTrue(pipeline.observationModel() == modelBefore,
                   "RESET_ESTIMATOR keeps the same observation-model instance (frontends hold it)");
        expectTrue(pipeline.resets() == 1, "RESET_ESTIMATOR is counted as a rebuild");

        // 回归：重建后不得丢掉重建前的那段 IMU。新估计器锚定在上一个成功
        // 关键帧（t=2.0）初始化，还没消费 [2.005, 2.5]；若 reset 清空窗口，
        // 下一个关键帧只会拿到 t>2.5 的尾段，而预积分因子声称覆盖
        // 「锚点 -> 新关键帧」。
        expectTrue(pipeline.imuSamplesSinceLastKeyframe() == 100,
                   "in-place rebuild preserves the unconsumed IMU window");

        for (int i = 101; i <= 200; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(created.size() == 1 && created.front() != nullptr, "the rebuilt estimator is tracked by the test");
        const auto afterRebuild = pipeline.processFrame(3, makeFrame(3.0, 60.0), 3.0);
        expectTrue(afterRebuild.producedPose(), "the rebuilt estimator produces a pose");
        if (!created.empty() && created.front() != nullptr)
        {
            expectTrue(created.front()->receivedSlices == 200,
                       "the rebuilt estimator receives the whole window, not just the post-reset tail");
            expectTrue(std::abs(created.front()->firstSliceTimestamp - kIntegrationStart) < 1e-9,
                       "the rebuilt estimator's window starts at the anchor keyframe, not at the reset moment");
        }
    }

    // ------------------------------------------------------------------
    // 连续就地重建必须有上限：否则窗口无界增长、每帧都做完整求解
    //
    // 就地重建保留窗口是正确性所需；但若重建后每帧都继续失败，保留窗口的
    // 代价会超过坐标系连续性的收益。超过上限必须退化为完整重置。
    // ------------------------------------------------------------------
    {
        class AlwaysFailingEstimator final : public myslam::domain::IEstimator
        {
        public:
            [[nodiscard]] std::string_view name() const noexcept override { return "always_failing"; }
            bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &,
                            const myslam::domain::PoseCovariance & = {}) override
            {
                return true;
            }
            myslam::domain::StepResult addKeyframe(const myslam::domain::Keyframe &,
                                                   std::span<const myslam::domain::ImuSlice>) override
            {
                myslam::domain::StepResult failed;
                failed.status = myslam::domain::StepStatus::FAILED;
                failed.recovery = myslam::domain::RecoveryAction::RESET_ESTIMATOR;
                failed.detail = "test: always ask for a local rebuild";
                return failed;
            }
            [[nodiscard]] myslam::domain::KinematicState latestState() const override
            {
                return myslam::domain::KinematicState{};
            }
            [[nodiscard]] myslam::domain::HealthReport health() const override { return {}; }
            [[nodiscard]] myslam::domain::EstimateSnapshot snapshot() const override { return {}; }
        };

        myslam::domain::EstimatorFactory factory =
            [](const myslam::domain::PipelineConfig &cfg) -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            return myslam::domain::EstimatorBundle{std::make_unique<AlwaysFailingEstimator>(),
                                                   myslam::domain::makeVirtualPinholeModel(cfg.camera.intrinsics)};
        };

        myslam::vio::VioPipeline pipeline(config, std::make_unique<AlwaysFailingEstimator>(),
                                          myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics), factory);
        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "always-failing pipeline initializes");

        auto makeFrame = [](double timestamp, double shift)
        {
            myslam::domain::Frame frame;
            frame.timestamp = timestamp;
            for (int i = 0; i < 10; ++i)
            {
                myslam::domain::Observation observation;
                observation.id = i;
                observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + shift, 100.0);
                frame.observations.push_back(observation);
            }
            return frame;
        };

        bool degraded = false;
        for (int attempt = 0; attempt < 8 && !degraded; ++attempt)
        {
            for (int i = 0; i < 20; ++i)
            {
                imu.timestamp = 2.0 + 0.005 * static_cast<double>(attempt * 20 + i + 1);
                pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
            }
            const auto outcome = pipeline.processFrame(
                2 + attempt, makeFrame(3.0 + 0.5 * attempt, 30.0 * (attempt + 1)), 3.0 + 0.5 * attempt);
            expectTrue(outcome.recovery == myslam::domain::RecoveryAction::RESET_ESTIMATOR,
                       "the stub keeps asking for a local rebuild");
            pipeline.reset(outcome.recovery);
            degraded = !pipeline.tracking();
        }

        expectTrue(degraded, "repeated local rebuilds eventually degrade to a full reset");
        expectTrue(pipeline.imuSamplesSinceLastKeyframe() == 0,
                   "the degraded path clears the IMU window instead of letting it grow without bound");
    }

    // ------------------------------------------------------------------
    // reset 路径的 bundle 也必须校验：不完整 bundle 只能失败闭合
    // ------------------------------------------------------------------
    {
        class TrivialEstimator final : public myslam::domain::IEstimator
        {
        public:
            [[nodiscard]] std::string_view name() const noexcept override { return "trivial"; }
            bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &,
                            const myslam::domain::PoseCovariance & = {}) override
            {
                return true;
            }
            myslam::domain::StepResult addKeyframe(const myslam::domain::Keyframe &,
                                                   std::span<const myslam::domain::ImuSlice>) override
            {
                myslam::domain::StepResult result;
                result.status = myslam::domain::StepStatus::FAILED;
                result.recovery = myslam::domain::RecoveryAction::RESET_ESTIMATOR;
                return result;
            }
            [[nodiscard]] myslam::domain::KinematicState latestState() const override { return {}; }
            [[nodiscard]] myslam::domain::HealthReport health() const override { return {}; }
            [[nodiscard]] myslam::domain::EstimateSnapshot snapshot() const override { return {}; }
        };

        int calls = 0;
        myslam::domain::EstimatorFactory partial = [&calls](const myslam::domain::PipelineConfig &)
            -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            ++calls;
            return myslam::domain::EstimatorBundle{std::make_unique<TrivialEstimator>(), nullptr};
        };

        myslam::vio::VioPipeline guarded(config, std::make_unique<TrivialEstimator>(),
                                         myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics), partial);
        myslam::domain::ImuSample sample;
        sample.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            sample.timestamp = 0.005 * static_cast<double>(i);
            guarded.feedImu(myslam::domain::ImuSlice{sample, 0.005});
        }
        expectTrue(guarded.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "guarded pipeline initializes");

        myslam::domain::Frame frame;
        frame.timestamp = 2.5;
        for (int i = 0; i < 10; ++i)
        {
            myslam::domain::Observation observation;
            observation.id = i;
            observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + 30.0, 100.0);
            frame.observations.push_back(observation);
        }
        const auto guardedOutcome = guarded.processFrame(2, frame, 2.5);
        guarded.reset(guardedOutcome.recovery);
        expectTrue(calls >= 1, "reset with a partial bundle still consults the factory");
        expectTrue(!guarded.valid(), "a partial bundle on the reset path fails closed instead of dereferencing null");
    }

    // ------------------------------------------------------------------
    // MEI 相机 + benchmark：仿真器必须使用估计器声明的观测模型
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig meiConfig = config;
        meiConfig.camera.intrinsics = meiIntrinsicsFromDefaults();

        myslam::vio::VioSimulator simulator(myslam::vio::makeSimulatorOptions(meiConfig),
                                            myslam::domain::makeVirtualPinholeModel(meiConfig.camera.intrinsics));
        myslam::domain::Frame frame = simulator.frameAt(0, 1.0);
        expectTrue(!frame.observations.empty(),
                   "simulator produces observations in the estimator's virtual-pinhole frame");

        myslam::domain::SystemConfig shortConfig;
        shortConfig.totalDuration = 3.0;
        myslam::vio::VioBenchmark meiBenchmark(meiConfig, shortConfig);
        const myslam::domain::RunReport meiReport = meiBenchmark.run();
        expectTrue(meiReport.initialized && meiReport.keyframeCount > 1,
                   "MEI 代码路径端到端跑通（默认标定下与针孔等价）");
    }

    // ------------------------------------------------------------------
    // 双后端 A/B：同一份数据、同一套流水线
    // ------------------------------------------------------------------
    {
        myslam::vio::VioSimulator simulator(myslam::vio::makeSimulatorOptions(config));
        myslam::domain::SystemConfig shortConfig;
        shortConfig.totalDuration = 3.0;

        myslam::vio::VioPipeline gtsamPipeline(config);
        const DirectRun gtsamRun = drivePipeline(gtsamPipeline, simulator, config, shortConfig);
        expectTrue(gtsamRun.initialized && gtsamRun.poses > 1, "gtsam backend produces a trajectory");

        myslam::domain::PipelineConfig imuOnlyConfig = config;
        imuOnlyConfig.estimator.name = "imu_only";
        myslam::vio::VioSimulator imuSimulator(myslam::vio::makeSimulatorOptions(imuOnlyConfig));
        myslam::vio::VioPipeline imuOnlyPipeline(imuOnlyConfig);
        expectTrue(imuOnlyPipeline.valid(), "switching estimator name creates the other backend");
        expectTrue(imuOnlyPipeline.estimator()->name() == "imu_only", "the selected backend is the one running");
        const DirectRun imuRun = drivePipeline(imuOnlyPipeline, imuSimulator, imuOnlyConfig, shortConfig);
        expectTrue(imuRun.initialized && imuRun.poses > 1,
                   "the same pipeline drives a non-gtsam backend without code changes");
    }

    // ------------------------------------------------------------------
    // 编排层可替换数据源
    // ------------------------------------------------------------------
    {
        myslam::domain::SystemConfig shortConfig;
        shortConfig.totalDuration = 3.0;

        auto source = std::make_unique<CountingSource>(config);
        CountingSource *const rawSource = source.get();
        myslam::vio::VioBenchmark custom(config, shortConfig, std::move(source));
        const myslam::domain::RunReport customReport = custom.run();

        expectTrue(rawSource->imuCalls > 0, "VioBenchmark drives the injected source for IMU");
        expectTrue(rawSource->cameraCalls > 0, "VioBenchmark drives the injected source for camera frames");
        expectTrue(customReport.initialized, "injected source reaches initialization");

        myslam::vio::VioSimulator simulator(myslam::vio::makeSimulatorOptions(config));
        myslam::vio::VioPipeline standalone(config);
        const DirectRun standaloneRun = drivePipeline(standalone, simulator, config, shortConfig);
        expectTrue(standaloneRun.initialized, "VioPipeline is drivable without VioBenchmark");
        expectTrue(standaloneRun.poses > 1, "standalone pipeline produces a trajectory, not just an anchor");
        expectTrue(standalone.resets() == 0, "standalone pipeline needs no rebuild on nominal data");
    }

    // ------------------------------------------------------------------
    // 无真值源与非法时序
    // ------------------------------------------------------------------
    {
        myslam::domain::SystemConfig shortConfig;
        shortConfig.totalDuration = 3.0;

        myslam::vio::VioBenchmark noTruth(config, shortConfig, std::make_unique<NoTruthSource>(config));
        const myslam::domain::RunReport reportNoTruth = noTruth.run();
        expectTrue(reportNoTruth.initialized, "a source without ground truth still runs and initializes");
        expectTrue(reportNoTruth.keyframeCount == 0, "no samples are claimed evaluable without ground truth");
        expectTrue(reportNoTruth.rmse == 0.0, "rmse stays zero when nothing was evaluated");
        expectTrue(!myslam::domain::isAcceptable(reportNoTruth, shortConfig.rmseThreshold),
                   "a run without ground truth must not be accepted on rmse alone");

        myslam::domain::SystemConfig badCamDt;
        badCamDt.totalDuration = 1.0;
        badCamDt.cameraDt = 0.0;
        myslam::vio::VioBenchmark badTimeline(config, badCamDt);
        expectTrue(!badTimeline.run().initialized, "non-positive cameraDt is rejected before the loop");

        myslam::domain::SystemConfig slowImu;
        slowImu.totalDuration = 1.0;
        slowImu.cameraDt = 0.001;
        myslam::domain::PipelineConfig slowConfig = config;
        slowConfig.imu.rateHz = 100.0; // 10 ms > 1 ms
        myslam::vio::VioBenchmark slowRun(slowConfig, slowImu);
        expectTrue(!slowRun.run().initialized, "imuDt greater than cameraDt is rejected");
    }

    // ------------------------------------------------------------------
    // χ² 短路比例：性能旋钮不得改变结果
    //
    // 短路比例决定「残差多小时的观测跳过统计复核」。压低它会让每个关键帧
    // 多做几十次贝叶斯树回代（实测单帧 35 ms → 106 ms），但如果那些计算
    // 不改变任何一个判定，就是纯粹的浪费。
    //
    // 这条断言把「省掉的计算确实是冗余的」钉成回归门禁：两种比例下关键帧
    // 数与 RMSE 必须**完全相同**，而不是「接近」。若哪天短路阈值被调低，
    // 或者门控逻辑变得依赖那部分协方差，这里会立刻失败。
    // ------------------------------------------------------------------
    {
        myslam::domain::SystemConfig shortSystem;
        shortSystem.totalDuration = 3.0;

        myslam::domain::PipelineConfig coarse = config;
        coarse.vision.chi2ShortCircuitRatio = 1.0; // 默认：只有超像素门限才复核
        myslam::vio::VioBenchmark coarseRun(coarse, shortSystem);
        const myslam::domain::RunReport coarseReport = coarseRun.run();

        myslam::domain::PipelineConfig fine = config;
        fine.vision.chi2ShortCircuitRatio = 0.25; // 旧默认：残差 2 px 就复核
        myslam::vio::VioBenchmark fineRun(fine, shortSystem);
        const myslam::domain::RunReport fineReport = fineRun.run();

        expectTrue(coarseReport.initialized && fineReport.initialized, "both short-circuit ratios initialize");
        expectTrue(coarseReport.keyframeCount == fineReport.keyframeCount,
                   "the short-circuit ratio does not change the keyframe count");
        expectTrue(std::abs(coarseReport.rmse - fineReport.rmse) < 1e-12,
                   "the short-circuit ratio does not change the trajectory (bit-identical rmse)");
        expectTrue(coarseReport.resets == fineReport.resets,
                   "the short-circuit ratio does not change the recovery behaviour");
    }

    // ------------------------------------------------------------------
    // 地图身份：reset 是「新一代坐标系」，不是「历史终结」
    //
    // 旧实现里两者是同一件事：reset 把 mapId 一起丢了，于是「重定位到
    // 旧地图」没有任何东西可以定位到。这里钉住的是：换地图后旧地图的节点
    // 仍然在位姿图里，而新地图有自己的编号与自己的锚点。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig mapConfig = config;
        mapConfig.map.poseGraphEnabled = true;
        // 每帧都优化，但 stub 默认不宣称修正：这样「版本号前进」只能来自
        // 节点/边登记，而不是 optimize() 的副作用。
        mapConfig.map.poseGraphOptimizeInterval = 1;

        auto graph = std::make_shared<StubPoseGraph>();
        myslam::vio::VioPipeline pipeline(mapConfig, {}, nullptr, graph);
        expectTrue(pipeline.valid(), "pipeline with a pose graph assembles");
        expectTrue(pipeline.mapInfo().activeMap == 0, "the first map is numbered 0");

        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, mapConfig.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }

        auto makeFrame = [](double timestamp, double shift)
        {
            myslam::domain::Frame frame;
            frame.timestamp = timestamp;
            for (int i = 0; i < 10; ++i)
            {
                myslam::domain::Observation observation;
                observation.id = i;
                observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + shift, 100.0);
                frame.observations.push_back(observation);
            }
            return frame;
        };

        expectTrue(pipeline.stepInitialization(1, makeFrame(2.0, 0.0), 2.0).tracking, "map test initializes");
        // 首帧必须成为位姿图的锚点：否则第一条边会因为 from 节点不存在而被丢弃。
        expectTrue(graph->nodes_.size() == 1, "the first keyframe is registered as a pose-graph node");
        expectTrue(!graph->nodes_.front().covariance.has_value(),
                   "the first node of a map is a gauge anchor (no covariance)");

        const auto beforeRevision = pipeline.mapInfo().revision;
        for (int i = 1; i <= 30; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        const auto posed = pipeline.processFrame(2, makeFrame(2.5, 30.0), 2.5);
        expectTrue(posed.producedPose(), "the second keyframe produces a pose");
        expectTrue(graph->nodes_.size() == 2, "the second keyframe joins the graph");
        expectTrue(graph->edges_.size() == 1, "exactly one edge connects the two keyframes");
        expectTrue(graph->nodes_.back().covariance.has_value(), "non-anchor nodes carry a covariance");
        expectTrue(graph->edges_.front().sharedObservations > 0,
                   "the shared-observation count is measured, not defaulted");
        expectTrue(pipeline.mapInfo().revision > beforeRevision, "registering keyframes advances the map revision");
        expectTrue(graph->optimizeCalls >= 1,
                   "the pipeline actually asks the pose graph to optimize on the configured interval");

        // 长期轨迹覆盖全部已登记关键帧（包括之后会被边缘化掉的）
        expectTrue(pipeline.longTermTrajectory().size() == graph->nodes_.size(),
                   "the long-term trajectory is the pose graph, not the sliding window");

        pipeline.reset(myslam::domain::RecoveryAction::RESET_PIPELINE);
        expectTrue(pipeline.mapInfo().activeMap == 1, "a full reset opens a new map identity");
        expectTrue(graph->nodes_.size() == 2, "the old map's nodes survive the reset");
        expectTrue(pipeline.mapInfo().mapCount >= 1, "the retained map is still counted");

        // 新地图重新初始化后，两个地图共存于同一张图
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 3.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(2, makeFrame(5.0, 0.0), 5.0).tracking, "the second map initializes");
        expectTrue(graph->mapCount() == 2, "two coordinate generations coexist after a reset");
        expectTrue(graph->nodes_.back().map == 1, "new nodes are tagged with the new map id");
    }

    // ------------------------------------------------------------------
    // 无位姿图时 mapCount 仍然跟随世代前进
    //
    // 旧实现用 max(自身, poseGraph_->mapCount()) 兜底：未注入位姿图时那个
    // 兜底不存在，两张地图跑完 mapCount 仍然是 1——「断过几次」这个事实
    // 就此消失。世代计数不能依赖一个可选的旁路组件。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig plainConfig = config;
        plainConfig.map.poseGraphEnabled = false;

        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = false;
        std::vector<RecoveryStubEstimator *> created;

        myslam::domain::EstimatorFactory factory = [&created](const myslam::domain::PipelineConfig &cfg)
            -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            auto rebuilt = std::make_unique<RecoveryStubEstimator>();
            rebuilt->fail = false;
            created.push_back(rebuilt.get());
            return myslam::domain::EstimatorBundle{std::move(rebuilt),
                                                   myslam::domain::makeVirtualPinholeModel(cfg.camera.intrinsics)};
        };

        myslam::vio::VioPipeline pipeline(plainConfig, std::move(estimator),
                                          myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics), factory);
        expectTrue(pipeline.poseGraph() == nullptr, "no pose graph is present in this configuration");
        expectTrue(pipeline.mapInfo().mapCount == 1, "a fresh run reports a single generation");

        pipeline.reset(myslam::domain::RecoveryAction::RESET_PIPELINE);
        expectTrue(pipeline.mapInfo().activeMap == 1, "the reset opens generation 1");
        expectTrue(pipeline.mapInfo().mapCount == 2, "generation count advances even without a pose graph");

        pipeline.reset(myslam::domain::RecoveryAction::RESET_PIPELINE);
        expectTrue(pipeline.mapInfo().mapCount == 3, "generations keep counting");
    }

    // ------------------------------------------------------------------
    // 共视数必须来自这条边的两个端点，不能借用更早的基线
    //
    // 视觉中断期间会插入「空观测关键帧」把 IMU 窗口消费掉。关键帧决策器
    // 的基线在此时刻意保留更早的一帧（否则恢复后无基线可比），若共视数
    // 也读那份基线，就会给一条「两帧之间毫无共同特征」的边一个虚高的
    // 权重——而权重正是位姿图判断该信谁的唯一依据。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig config2 = config;
        config2.map.poseGraphEnabled = true;
        config2.map.poseGraphOptimizeInterval = 0;
        config2.estimator.slidingWindowLag = 0.1; // 保留窗口 0.4 s，便于强制插帧
        config2.map.lostGraceSeconds = 0.0;

        auto graph = std::make_shared<StubPoseGraph>();
        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = false;
        myslam::vio::VioPipeline pipeline(config2, std::move(estimator),
                                          myslam::domain::makeVirtualPinholeModel(config2.camera.intrinsics), {},
                                          nullptr, graph);
        expectTrue(pipeline.valid(), "co-visibility pipeline assembles");

        auto makeFrame = [](double timestamp, double shift)
        {
            myslam::domain::Frame frame;
            frame.timestamp = timestamp;
            for (int i = 0; i < 10; ++i)
            {
                myslam::domain::Observation observation;
                observation.id = i;
                observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + shift, 100.0);
                frame.observations.push_back(observation);
            }
            return frame;
        };

        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config2.imu.gravityMagnitude);
        auto feed = [&imu, &pipeline](double from, double to)
        {
            for (double time = from; time < to; time += 0.005)
            {
                imu.timestamp = time;
                pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
            }
        };

        feed(0.0, 2.0);
        expectTrue(pipeline.stepInitialization(1, makeFrame(2.0, 0.0), 2.0).tracking,
                   "co-visibility pipeline initializes");

        // 第二条边：两帧共视 10 个特征
        feed(2.0, 2.5);
        expectTrue(pipeline.processFrame(2, makeFrame(2.5, 30.0), 2.5).producedPose(), "second keyframe is accepted");
        expectTrue(graph->edges_.size() == 1, "one edge so far");
        expectTrue(graph->edges_.back().sharedObservations == 10,
                   "co-visibility counts the actually shared feature ids");

        // 制造 IMU 窗口超时，强制插入一个**空观测**关键帧
        feed(2.5, 4.0);
        const auto forced = pipeline.processFrame(3, myslam::domain::Frame{}, 4.0);
        expectTrue(forced.kind == myslam::vio::FrameOutcome::Kind::KEYFRAME_OPTIMIZED,
                   "an overdue IMU window forces an observation-free keyframe");
        expectTrue(graph->edges_.size() == 2, "the forced keyframe still produces an edge");
        expectTrue(graph->edges_.back().sharedObservations == 0,
                   "an observation-free keyframe shares nothing with the previous one");

        // 恢复视觉：与空观测帧之间同样没有任何共视。旧实现会拿「更早那帧」
        // 的观测来算，得出 10 这个虚高值。
        feed(4.0, 4.5);
        const auto recovered = pipeline.processFrame(4, makeFrame(4.5, 60.0), 4.5);
        expectTrue(recovered.producedPose(), "a visual keyframe is accepted after the outage");
        expectTrue(graph->edges_.size() == 3, "the recovery keyframe adds an edge");
        expectTrue(graph->edges_.back().sharedObservations == 0,
                   "the recovery edge does not borrow the pre-outage baseline");
    }

    // ------------------------------------------------------------------
    // 位姿图宣称修正时，流水线的地图版本号必须前进
    //
    // 上一段证明「登记推进版本号」，这一段证明「修正也推进」——两者是不同
    // 的事件，任一缺失都会让消费者漏掉一次「历史被改写过」。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig config3 = config;
        config3.map.poseGraphEnabled = true;
        config3.map.poseGraphOptimizeInterval = 1;

        auto graph = std::make_shared<StubPoseGraph>();
        graph->reportCorrection = true;

        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = false;
        myslam::vio::VioPipeline pipeline(config3, std::move(estimator),
                                          myslam::domain::makeVirtualPinholeModel(config3.camera.intrinsics), {},
                                          nullptr, graph);
        expectTrue(pipeline.valid(), "revision pipeline assembles");

        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config3.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "revision pipeline initializes");
        const auto beforeEdge = pipeline.mapInfo().revision;

        auto frame = myslam::domain::Frame{};
        frame.timestamp = 2.5;
        for (int i = 0; i < 10; ++i)
        {
            myslam::domain::Observation observation;
            observation.id = i;
            observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + 30.0, 100.0);
            frame.observations.push_back(observation);
        }
        for (int i = 0; i < 100; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.processFrame(2, frame, 2.5).producedPose(), "the edge-producing keyframe is accepted");
        expectTrue(graph->optimizeCalls == 1, "the configured interval triggers exactly one optimization");
        // 版本号至少前进两次：登记节点/边一次，位姿图宣称修正一次。
        expectTrue(pipeline.mapInfo().revision >= beforeEdge + 2,
                   "a reported correction advances the map revision on top of registration");
    }

    // ------------------------------------------------------------------
    // 视觉退化不得触发坐标系重建
    //
    // 估计器说「我失败了，请重建」而 health 说是 VISUAL，两者直接冲突：
    // 重建对「图像是黑的」毫无作用，只会让已经在累积的轨迹反复归零。
    // ------------------------------------------------------------------
    {
        myslam::domain::HealthReport visualDegraded;
        visualDegraded.isHealthy = false;
        visualDegraded.degradation = myslam::domain::Degradation::VISUAL;
        visualDegraded.alertMessage = "test: observation starvation";

        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = true;
        estimator->recovery = myslam::domain::RecoveryAction::RESET_PIPELINE;
        estimator->report = visualDegraded;
        RecoveryStubEstimator *const raw = estimator.get();

        myslam::vio::VioPipeline pipeline(config, std::move(estimator),
                                          myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics));
        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "visual-degradation test initializes");

        // 推进到远超宽限的时刻：视觉退化仍然不重建
        for (int i = 0; i < 2000; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        auto frame = myslam::domain::Frame{};
        frame.timestamp = 12.0;
        for (int i = 0; i < 10; ++i)
        {
            myslam::domain::Observation observation;
            observation.id = i;
            observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i), 100.0);
            frame.observations.push_back(observation);
        }
        const auto outcome = pipeline.processFrame(2, frame, 12.0);
        expectTrue(raw->processCalls > 0, "the estimator was actually asked to solve");
        expectTrue(outcome.recovery == myslam::domain::RecoveryAction::NONE,
                   "visual degradation never triggers a coordinate-system rebuild");
        expectTrue(outcome.kind == myslam::vio::FrameOutcome::Kind::KEYFRAME_FAILED,
                   "the failure itself is still reported");
    }

    // ------------------------------------------------------------------
    // 丢失宽限：连续失败不足宽限时保持坐标系，超过后才重建
    //
    // 「丢一帧就重构」与「给几秒去抓回来」是两套不同的失败哲学。这里验证
    // 宽限真的是**连续**计时：中间成功一次就把计时清零。
    // ------------------------------------------------------------------
    {
        myslam::domain::PipelineConfig graceConfig = config;
        graceConfig.map.lostGraceSeconds = 1.0;

        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = true;
        estimator->recovery = myslam::domain::RecoveryAction::RESET_PIPELINE;
        estimator->report = myslam::domain::HealthReport{}; // 健康，只是求解失败
        RecoveryStubEstimator *const raw = estimator.get();

        int factoryCalls = 0;
        myslam::domain::EstimatorFactory factory = [&raw, &factoryCalls](const myslam::domain::PipelineConfig &cfg)
            -> std::expected<myslam::domain::EstimatorBundle, std::string>
        {
            ++factoryCalls;
            auto rebuilt = std::make_unique<RecoveryStubEstimator>();
            rebuilt->fail = true;
            rebuilt->recovery = myslam::domain::RecoveryAction::RESET_PIPELINE;
            return myslam::domain::EstimatorBundle{std::move(rebuilt),
                                                   myslam::domain::makeVirtualPinholeModel(cfg.camera.intrinsics)};
        };

        myslam::vio::VioPipeline pipeline(graceConfig, std::move(estimator),
                                          myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics), factory);
        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking, "grace test initializes");

        auto makeFrame = [](double timestamp, double shift)
        {
            myslam::domain::Frame frame;
            frame.timestamp = timestamp;
            for (int i = 0; i < 10; ++i)
            {
                myslam::domain::Observation observation;
                observation.id = i;
                observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i) + shift, 100.0);
                frame.observations.push_back(observation);
            }
            return frame;
        };

        auto feedImuUntil = [&imu, &pipeline](double from, double to)
        {
            for (double t = from; t < to; t += 0.005)
            {
                imu.timestamp = t;
                pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
            }
        };

        feedImuUntil(2.0, 2.4);
        const auto firstFailure = pipeline.processFrame(2, makeFrame(2.4, 30.0), 2.4);
        expectTrue(firstFailure.recovery == myslam::domain::RecoveryAction::NONE,
                   "the first failure starts the grace clock instead of rebuilding");
        expectTrue(raw->processCalls == 1, "the failure really came from the estimator");

        feedImuUntil(2.4, 3.0);
        const auto withinGrace = pipeline.processFrame(3, makeFrame(3.0, 60.0), 3.0);
        expectTrue(withinGrace.recovery == myslam::domain::RecoveryAction::NONE,
                   "failures inside the grace window keep the coordinate frame");
        expectTrue(factoryCalls == 0, "no rebuild was attempted inside the grace window");

        feedImuUntil(3.0, 3.6);
        const auto pastGrace = pipeline.processFrame(4, makeFrame(3.6, 90.0), 3.6);
        expectTrue(pastGrace.recovery == myslam::domain::RecoveryAction::RESET_PIPELINE,
                   "exceeding the grace window escalates to a coordinate-system rebuild");

        // 连续性的反例：中间成功一次之后，宽限重新计时
        auto healthy = std::make_unique<RecoveryStubEstimator>();
        healthy->fail = false;
        healthy->state = myslam::domain::KinematicState{};
        myslam::vio::VioPipeline second(graceConfig, std::move(healthy),
                                        myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics), factory);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            second.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(second.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking, "continuity test initializes");
        for (int i = 0; i < 200; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            second.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(second.processFrame(2, makeFrame(3.0, 30.0), 3.0).producedPose(), "a healthy keyframe is accepted");
    }

    // ------------------------------------------------------------------
    // RELOCALIZE 没有识别器时不得被执行，也不得静默换成重建
    //
    // 两个错误方向都不允许：真的去重定位（没有实现会崩）与降级成重建
    // （坐标系归零，正是 RELOCALIZE 想避免的）。
    // ------------------------------------------------------------------
    {
        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = true;
        estimator->recovery = myslam::domain::RecoveryAction::RELOCALIZE;
        RecoveryStubEstimator *const raw = estimator.get();

        myslam::vio::VioPipeline pipeline(config, std::move(estimator),
                                          myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics));
        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "relocalize test initializes");

        for (int i = 0; i < 2000; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        auto frame = myslam::domain::Frame{};
        frame.timestamp = 12.0;
        for (int i = 0; i < 10; ++i)
        {
            myslam::domain::Observation observation;
            observation.id = i;
            observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i), 100.0);
            frame.observations.push_back(observation);
        }
        const auto outcome = pipeline.processFrame(2, frame, 12.0);
        expectTrue(raw->processCalls > 0, "the relocalize request came from the estimator");
        expectTrue(outcome.recovery == myslam::domain::RecoveryAction::NONE,
                   "RELOCALIZE without a recognizer is not silently downgraded to a rebuild");
        expectTrue(pipeline.mapInfo().activeMap == 0,
                   "the coordinate-frame generation is untouched after an unexecutable relocalize");
    }

    // ------------------------------------------------------------------
    // 注入了识别器之后 RELOCALIZE 才可达
    //
    // 上一条用例证明「没有识别器时不会被假装执行」；这一条证明端口不是
    // 摆设：一旦注入，估计器的请求必须原样透传到编排层，由编排层去决定
    // 怎么用（当前编排层不做隐式重定位——实现不在这一层）。
    // ------------------------------------------------------------------
    {
        auto estimator = std::make_unique<RecoveryStubEstimator>();
        estimator->fail = true;
        estimator->recovery = myslam::domain::RecoveryAction::RELOCALIZE;

        auto recognizer = std::make_shared<StubPlaceRecognizer>();
        myslam::vio::VioPipeline pipeline(
            config, std::move(estimator), myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics),
            myslam::domain::EstimatorFactory{}, /*dynamicInitializer=*/nullptr, /*poseGraph=*/nullptr, recognizer);
        expectTrue(pipeline.placeRecognizer() != nullptr, "the injected recognizer is reachable from the pipeline");

        myslam::domain::ImuSample imu;
        imu.accel = myslam::domain::Vec3(0.0, 0.0, config.imu.gravityMagnitude);
        for (int i = 0; i < 400; ++i)
        {
            imu.timestamp = 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        expectTrue(pipeline.stepInitialization(1, myslam::domain::Frame{}, 2.0).tracking,
                   "recognizer test initializes");

        for (int i = 0; i < 2000; ++i)
        {
            imu.timestamp = 2.0 + 0.005 * static_cast<double>(i);
            pipeline.feedImu(myslam::domain::ImuSlice{imu, 0.005});
        }
        auto frame = myslam::domain::Frame{};
        frame.timestamp = 12.0;
        for (int i = 0; i < 10; ++i)
        {
            myslam::domain::Observation observation;
            observation.id = i;
            observation.uv = myslam::domain::Vec2(100.0 + static_cast<double>(i), 100.0);
            frame.observations.push_back(observation);
        }
        const auto outcome = pipeline.processFrame(2, frame, 12.0);
        expectTrue(outcome.recovery == myslam::domain::RecoveryAction::RELOCALIZE,
                   "with a recognizer present the request is passed through instead of swallowed");
        expectTrue(pipeline.mapInfo().activeMap == 0,
                   "the pipeline does not open a new map on its own for a relocalize request");
    }

    // ------------------------------------------------------------------
    // 生效配置摘要：所有分组必须出现
    //
    // 摘要的唯一价值是「不漏字段」。逐个分组断言比断言某个具体数值更
    // 有意义：数值会随默认值变化，而漏掉一个分组永远不应该是可接受的。
    // ------------------------------------------------------------------
    {
        const myslam::domain::SystemConfig runtime{};
        const std::string summary = myslam::common::summarizeConfig(config, runtime);
        for (const char *group : {"[相机]", "[外参]", "[IMU]", "[视觉]", "[前端]", "[门控]", "[初始化]", "[估计器]",
                                  "[健康]", "[地图]", "[运行时]"})
        {
            expectTrue(summary.find(group) != std::string::npos, "the effective-config summary covers every group");
        }
        // 地图分组必须真的反映配置，而不是写死一句说明
        expectTrue(summary.find("位姿图=on") != std::string::npos,
                   "the map group reflects the configured pose-graph switch");
        expectTrue(summary.find(config.estimator.name) != std::string::npos,
                   "the estimator group names the backend that will run");

        const auto lines = myslam::common::summarizeConfigLines(config, runtime);
        expectTrue(lines.size() == 11, "the summary is split into one line per group");

        // 回放入口没有 SystemConfig。传一个默认构造的实例会打印「时长=5s」，
        // 而回放时长由序列本身决定——那是与事实不符的数字，不是无害的占位。
        const std::string withoutRuntime = myslam::common::summarizeConfig(config, std::nullopt);
        expectTrue(withoutRuntime.find("[地图]") != std::string::npos,
                   "the map group is still reported without a runtime config");
        expectTrue(withoutRuntime.find("[运行时]") == std::string::npos,
                   "the runtime line is omitted when no runtime config exists");
    }

    myslam::flushLogging();

    std::printf("checks=%d failures=%d (keyframes=%zu rmse=%.4f max=%.4f)\n", g_checks, g_failures,
                report.keyframeCount, report.rmse, report.maxError);
    return g_failures == 0 ? 0 : 1;
}
