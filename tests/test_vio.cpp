/**
 * @file test_vio.cpp
 * @brief VIO 核心回归测试：契约自持、可替换性、恢复语义与时间同步。
 *
 * 测试只用领域层与公开接口，不包含 gtsam 头：如果某条断言需要 gtsam 类型
 * 才能表达，说明契约又漏回了实现细节。
 */

#include "adapters/register_builtin.hpp"
#include "adapters/simple/imu_only_estimator.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/health_monitor.hpp"
#include "domain/ports.hpp"
#include "domain/registry.hpp"
#include "domain/time_synchronizer.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "vio.hpp"
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
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

        bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &) override
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

        const auto mei = myslam::domain::makeCameraModel(myslam::camera::CameraIntrinsics::subtHandheld1Mei());
        expectTrue(dynamic_cast<const myslam::domain::IPinholeCalibration *>(mei.get()) == nullptr,
                   "true MEI model does not pretend to be a pinhole calibration");
        expectTrue(mei->name() == "mei", "MEI model reports its name");

        // 往返：MEI 投影 -> 反投影 -> 再投影，误差应在亚像素
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

        const auto meiIntrinsics = myslam::camera::CameraIntrinsics::subtHandheld1Mei();
        const auto virtualPinhole = myslam::domain::makeVirtualPinholeModel(meiIntrinsics);
        expectTrue(std::abs(virtualPinhole->focalLengthPx() -
                            0.5 * (meiIntrinsics.pinholeFx() + meiIntrinsics.pinholeFy())) < 1e-9,
                   "virtual pinhole uses the scaled focal length");
        expectTrue(virtualPinhole->focalLengthPx() < mei->focalLengthPx(),
                   "MEI virtual pinhole focal is narrower than the projection focal");
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
            bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &) override
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
            bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &) override
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
            bool initialize(const myslam::domain::Keyframe &, const myslam::domain::KinematicState &) override
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
        meiConfig.camera.intrinsics = myslam::camera::CameraIntrinsics::subtHandheld1Mei();

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
                   "MEI intrinsics still produce a trajectory (models agree end to end)");
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

    myslam::flushLogging();

    std::printf("checks=%d failures=%d (keyframes=%zu rmse=%.4f max=%.4f)\n", g_checks, g_failures,
                report.keyframeCount, report.rmse, report.maxError);
    return g_failures == 0 ? 0 : 1;
}
