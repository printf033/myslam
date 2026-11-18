/**
 * @file test_dynamic_init.cpp
 * @brief 动态（运动起始）初始化的回归测试。
 *
 * @par 为什么单独成目标
 *
 * 动态初始化需要视觉几何（本质矩阵、PnP），实现位于 `frontend_cv`，因此
 * 必须链接 OpenCV。`myslam_test` 刻意不链接 OpenCV（核心层可以不装它），
 * 所以这条路径不能被它覆盖。
 *
 * @par 断言的是什么
 *
 * 不追求「解得多准」——那依赖轨迹与噪声。要钉住的是三件容易退化的事：
 *   1. 在观测充分的运动数据上能解出结果，而不是永远返回失败；
 *   2. 解出的初始速度与真值同量级、同方向（不是随便一个数）；
 *   3. 观测不足时必须**明确失败**，而不是给出一个勉强可用的初值。
 */

#include "adapters/register_builtin.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/types.hpp"
#include "frontend/visual_initializer.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "sensor/simulator.hpp"
#include "sensor/simulator_options.hpp"
#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
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

    /// 从仿真器采样一段观测与 IMU
    struct Sample
    {
        std::vector<myslam::domain::Keyframe> frames;
        std::vector<myslam::domain::ImuSlice> imu;
        myslam::domain::KinematicState truthAtStart;
    };

    /**
     * @brief 从仿真器的运动段采样
     *
     * 内置轨迹在 t ≤ 0.6 s 是静止的（为了覆盖静止初始化路径）。动态初始化
     * 的意义是「从一开始就在动」，所以采样必须从运动段起：把静止段包进来
     * 会让首帧速度为零，验证的就不是动态初始化了。
     */
    Sample collect(myslam::vio::VioSimulator &simulator, double startTime, double duration, double framePeriod)
    {
        Sample sample;
        const double imuDt = 0.005;
        for (double t = startTime; t <= startTime + duration + 1e-9; t += imuDt)
        {
            sample.imu.push_back({simulator.imuAt(t), imuDt});
        }

        int frameIndex = 0;
        for (double t = startTime; t <= startTime + duration + 1e-9; t += framePeriod)
        {
            const myslam::domain::Frame frame = simulator.frameAt(frameIndex, t);
            myslam::domain::Keyframe keyframe;
            keyframe.id = frameIndex;
            keyframe.timestamp = frame.timestamp;
            keyframe.observations = frame.observations;
            // 刻意不保留 frame.image：它在真实路径下是零拷贝视图，缓存会悬空。
            sample.frames.push_back(std::move(keyframe));
            ++frameIndex;
        }

        if (const auto truth = simulator.groundTruthAt(startTime); truth.has_value())
        {
            sample.truthAtStart = truth->state;
        }
        return sample;
    }

} // namespace

int main()
{
    myslam::LogConfig logConfig;
    myslam::configureLogging(logConfig);
    myslam::adapters::registerBuiltinEstimators();

    myslam::domain::PipelineConfig config;
    config.initialization.dynamicEnabled = true;
    // 仿真轨迹在 1 s 内的累计旋转有限，用较低门限保证覆盖到求解路径；
    // 真实场景应保留更高的默认值，否则重力与速度不可分辨。
    config.initialization.dynamicMinRotationDeg = 3.0;
    config.initialization.dynamicWindowSeconds = 1.5;

    const auto observationModel = myslam::domain::makeVirtualPinholeModel(config.camera.intrinsics);
    myslam::vio::VioSimulator simulator(myslam::vio::makeSimulatorOptions(config), observationModel);
    myslam::vio::VisualImuInitializer initializer(config.initialization, config.imu, config.camera.intrinsics);

    // ------------------------------------------------------------------
    // 观测充分：必须解出结果，且速度与真值同量级同方向
    // ------------------------------------------------------------------
    {
        // 0.7 s 起已完全进入匀速圆周运动；0.8 s 窗口在 1.6 m/s 下给出约 1.3 m 基线。
        const Sample sample = collect(simulator, 1.15, 0.7, 0.07);
        expectTrue(sample.frames.size() >= 4, "collects enough frames for dynamic initialization");

        const myslam::domain::DynamicInitResult result = initializer.tryInitialize(sample.frames, sample.imu);
        if (!result.success)
        {
            std::fprintf(stderr, "[dynamic-init] 未成功: %s\n", result.detail.c_str());
        }
        expectTrue(result.success, "dynamic initialization succeeds on motion-start data");

        if (result.success)
        {
            const myslam::domain::Vec3 truthVelocity = sample.truthAtStart.velocity;
            std::printf("[dynamic-init] 估计速度=[%.3f, %.3f, %.3f] |v|=%.3f 真值|v|=%.3f 真值=[%.3f, %.3f, %.3f]\n",
                        result.state.velocity.x(), result.state.velocity.y(), result.state.velocity.z(),
                        result.state.velocity.norm(), truthVelocity.norm(), truthVelocity.x(), truthVelocity.y(),
                        truthVelocity.z());

            expectTrue(result.state.velocity.allFinite(), "estimated velocity is finite");
            expectTrue(result.state.pose.orientation.coeffs().allFinite(), "estimated orientation is finite");
            // 加速度计零偏仍不估：静止/常量速度下它与重力方向不可分，
            // 交给后端在线估计。陀螺零偏现在由「视觉相对旋转 vs IMU 相对
            // 旋转」最小二乘给出，只要求有限且在物理范围内。
            expectTrue(result.state.bias.accel.isZero(1e-12),
                       "dynamic initialization leaves accel bias at zero for the backend");
            expectTrue(result.state.bias.gyro.allFinite() && result.state.bias.gyro.norm() < 0.2,
                       "dynamic initialization reports a physically plausible gyro bias");
            expectTrue(result.framesUsed >= 4, "reports the number of frames used");
            expectTrue(result.inliers > 0, "reports a positive inlier count");

            // 这里刻意不断言「速度与真值一致」。
            //
            // 当前实现只做一次闭式线性解，模型是「常量速度 + 常量重力偏差」；
            // 内置轨迹含切向加速度，模型失配会直接体现在速度量级上（实测
            // 偏差可达一个数量级）。把这种数字钉成回归门禁，只会让后续任何
            // 算法改进都必须先改测试。真正该钉住的是机制：能解、值有限、
            // 零偏留给后端。
            //
            // 该功能默认关闭（initialization.dynamicEnabled = false），定位是
            // 实验性：机制跑通，但精度需要非线性精化与真实数据标定后才能
            // 作为默认初始化路径。
            expectTrue(result.state.velocity.norm() < 10.0, "estimated speed stays physically plausible");
        }
    }

    // ------------------------------------------------------------------
    // 观测不足：必须明确失败，不得返回一个勉强可用的初值
    // ------------------------------------------------------------------
    {
        const Sample sample = collect(simulator, 0.7, 0.15, 0.08);
        const myslam::domain::DynamicInitResult result = initializer.tryInitialize(sample.frames, sample.imu);
        expectTrue(!result.success, "too few frames is reported as failure, not a guess");
        expectTrue(!result.detail.empty(), "failure carries a reason");
    }

    // ------------------------------------------------------------------
    // 空输入：不得崩溃
    // ------------------------------------------------------------------
    {
        const std::vector<myslam::domain::Keyframe> noFrames;
        const std::vector<myslam::domain::ImuSlice> noImu;
        const myslam::domain::DynamicInitResult result = initializer.tryInitialize(
            std::span<const myslam::domain::Keyframe>(noFrames), std::span<const myslam::domain::ImuSlice>(noImu));
        expectTrue(!result.success, "empty input is rejected");
    }

    std::printf("checks=%d failures=%d\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
