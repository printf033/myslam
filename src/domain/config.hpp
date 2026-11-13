#pragma once

/**
 * @file config.hpp
 * @brief 分组后的流水线配置：每个组件只读自己那一组。
 *
 * @par 为什么拆分组
 *
 * 重构前 @c VioConfig 是一个约 40 字段的扁平结构，装配层逐字段手抄进五个
 * 不同的 Options 结构。新增任何参数都要改三处（Lua 解析、配置结构、装配），
 * 而读某个组件时根本看不出它真正依赖哪几个字段。
 *
 * 现在按「谁消费」分组：
 *   - @c CameraConfig    相机内参与外参；
 *   - @c ImuConfig       IMU 噪声、采样率、重力；
 *   - @c VisionConfig    像素噪声、门控、关键帧判据、特征数；
 *   - @c InitializationConfig 静止检测与重力对齐；
 *   - @c EstimatorConfig 估计器选择及其窗口/先验参数；
 *   - @c HealthConfig    健康判定阈值。
 *
 * 换一个估计器时，只有 @c EstimatorConfig::name 与它自己的参数需要变，
 * 其余分组保持原样。
 *
 * @par 为什么保留 Lua 的扁平字段名
 *
 * 配置文件是外部契约：已有序列的 @c config.lua 不应因为内部重构而失效。
 * 分组发生在 C++ 侧，Lua 解析器仍按原有键名写入对应分组。
 *
 * 约束：本文件只依赖领域类型与标准库。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/types.hpp"
#include <cmath>
#include <cstddef>
#include <string>

namespace myslam::domain
{

    /**
     * @brief 默认相机-IMU 外参（工业安装：前向 +X_b 对应光轴 +Z_c）
     */
    [[nodiscard]] inline Pose defaultBodyPSensor() noexcept
    {
        Mat3 rotation;
        rotation << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
        return Pose{Eigen::Quaterniond(rotation), Vec3(0.10, 0.0, 0.05)};
    }

    /**
     * @brief 相机与安装外参
     */
    struct CameraConfig
    {
        camera::CameraIntrinsics intrinsics{};

        /// 相机相对 IMU 的外参 tBc (Camera in Body frame)
        Pose bodyPSensor = defaultBodyPSensor();

        /// 相机-IMU 时间偏移 (s)：正值表示相机时间戳比 IMU 晚
        double timeOffsetSeconds{0.0};

        /// 曝光时长 (s)：图像内容对应曝光中点（触发时刻 + exposure/2）。
        /// 数据集通常只记录触发时刻，不做中点修正会让视觉观测与 IMU
        /// 积分区间产生半个曝光的系统性时移；无曝光元数据时保持 0。
        double exposureSeconds{0.0};
    };

    /**
     * @brief IMU 特性与重力
     */
    struct ImuConfig
    {
        double rateHz{200.0};

        /// 连续时间噪声谱密度：离散样本标准差 = sigma / sqrt(dt)
        double accelNoiseSigma{0.0057}; ///< m/s^2 / sqrt(Hz)
        double gyroNoiseSigma{3.6e-4};  ///< rad/s / sqrt(Hz)
        double accelBiasRw{0.01};       ///< m/s^3 / sqrt(Hz)，Allan 实测上界 ~0.12 的保守折扣（手持 MEMS）
        double gyroBiasRw{0.002};       ///< rad/s^2 / sqrt(Hz)，Allan 实测上界 ~0.012 的保守折扣（手持 MEMS）
        double integrationNoise{1e-8};

        double gravityMagnitude{9.81}; ///< m/s^2

        /// 相邻样本间隔超过该倍数的标称周期即视为时间戳断档
        double maxGapFactor{10.0};
    };

    /**
     * @brief 视觉前端与门控参数
     */
    struct VisionConfig
    {
        double pixelNoiseSigma{1.0};
        double huberLossDelta{1.345};

        /// 优化后外点软门限：连续超限帧数达到 outlierStreakThreshold 才标记
        double maxReprojectionError{8.0};
        /// 优化后外点硬门限：单帧残差超过即标记
        double maxReprojectionErrorHard{15.0};
        /// 预优化宽门限：只有预积分预测位姿可用，仅拦截灾难性误匹配
        double maxReprojectionErrorPreopt{50.0};
        /// 软门限连续超限多少帧后标记为外点（防单帧抖动误杀）
        int outlierStreakThreshold{3};
        bool postOptOutlierMarking{true};
        bool preoptTemporaryRejection{true};

        // 关键帧判据
        double minParallaxPx{20.0};
        double minTrackingRatio{0.65};
        double minKeyframeInterval{0.15};
        double maxKeyframeInterval{0.45};
        double minTriangulationParallaxPx{3.0};

        /// 新路标门控保护期（观测次数）
        int landmarkGateWarmupObs{4};
        double minLandmarkDepth{0.3};
        double maxLandmarkDepth{20.0};

        /// 视觉因子低于该值视为视觉失去约束能力
        int minVisualFactorsForHealth{5};

        /// 前端每帧最少保留的特征数。回放的**上限**来自
        /// @c DatasetRunConfig::maxFeatures（数据集配置），这里只表达下限，
        /// 避免同一个量出现在两处以不同默认值。
        int minFeatures{90};
    };

    /**
     * @brief 初始化（静止检测与重力对齐）
     */
    struct InitializationConfig
    {
        bool gravityAlignment{true};

        /// 消融：人为把初始姿态弄歪的角度 (deg)，仅在关闭重力对齐时生效
        double initialTiltDeg{0.0};
        double initialTiltAzimuthDeg{0.0};

        int minSamples{40};
        double windowSeconds{2.0};

        /// 超过该时刻仍未通过静止检测则放宽为「非静止对齐」；<= 0 表示不超时
        double timeoutSeconds{2.0};

        /// 世界系初始高度 (m)：自由平移量，不影响相对精度
        double initialHeightMeters{1.0};

        /// 初始速度（世界上系，m/s）。默认零，适用于从静止开始的序列；
        /// 从运动开始的序列（如 TUM RGB-D 手持数据）必须给出真值速度，
        /// 否则预积分预测会系统性偏离视觉观测，零偏估计被迫代偿而发散。
        Vec3 initialVelocity = Vec3::Zero();

        /// 首关键帧退化门控：有效观测数下限。低于此值说明特征太少，
        /// 初始化后视觉约束不足，保持 GRAVITY_ALIGNED 等下一帧。
        int minInitObservations{15};

        /// 首关键帧退化门控：观测射线最大夹角下限 (deg)。所有点都挤在图像
        /// 中心附近时三角化病态（深度不可观），同样延迟初始化。
        double minInitSpreadDeg{3.0};
    };

    /**
     * @brief 估计器选择与它自己的参数
     *
     * @c name 是注册表键。未知名字在装配期报错，而不是静默回退到某个默认
     * 实现——静默回退正是旧代码里「注入的后端被换掉」的同类错误。
     */
    struct EstimatorConfig
    {
        std::string name{"gtsam_fixed_lag"};

        /// 固定滞后窗口时长 (s)：窗口的权威边界，必须不小于 maxKeyframeInterval
        double slidingWindowLag{1.6};
        /// 期望窗口关键帧数上限（仅诊断告警用）
        std::size_t maxSlidingWindowSize{8};
        int extraSmootherIterations{1};

        // 零速更新
        double zuptWindowSeconds{0.5};
        double zuptVelocitySigma{0.02};

        // 先验强度
        double initialPoseSigma{0.05};
        double initialVelSigma{0.1};
        double initialBiasSigma{0.05};
        double landmarkPriorSigma{1.0};
        double recoveryPoseSigma{0.1};
        double recoveryVelSigma{0.2};
        double recoveryBiasSigma{0.1};
    };

    /**
     * @brief 健康判定阈值
     */
    struct HealthConfig
    {
        double maxAccelBiasNorm{1.0};
        double maxGyroBiasNorm{0.15};
        double maxSpeed{20.0};
        int minActiveFeatures{5};
        int alertStreakThreshold{3};
    };

    /**
     * @brief 一次运行的完整算法配置
     */
    struct PipelineConfig
    {
        CameraConfig camera{};
        ImuConfig imu{};
        VisionConfig vision{};
        InitializationConfig initialization{};
        EstimatorConfig estimator{};
        HealthConfig health{};

        /// 由重力方向的消融初始姿态
        [[nodiscard]] Pose ablationInitialPose() const noexcept
        {
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            if (initialization.initialTiltDeg == 0.0)
            {
                return Pose{};
            }
            const double azimuth = initialization.initialTiltAzimuthDeg * kDegToRad;
            const double tilt = initialization.initialTiltDeg * kDegToRad;
            const Vec3 axis(std::cos(azimuth), std::sin(azimuth), 0.0);
            return Pose{Eigen::Quaterniond(Eigen::AngleAxisd(tilt, axis.normalized())), Vec3::Zero()};
        }
    };

    /**
     * @brief 一次完整运行的精度报告
     */
    struct RunReport
    {
        std::size_t keyframeCount{0};
        double rmse{0.0};
        double maxError{0.0};
        /// 是否真正完成过初始化并产出关键帧；false 时 rmse 无意义
        bool initialized{false};
        std::size_t resets{0};
    };

    /**
     * @brief 验收判定：已完成初始化、至少两个样本、RMSE 在门限内
     *
     * 只看 rmse 会把「零关键帧」误判为「完美精度」；只要求 > 0 个样本会把
     * 「初始化锚点」当成轨迹。三个条件缺一不可。
     */
    [[nodiscard]] inline bool isAcceptable(const RunReport &report, double rmseThreshold) noexcept
    {
        constexpr std::size_t kMinEvaluableSamples = 2;
        return report.initialized && report.keyframeCount >= kMinEvaluableSamples && report.rmse < rmseThreshold;
    }

    /**
     * @brief 运行时序与验收口径（与算法配置无关）
     */
    struct SystemConfig
    {
        double cameraDt{0.050};     ///< 相机采样步长 (s)，对应 20 Hz
        double totalDuration{5.0};  ///< 仿真/运行时长 (s)
        double rmseThreshold{0.20}; ///< 平移 RMSE 验收门限 (m)

        /// 由 IMU 采样率派生采样步长
        [[nodiscard]] static constexpr double imuDtFrom(double imuRateHz) noexcept
        {
            return imuRateHz > 0.0 ? 1.0 / imuRateHz : 0.0;
        }
    };

} // namespace myslam::domain
