#pragma once

/**
 * @file types.hpp
 * @brief 传感器数据结构（IMU 测量、相机帧、特征观测）与系统配置 `VioConfig`。
 */

#include <cstddef>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <vector>

namespace myslam::vio
{

    // =========================================================================
    // 1. 传感器数据结构定义
    // =========================================================================

    /**
     * @brief 高频 IMU 原始测量帧
     */
    struct ImuMeasurement
    {
        double timestamp{0.0};         // 采样时间戳 (s)
        gtsam::Vector3 accel{0, 0, 0}; // 加速度计读数包含比力 (m/s^2)
        gtsam::Vector3 gyro{0, 0, 0};  // 陀螺仪角速度 (rad/s)
    };

    /**
     * @brief 视觉特征点观测
     */
    struct FeatureObservation
    {
        int featureId{-1};          // 全局唯一的特征点 ID
        gtsam::Point2 uv{0, 0};     // 像素坐标 (px)
        gtsam::Point2 uvNorm{0, 0}; // 归一化相机平面坐标 (x, y)
    };

    /**
     * @brief 图像特征帧
     */
    struct CameraFrame
    {
        int frameId{0};                           // 图像帧编号
        double timestamp{0.0};                    // 曝光时间戳 (s)
        std::vector<FeatureObservation> features; // 提取并跟踪的特征点集合
    };

    // =========================================================================
    // 2. 系统核心参数配置
    // =========================================================================

    struct VioConfig
    {
        // 相机内参 (Pinhole Camera Intrinsics)
        double fx{460.0};
        double fy{460.0};
        double cx{320.0};
        double cy{240.0};
        double imageWidth{640.0};
        double imageHeight{480.0};

        // 相机相对 IMU 的外参刚体变换 tBc (Camera in Body frame)
        // 工业标准定义: 机身前向 (+X_b) 对应相机光轴 (+Z_c), 机身右向 (-Y_b) 对应相机横轴 (+X_c), 机身下向 (-Z_b)
        // 对应相机纵轴 (+Y_c)
        gtsam::Pose3 bodyPSensor{gtsam::Rot3(0, 0, 1, -1, 0, 0, 0, -1, 0), gtsam::Point3(0.10, 0.0, 0.05)};

        // IMU 噪声特性 (连续时间功率谱密度与随机游走)
        // 连续时间噪声谱密度：200 Hz 采样下的样本标准差 = sigma / sqrt(dt)
        double accelNoiseSigma{0.0057}; // 加速度计白噪声 (m/s^2 / sqrt(Hz))
        double gyroNoiseSigma{3.6e-4};  // 陀螺仪白噪声 (rad/s / sqrt(Hz))
        double accelBiasRw{3e-4};       // 加速度计零偏随机游走 (m/s^3 / sqrt(Hz))
        double gyroBiasRw{3e-5};        // 陀螺仪零偏随机游走 (rad/s^2 / sqrt(Hz))
        double integrationNoise{1e-8};  // 数值积分不确定度
        /// IMU 采样率 (Hz)：连续时间噪声谱密度 sigma (单位 1/√Hz) 在离散采样下的
        /// 样本标准差为 sigma/√dt。仿真器按此换算，保证仿真噪声与因子图噪声模型一致。
        double imuRateHz{200.0};

        // 重力常数
        double gravityMagnitude{9.81}; // 重力加速度模长 (m/s^2)

        // 视觉观测噪声与鲁棒核
        double pixelNoiseSigma{1.0};      // 特征点重投影高斯噪声 (像素)
        double huberLossDelta{1.345};     // Huber 损失阈值
        double maxReprojectionError{8.0}; ///< 优化后外点软门限 (像素)：持续超限才标记路标为外点
        /// 优化后外点硬门限 (像素)：单帧残差超过即标记为外点。工业依据：优化后位姿
        /// 与路标均已收敛，真观测残差通常 < 3px，单帧硬门限几乎不会误杀。
        double maxReprojectionErrorHard{15.0};
        /// 预优化预门控宽门限 (像素)：优化前只有预积分预测位姿可用，早期零偏未收敛时
        /// 预测误差可达 20+px，因此这里仅拦截灾难性误匹配，精确门控留到优化之后。
        double maxReprojectionErrorPreopt{50.0};
        bool postOptOutlierMarking{true};    ///< 诊断开关：优化后是否永久标记外点
        bool preoptTemporaryRejection{true}; ///< 诊断开关：预优化是否临时拒绝超限观测

        // 关键帧选取策略参数
        double minParallaxPx{20.0};       // 触发关键帧的平均视差阈值 (像素)
        double minTrackingRatio{0.65};    // 触发关键帧的特征留存率下限
        double minKeyframeInterval{0.15}; // 两次关键帧的最小时间间隔 (保证立体视差基线充分)
        double maxKeyframeInterval{0.45}; // 强制插入关键帧的最大时间间隔 (s)
        /// 三角化所需最小视差 (像素)。低于此值的新关键帧无法产生有效深度，
        /// 因此超时保底判据也要求至少达到该视差 (静止/悬停时不空转插入关键帧)。
        double minTriangulationParallaxPx{3.0};
        /// 新路标门控保护期 (观测次数)：三角化初期位置未收敛，残差大属正常，
        /// 先让它参与优化若干轮再接受外点检验，避免误杀真实特征。
        int landmarkGateWarmupObs{4};
        /// 每个关键帧的额外平滑迭代次数：iSAM2 单次 update 只做一轮增量线性化，
        /// 新增视觉因子后视觉残差尚未收敛；工业 VIO 每关键帧多轮迭代 (VINS 为 8 轮)。
        int extraSmootherIterations{1};

        // 滑动窗口大小
        size_t maxSlidingWindowSize{8}; ///< 滑动窗口内最大关键帧数 (日志/位姿统计)
        /// 固定滞后平滑窗口时长 (秒)：工业固定滞后平滑的滑动窗口以时间定义，
        /// 覆盖 maxSlidingWindowSize 个关键帧 (关键帧间隔约 0.2s)
        double slidingWindowLag{1.6};
    };

} // namespace myslam::vio
