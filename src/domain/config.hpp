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

        /**
         * @brief 三角化的深度/基线比上限
         *
         * 视差门限只量「相差多少像素」，不量「这个差能不能定出深度」。
         * 深度远大于基线时，三角化的深度对观测噪声极敏感。
         *
         * 为什么不用 DLT 条件数：齐次坐标列的尺度差了约一个焦距倍，
         * 未做 Hartley 归一化时条件数在任何正常三角化下都很大，不是
         * 可用的判据。深度/基线比与焦距无关，且直接对应「深度可不可观」。
         *
         * 超过时**推迟**三角化（不标外点），下一帧基线更长时会自然重试。
         * <= 0 关闭。
         *
         * 默认 300 是刻意的保守值：在 5 s 与 20 s 两段仿真上扫过 40–300，
         * 中间区间（100–200）反而明显变差——短窗口内“哪些路标入图”
         * 对结果影响很大，把门限调到平均值上属于对单一序列过拟合。
         * 300 对应约 0.19° 最小张角，远低于关键帧视差门限，因此它在
         * 正常数据上几乎不动作，只在真正的退化几何（近纯旋转）上生效。
         */
        double maxDepthBaselineRatio{300.0};

        /**
         * @brief 三角化基线张角下限 (deg)
         *
         * 与条件数互补：纯旋转时两射线夹角趋近于零，深度不可观。
         */
        double minTriangulationBaselineDeg{0.5};

        /// 新路标门控保护期（观测次数）
        int landmarkGateWarmupObs{4};
        double minLandmarkDepth{0.3};
        double maxLandmarkDepth{20.0};

        /// 视觉因子低于该值视为视觉失去约束能力
        int minVisualFactorsForHealth{5};

        /**
         * @brief 是否用 χ² 统计检验代替纯像素阈值做外点判定
         *
         * 像素阈值是经验量：换分辨率就得重调，且不随估计器收敛程度变化。
         * χ² 阈值随协方差自动缩放：收敛好时门收紧，发散时门放宽。
         * 需要后端提供协方差；拿不到协方差时自动退回像素阈值。
         */
        bool chi2Gating{true};
        /// χ² 门限的松弛倍数：残差平方马氏距离超过 multiplier × 分位数才剔除
        double chi2Multiplier{5.0};
        /**
         * @brief 新观测（@c ObservationKind::NEW）的松弛倍数
         *
         * 新特征只有一次观测，还没有被任何残差检验过；用成熟的倍数去卡它
         * 会把「刚进视野、尚在收敛」的正确观测当成外点剔除。
         */
        double chi2MultiplierNew{12.0};
        /**
         * @brief 是否把路标自身协方差计入 χ² 的 S 矩阵
         *
         * 只用位姿协方差时 S 偏小、χ² 偏大：深度尚未收敛的路标会被误杀。
         * 代价是每个观测多一次 3x3 边际协方差分解，可关掉退回旧行为。
         */
        bool chi2UseLandmarkCovariance{true};
        /// χ² 分位数置信水平（单侧）
        double chi2Confidence{0.95};

        // 前端几何校验与空间均匀化
        /// 是否用基础矩阵 RANSAC 剔除几何不一致的跟踪点
        bool enableRansac{true};
        /// RANSAC 内点阈值（像素标准差倍数）：阈值 = multiplier × sigma / focal
        double ransacThresholdSigma{2.0};
        /// RANSAC 最小置信度
        double ransacConfidence{0.99};
        /// 均匀化网格列数（图像按网格分桶，每桶独立分配特征预算）
        int gridColumns{4};
        /// 均匀化网格行数
        int gridRows{3};

        /**
         * @brief 活动轨迹之间的最小间距 (px)
         *
         * 检测掩码只排除「新点与已有点重合」，管不住「两条已存在的轨迹逐渐
         * 收敛到同一片纹理」。同一物理点被两条轨迹各报一次，就是同一约束
         * 在因子图里被计两次信息，估计器会过度自信。每帧按该边长建占据栅格，
         * 同格只保留最早的一条。<= 0 关闭。
         */
        double minTrackSeparationPx{8.0};

        /**
         * @brief 是否在跟踪之前先补点
         *
         * 先补点（在上一帧）再跟踪，新点当帧就获得一次可被光流与几何校验
         * 验证的观测；后补点则让 @c ObservationKind::NEW 的点未经验证直接
         * 进入后端。保留开关以便对照实验。
         */
        bool detectBeforeTrack{true};

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

        /**
         * @brief 是否在静止检测失败时尝试动态（运动起始）初始化
         *
         * 静止初始化只覆盖「设备先静止、再出发」。从运动状态开始的序列
         * （手持、车载）会永远卡在静止检测上，只能被迫在配置里填写真值
         * 初速度。开启后用 IMU 预积分 + 视觉观测闭式求解初始状态。
         *
         * @warning **实验性，默认关闭**。机制已跑通（本质矩阵 → 三角化 →
         * PnP 统一尺度 → 线性对齐 → 世界系对齐），但只有一次闭式线性解，
         * 模型是「常量速度 + 常量重力偏差」：窗口内若有明显加速度，尺度与
         * 速度会出现数量级偏差。作为默认初始化路径之前，需要补非线性精化
         * 并在真实数据上重新标定。静止初始化只要成立就更准，因此顺序上永远
         * 先试静止、失败后才走这里。
         */
        bool dynamicEnabled{false};
        /// 触发动态初始化所需的累计旋转角下限 (deg)：太小则陀螺零偏与
        /// 重力方向不可分辨，线性系统病态
        double dynamicMinRotationDeg{10.0};
        /// 动态初始化窗口内均匀取多少个位姿做闭式解
        int dynamicPoseCount{6};
        /// 动态初始化窗口时长 (s)：窗口越长可观性越好，但延迟越大
        double dynamicWindowSeconds{1.0};
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

        /**
         * @brief 零速判定的速度上限 (m/s)
         *
         * 纯 IMU 方差判据在匀速直线运动时全部通过（加速度与角速度都为零），
         * 于是 ZUPT 会把真实运动钉死。速度估计是唯一能区分「静止」与「匀速」
         * 的状态量，必须一起判。
         */
        double zuptMaxVelocity{0.1};

        /**
         * @brief 零速判定的图像视差上限 (px)
         *
         * 视差是最后一个独立信号：IMU 与速度估计可能同时错在同一方向上，
         * 但图像在这时仍然会动。
         */
        double zuptMaxDisparity{0.5};
        /// 参与视差判据所需的最少共视特征数
        int zuptMinFeatures{20};

        /**
         * @brief ZUPT 噪声放大倍数
         *
         * 零速先验必须比「理论上应该的」更保守：一旦判定错了，它会把整段
         * 运动强行拉停。乘子在 @c zuptVelocitySigma 之上再放大一次。
         */
        double zuptNoiseMultiplier{10.0};

        /**
         * @brief 是否用初始化阶段给出的协方差覆盖固定先验 sigma
         *
         * 固定 sigma 对「静止起步」与「运动起步」用同一套数字，前者过宽、
         * 后者过窄。初始化器能给出按样本数与噪声推导的方差时，优先用它。
         */
        bool initialCovarianceFromInit{true};

        /**
         * @brief 是否在每次求解后计算并上报位姿边际协方差
         *
         * 协方差是 NEES / χ² 门控的输入。iSAM2 的 @c marginalCovariance
         * 需要对相关块做一次分解，在窗口较大时有可见开销；对延迟敏感而又
         * 不做一致性分析的部署可以关掉，此时下游自动退化为像素阈值。
         */
        bool publishCovariance{true};

        /**
         * @brief 路标参数化方式
         *
         * - @c global3d：世界系 3D 点，配一个固定强度先验；
         * - @c anchored_inverse_depth：锚定逆深度。对远点与小基线更友好，
         *   深度不确定性沿视线方向表达，不会把点硬拽到错误的距离上。
         */
        std::string landmarkRepresentation{"global3d"};

        /**
         * @brief 逆深度模式下 (u, v) 分量的先验标准差（归一化平面单位）
         *
         * 这个量不是「一个像素的角尺寸」，而是**锚点帧位姿不确定性的角度投影**：
         * 锚点不入优化，它的位姿误差会固化成路标的横向误差。取 1e-3 量级
         * （约 1/focal）会让 NEES 飙到 400（过度自信），取 1.0 会掉到 0.46
         * （过度保守）；0.09 是仿真标定出的、让 NEES 回到 3 附近的量级
         * （下文默认值与 config 目录下的各 lua 文件均为 0.09）。
         *
         * 换平台（焦距、深度范围变化）应重新标定，而不是沿用默认值。
         */
        double inverseDepthAngularSigma{0.09};

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

        /// 参与 NEES 的样本数（后端未提供协方差时为 0）
        std::size_t neesSamples{0};
        /// 位置 NEES 均值；理论期望值 = 3（三维位置自由度）
        double neesMean{0.0};
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
     * @brief 一致性验收：NEES 均值是否落在给定区间内
     *
     * 与 @c isAcceptable 分开而不合并：精度与一致性是两个独立的结论。
     * 「误差小但 NEES 巨大」说明估计器在某个错误方向上极自信，「误差小且
     * NEES 正常」才是可外推的准。没有协方差样本时返回 @c false 而不是
     * 默认通过——无法检验不等于检验通过。
     *
     * @param lower 允许的 NEES 均值下限（过度保守的检测）
     * @param upper 允许的 NEES 均值上限（过度自信的检测）
     */
    [[nodiscard]] inline bool isConsistent(const RunReport &report, double lower, double upper) noexcept
    {
        return report.neesSamples > 0 && report.neesMean >= lower && report.neesMean <= upper;
    }

    /**
     * @brief 运行时序与验收口径（与算法配置无关）
     */
    struct SystemConfig
    {
        double cameraDt{0.050};     ///< 相机采样步长 (s)，对应 20 Hz
        double totalDuration{5.0};  ///< 仿真/运行时长 (s)
        double rmseThreshold{0.20}; ///< 平移 RMSE 验收门限 (m)

        /**
         * @brief 分段耗时 CSV 的输出路径；空表示不写
         *
         * 写文件而不是只打日志：把多次运行的 CSV 拼起来才能看出「这段
         * 改动到底是变快还是变慢」，而日志里的数字无法被脚本消费。
         */
        std::string timingCsvPath{};

        /// 由 IMU 采样率派生采样步长
        [[nodiscard]] static constexpr double imuDtFrom(double imuRateHz) noexcept
        {
            return imuRateHz > 0.0 ? 1.0 / imuRateHz : 0.0;
        }
    };

} // namespace myslam::domain
