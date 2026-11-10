#pragma once

/**
 * @file types.hpp
 * @brief 领域值类型：全项目共享的数据契约，不依赖任何第三方库（Eigen 除外）。
 *
 * @par 为什么这层必须自持
 *
 * 重构前这些概念全部由 gtsam 类型承担：@c gtsam::Pose3、@c NavState、
 * @c ConstantBias、@c Values。于是「换掉 gtsam」这件事在接口层就不成立——
 * 新实现必须先引入 gtsam 才能实现 @c IVioBackend。契约类型被某个库绑定，
 * 可替换性就只是注释。
 *
 * 本文件定义领域自己的位姿、状态、观测、关键帧与估计快照。适配器负责与
 * 具体库互转（见 @c adapters/gtsam/types.hpp），领域层不认识 gtsam。
 *
 * @par 为什么允许 Eigen
 *
 * Eigen 是纯头文件的数值标量扩展，不携带求解器、线程或内存管理语义；
 * 位姿/向量的表达若绕开它只会得到一套更差的手写线性代数。约束是**只允许
 * 几何与数值类型**：Eigen 的矩阵分解、优化与稀疏模块不得出现在本层。
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace myslam::domain
{

    using Vec2 = Eigen::Vector2d;
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    /// 关键帧编号：由编排层单调分配，重建后继续递增（跨段不重号）
    using KeyframeId = int;
    /// 特征编号：由前端跨帧维护，后端只当作不透明标识
    using FeatureId = int;
    /// 路标编号：与特征编号同源，便于日志对照
    using LandmarkId = int;

    /**
     * @brief 刚体位姿（World <- Body）
     *
     * 用四元数 + 平移表达，避免把 4x4 齐次矩阵当成通用容器。四元数在
     * 优化器之外不会被反复归一化，做复合/求逆时的数值行为与 gtsam 的
     * @c Rot3 一致（后者内部也是四元数）。
     */
    struct Pose
    {
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Vec3 t = Vec3::Zero();

        [[nodiscard]] static Pose identity() noexcept { return {}; }

        [[nodiscard]] Mat3 rotation() const noexcept { return q.toRotationMatrix(); }

        /// 变换一个点：R * p + t
        [[nodiscard]] Vec3 operator*(const Vec3 &point) const noexcept { return q * point + t; }

        /// 复合：this * rhs（先 rhs 后 this）
        [[nodiscard]] Pose operator*(const Pose &rhs) const noexcept { return Pose{q * rhs.q, q * rhs.t + t}; }

        [[nodiscard]] Pose inverse() const noexcept
        {
            const Eigen::Quaterniond inv = q.conjugate();
            return Pose{inv, -(inv * t)};
        }

        /// 把世界系点变换到本机体系
        [[nodiscard]] Vec3 transformTo(const Vec3 &point) const noexcept { return inverse() * point; }

        /// 只反向旋转一个向量（不含平移）
        [[nodiscard]] Vec3 unrotate(const Vec3 &vector) const noexcept { return q.conjugate() * vector; }
    };

    /**
     * @brief IMU 零偏
     */
    struct Bias
    {
        Vec3 accel = Vec3::Zero();
        Vec3 gyro = Vec3::Zero();
    };

    /**
     * @brief 一个时刻的完整导航状态
     */
    struct KinematicState
    {
        double timestamp{0.0};
        Pose pose{};
        Vec3 velocity = Vec3::Zero();
        Bias bias{};
    };

    /**
     * @brief 一条 IMU 原始测量
     *
     * 不含积分步长：步长属于「时间同步」的产物（见 @c ImuSlice），
     * 而不是传感器读数本身的属性。
     */
    struct ImuSample
    {
        double timestamp{0.0};
        Vec3 accel = Vec3::Zero();
        Vec3 gyro = Vec3::Zero();
    };

    /**
     * @brief 带积分步长的 IMU 样本
     *
     * @c dt 是该样本实际覆盖的时间长度。回放场景下相机时刻通常落在两条
     * IMU 样本之间，跨边界样本只消费其前半段，因此 @c dt 不能由相邻时间戳
     * 相减得到，必须由时间同步器给出。
     */
    struct ImuSlice
    {
        ImuSample sample{};
        double dt{0.0};
    };

    /**
     * @brief 一帧图像的像素格式
     */
    enum class PixelFormat : std::uint8_t
    {
        GRAY8,
        BGR8,
        RGB8,
        GRAY16,
        UNKNOWN,
    };

    /**
     * @brief 只读图像视图（零拷贝）
     *
     * 领域层不拥有像素。直接法、稀疏法和学习型前端都能从同一视图取图，
     * 不必为了「后端可能需要图像」而把 cv::Mat 塞进跨层契约。
     */
    struct ImageView
    {
        const std::uint8_t *data{nullptr};
        int width{0};
        int height{0};
        int stride{0}; ///< 一行字节数（含 padding）
        PixelFormat format{PixelFormat::UNKNOWN};

        [[nodiscard]] bool valid() const noexcept
        {
            return data != nullptr && width > 0 && height > 0 && stride > 0 && format != PixelFormat::UNKNOWN;
        }
    };

    /**
     * @brief 观测来源：区分跨帧跟踪点与本帧新检出的点
     */
    enum class ObservationKind : std::uint8_t
    {
        TRACKED, ///< 由上一帧跟踪而来，具备跨帧 ID
        NEW,     ///< 本帧新检出
    };

    /**
     * @brief 一个特征观测
     *
     * @c uv 是**该相机模型的像素坐标**（由前端用同一个 @c ICameraModel 产生）。
     * 后端不再假设它一定是虚拟针孔像素：模型是什么，坐标就是什么。
     *
     * @c normalized 是可选的相机系射线（方向向量，不要求单位长度）。提供它
     * 之后，关键帧决策与几何判断可以完全脱离像素尺度；未提供时调用方退回
     * 用 @c uv。
     *
     * @c sigmaPx 允许单个观测携带自己的噪声尺度（鲁棒核、加权最小二乘、
     * 学习型前端的不确定度都能用上），缺省值表示使用全局配置。
     *
     * @c sourcePixel 是原始图像上的像素坐标（例如 MEI 鱼眼原图）。可视化
     * 需要它把点画在原始图上；以前这是回放层的旁路数组，现在随观测一起流动。
     */
    struct Observation
    {
        FeatureId id{-1};
        Vec2 uv = Vec2::Zero();
        std::optional<Vec3> normalized{};
        std::optional<Vec2> sourcePixel{};
        double sigmaPx{0.0}; ///< <= 0 表示使用全局像素噪声
        ObservationKind kind{ObservationKind::TRACKED};
    };

    /**
     * @brief 一个关键帧：后端消费的最小观测单元
     *
     * 不携带「上一关键帧」：谁需要历史观测，谁自己保存。把两视图假设从类型
     * 里拿掉之后，多视图三角化、批量优化、学习型后端都不必再改写契约。
     */
    struct Keyframe
    {
        KeyframeId id{0};
        double timestamp{0.0};
        std::vector<Observation> observations;
        ImageView image{}; ///< 可选：稀疏前端留空，直接法/学习型前端填充
    };

    /**
     * @brief 一条路标估计
     */
    struct LandmarkEstimate
    {
        LandmarkId id{0};
        Vec3 position = Vec3::Zero();
    };

    /**
     * @brief 一个关键帧的状态估计
     */
    struct KeyframeState
    {
        KeyframeId id{0};
        double timestamp{0.0};
        Pose pose{};
    };

    /**
     * @brief 估计器对外暴露的只读快照
     *
     * 用值返回而不是引用：旧接口承诺「引用在下一次调用修改状态前有效」，
     * 这条约定无法被类型检查，调用方稍有不慎就握着悬空引用。快照的拷贝
     * 成本只在导出点云/轨迹时发生，与每帧求解相比可以忽略。
     */
    struct EstimateSnapshot
    {
        std::vector<KeyframeState> keyframes;
        std::vector<LandmarkEstimate> landmarks;
    };

    /**
     * @brief 仿真真值（只有带解析真值的来源才提供）
     */
    struct GroundTruth
    {
        KinematicState state{};
        Vec3 accel = Vec3::Zero(); ///< 世界系线加速度 (m/s^2)
        Vec3 gyro = Vec3::Zero();  ///< 机体系角速度 (rad/s)
    };

} // namespace myslam::domain
