#pragma once

/**
 * @file ports.hpp
 * @brief 端口定义：估计器、观测提取器、状态外推器、传感器查询源。
 *
 * @par 依赖方向
 *
 * 编排层只认识本文件里的接口与 @c domain 值类型；具体实现（gtsam 固定滞后、
 * IMU 航位推算、OpenCV 光流）位于 @c adapters/ 与 @c frontend/，单向依赖
 * 领域层。任何适配器都不被领域层反向引用。
 *
 * @par 为什么错误用返回值表达
 *
 * 项目约束是不引入跨层异常（日志与编排层没有上层调用者可以兜住异常）。
 * 端口因此把失败编码为 @c StepResult / @c bool，并要求实现内部截住第三方
 * 库抛出的异常、转成返回值与诊断文本。
 */

#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace myslam::domain
{

    /**
     * @brief 一帧图像观测（前端输出，尚未被判定为关键帧）
     */
    struct Frame
    {
        int index{0}; ///< 图像帧号（由数据源分配）
        double timestamp{0.0};
        std::vector<Observation> observations;
        ImageView image{};
    };

    /**
     * @brief 单步估计的完整结果
     */
    struct StepResult
    {
        StepStatus status{StepStatus::FAILED};
        RecoveryAction recovery{RecoveryAction::NONE};

        /// @c status == OK 时有效
        std::optional<KinematicState> state;

        HealthReport health{};
        /// 失败/跳过原因，供日志使用
        std::string detail;
    };

    /**
     * @brief 估计器端口：只谈数据与命令，不谈任何第三方类型
     *
     * 契约要点：
     *   - @c initialize 注入首帧状态（位姿、速度、零偏）；
     *   - @c addKeyframe 接收关键帧**以及它覆盖的 IMU 区间**。IMU 缓冲与
     *     时间同步归属编排层，估计器自行决定是预积分、批量积分还是学习型
     *     积分；「必须逐样本流式喂入」不再是被强加的假设；
     *   - 失败必须由实现内部转成 @c StepResult（不抛异常），并给出恢复建议；
     *   - 所有查询方法返回**值**，不承诺引用的生命周期。
     */
    class IEstimator
    {
    public:
        virtual ~IEstimator() = default;

        /// 注册名（与 @c EstimatorConfig::name 对应）
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;

        /**
         * @brief 注入首个关键帧并建立内部状态
         *
         * 首个关键帧的**观测**必须随先验一起传入：它是下一次三角化的基线。
         * 只传位姿会让估计器丢掉这条基线，第一个关键帧无法建立任何路标，
         * 精度在起步阶段就出现可见损失（这是把接口从「四个标量」收窄为
         * 「一个关键帧」的直接原因）。
         *
         * @param firstKeyframe 首个关键帧（编号与观测）
         * @param initialState 对齐后的初始位姿、速度与零偏
         * @return false 表示本帧注入失败（实现已截住异常）。流水线会保持
         *         `GRAVITY_ALIGNED` 并在下一帧重试；若持续失败，它始终进不了
         *         `TRACKING_OK`，由编排层按"未完成初始化"上报，而不是带着
         *         空估计器继续跑。
         */
        virtual bool initialize(const Keyframe &firstKeyframe, const KinematicState &initialState) = 0;

        /**
         * @brief 处理一个关键帧
         *
         * @param keyframe 关键帧观测（ID 由编排层单调分配）
         * @param imu 自上个关键帧以来、按时间截断的 IMU 区间
         */
        virtual StepResult addKeyframe(const Keyframe &keyframe, std::span<const ImuSlice> imu) = 0;

        /// 最近一次成功优化的状态
        [[nodiscard]] virtual KinematicState latestState() const = 0;

        /// 最近一次关键帧后的健康结论
        [[nodiscard]] virtual HealthReport health() const = 0;

        /// 只读快照：活跃关键帧与路标，供轨迹/点云导出
        [[nodiscard]] virtual EstimateSnapshot snapshot() const = 0;

        /// 诊断计数（零速更新次数、内部恢复次数等）
        [[nodiscard]] virtual EstimatorDiagnostics diagnostics() const { return {}; }
    };

    /**
     * @brief 观测提取器端口：图像 -> 带跨帧 ID 的观测
     *
     * 稀疏光流、直接法、学习型特征都实现同一接口。实现必须自己维护跨帧
     * 特征表，并在 @c reset 时清空（流水线重建后不得沿用旧 ID）。
     */
    class IObservationExtractor
    {
    public:
        virtual ~IObservationExtractor() = default;

        /// 清空跨帧状态（流水线重建时调用）
        virtual void reset() = 0;

        /**
         * @brief 处理一帧图像
         *
         * @param image 只读图像视图（布局由 @c ImageView 描述）
         * @param frameIndex 图像帧号
         * @param timestamp 曝光时刻 (s)
         * @param[out] out 观测集合；实现负责清空并填充
         * @return false 表示本帧无法提取（图像为空或格式不支持）
         */
        virtual bool extract(const ImageView &image, int frameIndex, double timestamp,
                             std::vector<Observation> &out) = 0;
    };

    /**
     * @brief 高频状态外推器端口
     *
     * 只用于「优化之间的低延迟输出」。不做外推的后端可以不实现它，编排层
     * 在工厂返回空指针时跳过高频输出。
     */
    class IStatePropagator
    {
    public:
        virtual ~IStatePropagator() = default;

        /// 用优化后的状态重置外推基线
        virtual void reset(const KinematicState &state) = 0;

        /// 用一条 IMU 切片推进
        virtual KinematicState propagate(const ImuSlice &slice) = 0;
    };

    /**
     * @brief 查询式传感器源（仿真与离线批处理）
     *
     * 真实流式数据不走这里：它由编排层用 @c TimeSynchronizer 自己驱动，
     * 因为「按时间点查询」这个 API 无法表达乱序、断档与延迟。
     */
    class ISensorSource
    {
    public:
        virtual ~ISensorSource() = default;

        [[nodiscard]] virtual ImuSample imuAt(double time) = 0;
        [[nodiscard]] virtual Frame frameAt(int frameIndex, double time) = 0;

        /// 无真值的来源返回 @c std::nullopt
        [[nodiscard]] virtual std::optional<GroundTruth> groundTruthAt(double time)
        {
            (void)time;
            return std::nullopt;
        }
    };

    /**
     * @brief 工厂产出：估计器实例 + 它消费的观测模型
     *
     * 观测模型由估计器声明而不是由编排层猜：只接受针孔标定的优化器会选
     * 虚拟针孔模型，支持全向重投影的估计器会选真实 MEI 模型。前端必须用
     * 同一个模型产生观测，否则像素语义不一致。
     */
    struct EstimatorBundle
    {
        std::unique_ptr<IEstimator> estimator;
        std::shared_ptr<const ICameraModel> observationModel;
    };

    /**
     * @brief 估计器工厂
     *
     * 同一个工厂既用于首次构造，也用于重建。旧实现的 @c reset() 直接
     * @c new 出默认 gtsam 后端，注入的自定义实现会在第一次重建时被静默
     * 换掉——工厂把「重建后仍是我注入的实现」变成结构上的保证。
     *
     * 返回 @c std::expected 而不是空指针：装配错误（未知名字、模型不支持）
     * 必须带着原因上报，不能静默回退。
     */
    using EstimatorFactory = std::function<std::expected<EstimatorBundle, std::string>(const PipelineConfig &)>;

} // namespace myslam::domain
