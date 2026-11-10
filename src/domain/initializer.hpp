#pragma once

/**
 * @file initializer.hpp
 * @brief 多阶段鲁棒初始化：静止检测、重力对齐、陀螺零偏标定。
 *
 * @par 与重构前的关系
 *
 * 状态机与判据逻辑保持原样，只把 gtsam 几何类型换成领域类型。初始化本身
 * 是纯数学（均值、方差、两向量对齐），没有任何理由让它依赖因子图库。
 *
 * 状态转移：
 * @code
 * UNINITIALIZED ──feedImu──> STATIONARY_COLLECTING
 *        STATIONARY_COLLECTING ──tryGravityAlignment() 成功──> GRAVITY_ALIGNED
 *        GRAVITY_ALIGNED ──调用方注入先验后→ TRACKING_OK（由流水线持有）
 *        TRACKING_OK ──估计器给出 RESET_PIPELINE──> resetSamples() 回到 UNINITIALIZED
 * @endcode
 *
 * 降级不是状态：估计器用 @c RecoveryAction 表达处置意图，初始化器本身不
 * 需要知道"降级"这个词。
 *
 * 约束：只依赖领域类型、标准库与日志。
 */

#include "domain/config.hpp"
#include "domain/status.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace myslam::domain
{

    /**
     * @brief VIO 多阶段初始化器
     */
    class Initializer
    {
    public:
        Initializer(const InitializationConfig &config, const ImuConfig &imu)
            : config_(config),
              imu_(imu)
        {
        }

        /**
         * @brief 喂入初始静止阶段的 IMU 测量
         *
         * 只保留最近一段窗口：设备可能先动后静，历史动态样本会污染均值，
         * 而无限累积又会随长时间无法初始化而无界增长。
         */
        void feedImu(const ImuSample &sample)
        {
            if (state_ == SystemState::UNINITIALIZED || state_ == SystemState::STATIONARY_COLLECTING)
            {
                if (samples_.size() >= windowCapacity())
                {
                    samples_.erase(samples_.begin());
                }
                samples_.push_back(sample);
                state_ = SystemState::STATIONARY_COLLECTING;
            }
        }

        /**
         * @brief 尝试执行静态检测与重力粗对齐
         *
         * @param enforceStationary 是否强制静止检测。真实序列常从运动状态开始，
         *        长时间无法满足静止条件时以 false 调用：直接用最近一段 IMU
         *        均值做重力对齐（世界系取初始 IMU 系）。
         * @param[out] alignedPose 对齐得到的初始姿态（World <- Body）
         * @param[out] alignedBias 对齐得到的初始零偏
         * @return 是否成功完成对齐
         */
        bool tryGravityAlignment(Pose &alignedPose, Bias &alignedBias, bool enforceStationary = true)
        {
            if (samples_.size() < static_cast<std::size_t>(std::max(1, config_.minSamples)))
            {
                mylog::debug("初始化样本不足: 当前 {} / 目标 {}", samples_.size(), config_.minSamples);
                return false;
            }

            Vec3 meanAcc = Vec3::Zero();
            Vec3 meanGyro = Vec3::Zero();
            for (const auto &sample : samples_)
            {
                meanAcc += sample.accel;
                meanGyro += sample.gyro;
            }
            const double invCount = 1.0 / static_cast<double>(samples_.size());
            meanAcc *= invCount;
            meanGyro *= invCount;

            Vec3 varAcc = Vec3::Zero();
            Vec3 varGyro = Vec3::Zero();
            for (const auto &sample : samples_)
            {
                varAcc += (sample.accel - meanAcc).cwiseAbs2();
                varGyro += (sample.gyro - meanGyro).cwiseAbs2();
            }
            varAcc *= invCount;
            varGyro *= invCount;

            const double accStd = std::sqrt(varAcc.sum() / 3.0);
            const double gyroStd = std::sqrt(varGyro.sum() / 3.0);

            // 静止门限随噪声配置缩放：单样本标准差 = sigma * sqrt(rate)，取 5 倍并保留工业下限
            const double rateScale = std::sqrt(std::max(1.0, rateHz()));
            const double accThreshold = std::max(0.3, 5.0 * noiseAccel() * rateScale);
            const double gyroThreshold = std::max(0.05, 5.0 * noiseGyro() * rateScale);

            mylog::info("[初始化-步骤1: 静态与激励检测] 样本数: {}, 加速度标准差: {:.4f} m/s^2 (门限 {:.3f}), "
                        "陀螺标准差: {:.5f} rad/s (门限 {:.4f})",
                        samples_.size(), accStd, accThreshold, gyroStd, gyroThreshold);

            if (enforceStationary && (accStd > accThreshold || gyroStd > gyroThreshold))
            {
                mylog::warn("[初始化-步骤1] 设备处于动态激励中，未能满足静止条件，重置样本集");
                samples_.clear();
                return false;
            }

            // 重力对齐：静止时测得的比力方向应对齐世界 +z
            const Vec3 gravityWorld(0.0, 0.0, -imu_.gravityMagnitude);
            const Vec3 expectedUp = -gravityWorld.normalized();
            if (!(meanAcc.norm() > 1e-9))
            {
                return false;
            }
            const Vec3 measuredUp = meanAcc.normalized();

            Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(measuredUp, expectedUp);

            // 航向角绕重力不可观，工业约定初始航向为 0：只保留 roll/pitch
            const double pitch = std::asin(std::clamp(-rotation.toRotationMatrix()(2, 0), -1.0, 1.0));
            const double roll = std::atan2(rotation.toRotationMatrix()(2, 1), rotation.toRotationMatrix()(2, 2));
            rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

            alignedPose_ = Pose{rotation, Vec3::Zero()};

            if (enforceStationary)
            {
                alignedBias_ = Bias{Vec3::Zero(), meanGyro};
                mylog::info("[初始化-步骤2: 零偏校准] 初始陀螺仪偏置估计: [{:.5f}, {:.5f}, {:.5f}] rad/s", meanGyro(0),
                            meanGyro(1), meanGyro(2));
            }
            else
            {
                alignedBias_ = Bias{};
                mylog::warn("[初始化-步骤2: 零偏校准] 非静止对齐：角速度均值 [{:.4f}, {:.4f}, {:.4f}] rad/s 混入了"
                            "真实运动，不能作为陀螺零偏先验，已置零并交由估计器在线估计",
                            meanGyro(0), meanGyro(1), meanGyro(2));

                const double accNormDeviation = std::abs(meanAcc.norm() - imu_.gravityMagnitude);
                if (accNormDeviation > 0.1)
                {
                    const double tiltDeg =
                        std::atan2(accNormDeviation, imu_.gravityMagnitude) * 180.0 / std::numbers::pi;
                    mylog::warn("[初始化-步骤2: 重力粗对齐] 比力模长偏离重力 {:.3f} m/s^2，重力方向误差约 {:.1f}°，"
                                "该偏差会由后续紧耦合优化逐步修正",
                                accNormDeviation, tiltDeg);
                }
            }

            alignedPose = alignedPose_;
            alignedBias = alignedBias_;

            const auto rpy = alignedPose_.q.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
            mylog::info("[初始化-步骤2: 重力粗对齐] 测得比力: [{:.3f}, {:.3f}, {:.3f}] m/s^2, 估算 Roll: {:.2f}°, "
                        "Pitch: {:.2f}°, Yaw: {:.2f}°",
                        meanAcc(0), meanAcc(1), meanAcc(2), rpy(2) * 180.0 / std::numbers::pi,
                        rpy(1) * 180.0 / std::numbers::pi, rpy(0) * 180.0 / std::numbers::pi);

            state_ = SystemState::GRAVITY_ALIGNED;
            return true;
        }

        /**
         * @brief 跳过重力对齐：不做估计，直接给出一条对齐结果（仅消融实验）
         */
        bool skipGravityAlignment(Pose &alignedPose, Bias &alignedBias, const Pose &initialPose = {},
                                  std::size_t minSamples = 0)
        {
            const std::size_t required =
                minSamples > 0 ? minSamples : static_cast<std::size_t>(std::max(1, config_.minSamples));
            if (samples_.size() < required)
            {
                mylog::debug("初始化样本不足: 当前 {} / 目标 {}", samples_.size(), required);
                return false;
            }

            alignedPose_ = Pose{initialPose.q, Vec3::Zero()};
            alignedBias_ = Bias{};
            alignedPose = alignedPose_;
            alignedBias = alignedBias_;

            const auto rpy = alignedPose_.q.toRotationMatrix().eulerAngles(2, 1, 0);
            mylog::warn("[初始化-跳过对齐] 未做静止检测与重力估计：初始姿态固定为 Roll: {:.2f}°, Pitch: {:.2f}°, "
                        "Yaw: {:.2f}°，零偏置零。世界系 z 轴不再与真实重力对齐，"
                        "该偏差只能由估计器自行收敛",
                        rpy(2) * 180.0 / std::numbers::pi, rpy(1) * 180.0 / std::numbers::pi,
                        rpy(0) * 180.0 / std::numbers::pi);

            state_ = SystemState::GRAVITY_ALIGNED;
            return true;
        }

        [[nodiscard]] SystemState state() const noexcept { return state_; }
        void setState(SystemState state) noexcept { state_ = state; }
        [[nodiscard]] bool hasAlignment() const noexcept { return state_ == SystemState::GRAVITY_ALIGNED; }

        [[nodiscard]] const Pose &alignedPose() const noexcept { return alignedPose_; }
        [[nodiscard]] const Bias &alignedBias() const noexcept { return alignedBias_; }

        /// 丢弃采样窗口（重建时调用）
        void resetSamples()
        {
            samples_.clear();
            state_ = SystemState::UNINITIALIZED;
            alignedPose_ = Pose{};
            alignedBias_ = Bias{};
        }

    private:
        [[nodiscard]] std::size_t windowCapacity() const
        {
            const auto byTime = static_cast<std::size_t>(std::max(0.0, config_.windowSeconds * rateHz()));
            return std::max(static_cast<std::size_t>(std::max(1, config_.minSamples)), byTime);
        }

        [[nodiscard]] double rateHz() const noexcept { return imu_.rateHz; }
        [[nodiscard]] double noiseAccel() const noexcept { return imu_.accelNoiseSigma; }
        [[nodiscard]] double noiseGyro() const noexcept { return imu_.gyroNoiseSigma; }

        InitializationConfig config_;
        ImuConfig imu_;
        SystemState state_{SystemState::UNINITIALIZED};
        std::vector<ImuSample> samples_;
        Pose alignedPose_{};
        Bias alignedBias_{};
    };

} // namespace myslam::domain
