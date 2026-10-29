#pragma once

/**
 * @file initializer.hpp
 * @brief 多阶段鲁棒初始化状态机（静止检测、重力对齐、陀螺零偏标定）。
 *
 * 状态转移（与 README 的描述一致）：
 * @code
 * UNINITIALIZED ──feedImu──> STATIONARY_COLLECTING
 *        STATIONARY_COLLECTING ──tryGravityAlignment() 成功──> GRAVITY_ALIGNED
 *        GRAVITY_ALIGNED ──调用方注入先验后 setState()──> TRACKING_OK
 *        TRACKING_OK ──健康监控判定持续异常──> LOST_DEGRADED
 *        LOST_DEGRADED ──调用方重建流水线──> UNINITIALIZED
 * @endcode
 *
 * 对齐结果同时写入 @c alignedRotation / @c alignedBias，因此即使调用方把状态
 * 推进与结果消费拆到不同时刻（例如下一帧才注入先验），@c GRAVITY_ALIGNED
 * 仍是可被后续代码完整处理的状态，不存在“进去了出不来”的空转分支。
 */

#include "logger.hpp"
#include "vio/configHolder.hpp"
#include <algorithm>
#include <cmath>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <numbers>
#include <string_view>
#include <vector>

namespace myslam::vio
{

    // =========================================================================
    // 3. 多阶段鲁棒初始化状态机
    // =========================================================================

    enum class SystemState
    {
        UNINITIALIZED,         // 未初始化
        STATIONARY_COLLECTING, // 收集静止数据中
        GRAVITY_ALIGNED,       // 重力完成对齐，等待视觉视差
        TRACKING_OK,           // 正常紧耦合追踪
        LOST_DEGRADED          // 跟踪异常退化
    };

    inline std::string_view systemStateToString(SystemState s)
    {
        switch (s)
        {
        case SystemState::UNINITIALIZED:
            return "UNINITIALIZED";
        case SystemState::STATIONARY_COLLECTING:
            return "STATIONARY_COLLECTING";
        case SystemState::GRAVITY_ALIGNED:
            return "GRAVITY_ALIGNED";
        case SystemState::TRACKING_OK:
            return "TRACKING_OK";
        case SystemState::LOST_DEGRADED:
            return "LOST_DEGRADED";
        }
        return "UNKNOWN";
    }

    /**
     * @brief VIO 系统多阶段初始化器
     */
    class VioInitializer
    {
    public:
        explicit VioInitializer(const VioConfig &config)
            : config_(config)
        {
        }

        /**
         * @brief 喂入初始静止阶段的 IMU 测量
         *
         * 只保留最近一段窗口：设备可能先动后静，历史动态样本会污染均值，
         * 而无限累积又会随长时间无法初始化而无界增长。
         */
        void feedImu(const ImuMeasurement &imu)
        {
            if (state_ == SystemState::UNINITIALIZED || state_ == SystemState::STATIONARY_COLLECTING)
            {
                if (imuSamples_.size() >= windowCapacity())
                {
                    imuSamples_.erase(imuSamples_.begin());
                }
                imuSamples_.push_back(imu);
                state_ = SystemState::STATIONARY_COLLECTING;
            }
        }

        /**
         * @brief 尝试执行静态检测与重力粗对齐
         *
         * @param enforceStationary 是否强制静止检测。数据集回放时设备往往从运动
         *        状态开始，长时间无法满足静止条件；此时以 false 调用，直接用
         *        最近一段 IMU 的均值做重力对齐（世界系取初始 IMU 系）。
         * @return 是否成功完成对齐；成功时状态推进到 @c GRAVITY_ALIGNED
         */
        bool tryGravityAlignment(gtsam::Rot3 &outRWb, gtsam::imuBias::ConstantBias &outBias,
                                 bool enforceStationary = true)
        {
            if (imuSamples_.size() < 40)
            {
                mylog::debug("初始化样本不足: 当前 {} / 目标 40", imuSamples_.size());
                return false;
            }

            // 1. 计算均值与方差
            gtsam::Vector3 meanAcc = gtsam::Vector3::Zero();
            gtsam::Vector3 meanGyro = gtsam::Vector3::Zero();
            for (const auto &sample : imuSamples_)
            {
                meanAcc += sample.accel;
                meanGyro += sample.gyro;
            }
            meanAcc /= static_cast<double>(imuSamples_.size());
            meanGyro /= static_cast<double>(imuSamples_.size());

            gtsam::Vector3 varAcc = gtsam::Vector3::Zero();
            gtsam::Vector3 varGyro = gtsam::Vector3::Zero();
            for (const auto &sample : imuSamples_)
            {
                varAcc += (sample.accel - meanAcc).cwiseAbs2();
                varGyro += (sample.gyro - meanGyro).cwiseAbs2();
            }
            varAcc /= static_cast<double>(imuSamples_.size());
            varGyro /= static_cast<double>(imuSamples_.size());

            double accStd = std::sqrt(varAcc.sum() / 3.0);
            double gyroStd = std::sqrt(varGyro.sum() / 3.0);

            // 静止门限必须随噪声配置缩放：原先的硬编码 0.25 m/s^2 只给默认噪声
            // 留了约 3σ 余量，把 accelNoiseSigma 调到真实 IMU 常见量级后静止检测
            // 会开始随机误判并清空样本，初始化永远无法完成。
            // 这里取 max(工业经验下限, 5 倍理论单样本标准差)。
            // 单样本标准差 sigma_sample = sigma / sqrt(dt) = sigma * sqrt(imuRateHz)。
            const double rateScale = std::sqrt(std::max(1.0, config_.imuRateHz));
            const double accThreshold = std::max(0.3, 5.0 * config_.accelNoiseSigma * rateScale);
            const double gyroThreshold = std::max(0.05, 5.0 * config_.gyroNoiseSigma * rateScale);

            mylog::info("[初始化-步骤1: 静态与激励检测] 样本数: {}, 加速度标准差: {:.4f} m/s^2 (门限 {:.3f}), "
                        "陀螺标准差: {:.5f} rad/s (门限 {:.4f})",
                        imuSamples_.size(), accStd, accThreshold, gyroStd, gyroThreshold);

            // 静态检测门限：若方差过大说明有剧烈晃动，拒绝静态初始化
            if (enforceStationary && (accStd > accThreshold || gyroStd > gyroThreshold))
            {
                mylog::warn("[初始化-步骤1] 设备处于动态激励中，未能满足静止条件，重置样本集");
                imuSamples_.clear();
                return false;
            }

            // 2. 重力粗对齐
            // 静止时加速度计测得的比力 f_b = - rWb^T * g_world
            // 在以 z 向上定义的世界系中，g_world = [0, 0, -9.81]^T，则 -g_world = [0, 0, 9.81]^T
            // 故比力方向应大致对齐世界坐标系 +z 轴
            gtsam::Vector3 gravityWorld(0.0, 0.0, -config_.gravityMagnitude);
            gtsam::Vector3 expectedUp = -gravityWorld.normalized(); // [0, 0, 1]
            gtsam::Vector3 measuredUp = meanAcc.normalized();

            // 计算从 measuredUp 旋转到 expectedUp 的初始旋转矩阵 rWb
            // (即 rWb * measuredUp = expectedUp)
            gtsam::Vector3 axis = measuredUp.cross(expectedUp);
            double axisNorm = axis.norm();
            gtsam::Rot3 rWb = gtsam::Rot3::Identity();
            if (axisNorm > 1e-5)
            {
                double angle = std::atan2(axisNorm, measuredUp.dot(expectedUp));
                rWb = gtsam::Rot3::Rodrigues(axis.normalized() * angle);
            }

            // 航向角 Yaw 绕重力不可观，工业上约定初始航向设为 0
            gtsam::Vector3 rpy = rWb.rpy();
            alignedRotation_ = gtsam::Rot3::Ypr(0.0, rpy(1), rpy(0));

            // 3. 初始陀螺仪偏置估计
            //    只有静止窗口的角速度均值才近似为陀螺零偏。非静止路径下均值混入
            //    真实转动的角速度（可远大于零偏 2 个数量级），把它当先验会以数十
            //    倍 σ 的偏离污染初始状态，因此这里置零，把误差留给先验不确定度。
            if (enforceStationary)
            {
                alignedBias_ = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), meanGyro);
                mylog::info("[初始化-步骤2: 零偏校准] 初始陀螺仪偏置估计: [{:.5f}, {:.5f}, {:.5f}] rad/s", meanGyro(0),
                            meanGyro(1), meanGyro(2));
            }
            else
            {
                alignedBias_ = gtsam::imuBias::ConstantBias();
                mylog::warn("[初始化-步骤2: 零偏校准] 非静止对齐：角速度均值 [{:.4f}, {:.4f}, {:.4f}] rad/s 混入了"
                            "真实运动，不能作为陀螺零偏先验，已置零并交由后端在线估计",
                            meanGyro(0), meanGyro(1), meanGyro(2));

                const double accNormDeviation = std::abs(meanAcc.norm() - config_.gravityMagnitude);
                if (accNormDeviation > 0.1)
                {
                    const double tiltDeg =
                        std::atan2(accNormDeviation, config_.gravityMagnitude) * 180.0 / std::numbers::pi;
                    mylog::warn("[初始化-步骤2: 重力粗对齐] 比力模长偏离重力 {:.3f} m/s^2，重力方向误差约 {:.1f}°，"
                                "该偏差会由后续紧耦合优化逐步修正",
                                accNormDeviation, tiltDeg);
                }
            }

            outRWb = alignedRotation_;
            outBias = alignedBias_;

            mylog::info("[初始化-步骤2: 重力粗对齐] 测得比力: [{:.3f}, {:.3f}, {:.3f}] m/s^2, 估算 Roll: {:.2f}°, "
                        "Pitch: {:.2f}°, Yaw: {:.2f}°",
                        meanAcc(0), meanAcc(1), meanAcc(2), alignedRotation_.roll() * 180.0 / std::numbers::pi,
                        alignedRotation_.pitch() * 180.0 / std::numbers::pi,
                        alignedRotation_.yaw() * 180.0 / std::numbers::pi);

            state_ = SystemState::GRAVITY_ALIGNED;
            return true;
        }

        SystemState state() const { return state_; }
        void setState(SystemState s) { state_ = s; }

        /// 对齐是否已完成、正等待调用方注入先验
        bool hasAlignment() const { return state_ == SystemState::GRAVITY_ALIGNED; }

        /// 对齐得到的初始姿态（World<-Body）
        const gtsam::Rot3 &alignedRotation() const { return alignedRotation_; }

        /// 对齐得到的初始零偏
        const gtsam::imuBias::ConstantBias &alignedBias() const { return alignedBias_; }

    private:
        /// 初始化窗口容量：至少 40 条（判定所需），最多保留最近 2 秒
        [[nodiscard]] std::size_t windowCapacity() const
        {
            const auto byTime = static_cast<std::size_t>(std::max(0.0, 2.0 * config_.imuRateHz));
            return std::max<std::size_t>(40, byTime);
        }

        VioConfig config_;
        SystemState state_{SystemState::UNINITIALIZED};
        std::vector<ImuMeasurement> imuSamples_;
        gtsam::Rot3 alignedRotation_{gtsam::Rot3::Identity()};
        gtsam::imuBias::ConstantBias alignedBias_;
    };

} // namespace myslam::vio
