#pragma once

/**
 * @file initializer.hpp
 * @brief 工业级多阶段鲁棒初始化状态机（静止检测、重力对齐、陀螺零偏标定）。
 */

#include "logger.hpp"
#include "vio/types.hpp"
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
         */
        void feedImu(const ImuMeasurement &imu)
        {
            if (state_ == SystemState::UNINITIALIZED || state_ == SystemState::STATIONARY_COLLECTING)
            {
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
         * @return 是否成功完成对齐
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

            mylog::info(
                "[初始化-步骤1: 静态与激励检测] 样本数: {}, 加速度标准差: {:.4f} m/s^2, 陀螺标准差: {:.5f} rad/s",
                imuSamples_.size(), accStd, gyroStd);

            // 静态检测门限：若方差过大说明有剧烈晃动，拒绝静态初始化
            if (enforceStationary && (accStd > 0.25 || gyroStd > 0.05))
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
            outRWb = gtsam::Rot3::Ypr(0.0, rpy(1), rpy(0));

            // 3. 初始陀螺仪偏置估计 (静止阶段角速度均值直接作为先验陀螺零偏)
            outBias = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), meanGyro);

            mylog::info("[初始化-步骤2: 重力粗对齐] 测得比力: [{:.3f}, {:.3f}, {:.3f}] m/s^2, 估算 Roll: {:.2f}°, "
                        "Pitch: {:.2f}°, Yaw: {:.2f}°",
                        meanAcc(0), meanAcc(1), meanAcc(2), outRWb.roll() * 180.0 / std::numbers::pi,
                        outRWb.pitch() * 180.0 / std::numbers::pi, outRWb.yaw() * 180.0 / std::numbers::pi);
            mylog::info("[初始化-步骤2: 零偏校准] 初始陀螺仪偏置估计: [{:.5f}, {:.5f}, {:.5f}] rad/s", meanGyro(0),
                        meanGyro(1), meanGyro(2));

            state_ = SystemState::GRAVITY_ALIGNED;
            return true;
        }

        SystemState state() const { return state_; }
        void setState(SystemState s) { state_ = s; }

    private:
        VioConfig config_;
        SystemState state_{SystemState::UNINITIALIZED};
        std::vector<ImuMeasurement> imuSamples_;
    };

} // namespace myslam::vio
