#pragma once

/**
 * @file simulator.hpp
 * @brief 自包含 3D 轨迹与 IMU / 相机传感器仿真器（示例与测试基准）。
 */

#include "vio/configHolder.hpp"
#include <algorithm>
#include <cmath>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/BetweenFactor.h>
#include <map>
#include <numbers>
#include <random>

namespace myslam::vio
{

    // =========================================================================
    // 8. 真实 3D 轨迹与传感器仿真器
    // =========================================================================

    class VioSimulator
    {
    public:
        explicit VioSimulator(const VioConfig &config)
            : config_(config),
              rng_(42),
              // 连续谱密度 -> 离散样本标准差：sigma_sample = sigma / sqrt(dt)
              distAcc_(0.0, config.accelNoiseSigma / std::sqrt(1.0 / config.imuRateHz)),
              distGyro_(0.0, config.gyroNoiseSigma / std::sqrt(1.0 / config.imuRateHz)),
              distPix_(0.0, config.pixelNoiseSigma)
        {
            // 真实物理零偏注入
            trueBiasAcc_ = gtsam::Vector3(0.04, -0.03, 0.02);
            trueBiasGyro_ = gtsam::Vector3(0.003, -0.002, 0.001);

            // 在圆周运动轨迹周围立体空间中结构化布置 120 个特征路标点
            // (内外两层圆柱环壁与中间特征，高度覆盖 0.4m ~ 2.0m)
            // VIO 每帧通常有数十到数百个特征观测，才能与高精度 IMU 的
            // 信息量匹配并保证 bias / 重力的可观测性。此处用 5 层 × 100 个
            // 路标 (内外圆柱环壁 + 高度分层) 覆盖相机视场。
            constexpr int kLandmarkRings = 100;
            constexpr int kLandmarkLayers = 5;
            int landmarkId = 0;
            for (int layer = 0; layer < kLandmarkLayers; ++layer)
            {
                for (int i = 0; i < kLandmarkRings; ++i)
                {
                    const double theta = 2.0 * std::numbers::pi * i / kLandmarkRings;
                    const double rad = (i % 3 == 0) ? 3.5 : ((i % 3 == 1) ? 2.6 : 1.6);
                    const double x = rad * std::cos(theta);
                    const double y = rad * std::sin(theta) - 2.0;
                    const double z = 0.4 + 0.4 * layer;
                    landmarks_[landmarkId++] = gtsam::Point3(x, y, z);
                }
            }
        }

        /**
         * @brief 真实空间三维运动方程 (带俯仰起伏的圆周/双纽线轨迹)
         */
        struct GroundTruthState
        {
            gtsam::Pose3 pose;
            gtsam::Vector3 velocity;
            gtsam::Vector3 accel;
            gtsam::Vector3 gyro;
        };

        GroundTruthState evaluateGroundTruth(double t)
        {
            const double w = 0.8;
            const double r = 2.0;
            const double tStart = 0.6; ///< 静止初始化结束、运动开始时刻
            const double ramp = 0.5;   ///< 平滑加速时长：保证速度/加速度连续，避免 IMU 冲激

            // 相位 φ(t)：静止 -> 平滑加速 (smoothstep) -> 恒定角速度。
            // 真实的 VIO 系统开机后从静止平滑起步，此处显式建模，避免 tStart 处
            // 速度阶跃导致的预积分冲激误差 (工业仿真的必要条件)。
            double u = 0.0;     ///< 相位速度系数 dφ/dt / w
            double uDot = 0.0;  ///< du/dt
            double phase = 0.0; ///< ∫u dt
            if (t > tStart)
            {
                const double s = std::min(1.0, (t - tStart) / ramp);
                u = 3.0 * s * s - 2.0 * s * s * s;
                uDot = (6.0 * s - 6.0 * s * s) / ramp;
                phase = ramp * (s * s * s - 0.5 * s * s * s * s) + std::max(0.0, t - tStart - ramp);
                if (s >= 1.0)
                {
                    u = 1.0;
                    uDot = 0.0;
                }
            }

            const double phi = w * phase;
            const double phiDot = w * u;
            const double phiDdot = w * uDot;

            // 位置
            const double x = r * std::sin(phi);
            const double y = r * std::cos(phi) - r;
            const double z = 1.0 + 0.3 * std::sin(0.5 * phi);

            // 速度 (一阶导)
            const double vx = r * phiDot * std::cos(phi);
            const double vy = -r * phiDot * std::sin(phi);
            const double vz = 0.15 * phiDot * std::cos(0.5 * phi);

            // 加速度 (二阶导)
            const double ax = r * phiDdot * std::cos(phi) - r * phiDot * phiDot * std::sin(phi);
            const double ay = -r * phiDdot * std::sin(phi) - r * phiDot * phiDot * std::cos(phi);
            const double az = 0.15 * phiDdot * std::cos(0.5 * phi) - 0.075 * phiDot * phiDot * std::sin(0.5 * phi);

            // 姿态：静止段严格为单位姿态 (初始化先验假设单位旋转)，运动后平滑引入俯仰/横滚起伏
            const double yaw = -phi;
            const double pitch = 0.05 * std::cos(phi) * u;
            const double roll = 0.08 * std::sin(phi) * u;

            // 角速度：由 ZYX 欧拉角率严格变换到体坐标系
            // ω_b = [φ_dot - ψ_dot·sinθ, θ_dot·cosφ + ψ_dot·sinφ·cosθ, -θ_dot·sinφ + ψ_dot·cosφ·cosθ]
            const double yawDot = -phiDot;
            const double pitchDot = 0.05 * (-std::sin(phi) * phiDot * u + std::cos(phi) * uDot);
            const double rollDot = 0.08 * (std::cos(phi) * phiDot * u + std::sin(phi) * uDot);
            const double sp = std::sin(pitch), cp = std::cos(pitch);
            const double sr = std::sin(roll), cr = std::cos(roll);
            const gtsam::Vector3 gyro(rollDot - yawDot * sp, pitchDot * cr + yawDot * sr * cp,
                                      -pitchDot * sr + yawDot * cp * cr);

            const gtsam::Rot3 R = gtsam::Rot3::Ypr(yaw, pitch, roll);
            const gtsam::Vector3 vel(vx, vy, vz);
            const gtsam::Vector3 acc(ax, ay, az);

            return {gtsam::Pose3(R, gtsam::Point3(x, y, z)), vel, acc, gyro};
        }

        /**
         * @brief 生成带有重力、零偏与高斯白噪声的 IMU 读数
         */
        ImuMeasurement generateImu(double t)
        {
            auto gt = evaluateGroundTruth(t);

            // 比力 = R^T * (a_world - g_world)
            gtsam::Vector3 gravityWorld(0.0, 0.0, -config_.gravityMagnitude);
            gtsam::Vector3 specificForce = gt.pose.rotation().unrotate(gt.accel - gravityWorld);

            gtsam::Vector3 noiseAcc(distAcc_(rng_), distAcc_(rng_), distAcc_(rng_));
            gtsam::Vector3 noiseGyro(distGyro_(rng_), distGyro_(rng_), distGyro_(rng_));

            return {t, specificForce + trueBiasAcc_ + noiseAcc, gt.gyro + trueBiasGyro_ + noiseGyro};
        }

        /**
         * @brief 生成图像特征观测，并模拟少量误匹配外点
         */
        CameraFrame generateCameraFrame(int frameId, double t)
        {
            auto gt = evaluateGroundTruth(t);
            gtsam::Pose3 tWc = gt.pose.compose(config_.bodyPSensor);
            gtsam::Cal3_S2 K(config_.fx, config_.fy, 0.0, config_.cx, config_.cy);
            gtsam::PinholeCamera<gtsam::Cal3_S2> camera(tWc, K);

            CameraFrame frame;
            frame.frameId = frameId;
            frame.timestamp = t;

            for (const auto &[id, ptW] : landmarks_)
            {
                try
                {
                    gtsam::Point2 uv = camera.project(ptW);

                    // 视场角裁剪 (FOV check)
                    if (uv.x() >= 0 && uv.x() < config_.imageWidth && uv.y() >= 0 && uv.y() < config_.imageHeight)
                    {
                        // 故意在 frame 15-17 且 id == 12 时注入大幅误匹配外点 (跨帧持续，
                        // 确保覆盖到关键帧；关键帧序列由数据驱动，单帧注入可能被跳过)，
                        // 用于验证 Huber 鲁棒核与三级外点门控。
                        double noiseU = distPix_(rng_);
                        double noiseV = distPix_(rng_);
                        if (frameId >= 15 && frameId <= 22 && id == 0)
                        {
                            // 多个候选特征 × 多帧注入，保证至少有一个落在关键帧视野内
                            // (特征可见性随视角变化，单点单帧注入可能完全不被观测)
                            noiseU += 25.0; // 显著误匹配外点
                        }

                        gtsam::Point2 uvNoisy(uv.x() + noiseU, uv.y() + noiseV);
                        gtsam::Point2 uvNorm((uvNoisy.x() - config_.cx) / config_.fx,
                                             (uvNoisy.y() - config_.cy) / config_.fy);

                        frame.features.push_back({id, uvNoisy, uvNorm});
                    }
                }
                catch (const gtsam::CheiralityException &)
                {
                    // 点在相机后方，忽略
                }
            }

            return frame;
        }

        const std::map<int, gtsam::Point3> &landmarks() const { return landmarks_; }

    private:
        VioConfig config_;
        std::mt19937 rng_;
        std::normal_distribution<double> distAcc_;
        std::normal_distribution<double> distGyro_;
        std::normal_distribution<double> distPix_;

        gtsam::Vector3 trueBiasAcc_;
        gtsam::Vector3 trueBiasGyro_;
        std::map<int, gtsam::Point3> landmarks_;
    };

} // namespace myslam::vio
