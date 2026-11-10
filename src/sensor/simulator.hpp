#pragma once

/**
 * @file simulator.hpp
 * @brief 自包含 3D 轨迹与 IMU / 相机传感器仿真器。
 *
 * 实现 @c ISensorSource：按解析轨迹生成 IMU 与图像观测，并额外提供真值。
 * 换一条轨迹或换一套路标布局只需替换本类，流水线不动。
 *
 * 观测由一组相机模型产生——仿真不再自己拼 @c PinholeCamera。
 */

#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/types.hpp"
#include "sensor/simulator_options.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <random>

namespace myslam::vio
{

    /**
     * @brief 自包含传感器仿真器
     */
    class VioSimulator final : public domain::ISensorSource
    {
    public:
        explicit VioSimulator(const SimulatorOptions &options,
                              std::shared_ptr<const domain::ICameraModel> observationModel = nullptr)
            : options_(options),
              rng_(42),
              // 连续谱密度 -> 离散样本标准差：sigma_sample = sigma / sqrt(dt)
              distAcc_(0.0, options.accelNoiseSigma / std::sqrt(1.0 / options.imuRateHz)),
              distGyro_(0.0, options.gyroNoiseSigma / std::sqrt(1.0 / options.imuRateHz)),
              distPix_(0.0, options.pixelNoiseSigma),
              // 观测模型默认按内参自建；编排层可以传入流水线实际消费的模型，
              // 保证"产生观测"与"消费观测"用的是同一套像素语义。
              model_(observationModel ? std::move(observationModel) : domain::makeCameraModel(options.intrinsics))
        {
            trueBiasAcc_ = domain::Vec3(0.04, -0.03, 0.02);
            trueBiasGyro_ = domain::Vec3(0.003, -0.002, 0.001);

            constexpr int kLandmarkRings = 100;
            constexpr int kLandmarkLayers = 5;
            int landmarkId = 0;
            for (int layer = 0; layer < kLandmarkLayers; ++layer)
            {
                for (int i = 0; i < kLandmarkRings; ++i)
                {
                    const double theta = 2.0 * std::numbers::pi * i / kLandmarkRings;
                    const double radius = (i % 3 == 0) ? 3.5 : ((i % 3 == 1) ? 2.6 : 1.6);
                    const double x = radius * std::cos(theta);
                    const double y = radius * std::sin(theta) - 2.0;
                    const double z = 0.4 + 0.4 * layer;
                    landmarks_[landmarkId++] = domain::Vec3(x, y, z);
                }
            }
        }

        /**
         * @brief 带俯仰起伏的圆周轨迹真值
         */
        domain::GroundTruth evaluateGroundTruth(double t)
        {
            const double w = 0.8;
            const double r = 2.0;
            const double tStart = 0.6;
            const double ramp = 0.5;

            double u = 0.0;
            double uDot = 0.0;
            double phase = 0.0;
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

            const double x = r * std::sin(phi);
            const double y = r * std::cos(phi) - r;
            const double z = 1.0 + 0.3 * std::sin(0.5 * phi);

            const double vx = r * phiDot * std::cos(phi);
            const double vy = -r * phiDot * std::sin(phi);
            const double vz = 0.15 * phiDot * std::cos(0.5 * phi);

            const double ax = r * phiDdot * std::cos(phi) - r * phiDot * phiDot * std::sin(phi);
            const double ay = -r * phiDdot * std::sin(phi) - r * phiDot * phiDot * std::cos(phi);
            const double az = 0.15 * phiDdot * std::cos(0.5 * phi) - 0.075 * phiDot * phiDot * std::sin(0.5 * phi);

            const double yaw = -phi;
            const double pitch = 0.05 * std::cos(phi) * u;
            const double roll = 0.08 * std::sin(phi) * u;

            const double yawDot = -phiDot;
            const double pitchDot = 0.05 * (-std::sin(phi) * phiDot * u + std::cos(phi) * uDot);
            const double rollDot = 0.08 * (std::cos(phi) * phiDot * u + std::sin(phi) * uDot);
            const double sp = std::sin(pitch), cp = std::cos(pitch);
            const double sr = std::sin(roll), cr = std::cos(roll);
            const domain::Vec3 gyro(rollDot - yawDot * sp, pitchDot * cr + yawDot * sr * cp,
                                    -pitchDot * sr + yawDot * cp * cr);

            const Eigen::Quaterniond rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

            domain::GroundTruth truth;
            truth.state.timestamp = t;
            truth.state.pose = domain::Pose{rotation, domain::Vec3(x, y, z)};
            truth.state.velocity = domain::Vec3(vx, vy, vz);
            truth.state.bias = domain::Bias{trueBiasAcc_, trueBiasGyro_};
            truth.accel = domain::Vec3(ax, ay, az);
            truth.gyro = gyro;
            return truth;
        }

        /**
         * @brief 生成带重力、零偏与高斯白噪声的 IMU 读数
         */
        domain::ImuSample generateImu(double t)
        {
            const domain::GroundTruth truth = evaluateGroundTruth(t);

            // 比力 = R^T * (a_world - g_world)
            const domain::Vec3 gravityWorld(0.0, 0.0, -options_.gravityMagnitude);
            const domain::Vec3 specificForce = truth.state.pose.unrotate(truth.accel - gravityWorld);

            const domain::Vec3 noiseAcc(distAcc_(rng_), distAcc_(rng_), distAcc_(rng_));
            const domain::Vec3 noiseGyro(distGyro_(rng_), distGyro_(rng_), distGyro_(rng_));

            return domain::ImuSample{t, specificForce + trueBiasAcc_ + noiseAcc,
                                     truth.gyro + trueBiasGyro_ + noiseGyro};
        }

        /**
         * @brief 生成图像观测，并模拟少量误匹配外点
         */
        domain::Frame generateCameraFrame(int frameId, double t)
        {
            const domain::GroundTruth truth = evaluateGroundTruth(t);
            const domain::Pose tWc = truth.state.pose * options_.bodyPSensor;

            domain::Frame frame;
            frame.index = frameId;
            frame.timestamp = t;

            for (const auto &[id, point] : landmarks_)
            {
                const domain::Vec3 pointCamera = tWc.transformTo(point);
                const auto uv = model_->project(pointCamera);
                if (!uv)
                {
                    continue; // 相机后方或超出视场
                }
                if (uv->x() < 0.0 || uv->x() >= model_->imageWidth() || uv->y() < 0.0 ||
                    uv->y() >= model_->imageHeight())
                {
                    continue;
                }

                double noiseU = distPix_(rng_);
                double noiseV = distPix_(rng_);
                if (frameId >= 15 && frameId <= 22 && id == 0)
                {
                    noiseU += 25.0; // 故意注入的误匹配外点
                }

                domain::Observation observation;
                observation.id = id;
                observation.uv = *uv + domain::Vec2(noiseU, noiseV);
                observation.normalized = pointCamera;
                observation.sourcePixel = observation.uv;
                observation.kind = domain::ObservationKind::TRACKED;
                frame.observations.push_back(observation);
            }

            return frame;
        }

        [[nodiscard]] const std::map<int, domain::Vec3> &landmarks() const { return landmarks_; }

        // --- ISensorSource 实现 ---

        [[nodiscard]] domain::ImuSample imuAt(double time) override { return generateImu(time); }

        [[nodiscard]] domain::Frame frameAt(int frameIndex, double time) override
        {
            return generateCameraFrame(frameIndex, time);
        }

        [[nodiscard]] std::optional<domain::GroundTruth> groundTruthAt(double time) override
        {
            return evaluateGroundTruth(time);
        }

    private:
        SimulatorOptions options_;
        std::mt19937 rng_;
        std::normal_distribution<double> distAcc_;
        std::normal_distribution<double> distGyro_;
        std::normal_distribution<double> distPix_;

        domain::Vec3 trueBiasAcc_;
        domain::Vec3 trueBiasGyro_;
        std::map<int, domain::Vec3> landmarks_;
        std::shared_ptr<const domain::ICameraModel> model_;
    };

} // namespace myslam::vio
