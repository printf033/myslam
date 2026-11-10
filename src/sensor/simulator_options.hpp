#pragma once

/**
 * @file simulator_options.hpp
 * @brief 仿真器（@c VioSimulator）独享的参数集。
 *
 * 只保留仿真真正读取的字段：相机内外参、三项噪声、采样率与重力。配置
 * 镜像到本结构的映射放在这里，而不是流水线装配层——后者不该依赖传感器层。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include "domain/types.hpp"

namespace myslam::vio
{

    /**
     * @brief 仿真器参数：几何、噪声与采样率
     */
    struct SimulatorOptions
    {
        camera::CameraIntrinsics intrinsics{};
        domain::Pose bodyPSensor = domain::defaultBodyPSensor();
        double accelNoiseSigma{0.0057};
        double gyroNoiseSigma{3.6e-4};
        double pixelNoiseSigma{1.0};
        double imuRateHz{200.0};
        double gravityMagnitude{9.81};
    };

    /// 配置镜像 -> 仿真器参数
    [[nodiscard]] inline SimulatorOptions makeSimulatorOptions(const domain::PipelineConfig &config)
    {
        SimulatorOptions options;
        options.intrinsics = config.camera.intrinsics;
        options.bodyPSensor = config.camera.bodyPSensor;
        options.accelNoiseSigma = config.imu.accelNoiseSigma;
        options.gyroNoiseSigma = config.imu.gyroNoiseSigma;
        options.pixelNoiseSigma = config.vision.pixelNoiseSigma;
        options.imuRateHz = config.imu.rateHz;
        options.gravityMagnitude = config.imu.gravityMagnitude;
        return options;
    }

} // namespace myslam::vio
