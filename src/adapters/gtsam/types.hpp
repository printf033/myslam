#pragma once

/**
 * @file types.hpp
 * @brief 领域类型 ↔ gtsam 类型的转换。适配器之外的代码不得出现本文件。
 */

#include "domain/types.hpp"
#include <Eigen/Geometry>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace myslam::adapters
{

    [[nodiscard]] inline gtsam::Point3 toGtsam(const domain::Vec3 &value) noexcept
    {
        return gtsam::Point3(value.x(), value.y(), value.z());
    }

    [[nodiscard]] inline domain::Vec3 fromGtsam(const gtsam::Point3 &value) noexcept
    {
        return domain::Vec3(value.x(), value.y(), value.z());
    }

    [[nodiscard]] inline gtsam::Pose3 toGtsam(const domain::Pose &value)
    {
        return gtsam::Pose3(gtsam::Rot3(value.orientation.toRotationMatrix()), toGtsam(value.position));
    }

    [[nodiscard]] inline domain::Pose fromGtsam(const gtsam::Pose3 &value)
    {
        return domain::Pose{Eigen::Quaterniond(value.rotation().matrix()), fromGtsam(value.translation())};
    }

    [[nodiscard]] inline gtsam::imuBias::ConstantBias toGtsam(const domain::Bias &value)
    {
        return gtsam::imuBias::ConstantBias(toGtsam(value.accel), toGtsam(value.gyro));
    }

    [[nodiscard]] inline domain::Bias fromGtsam(const gtsam::imuBias::ConstantBias &value)
    {
        return domain::Bias{fromGtsam(value.accelerometer()), fromGtsam(value.gyroscope())};
    }

    [[nodiscard]] inline gtsam::NavState toGtsam(const domain::KinematicState &value)
    {
        return gtsam::NavState(toGtsam(value.pose), toGtsam(value.velocity));
    }

    [[nodiscard]] inline domain::KinematicState fromGtsam(double timestamp, const gtsam::NavState &state,
                                                          const domain::Bias &bias)
    {
        return domain::KinematicState{timestamp, fromGtsam(state.pose()), fromGtsam(state.velocity()), bias};
    }

} // namespace myslam::adapters
