#pragma once

/**
 * @file register_builtin.hpp
 * @brief 内置估计器的编译期注册。
 *
 * 注册只发生一次（@c std::call_once）。键名与配置 @c estimator.name 对应：
 *   - @c gtsam_fixed_lag：gtsam 固定滞后平滑（参考实现，只接受针孔标定）；
 *   - @c imu_only：纯 IMU 航位推算（可替换性验证与 A/B 基线）。
 *
 * 新增一个后端只需要在这里加一行；流水线、编排层与配置结构都不动。
 */

#include "adapters/gtsam/fixed_lag_estimator.hpp"
#include "adapters/simple/imu_only_estimator.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/ports.hpp"
#include "domain/registry.hpp"
#include <expected>
#include <memory>
#include <mutex>
#include <string>

namespace myslam::adapters
{

    /**
     * @brief 注册全部内置估计器（幂等）
     */
    inline void registerBuiltinEstimators()
    {
        static std::once_flag once;
        std::call_once(
            once,
            []()
            {
                auto &registry = domain::EstimatorRegistry::instance();

                registry.add(
                    "gtsam_fixed_lag",
                    [](const domain::PipelineConfig &config) -> std::expected<domain::EstimatorBundle, std::string>
                    {
                        // 该后端只做针孔重投影：MEI 相机由虚拟针孔承载
                        auto model = domain::makeVirtualPinholeModel(config.camera.intrinsics);
                        auto estimator = std::make_unique<FixedLagEstimator>(config, model);
                        if (!estimator->valid())
                        {
                            return std::unexpected("gtsam_fixed_lag 需要针孔标定；当前相机模型不支持");
                        }
                        return domain::EstimatorBundle{std::move(estimator), std::move(model)};
                    });

                registry.add(
                    "imu_only",
                    [](const domain::PipelineConfig &config) -> std::expected<domain::EstimatorBundle, std::string>
                    {
                        // 纯 IMU 后端对观测像素没有要求，使用与相机种类一致的模型
                        auto model = domain::makeCameraModel(config.camera.intrinsics);
                        auto estimator = std::make_unique<ImuOnlyEstimator>(config);
                        return domain::EstimatorBundle{std::move(estimator), std::move(model)};
                    });
            });
    }

} // namespace myslam::adapters
