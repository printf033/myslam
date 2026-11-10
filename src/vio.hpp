#pragma once

/**
 * @file vio.hpp
 * @brief myslam VIO 的聚合头：一次引入领域契约、流水线与内置适配器。
 *
 * @par 目录即分层
 *
 * `src/` 下的目录按职责划分，依赖严格单向：
 *
 * | 目录 | 职责 |
 * | :--- | :--- |
 * | `domain/` | 契约与值类型：位姿、观测、关键帧、端口、配置、初始化、时间同步 |
 * | `common/` | 非算法数据：点云导出、数据集回放配置 |
 * | `config/` | 配置加载（Lua 沙箱 + 区间钳位 + 交叉校验） |
 * | `sensor/` | 数据来源：仿真器、数据集读取、标定文件 |
 * | `frontend/` | 图像 → 观测（OpenCV 光流，实现 @c domain::IObservationExtractor） |
 * | `adapters/` | 估计器实现：gtsam 固定滞后、纯 IMU 航位推算 |
 * | `pipeline/` | 编排核心：只依赖 `domain/`，按注册名选择估计器 |
 * | `app/` | 两个入口：`VioBenchmark`（有真值，打分）、`ReplayRunner`（无真值，只统计） |
 * | `viz/` | 输出与显示：窗口、去畸变图、点云 |
 *
 * @par 替换估计器
 *
 * @code
 * domain::PipelineConfig config;
 * config.estimator.name = "imu_only";   // 或 "gtsam_fixed_lag"
 * vio::VioBenchmark benchmark(config);
 * auto report = benchmark.run();
 * @endcode
 *
 * 流水线不认识任何具体估计器；新增实现只需在
 * @c adapters/register_builtin.hpp 注册工厂。
 */

#include "adapters/register_builtin.hpp"
#include "app/benchmark.hpp"
#include "domain/camera_intrinsics.hpp"
#include "domain/camera_model.hpp"
#include "domain/config.hpp"
#include "domain/health_monitor.hpp"
#include "domain/initializer.hpp"
#include "domain/keyframe_selector.hpp"
#include "domain/ports.hpp"
#include "domain/propagator.hpp"
#include "domain/registry.hpp"
#include "domain/status.hpp"
#include "domain/time_synchronizer.hpp"
#include "domain/types.hpp"
#include "pipeline/pipeline.hpp"
#include "sensor/simulator.hpp"
