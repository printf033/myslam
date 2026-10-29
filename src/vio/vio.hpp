#pragma once

/**
 * @file vio.hpp
 * @brief myslam VIO 最小系统聚合头文件
 *
 * 一次引入全部 VIO 组件：数据结构与配置、初始化状态机、关键帧决策、
 * 高频外推、健康监控、紧耦合固定滞后后端、传感器仿真器与系统编排。
 *
 * 典型用法：
 * @code
 * myslam::vio::VioConfig config;
 * myslam::vio::VioSystem system(config);
 * auto report = system.run();
 * @endcode
 */

#include "vio/backend.hpp"
#include "vio/configHolder.hpp"
#include "vio/health_monitor.hpp"
#include "vio/initializer.hpp"
#include "vio/keyframe_selector.hpp"
#include "vio/propagator.hpp"
#include "vio/simulator.hpp"
#include "vio/system.hpp"
