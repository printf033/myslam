#pragma once

/**
 * @file lua_sections.hpp
 * @brief 各配置节的字段解析与交叉校验（@c myslam::config::detail）。
 *
 * 每个 @c load*Section 负责一节：读取字段、调用 @c lua_primitives.hpp 的通用
 * 原语、并校验该节内部的交叉约束（例如"深度区间必须非空"、"关闭重力对齐时
 * 才允许注入初始倾斜"）。解析结果写入调用方传入的结构体，失败时不构造半成品。
 *
 * 与 @c lua_primitives.hpp 的分工：本文件知道"有哪些字段"，原语文件只知道
 * "怎么安全取值"。新增配置字段改这里，改取值规则改原语。
 *
 * 未启用 Lua 时整段实现不参与编译，文件退化为空壳。
 */

#include "config/lua_primitives.hpp"
#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <string_view>

#if defined(MYSLAM_HAS_LUA) && MYSLAM_HAS_LUA

namespace myslam::config
{

    namespace detail
    {

        inline bool loadVioSection(lua_State *L, int idx, domain::PipelineConfig &cfg)
        {
            constexpr std::string_view kSection = "vio";
            checkUnknownKeys(L, idx, kSection,
                             {"model",
                              "fx",
                              "fy",
                              "cx",
                              "cy",
                              "xi",
                              "k1",
                              "k2",
                              "p1",
                              "p2",
                              "pinholeScale",
                              "imageWidth",
                              "imageHeight",
                              "extrinsic",
                              "accelNoiseSigma",
                              "gyroNoiseSigma",
                              "accelBiasRw",
                              "gyroBiasRw",
                              "integrationNoise",
                              "imuRateHz",
                              "gravityMagnitude",
                              "gravityAlignment",
                              "initialTiltDeg",
                              "initialTiltAzimuthDeg",
                              "pixelNoiseSigma",
                              "huberLossDelta",
                              "maxReprojectionError",
                              "maxReprojectionErrorHard",
                              "maxReprojectionErrorPreopt",
                              "outlierStreakThreshold",
                              "postOptOutlierMarking",
                              "preoptTemporaryRejection",
                              "minParallaxPx",
                              "minTrackingRatio",
                              "minKeyframeInterval",
                              "maxKeyframeInterval",
                              "minTriangulationParallaxPx",
                              "landmarkGateWarmupObs",
                              "minLandmarkDepth",
                              "maxLandmarkDepth",
                              "minVisualFactorsForHealth",
                              "zuptWindowSeconds",
                              "zuptVelocitySigma",
                              "extraSmootherIterations",
                              "maxSlidingWindowSize",
                              "slidingWindowLag",
                              "initializerMinSamples",
                              "initializerWindowSeconds",
                              "minInitObservations",
                              "minInitSpreadDeg",
                              "healthMaxAccelBiasNorm",
                              "healthMaxGyroBiasNorm",
                              "healthMaxSpeed",
                              "healthMinActiveFeatures",
                              "healthAlertStreakThreshold",
                              "initialPoseSigma",
                              "initialVelSigma",
                              "initialBiasSigma",
                              "landmarkPriorSigma",
                              "recoveryPoseSigma",
                              "recoveryVelSigma",
                              "recoveryBiasSigma",
                              "estimator",
                              "timeOffsetSeconds",
                              "exposureSeconds"});

            bool valid = true;
            fieldTypeErrors() = 0;

            // 估计器选择：注册表键。未知名字在装配期报错，不会静默回退。
            // 必须用 lua_type 严格判断：lua_isstring 对数字也返回真（Lua 会
            // 把数字协变为字符串），于是 estimator = 42 会静默变成名字 "42"。
            lua_getfield(L, idx, "estimator");
            if (lua_isnil(L, -1))
            {
                // 未配置：沿用代码默认值（gtsam_fixed_lag）
            }
            else if (lua_type(L, -1) == LUA_TSTRING)
            {
                cfg.estimator.name = lua_tostring(L, -1);
            }
            else
            {
                reportError("vio.estimator 应为字符串（估计器注册名），实际为 {}，配置不合法", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            // 相机-IMU 时间偏移：正值表示相机时间戳比 IMU 晚。目前是固定常数
            // （不在线估计），由 TimeSynchronizer 在回放时应用。
            loadDouble(L, idx, kSection, "timeOffsetSeconds", -1.0, 1.0, cfg.camera.timeOffsetSeconds);
            // 曝光时长：图像内容对应曝光中点，回放时加 exposure/2。数据集无
            // 曝光元数据时保持默认 0（行为与旧版一致）。
            loadDouble(L, idx, kSection, "exposureSeconds", 0.0, 10.0, cfg.camera.exposureSeconds);

            // 相机内参：@c model 决定装配层选择哪个投影模型
            if (!loadCameraModel(L, idx, kSection, cfg.camera.intrinsics.model))
            {
                valid = false;
            }

            loadDouble(L, idx, kSection, "fx", 1.0, 1e6, cfg.camera.intrinsics.fx);
            loadDouble(L, idx, kSection, "fy", 1.0, 1e6, cfg.camera.intrinsics.fy);
            loadDouble(L, idx, kSection, "cx", -1e6, 1e6, cfg.camera.intrinsics.cx);
            loadDouble(L, idx, kSection, "cy", -1e6, 1e6, cfg.camera.intrinsics.cy);
            // 图像尺寸是 FOV 裁剪的依据：0 宽会让仿真器丢掉全部特征并静默空转，
            // 因此下界取 1。cx/cy 必须落在画幅内部的交叉检查见本节末。
            loadInteger(L, idx, kSection, "imageWidth", 1, 100'000, cfg.camera.intrinsics.imageWidth);
            loadInteger(L, idx, kSection, "imageHeight", 1, 100'000, cfg.camera.intrinsics.imageHeight);
            // MEI 参数：xi 与 pinholeScale 只在 model = "mei" 下参与几何（针孔
            // 下 pinholeScale 按 1 处理、xi 被归一化为 0）；k1/k2/p1/p2 则在两种
            // 模型下都作为 radtan 畸变生效——带畸变的针孔相机需要它们。
            // 所有值都解析并保存，切换 model 时不必重新录入整套标定。
            loadDouble(L, idx, kSection, "xi", -10.0, 10.0, cfg.camera.intrinsics.xi);
            loadDouble(L, idx, kSection, "k1", -10.0, 10.0, cfg.camera.intrinsics.k1);
            loadDouble(L, idx, kSection, "k2", -10.0, 10.0, cfg.camera.intrinsics.k2);
            loadDouble(L, idx, kSection, "p1", -10.0, 10.0, cfg.camera.intrinsics.p1);
            loadDouble(L, idx, kSection, "p2", -10.0, 10.0, cfg.camera.intrinsics.p2);
            // pinholeScale 与 fx / fy 同属几何硬前提：钳位会把“单位写错”
            // 静默洗成一个能跑但全错的焦距（1e-6 → 1e-3 → 虚拟焦距 0.46 px），
            // 因此同样不允许自动纠正。
            valid =
                loadDoubleStrict(L, idx, kSection, "pinholeScale", 1e-3, 10.0, cfg.camera.intrinsics.pinholeScale) &&
                valid;

            // 相机相对 IMU 的外参
            if (!loadExtrinsic(L, idx, cfg.camera.bodyPSensor))
            {
                valid = false;
            }

            // IMU 噪声特性与采样率
            // 噪声谱密度与零偏随机游走必须严格为正：它们不是“开关”，
            // 而是决定信息矩阵的量。0 不会得到“无噪声”，而是得到奇异系统：
            //   - 仿真器把 0 传给 std::normal_distribution，σ 必须 > 0，libstdc++ 断言 abort；
            //   - 因子图侧 Isotropic::Sigma(…, 0) 给出无穷信息矩阵，gtsam 抛异常；
            //   - 即使钳到 1e-12，信息矩阵会到 1e18 量级，Cholesky 仍然失败。
            // 因此与 fx / pinholeScale 一样按“不允许自动纠正”处理：钳位会把
            // “我想关掉噪声”这种错误意图洗成一个看不见的魔法数值，而用户会
            // 据此得出错误的实验结论。下界取物理上不可能更小的量。
            bool noiseValid = true;
            noiseValid =
                loadDoubleStrict(L, idx, kSection, "accelNoiseSigma", 1e-6, 1.0, cfg.imu.accelNoiseSigma) && noiseValid;
            noiseValid =
                loadDoubleStrict(L, idx, kSection, "gyroNoiseSigma", 1e-8, 1.0, cfg.imu.gyroNoiseSigma) && noiseValid;
            noiseValid =
                loadDoubleStrict(L, idx, kSection, "accelBiasRw", 1e-8, 1.0, cfg.imu.accelBiasRw) && noiseValid;
            noiseValid = loadDoubleStrict(L, idx, kSection, "gyroBiasRw", 1e-9, 1.0, cfg.imu.gyroBiasRw) && noiseValid;
            valid = noiseValid && valid;
            loadDouble(L, idx, kSection, "integrationNoise", 0.0, 1e-2, cfg.imu.integrationNoise);
            loadDouble(L, idx, kSection, "imuRateHz", 1.0, 10'000.0, cfg.imu.rateHz);
            loadDouble(L, idx, kSection, "gravityMagnitude", 1e-3, 100.0, cfg.imu.gravityMagnitude);
            loadBool(L, idx, kSection, "gravityAlignment", cfg.initialization.gravityAlignment);
            loadDouble(L, idx, kSection, "initialTiltDeg", 0.0, 90.0, cfg.initialization.initialTiltDeg);
            loadDouble(L, idx, kSection, "initialTiltAzimuthDeg", -180.0, 180.0,
                       cfg.initialization.initialTiltAzimuthDeg);

            // 视觉观测噪声与鲁棒核
            // 像素噪声同样必须为正：它决定重投影因子的信息矩阵。
            // 1e-3 px 已远小于任何真实特征定位精度，再小就进入
            // Cholesky 数值失效区（实测 1e-12 时 Hessian 到 1e18，直接失败）。
            valid =
                loadDoubleStrict(L, idx, kSection, "pixelNoiseSigma", 1e-3, 100.0, cfg.vision.pixelNoiseSigma) && valid;
            loadDouble(L, idx, kSection, "huberLossDelta", 1e-3, 100.0, cfg.vision.huberLossDelta);
            loadDouble(L, idx, kSection, "maxReprojectionError", 0.0, 1e4, cfg.vision.maxReprojectionError);
            loadDouble(L, idx, kSection, "maxReprojectionErrorHard", 0.0, 1e4, cfg.vision.maxReprojectionErrorHard);
            loadDouble(L, idx, kSection, "maxReprojectionErrorPreopt", 0.0, 1e4, cfg.vision.maxReprojectionErrorPreopt);
            loadInteger(L, idx, kSection, "outlierStreakThreshold", 1, 1'000, cfg.vision.outlierStreakThreshold);
            loadBool(L, idx, kSection, "postOptOutlierMarking", cfg.vision.postOptOutlierMarking);
            loadBool(L, idx, kSection, "preoptTemporaryRejection", cfg.vision.preoptTemporaryRejection);

            // 关键帧选取策略
            loadDouble(L, idx, kSection, "minParallaxPx", 0.0, 1e4, cfg.vision.minParallaxPx);
            loadDouble(L, idx, kSection, "minTrackingRatio", 0.0, 1.0, cfg.vision.minTrackingRatio);
            loadDouble(L, idx, kSection, "minKeyframeInterval", 0.0, 60.0, cfg.vision.minKeyframeInterval);
            loadDouble(L, idx, kSection, "maxKeyframeInterval", 0.0, 60.0, cfg.vision.maxKeyframeInterval);
            loadDouble(L, idx, kSection, "minTriangulationParallaxPx", 0.0, 1e4, cfg.vision.minTriangulationParallaxPx);
            loadInteger(L, idx, kSection, "landmarkGateWarmupObs", 0, 1'000, cfg.vision.landmarkGateWarmupObs);
            loadDouble(L, idx, kSection, "minLandmarkDepth", 0.0, 1e4, cfg.vision.minLandmarkDepth);
            loadDouble(L, idx, kSection, "maxLandmarkDepth", 0.0, 1e4, cfg.vision.maxLandmarkDepth);
            loadInteger(L, idx, kSection, "minVisualFactorsForHealth", 0, 10'000, cfg.vision.minVisualFactorsForHealth);
            loadDouble(L, idx, kSection, "zuptWindowSeconds", 1e-3, 10.0, cfg.estimator.zuptWindowSeconds);
            loadDouble(L, idx, kSection, "zuptVelocitySigma", 1e-4, 10.0, cfg.estimator.zuptVelocitySigma);
            loadInteger(L, idx, kSection, "extraSmootherIterations", 0, 100, cfg.estimator.extraSmootherIterations);

            // 滑动窗口
            loadInteger(L, idx, kSection, "maxSlidingWindowSize", 1, 1'000, cfg.estimator.maxSlidingWindowSize);
            // 固定滞后窗口必须至少容纳一个关键帧间隔，否则每帧进来就立刻被
            // 边缘化（实测 slidingWindowLag=0 时活跃关键帧恒为 1、RMSE 从 0.04
            // 恶化到 0.18 且无任何告警）。真正的下限由下面的交叉检查给出。
            loadDouble(L, idx, kSection, "slidingWindowLag", 1e-3, 60.0, cfg.estimator.slidingWindowLag);

            // 原先硬编码在初始化器与健康监控里的判据：
            // 最少样本数与窗口时长决定初始化速度，零偏/速度/观测数阈值决定
            // 何时降级。它们是最该拿来做消融的量，因此必须进配置。
            loadInteger(L, idx, kSection, "initializerMinSamples", 1, 1'000'000, cfg.initialization.minSamples);
            loadDouble(L, idx, kSection, "initializerWindowSeconds", 0.0, 600.0, cfg.initialization.windowSeconds);
            // 首关键帧退化门控：观测数下限与射线最大夹角下限 (deg)
            loadInteger(L, idx, kSection, "minInitObservations", 0, 1'000'000, cfg.initialization.minInitObservations);
            loadDouble(L, idx, kSection, "minInitSpreadDeg", 0.0, 90.0, cfg.initialization.minInitSpreadDeg);
            loadDouble(L, idx, kSection, "healthMaxAccelBiasNorm", 1e-3, 1e3, cfg.health.maxAccelBiasNorm);
            loadDouble(L, idx, kSection, "healthMaxGyroBiasNorm", 1e-4, 1e2, cfg.health.maxGyroBiasNorm);
            loadDouble(L, idx, kSection, "healthMaxSpeed", 1e-3, 1e4, cfg.health.maxSpeed);
            loadInteger(L, idx, kSection, "healthMinActiveFeatures", 0, 100'000, cfg.health.minActiveFeatures);
            loadInteger(L, idx, kSection, "healthAlertStreakThreshold", 1, 1'000, cfg.health.alertStreakThreshold);

            // 路标软先验与局部恢复先验：均为标准差 (m)，必须为正
            valid =
                loadDoubleStrict(L, idx, kSection, "landmarkPriorSigma", 1e-3, 1e3, cfg.estimator.landmarkPriorSigma) &&
                valid;
            valid =
                loadDoubleStrict(L, idx, kSection, "recoveryPoseSigma", 1e-3, 1e3, cfg.estimator.recoveryPoseSigma) &&
                valid;
            valid = loadDoubleStrict(L, idx, kSection, "recoveryVelSigma", 1e-3, 1e3, cfg.estimator.recoveryVelSigma) &&
                    valid;
            valid =
                loadDoubleStrict(L, idx, kSection, "recoveryBiasSigma", 1e-3, 1e3, cfg.estimator.recoveryBiasSigma) &&
                valid;

            // 交叉约束：单字段合法但组合自相矛盾时不做自动纠正，交由使用者决策
            if (cfg.vision.maxKeyframeInterval < cfg.vision.minKeyframeInterval)
            {
                reportError("vio.maxKeyframeInterval ({}) 小于 minKeyframeInterval ({}): 强制插入间隔不应短于最小间隔",
                            cfg.vision.maxKeyframeInterval, cfg.vision.minKeyframeInterval);
                valid = false;
            }
            if (cfg.vision.maxReprojectionErrorHard < cfg.vision.maxReprojectionError)
            {
                reportError("vio.maxReprojectionErrorHard ({}) 小于 maxReprojectionError ({}): 硬门限应不严于软门限",
                            cfg.vision.maxReprojectionErrorHard, cfg.vision.maxReprojectionError);
                valid = false;
            }
            if (cfg.vision.maxReprojectionErrorPreopt < cfg.vision.maxReprojectionError)
            {
                reportError("vio.maxReprojectionErrorPreopt ({}) 小于 maxReprojectionError ({}): 预优化门控应更宽松",
                            cfg.vision.maxReprojectionErrorPreopt, cfg.vision.maxReprojectionError);
                valid = false;
            }
            // 窗口时长连一个关键帧间隔都覆盖不了时，每帧进来就会被边缘化：
            // 活跃窗口降到 1 个关键帧，IMU 预积分因子退化为孤立约束。
            if (cfg.estimator.slidingWindowLag < cfg.vision.maxKeyframeInterval)
            {
                reportError("vio.slidingWindowLag ({}) 小于 maxKeyframeInterval ({}): 固定滞后窗口连一个关键帧间隔都"
                            "容纳不了，滑动窗口将退化为单帧",
                            cfg.estimator.slidingWindowLag, cfg.vision.maxKeyframeInterval);
                valid = false;
            }
            // 深度区间必须非空且有序：反了就相当于“没有任何路标几何有效”，
            // 后端会静默丢掉全部视觉约束，退化成纯 IMU 积分而不报错。
            if (cfg.vision.maxLandmarkDepth <= cfg.vision.minLandmarkDepth)
            {
                reportError("vio.maxLandmarkDepth ({}) 不大于 minLandmarkDepth ({}): 深度有效区间为空，"
                            "视觉观测将被全部丢弃",
                            cfg.vision.maxLandmarkDepth, cfg.vision.minLandmarkDepth);
                valid = false;
            }
            // 主点必须在画幅内：cx/cy 越界时后端的全部重投影都会落到图像外，
            // 仿真器则因为 FOV 裁剪直接丢掉所有特征而静默空转。
            if (cfg.camera.intrinsics.cx < 0.0 || cfg.camera.intrinsics.cx >= cfg.camera.intrinsics.imageWidth ||
                cfg.camera.intrinsics.cy < 0.0 || cfg.camera.intrinsics.cy >= cfg.camera.intrinsics.imageHeight)
            {
                reportError("vio 主点 ({}, {}) 落在画幅 {}x{} 之外: 全部重投影都会越界", cfg.camera.intrinsics.cx,
                            cfg.camera.intrinsics.cy, cfg.camera.intrinsics.imageWidth,
                            cfg.camera.intrinsics.imageHeight);
                valid = false;
            }

            // 类型错误是使用者错误，不能靠钳位/忽略洗白：否则 cameraDt = "0.05"
            // 这种从 YAML 拷来的写法会静默保留默认值，用户以为参数已生效。
            if (fieldTypeErrors() > 0)
            {
                reportError("vio 节共 {} 个字段类型不匹配，配置不合法", fieldTypeErrors());
                valid = false;
            }

            // 内参自洽性交给统一模型表达：MEI 需要有效的 pinholeScale，
            // 针孔只需要正焦距与画幅——避免这里再写一份按模型的规则
            if (!cfg.camera.intrinsics.isValid())
            {
                reportError("vio 相机内参未通过校验（model={}）", camera::cameraModelName(cfg.camera.intrinsics.model));
                valid = false;
            }

            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c system 节并校验时序一致性
         *
         * IMU 采样步长不在这里配置：它由 @c vio.imuRateHz 派生
         * （见 @c SystemConfig::imuDtFrom），因此不存在两处不一致的可能。
         * 旧配置里残留的 @c imuDt 会得到一条明确的迁移提示。
         *
         * @param L Lua 状态机
         * @param idx system 表在栈上的索引
         * @param cfg 待填充配置
         * @param vioCfg 已解析的视觉惯性配置，提供 IMU 采样率
         * @return true 解析成功
         * @return false 存在自相矛盾的时序配置
         */
        inline bool loadSystemSection(lua_State *L, int idx, domain::SystemConfig &cfg,
                                      domain::InitializationConfig &initCfg, const domain::PipelineConfig &vioCfg)
        {
            constexpr std::string_view kSection = "system";
            fieldTypeErrors() = 0;
            checkUnknownKeys(
                L, idx, kSection,
                {"imuDt", "cameraDt", "totalDuration", "rmseThreshold", "initTimeout", "initialHeightMeters"});

            loadDouble(L, idx, kSection, "cameraDt", 1e-6, 10.0, cfg.cameraDt);
            loadDouble(L, idx, kSection, "totalDuration", 1e-3, 86'400.0, cfg.totalDuration);
            loadDouble(L, idx, kSection, "rmseThreshold", 0.0, 1e3, cfg.rmseThreshold);
            // 0 是合法取值：表示不超时，严格等待静止条件（与旧的硬等待行为一致）
            loadDouble(L, idx, kSection, "initTimeout", 0.0, 3'600.0, initCfg.timeoutSeconds);
            // 初始化写入的世界系高度：消去固定滞后优化的高度自由度，
            // 使估计轨迹可与仿真真值逐帧比较。区间取正值以排除笔误。
            loadDouble(L, idx, kSection, "initialHeightMeters", -1e3, 1e3, initCfg.initialHeightMeters);

            // 旧配置兼容：imuDt 已改为派生量，单独提示比笼统的“未知字段”更可操作
            lua_getfield(L, idx, "imuDt");
            if (!lua_isnil(L, -1))
            {
                reportError("system.imuDt 已移除（现由 vio.imuRateHz 派生为 {} s），该字段被忽略",
                            domain::SystemConfig::imuDtFrom(vioCfg.imu.rateHz));
            }
            lua_pop(L, 1);

            const double imuDt = domain::SystemConfig::imuDtFrom(vioCfg.imu.rateHz);
            if (imuDt <= 0.0)
            {
                reportError("vio.imuRateHz ({}) 非正，无法派生 IMU 采样步长", vioCfg.imu.rateHz);
                return false;
            }
            if (cfg.cameraDt < imuDt)
            {
                reportError("system.cameraDt ({}) 小于 vio.imuRateHz ({}) 派生的步长 ({}): 本系统假设 IMU 采样快于相机",
                            cfg.cameraDt, vioCfg.imu.rateHz, imuDt);
                return false;
            }
            if (fieldTypeErrors() > 0)
            {
                reportError("system 节共 {} 个字段类型不匹配，配置不合法", fieldTypeErrors());
                return false;
            }
            return true;
        }

        /**
         * @brief 解析 Lua 的 @c dataset.intrinsics 节（数据集相机内参，针孔或 MEI）
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param cfg 待填充内参
         * @return true 解析成功（含钳位告警）
         */
        inline bool loadIntrinsicsSection(lua_State *L, int idx, dataset::MeiIntrinsics &cfg,
                                          dataset::DatasetInputConfig &input)
        {
            constexpr std::string_view kSection = "dataset.intrinsics";
            fieldTypeErrors() = 0;
            lua_getfield(L, idx, "intrinsics");
            if (!lua_istable(L, -1))
            {
                if (!lua_isnil(L, -1))
                {
                    reportError("dataset.intrinsics 应为 table，实际为 {}，已忽略", typeName(L));
                    lua_pop(L, 1);
                    return false;
                }
                lua_pop(L, 1);
                return true;
            }

            const int tableIdx = lua_gettop(L);
            checkUnknownKeys(L, tableIdx, kSection,
                             {"model", "fx", "fy", "cx", "cy", "xi", "k1", "k2", "p1", "p2", "pinholeScale",
                              "imageWidth", "imageHeight", "source", "file"});

            // 内参来源：来源决定数值字段是否生效，因此先解析它。
            bool valid = true;
            std::string sourceText;
            loadString(L, tableIdx, kSection, "source", sourceText);
            if (!sourceText.empty())
            {
                if (sourceText == "config")
                {
                    input.intrinsicsSource = dataset::IntrinsicsSource::CONFIG;
                }
                else if (sourceText == "file")
                {
                    input.intrinsicsSource = dataset::IntrinsicsSource::FILE;
                }
                else if (sourceText == "mast3r")
                {
                    input.intrinsicsSource = dataset::IntrinsicsSource::MAST3R;
                }
                else
                {
                    reportError("dataset.intrinsics.source = \"{}\" 不是受支持的来源（config / file / mast3r）",
                                sourceText);
                    valid = false;
                }
            }
            loadPath(L, tableIdx, kSection, "file", input.intrinsicsFile);

            // 数据集侧的 model 决定前端如何反解畸变，退错模型会让整段回放作废
            if (!loadCameraModel(L, tableIdx, kSection, cfg.model))
            {
                lua_pop(L, 1);
                return false;
            }

            loadDouble(L, tableIdx, kSection, "cx", -1e6, 1e6, cfg.cx);
            loadDouble(L, tableIdx, kSection, "cy", -1e6, 1e6, cfg.cy);
            loadDouble(L, tableIdx, kSection, "xi", -10.0, 10.0, cfg.xi);
            loadDouble(L, tableIdx, kSection, "k1", -10.0, 10.0, cfg.k1);
            loadDouble(L, tableIdx, kSection, "k2", -10.0, 10.0, cfg.k2);
            loadDouble(L, tableIdx, kSection, "p1", -10.0, 10.0, cfg.p1);
            loadDouble(L, tableIdx, kSection, "p2", -10.0, 10.0, cfg.p2);
            // 焦距、缩放与图像尺寸是几何链路的硬前提：越界时报错而非钳位，
            // 否则 fx=0 被钳到 1.0 会“表面合法、运行期崩”。
            valid = loadDoubleStrict(L, tableIdx, kSection, "fx", 1.0, 1e6, cfg.fx) && valid;
            valid = loadDoubleStrict(L, tableIdx, kSection, "fy", 1.0, 1e6, cfg.fy) && valid;
            valid = loadDoubleStrict(L, tableIdx, kSection, "pinholeScale", 1e-3, 10.0, cfg.pinholeScale) && valid;
            valid = loadIntegerStrict(L, tableIdx, kSection, "imageWidth", 1, 100'000, cfg.imageWidth) && valid;
            valid = loadIntegerStrict(L, tableIdx, kSection, "imageHeight", 1, 100'000, cfg.imageHeight) && valid;

            // 逐维上限不足以约束内存：100000x100000 逐维合法，但特征跟踪器会
            // cv::Mat::zeros(1e10) 并在 int 域计算宽*高（有符号溢出，UB）。
            // 这里按像素总数上限兜底，代表真实相机画幅的上限（约 8K 正方形的 6 倍）。
            constexpr long long kMaxPixels = 50'000'000;
            const long long pixels = static_cast<long long>(cfg.imageWidth) * static_cast<long long>(cfg.imageHeight);
            if (pixels > kMaxPixels)
            {
                reportError("dataset.intrinsics 画幅 {}x{} = {} 像素超过上限 {}（防止特征跟踪器分配失控）",
                            cfg.imageWidth, cfg.imageHeight, pixels, kMaxPixels);
                valid = false;
            }
            if (fieldTypeErrors() > 0)
            {
                reportError("dataset.intrinsics 节共 {} 个字段类型不匹配，配置不合法", fieldTypeErrors());
                valid = false;
            }

            // 相机模型决定前端如何把像素投到虚拟针孔平面：前端不做模型分支，
            // 而是靠 xi = 0 让 Mei & Rives 公式精确退化为标准针孔。因此针孔
            // 模型下必须把 xi 归零（normalizeForModel），否则从 SubT 基线继承
            // 的 xi = 1.58 会让针孔图被当作鱼眼图反解，边缘特征点被挤到镜头边框上。
            //
            // 只有使用者显式写了 xi 才提示：从 MEI 基线切到 pinhole 时 xi 必然
            // 非零，那是模型切换的正常结果，不是配置错误。
            bool xiPresent = false;
            {
                double probe = cfg.xi;
                (void)readFiniteDoubleStrict(L, tableIdx, kSection, "xi", probe, &xiPresent);
            }
            if (cfg.normalizeForModel() && xiPresent)
            {
                reportError("dataset.intrinsics.model = pinhole 时 xi 不生效，已置 0"
                            "（针孔模型没有球面反投影）");
            }
            // k1 / k2 / p1 / p2 不清零：对带径向/切向畸变的针孔相机它们是合法参数，
            // 在这里清零会毁掉真实的 radtan 标定。它们的来源由基线或配置决定，
            // 不在本函数里判。

            lua_pop(L, 1);
            return valid && cfg.isValid();
        }

        /**
         * @brief 解析 Lua 的 @c dataset.replay 与 @c dataset.viz 两节
         *
         * 路径字段允许写空串表示“不输出”，因此用 @c "" 可以清掉代码默认值。
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param cfg 待填充回放配置（含嵌套的可视化配置）
         * @return true 解析成功
         * @return false 存在自相矛盾或类型错误的配置
         */
        inline bool loadReplaySection(lua_State *L, int idx, dataset::DatasetRunConfig &cfg)
        {
            bool valid = true;
            fieldTypeErrors() = 0;

            lua_getfield(L, idx, "replay");
            if (lua_istable(L, -1))
            {
                const int tableIdx = lua_gettop(L);
                constexpr std::string_view kSection = "dataset.replay";
                checkUnknownKeys(L, tableIdx, kSection,
                                 {"outputTum", "startFrame", "maxFrames", "stride", "maxFeatures", "maxDuration",
                                  "plyPath", "maxImuGapFactor"});

                loadPath(L, tableIdx, kSection, "outputTum", cfg.outputTum);
                loadPath(L, tableIdx, kSection, "plyPath", cfg.plyPath);
                loadInteger(L, tableIdx, kSection, "startFrame", 0, 10'000'000, cfg.startFrame);
                loadInteger(L, tableIdx, kSection, "maxFrames", 0, 10'000'000, cfg.maxFrames);
                loadInteger(L, tableIdx, kSection, "stride", 1, 10'000, cfg.stride);
                loadInteger(L, tableIdx, kSection, "maxFeatures", 1, 1'000'000, cfg.maxFeatures);
                loadDouble(L, tableIdx, kSection, "maxDuration", 0.0, 86'400.0, cfg.maxDuration);
                // 倍数低于 2 会把正常抖动误判为断档，上限 1000 已远超任何合理缺样
                loadDouble(L, tableIdx, kSection, "maxImuGapFactor", 2.0, 1'000.0, cfg.maxImuGapFactor);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("dataset.replay 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            lua_getfield(L, idx, "viz");
            if (lua_istable(L, -1))
            {
                const int tableIdx = lua_gettop(L);
                constexpr std::string_view kSection = "dataset.viz";
                checkUnknownKeys(L, tableIdx, kSection,
                                 {"showWindow", "videoPath", "showUndistorted", "displayStride", "panelHeight",
                                  "trailMaxPoints", "videoFps", "playbackRate"});

                loadBool(L, tableIdx, kSection, "showWindow", cfg.viz.showWindow);
                loadPath(L, tableIdx, kSection, "videoPath", cfg.viz.videoPath);
                loadBool(L, tableIdx, kSection, "showUndistorted", cfg.viz.showUndistorted);
                loadInteger(L, tableIdx, kSection, "displayStride", 1, 10'000, cfg.viz.displayStride);
                // 状态栏高度为 0 时下面的面板是空 Mat，cv::rectangle / cv::vconcat
                // 会直接断言失败并 terminate。
                loadInteger(L, tableIdx, kSection, "panelHeight", 1, 4'000, cfg.viz.panelHeight);
                loadInteger(L, tableIdx, kSection, "trailMaxPoints", 1, 10'000'000, cfg.viz.trailMaxPoints);
                // 0 是合法值：表示跟随序列频率，由回放器用时间戳估算后填入
                loadInteger(L, tableIdx, kSection, "videoFps", 0, 1'000, cfg.viz.videoFps);
                loadDouble(L, tableIdx, kSection, "playbackRate", 0.0, 100.0, cfg.viz.playbackRate);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("dataset.viz 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            // 窗口与视频都没有启用时，渲染参数无意义但不致命；
            // 但类型不匹配始终是使用者错误，仍然判失败。
            if (fieldTypeErrors() > 0)
            {
                reportError("dataset.replay / viz 节共 {} 个字段类型不匹配，配置不合法", fieldTypeErrors());
                valid = false;
            }
            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c dataset.sequence 节（回放序列位置）
         *
         * @c root 为空表示没配置序列：仿真路径不需要它，回放器会在加载前
         * 给出可操作的错误，而不是拿当前目录去拼一个不存在的路径。
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param cfg 待填充序列配置
         * @return true 解析成功
         */
        inline bool loadSequenceSection(lua_State *L, int idx, dataset::SequenceConfig &cfg)
        {
            bool valid = true;
            fieldTypeErrors() = 0;
            lua_getfield(L, idx, "sequence");
            if (lua_istable(L, -1))
            {
                const int tableIdx = lua_gettop(L);
                constexpr std::string_view kSection = "dataset.sequence";
                checkUnknownKeys(L, tableIdx, kSection, {"root", "cameraDir", "imuFile", "stampFile"});

                loadPath(L, tableIdx, kSection, "root", cfg.root);
                loadPath(L, tableIdx, kSection, "cameraDir", cfg.cameraDir);
                loadPath(L, tableIdx, kSection, "imuFile", cfg.imuFile);
                loadPath(L, tableIdx, kSection, "stampFile", cfg.stampFile);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("dataset.sequence 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            if (fieldTypeErrors() > 0)
            {
                reportError("dataset.sequence 节共 {} 个字段类型不匹配，配置不合法", fieldTypeErrors());
                valid = false;
            }
            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c dataset.mast3r 节（内参估计选项）
         *
         * 只在 @c dataset.intrinsics.source = "mast3r" 时被消费；其它来源下
         * 配置了也不报错，避免切换来源时被迫删字段。
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param cfg 待填充估计选项
         * @return true 解析成功
         */
        inline bool loadMast3rSection(lua_State *L, int idx, dataset::Mast3rConfig &cfg)
        {
            bool valid = true;
            fieldTypeErrors() = 0;
            lua_getfield(L, idx, "mast3r");
            if (lua_istable(L, -1))
            {
                const int tableIdx = lua_gettop(L);
                constexpr std::string_view kSection = "dataset.mast3r";
                checkUnknownKeys(L, tableIdx, kSection, {"python", "weights", "pairs", "device", "cache", "force"});

                loadString(L, tableIdx, kSection, "python", cfg.python);
                loadString(L, tableIdx, kSection, "weights", cfg.weights);
                // 下限 1：0 对图像对会让估计脚本无事可做；上限 128 已超过
                // ViT-Large 在常见显存下的可行上限。
                loadInteger(L, tableIdx, kSection, "pairs", 1, 128, cfg.pairs);

                std::string device;
                loadString(L, tableIdx, kSection, "device", device);
                if (!device.empty())
                {
                    if (device == "auto" || device == "cpu" || device == "cuda")
                    {
                        cfg.device = device;
                    }
                    else
                    {
                        reportError("dataset.mast3r.device = \"{}\" 不是受支持的设备（auto / cpu / cuda）", device);
                        valid = false;
                    }
                }

                loadPath(L, tableIdx, kSection, "cache", cfg.cache);
                loadBool(L, tableIdx, kSection, "force", cfg.force);
            }
            else if (!lua_isnil(L, -1))
            {
                reportError("dataset.mast3r 应为 table，实际为 {}，已忽略", typeName(L));
                valid = false;
            }
            lua_pop(L, 1);

            if (fieldTypeErrors() > 0)
            {
                reportError("dataset.mast3r 节共 {} 个字段类型不匹配，配置不合法", fieldTypeErrors());
                valid = false;
            }
            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c dataset.extrinsic 节（回放专用相机-IMU 外参）
         *
         * 字段形状与 @c vio.extrinsic 完全一致；差别只在于语义——它只作用于回放，
         * 不改变仿真与其它入口的安装参数。未写该节时 @c hasExtrinsic 保持 false，
         * 调用方沿用 @c vio.extrinsic。
         *
         * @param L Lua 状态机
         * @param idx 父表（dataset）在栈上的索引
         * @param input 待填充的输入配置
         * @return true 解析成功或无该节
         */
        inline bool loadDatasetExtrinsic(lua_State *L, int idx, dataset::DatasetInputConfig &input)
        {
            lua_getfield(L, idx, "extrinsic");
            const int fieldType = lua_type(L, -1);
            // typeName 读的是栈顶，必须在弹栈之前取，否则报出的是父表的类型
            const std::string actualType = typeName(L);
            lua_pop(L, 1);

            if (fieldType != LUA_TTABLE && fieldType != LUA_TNIL)
            {
                reportError("dataset.extrinsic 应为 table，实际为 {}，已忽略", actualType);
                return false;
            }
            if (fieldType == LUA_TNIL)
            {
                return true;
            }

            domain::Pose pose = input.extrinsic;
            const bool valid = loadExtrinsic(L, idx, pose, "dataset.extrinsic");
            if (valid)
            {
                input.extrinsic = pose;
                input.hasExtrinsic = true;
            }
            return valid;
        }

        /**
         * @brief 解析 Lua 的 @c dataset 节
         *
         * @param L Lua 状态机
         * @param idx dataset 表在栈上的索引
         * @param intrinsics 待填充相机内参
         * @param replay 待填充回放配置
         * @param input 待填充序列位置、内参来源与估计选项
         * @return true 解析成功
         * @return false 存在无法自动纠正的配置错误
         */
        inline bool loadDatasetSection(lua_State *L, int idx, dataset::MeiIntrinsics &intrinsics,
                                       dataset::DatasetRunConfig &replay, dataset::DatasetInputConfig &input,
                                       domain::ImuConfig &imu)
        {
            checkUnknownKeys(L, idx, "dataset", {"intrinsics", "replay", "viz", "sequence", "mast3r", "extrinsic"});

            bool valid = true;
            if (!loadIntrinsicsSection(L, idx, intrinsics, input))
            {
                reportError("dataset.intrinsics 校验失败：内参或图像尺寸非法");
                valid = false;
            }
            if (!loadReplaySection(L, idx, replay))
            {
                valid = false;
            }
            if (!loadSequenceSection(L, idx, input.sequence))
            {
                valid = false;
            }
            if (!loadMast3rSection(L, idx, input.mast3r))
            {
                valid = false;
            }
            if (!loadDatasetExtrinsic(L, idx, input))
            {
                valid = false;
            }

            // 来源与取值必须自洽：选了 file 却没给文件，回放时只会得到
            // 「标定文件不存在: (空)」这类无法操作的错误，这里直接拒绝。
            if (input.intrinsicsSource == dataset::IntrinsicsSource::FILE && input.intrinsicsFile.empty())
            {
                reportError("dataset.intrinsics.source = \"file\" 时 dataset.intrinsics.file 不得为空");
                valid = false;
            }

            // dataset.replay.maxImuGapFactor 是历史上唯一的断档配置入口，已有序列
            // 都这么写。真实生效的阈值只有一个来源（ImuConfig::maxGapFactor，
            // 由 TimeSynchronizer 读取），因此这里显式搬过去，而不是让两个字段
            // 各自表示同一件事——后者迟早会有一侧改了而另一侧没改。
            imu.maxGapFactor = replay.maxImuGapFactor;
            return valid;
        }

    } // namespace detail

} // namespace myslam::config

#endif // MYSLAM_HAS_LUA
