#pragma once

/**
 * @file config_summary.hpp
 * @brief 生效配置的结构化摘要：启动时把「这一跑究竟用了哪些参数」打出来。
 *
 * @par 为什么需要它
 *
 * 配置有七个分组、近百个字段，绝大多数在某个 lua 文件里被覆盖过，而
 * 「默认值 + 覆盖」叠加之后的结果没有任何地方看得见。于是排查精度变化时
 * 第一件事永远是逐个字段反推「它到底取了多少」。
 *
 * @par 为什么只有一份实现
 *
 * 仿真入口与回放入口都要打这份摘要。两处各写一份的后果不是"多写几行"，
 * 而是两份会各自漏掉不同的字段——**哪一个漏了都没有任何机制能发现**，
 * 而这正是摘要存在的理由。因此这里返回完整字符串，两个入口只负责把它
 * 交给日志。
 *
 * @par 为什么返回字符串而不是直接打日志
 *
 * 返回值可以被测试断言（哪些分组必须出现），也可以写进运行记录。直接调用
 * 日志宏的话，「摘要漏了一个分组」这种缺陷在 CI 里永远不会被发现。
 *
 * @par 门控结论为什么要单独一行
 *
 * 外点判定现在有两条路径（χ² 统计检验 / 像素阈值），由两个开关共同决定
 * （@c vision.chi2Gating 与 @c estimator.publishCovariance）。读配置看不出
 * 实际生效的是哪一条，而它决定出错时该去哪个模块找原因。这一行是**推导
 * 出来的结论**，不是配置项的复述。
 *
 * 约束：只依赖领域层与标准库。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/config.hpp"
#include <format>
#include <optional>
#include <string>
#include <vector>

namespace myslam::common
{

    /**
     * @brief 生成分组摘要（每行一个分组，结尾无换行）
     *
     * 数值用 @c std::format 的默认格式，不手工拼精度：本摘要的用途是
     * 「哪些字段被覆盖成了多少」，不是可复现的数值文件。
     *
     * @param system 运行时口径；为空时不输出「运行时」行。回放入口没有
     *        @c SystemConfig（时长由序列本身决定），传一个默认构造的实例
     *        会打印出「时长=5s」这种与事实不符的数字。
     */
    [[nodiscard]] inline std::string summarizeConfig(const domain::PipelineConfig &config,
                                                     const std::optional<domain::SystemConfig> &system)
    {
        const auto &intr = config.camera.intrinsics;
        const auto &extrinsic = config.camera.bodyPSensor.position;
        std::string out;

        out += std::format("[相机] {} {}x{} fx={} fy={} cx={} cy={} scale={}\n", camera::cameraModelName(intr.model),
                           intr.imageWidth, intr.imageHeight, intr.fx, intr.fy, intr.cx, intr.cy, intr.pinholeScale);
        out += std::format("[外参] 平移=({}, {}, {}) 时间偏移={}s 曝光={}s\n", extrinsic.x(), extrinsic.y(),
                           extrinsic.z(), config.camera.timeOffsetSeconds, config.camera.exposureSeconds);
        out += std::format("[IMU] rate={}Hz 噪声 acc={} gyro={} 零偏游走 acc={} gyro={} g={} 断档={}x\n",
                           config.imu.rateHz, config.imu.accelNoiseSigma, config.imu.gyroNoiseSigma,
                           config.imu.accelBiasRw, config.imu.gyroBiasRw, config.imu.gravityMagnitude,
                           config.imu.maxGapFactor);
        out += std::format("[视觉] 像素噪声={} 视差>={}px 共视<{} 间隔=[{},{}]s 三角化视差>={}px 深度/基线<={}\n",
                           config.vision.pixelNoiseSigma, config.vision.minParallaxPx, config.vision.minTrackingRatio,
                           config.vision.minKeyframeInterval, config.vision.maxKeyframeInterval,
                           config.vision.minTriangulationParallaxPx, config.vision.maxDepthBaselineRatio);
        out += std::format("[前端] 特征数下限={} 网格={}x{} RANSAC={} 阈值={}σ 置信={} 最小间距={}px\n",
                           config.vision.minFeatures, config.vision.gridColumns, config.vision.gridRows,
                           config.vision.enableRansac ? "on" : "off", config.vision.ransacThresholdSigma,
                           config.vision.ransacConfidence, config.vision.minTrackSeparationPx);
        out += std::format("[门控] 外点判定={} 像素软/硬/预优化={}/{}/{}px streak={} χ²multiplier={}/{}new "
                           "置信={} 路标协方差={}\n",
                           (config.vision.chi2Gating && config.estimator.publishCovariance) ? "χ²统计检验" : "像素阈值",
                           config.vision.maxReprojectionError, config.vision.maxReprojectionErrorHard,
                           config.vision.maxReprojectionErrorPreopt, config.vision.outlierStreakThreshold,
                           config.vision.chi2Multiplier, config.vision.chi2MultiplierNew, config.vision.chi2Confidence,
                           config.vision.chi2UseLandmarkCovariance ? "计入" : "不计入");
        out += std::format("[初始化] 重力对齐={} 动态={} 最少样本={} 窗口={}s 超时={}s 首帧观测>={}\n",
                           config.initialization.gravityAlignment ? "on" : "off",
                           config.initialization.dynamicEnabled ? "on" : "off", config.initialization.minSamples,
                           config.initialization.windowSeconds, config.initialization.timeoutSeconds,
                           config.initialization.minInitObservations);
        out +=
            std::format("[估计器] {} 窗口={}s 最大窗口={} 迭代={} 路标={} 协方差={} 零速窗口={}s\n",
                        config.estimator.name, config.estimator.slidingWindowLag, config.estimator.maxSlidingWindowSize,
                        config.estimator.extraSmootherIterations, config.estimator.landmarkRepresentation,
                        config.estimator.publishCovariance ? "上报" : "不上报", config.estimator.zuptWindowSeconds);
        out += std::format("[健康] 加速度零偏<={} 陀螺零偏<={} 速度<={} 最少特征={} 连续={}次\n",
                           config.health.maxAccelBiasNorm, config.health.maxGyroBiasNorm, config.health.maxSpeed,
                           config.health.minActiveFeatures, config.health.alertStreakThreshold);
        out += std::format("[地图] 位姿图={} 每{}帧优化 丢失宽限={}s 位置识别候选={}\n",
                           config.map.poseGraphEnabled ? "on" : "off", config.map.poseGraphOptimizeInterval,
                           config.map.lostGraceSeconds, config.map.placeRecognitionCandidates);
        if (system.has_value())
        {
            out += std::format("[运行时] 相机步长={}s 时长={}s RMSE门限={}m", system->cameraDt, system->totalDuration,
                               system->rmseThreshold);
        }
        return out;
    }

    /**
     * @brief 按行切分摘要：每个元素就是一个分组的完整一行
     *
     * 调用方逐行打日志（而不是把多行塞进一条日志），因为日志系统需要
     * 单行才能正确与时间戳配对。
     */
    [[nodiscard]] inline std::vector<std::string>
    summarizeConfigLines(const domain::PipelineConfig &config, const std::optional<domain::SystemConfig> &system)
    {
        const std::string text = summarizeConfig(config, system);
        std::vector<std::string> lines;
        std::size_t start = 0;
        while (start < text.size())
        {
            const auto end = text.find('\n', start);
            if (end == std::string::npos)
            {
                lines.push_back(text.substr(start));
                break;
            }
            lines.push_back(text.substr(start, end - start));
            start = end + 1;
        }
        return lines;
    }

} // namespace myslam::common
