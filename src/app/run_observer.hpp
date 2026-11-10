#pragma once

/**
 * @file run_observer.hpp
 * @brief 回放过程中的外部观察者接口：把「算法回放」与「怎么显示/记录」分开。
 *
 * 重构后观察者只见领域类型：图像帧就是 @c domain::Frame（观测里已带原始
 * 像素与目标像素），位姿就是 @c domain::Pose。回放器不再为可视化维护
 * rawPixels / validMask 这类旁路数组。
 *
 * @par 为什么接口里仍然出现 @c cv::Mat
 *
 * 回放器本身就要读图（@c cv::imread），所以编排层已经依赖 OpenCV；把图像
 * 藏在 @c void* 后面只会让实现方做一次不必要的反向转换。领域层的
 * @c ImageView 用于算法，@c cv::Mat 只用于显示。
 */

#include "domain/types.hpp"
#include <cstddef>
#include <opencv2/core.hpp>
#include <string_view>

namespace myslam::vio
{

    /**
     * @brief 回放中一帧的可观察信息
     *
     * 全部用指针：对象由回放器在调用期间持有，观察者不得保存。
     */
    struct FrameObservation
    {
        std::size_t frameIndex{0};
        double cameraTime{0.0};
        const cv::Mat *gray{nullptr};          ///< 原始灰度图
        const domain::Frame *frame{nullptr};   ///< 本帧观测（含 sourcePixel）
        std::string_view stateName{"UNKNOWN"}; ///< 当前状态机状态
        std::size_t imuSamples{0};
        std::size_t resets{0};
        int keyframeId{0};
    };

    /**
     * @brief 回放观察者
     *
     * 实现不得抛异常：回放循环没有更上层可以兜住它。
     */
    class IRunObserver
    {
    public:
        virtual ~IRunObserver() = default;

        /// 是否需要投入工作（例如窗口创建失败后返回 false）
        [[nodiscard]] virtual bool active() const { return true; }

        /// 每帧调用一次；回放器已把 IMU 积分到该相机时刻
        virtual void onFrame(const FrameObservation &observation) = 0;

        /// 每当产出一个关键帧位姿（含初始化首帧）时调用
        virtual void onPose(const domain::Pose &pose) { (void)pose; }

        /// 观察者要求提前结束回放（例如用户按下 ESC/q）
        [[nodiscard]] virtual bool quitRequested() const { return false; }
    };

} // namespace myslam::vio
