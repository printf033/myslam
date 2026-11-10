#pragma once

/**
 * @file time_synchronizer.hpp
 * @brief IMU / 相机时间同步：零阶保持、断档检测、时间偏移、乱序丢弃。
 *
 * @par 为什么单独成组件
 *
 * 重构前这段逻辑手写在 @c ReplayRunner 的循环里（约 90 行），与读图、特征
 * 提取、状态机、日志交织在一起，无法单测；仿真路径又是另一套固定步长。
 * 时间同步是最容易出错、也最需要回归保护的地方：样本跨相机时刻被截断、
 * 断档不得被外推、乱序必须丢弃。
 *
 * 现在它是纯逻辑组件：输入 IMU 样本与相机时刻，输出带 @c dt 的切片。
 * 领域层不碰 I/O，因此可以在单元测试里精确断言每一条样本的消费区间。
 *
 * @par 零阶保持约定
 *
 * 样本 k 的测量代表区间 @c [t_k, t_{k+1})。积分在相机时刻处截断，跨边界
 * 的样本只消费其前半段，剩余部分留给下一帧。这样预积分区间末端与关键帧
 * 时间戳严格一致，固定滞后窗口的边缘化边界不会偏移。
 */

#include "domain/config.hpp"
#include "domain/types.hpp"
#include <algorithm>
#include <cstddef>
#include <deque>
#include <vector>

namespace myslam::domain
{

    /**
     * @brief 一次同步的结果
     */
    struct SyncResult
    {
        /// 对齐后的相机时刻（已扣除 @c timeOffsetSeconds）
        double alignedCameraTime{0.0};
        /// 自上次调用以来可供积分的样本切片
        std::vector<ImuSlice> slices;
        /// 检测到采集断档：调用方不得把本批切片送进估计器，应重建并重置
        bool gapDetected{false};
        double gapSeconds{0.0};
        /// 本批切片覆盖的时间跨度 (s)
        double integratedSeconds{0.0};
    };

    /**
     * @brief 时间同步统计（供回放报告）
     */
    struct SyncStats
    {
        std::size_t acceptedSamples{0};
        std::size_t droppedOutOfOrder{0};
        std::size_t droppedOverflow{0};
        std::size_t gapsDetected{0};
    };

    /**
     * @brief IMU 缓冲与相机时刻对齐
     */
    class TimeSynchronizer
    {
    public:
        explicit TimeSynchronizer(const ImuConfig &config)
            : nominalDt_(config.rateHz > 0.0 ? 1.0 / config.rateHz : 0.005),
              maxGapSeconds_(nominalDt_ * (config.maxGapFactor > 0.0 ? config.maxGapFactor : 10.0)),
              timeOffsetSeconds_(0.0),
              maxBufferSamples_(static_cast<std::size_t>(std::max(1.0, 4096.0 * std::max(1.0, config.rateHz / 200.0))))
        {
        }

        void setTimeOffset(double seconds) noexcept { timeOffsetSeconds_ = seconds; }

        /**
         * @brief 推入一条 IMU 样本
         *
         * 时间戳非严格递增的样本被丢弃并计数：把它们插回序列会伪造顺序，
         * 而重排序需要无界缓冲。真实驱动应在源头保证顺序。
         */
        bool push(const ImuSample &sample)
        {
            ++stats_.acceptedSamples;
            if (!buffer_.empty() && sample.timestamp <= buffer_.back().timestamp)
            {
                ++stats_.droppedOutOfOrder;
                --stats_.acceptedSamples;
                return false;
            }
            if (buffer_.size() >= maxBufferSamples_)
            {
                buffer_.pop_front();
                ++stats_.droppedOverflow;
            }
            buffer_.push_back(sample);
            return true;
        }

        /**
         * @brief 把缓冲推进到相机时刻，返回本次可积分的切片
         *
         * @param cameraTime 相机曝光时刻 (s)
         *
         * @note 调用方必须在 @c gapDetected 为 true 时丢弃 @c slices 并重建
         *       估计器：跨断档的积分会凭空发明一段匀速运动。
         */
        SyncResult advanceTo(double cameraTime)
        {
            SyncResult result;
            result.alignedCameraTime = cameraTime - timeOffsetSeconds_;
            const double target = result.alignedCameraTime;

            while (!buffer_.empty())
            {
                const ImuSample &sample = buffer_.front();

                // 断档检查只对本样本做一次（跨帧保留的样本会被再次访问）
                if (sample.timestamp > checkedThrough_ + 1e-12)
                {
                    if (lastSeenTimestamp_ >= 0.0 && sample.timestamp - lastSeenTimestamp_ > maxGapSeconds_)
                    {
                        result.gapDetected = true;
                        result.gapSeconds = sample.timestamp - lastSeenTimestamp_;
                        ++stats_.gapsDetected;
                        // 断档后的积分基线重置到本样本，绝不跨越缺失区间
                        integratedUntil_ = sample.timestamp;
                    }
                    checkedThrough_ = sample.timestamp;
                }

                if (sample.timestamp > target)
                {
                    break; // 样本完全位于相机时刻之后
                }

                // 样本代表 [t_k, next)；缺失后继或间隔异常时只按标称周期外推
                double segmentEnd = buffer_.size() > 1 ? buffer_[1].timestamp : sample.timestamp + nominalDt_;
                if (segmentEnd - sample.timestamp > maxGapSeconds_)
                {
                    segmentEnd = sample.timestamp + nominalDt_;
                }

                const double from = hasBaseline_ ? std::max(integratedUntil_, sample.timestamp) : sample.timestamp;
                const double to = std::min(segmentEnd, target);
                if (to > from)
                {
                    result.slices.push_back(ImuSlice{sample, to - from});
                    result.integratedSeconds += to - from;
                    integratedUntil_ = to;
                    hasBaseline_ = true;
                }

                lastSeenTimestamp_ = sample.timestamp;

                if (segmentEnd > target)
                {
                    break; // 已截断到相机时刻，样本残余留给下一帧
                }
                buffer_.pop_front();
            }

            return result;
        }

        /// 重置同步基线（重建流水线时调用；@c timestamp < 0 表示尚未开始）
        void reset(double timestamp = -1.0)
        {
            buffer_.clear();
            integratedUntil_ = timestamp;
            hasBaseline_ = timestamp >= 0.0;
            lastSeenTimestamp_ = -1.0;
            checkedThrough_ = -1e300;
        }

        [[nodiscard]] double integratedUntil() const noexcept { return integratedUntil_; }
        [[nodiscard]] std::size_t buffered() const noexcept { return buffer_.size(); }
        [[nodiscard]] const SyncStats &stats() const noexcept { return stats_; }

    private:
        double nominalDt_;
        double maxGapSeconds_;
        double timeOffsetSeconds_;
        std::size_t maxBufferSamples_;

        std::deque<ImuSample> buffer_;
        double integratedUntil_{-1.0};
        bool hasBaseline_{false};
        double lastSeenTimestamp_{-1.0};
        double checkedThrough_{-1e300};
        SyncStats stats_{};
    };

} // namespace myslam::domain
