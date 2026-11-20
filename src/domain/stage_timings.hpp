#pragma once

/**
 * @file stage_timings.hpp
 * @brief 分段耗时统计：把一次运行拆成几个互斥的阶段分别累计。
 *
 * @par 为什么放在 domain
 *
 * 它与 @c RunReport 是同一类东西：一次运行的可观测结果值。编排层需要
 * 在阶段边界打点，而分层规则要求 @c pipeline 只依赖 @c domain——把计时
 * 设施放这里，既不让编排层认识具体实现，也不弱化分层。
 *
 * @par 只统计、不改行为
 *
 * 计时器在构造时记起点、析构时累加，属于纯观测设施：不参与任何算法分支，
 * 不使用第三方库，不影响结果。
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>

namespace myslam::domain
{

    /**
     * @brief 一次运行的阶段划分
     *
     * 顺序即 CSV 输出顺序，改动会影响下游脚本的解读。
     */
    enum class TimingStage : std::uint8_t
    {
        kInitialization, ///< 初始化状态机推进（stepInitialization 整体）
        kPropagation,    ///< 优化之间的 IMU 外推
        kEstimator,      ///< 估计器单次 addKeyframe（含求解与门控）
        kPoseGraph,      ///< 位姿图登记与批量优化（跨窗口长期一致性）
        kCount
    };

    [[nodiscard]] constexpr std::string_view timingStageName(TimingStage stage) noexcept
    {
        switch (stage)
        {
        case TimingStage::kInitialization:
            return "initialization";
        case TimingStage::kPropagation:
            return "propagation";
        case TimingStage::kEstimator:
            return "estimator";
        case TimingStage::kPoseGraph:
            return "pose_graph";
        case TimingStage::kCount:
            break;
        }
        return "unknown";
    }

    /// 单个阶段的累计值
    struct StageStats
    {
        std::uint64_t microseconds{0};
        std::size_t calls{0};

        [[nodiscard]] double meanMilliseconds() const noexcept
        {
            return calls > 0 ? static_cast<double>(microseconds) / static_cast<double>(calls) / 1000.0 : 0.0;
        }

        [[nodiscard]] double totalMilliseconds() const noexcept { return static_cast<double>(microseconds) / 1000.0; }
    };

    /**
     * @brief 累计各阶段耗时（单线程，不加锁）
     */
    class StageTimings
    {
    public:
        /**
         * @brief RAII 计时器：构造记起点，析构累加
         *
         * 用 guard 而不是成对的 start/stop，异常路径也不会漏记。
         */
        class Scoped
        {
        public:
            Scoped(StageTimings &timings, TimingStage stage) noexcept
                : timings_(timings),
                  stage_(stage),
                  start_(std::chrono::steady_clock::now())
            {
            }

            ~Scoped()
            {
                const auto elapsed = std::chrono::steady_clock::now() - start_;
                const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
                timings_.add(stage_, static_cast<std::uint64_t>(microseconds > 0 ? microseconds : 0));
            }

            Scoped(const Scoped &) = delete;
            Scoped &operator=(const Scoped &) = delete;
            Scoped(Scoped &&) = delete;
            Scoped &operator=(Scoped &&) = delete;

        private:
            StageTimings &timings_;
            TimingStage stage_;
            std::chrono::steady_clock::time_point start_;
        };

        void add(TimingStage stage, std::uint64_t microseconds) noexcept
        {
            const auto index = static_cast<std::size_t>(stage);
            if (index >= stats_.size())
            {
                return;
            }
            stats_[index].microseconds += microseconds;
            stats_[index].calls += 1;
        }

        [[nodiscard]] const StageStats &stats(TimingStage stage) const noexcept
        {
            // 与 add() 对称：枚举可被 static_cast 到 kCount 之外，越界读
            // std::array 是 UB。返回静态空值比静默越界安全。
            const auto index = static_cast<std::size_t>(stage);
            if (index >= stats_.size())
            {
                static const StageStats kEmpty{};
                return kEmpty;
            }
            return stats_[index];
        }

        [[nodiscard]] double totalMilliseconds() const noexcept
        {
            std::uint64_t total = 0;
            for (const StageStats &entry : stats_)
            {
                total += entry.microseconds;
            }
            return static_cast<double>(total) / 1000.0;
        }

        [[nodiscard]] bool empty() const noexcept
        {
            for (const StageStats &entry : stats_)
            {
                if (entry.calls > 0)
                {
                    return false;
                }
            }
            return true;
        }

        /// CSV 表头：列序与 @c toCsv 严格一致
        [[nodiscard]] static std::string csvHeader() { return "stage,total_ms,mean_ms,calls\n"; }

        /**
         * @brief 每阶段一行 CSV
         *
         * 每阶段单占一行而不是横向铺开：阶段数将来会变，纵向追加不会打破
         * 已有脚本的列下标。
         */
        [[nodiscard]] std::string toCsv() const
        {
            std::string out;
            for (std::size_t i = 0; i < stats_.size(); ++i)
            {
                const StageStats &entry = stats_[i];
                if (entry.calls == 0)
                {
                    continue;
                }
                out += std::format("{},{:.3f},{:.4f},{}\n", timingStageName(static_cast<TimingStage>(i)),
                                   entry.totalMilliseconds(), entry.meanMilliseconds(), entry.calls);
            }
            return out;
        }

        /// 一行汇总日志，供终端阅读
        [[nodiscard]] std::string summary() const
        {
            std::string out;
            const double total = totalMilliseconds();
            for (std::size_t i = 0; i < stats_.size(); ++i)
            {
                const StageStats &entry = stats_[i];
                if (entry.calls == 0)
                {
                    continue;
                }
                out += std::format("{}: {:.1f} ms/{}次", timingStageName(static_cast<TimingStage>(i)),
                                   entry.meanMilliseconds(), entry.calls);
                if (total > 0.0)
                {
                    out += std::format(" ({:.1f}%)", entry.totalMilliseconds() / total * 100.0);
                }
                out += "  ";
            }
            return out;
        }

        /// 合并另一份统计（多段回放时把每段的耗时累加起来）
        void merge(const StageTimings &other) noexcept
        {
            for (std::size_t i = 0; i < stats_.size(); ++i)
            {
                stats_[i].microseconds += other.stats_[i].microseconds;
                stats_[i].calls += other.stats_[i].calls;
            }
        }

        void reset() noexcept { stats_ = {}; }

    private:
        std::array<StageStats, static_cast<std::size_t>(TimingStage::kCount)> stats_{};
    };

} // namespace myslam::domain
