#pragma once

/**
 * @file place_recognition.hpp
 * @brief 位置识别端口：把「我在哪里」从「我怎么走到这里」里拆出来。
 *
 * @par 为什么单独成端口
 *
 * 重定位与回环是两个不同的问题，却共享同一条检索链：**索引**历史关键帧 →
 * 为当前帧给出**候选** → 由调用方做几何验证与融合。这条链的每一步都可以
 * 换实现（词袋、学习型描述子、几何哈希），而估计器只关心最后的结论
 * 「当前帧属于哪张地图、相对哪个历史关键帧的位姿是多少」。
 *
 * 把检索留在估计器内部会让三种完全不同的技术被焊死在一个后端里；抽成
 * 端口之后，换检索算法不改估计器，换估计器也不改检索。
 *
 * @par 为什么本层只定义契约，不提供参考实现
 *
 * 可靠的跨地图检索需要**外观描述子**：像素坐标在两张不同的地图之间没有
 * 可比性，而 @c Observation 当前只携带像素、射线与编号——编号由前端跨帧
 * 维护，重建后必然重置，不能当作跨会话的身份。
 *
 * 因此本端口的实现方必须先带来描述子（DBoW2 词汇表、学习型描述子，或
 * 第三方的匹配器）。用一个"像素直方图相似度"凑一个参考实现是可以编译
 * 通过的，但它给出的候选没有任何可验证的召回率——那比缺席更危险，因为
 * 它会诱使上层把 @c RELOCALIZE 当成已经可用。缺席必须表现为**明确失败**，
 * 而不是一个安静跑错的重定位。
 *
 * 约束：只依赖领域类型与标准库。
 */

#include "domain/types.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace myslam::domain
{

    /**
     * @brief 一个历史位置候选
     *
     * @c relativePose 是「把当前帧的点变到候选帧系」的变换
     * （@c candidate.pose.inverse() * current.pose）。实现若无法给出相对位姿，
     * 必须留空——调用方据此把候选交给独立的几何验证，而不是拿一个编造的
     * 位姿去做融合。
     *
     * 用 @c std::optional 而不是「bool + 默认构造位姿」两个字段：后者允许
     * 构造出「hasRelativePose = true，位姿却是单位阵」这种自相矛盾的状态，
     * 而没有任何检查能发现它。项目里 @c PoseCovariance 出于同一理由用了
     * @c optional（「没有」与「恰好为零」是两件事）。
     */
    struct PlaceCandidate
    {
        MapId map{0};
        KeyframeId id{0};
        /// 为空表示只是"像"，需要外部做几何验证
        std::optional<Pose> relativePose{};
        /// 相似度得分，越大越像；绝对值无跨实现意义，只用于排序
        double score{0.0};
    };

    /**
     * @brief 位置识别端口
     *
     * 实现必须以**只读**方式消费 @c Keyframe：索引可以保存观测（这是它
     * 存在的意义），但不得修改调用方传入的对象。
     */
    class IPlaceRecognizer
    {
    public:
        virtual ~IPlaceRecognizer() = default;

        [[nodiscard]] virtual std::string_view name() const noexcept = 0;

        /// 把一张地图里的关键帧加入索引；重复索引同一 `(map, id)` 返回 false
        virtual bool index(MapId map, const Keyframe &keyframe) = 0;

        /**
         * @brief 查询与当前帧最相似的历史位置
         *
         * @param keyframe 当前帧（观测必须完整，索引侧依赖它们）
         * @param maxCandidates 返回候选数上限
         * @return 按 @c score 降序的候选；无法判断时返回空，**不得**降级为
         *         "猜一个最近的"
         */
        [[nodiscard]] virtual std::vector<PlaceCandidate> query(const Keyframe &keyframe,
                                                                std::size_t maxCandidates) = 0;

        /// 清空索引（整条流水线作废时调用；跨地图保留索引才有重定位价值）
        virtual void reset() = 0;

        /// 已索引的关键帧数
        [[nodiscard]] virtual std::size_t indexedKeyframes() const = 0;
    };

} // namespace myslam::domain
