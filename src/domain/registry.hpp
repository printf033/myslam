#pragma once

/**
 * @file registry.hpp
 * @brief 估计器注册表：按名字选择实现，未知名字明确失败。
 *
 * @par 为什么需要它
 *
 * 旧代码里「选择哪个后端」不存在：@c VioPipeline 的构造函数直接
 * @c make_unique<VioBackend>，换实现必须改流水线源码。注册表把选择变成
 * 运行期配置（@c estimator.name），也让同一份序列可以跑两个后端做 A/B ——
 * 这是「可替换」唯一可信的验收方式。
 *
 * @par 不做动态插件
 *
 * 当前规模下共享库加载带来 ABI、搜索路径与生命周期问题，收益为零。注册
 * 发生在编译期（@c registerBuiltinEstimators），键是字符串。
 *
 * 约束：本文件只依赖领域层与标准库。
 */

#include "domain/ports.hpp"
#include <expected>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace myslam::domain
{

    /**
     * @brief 估计器工厂注册表（进程级单例）
     */
    class EstimatorRegistry
    {
    public:
        [[nodiscard]] static EstimatorRegistry &instance()
        {
            static EstimatorRegistry registry;
            return registry;
        }

        /// 注册实现；重名视为配置错误，保留先注册者并返回 false
        bool add(std::string name, EstimatorFactory factory)
        {
            if (name.empty() || !factory)
            {
                return false;
            }
            std::scoped_lock lock(mutex_);
            return factories_.emplace(std::move(name), std::move(factory)).second;
        }

        /// 按名字创建；未知名字返回错误文本（不回退到任何默认实现）
        [[nodiscard]] std::expected<EstimatorBundle, std::string> create(std::string_view name,
                                                                         const PipelineConfig &config) const
        {
            EstimatorFactory factory;
            {
                std::scoped_lock lock(mutex_);
                const auto it = factories_.find(std::string(name));
                if (it == factories_.end())
                {
                    return std::unexpected("未知估计器 '" + std::string(name) + "'；已注册: " + joinNames());
                }
                factory = it->second;
            }
            auto bundle = factory(config);
            if (!bundle)
            {
                return std::unexpected("估计器 '" + std::string(name) + "' 创建失败: " + bundle.error());
            }
            if (!bundle->estimator || !bundle->observationModel)
            {
                return std::unexpected("估计器 '" + std::string(name) + "' 返回了不完整的工厂结果");
            }
            return bundle;
        }

        [[nodiscard]] bool contains(std::string_view name) const
        {
            std::scoped_lock lock(mutex_);
            return factories_.contains(std::string(name));
        }

        [[nodiscard]] std::vector<std::string> names() const
        {
            std::scoped_lock lock(mutex_);
            std::vector<std::string> out;
            out.reserve(factories_.size());
            for (const auto &[name, factory] : factories_)
            {
                (void)factory;
                out.push_back(name);
            }
            return out;
        }

    private:
        EstimatorRegistry() = default;

        [[nodiscard]] std::string joinNames() const
        {
            std::string joined;
            for (const auto &[name, factory] : factories_)
            {
                (void)factory;
                if (!joined.empty())
                {
                    joined += ", ";
                }
                joined += name;
            }
            return joined.empty() ? "(空)" : joined;
        }

        mutable std::mutex mutex_;
        std::map<std::string, EstimatorFactory> factories_;
    };

} // namespace myslam::domain
