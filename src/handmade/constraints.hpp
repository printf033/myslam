#pragma once

#include <concepts>
#include <utility>

namespace myslam::constraints
{
    template <typename T, typename... Args>
    concept Initializable = std::destructible<T> && requires(T &object, Args &&...args) {
        { object.initialize(std::forward<Args>(args)...) } noexcept -> std::convertible_to<bool>;
        { object.isValid() } noexcept -> std::convertible_to<bool>;
    };

    template <typename T>
    concept Resettable = std::destructible<T> && requires(T &object) {
        { object.reset() } noexcept -> std::same_as<void>;
        { object.isValid() } noexcept -> std::convertible_to<bool>;
    };
} // namespace myslam::constraints
