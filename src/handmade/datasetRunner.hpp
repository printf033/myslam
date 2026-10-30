#pragma once

#include "configHolder.hpp"
#include "constraints.hpp"
#include "vio/configHolder.hpp"
#include <concepts>
#include <stdexcept>
#include <utility>

namespace myslam::dataset
{
    template <typename Intrinsics = camera::MeiIntrinsics>
    class DatasetRunner
    {
    public:
        template <typename... Args>
            requires std::default_initializable<Intrinsics> && constraints::Initializable<Intrinsics, Args...>
        DatasetRunner(const vio::VioConfig &config, Args &&...args)
            : config_(config)
        {
            if (!intrinsics_.initialize(std::forward<Args>(args)...))
                throw std::invalid_argument("DatasetRunner received invalid camera intrinsics");
        }

        ~DatasetRunner()
            requires constraints::Resettable<Intrinsics>
        {
            intrinsics_.reset();
        }

    private:
        vio::VioConfig config_;
        Intrinsics intrinsics_{};
    };
} // namespace myslam::dataset
