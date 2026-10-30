#pragma once

#include "constraints.hpp"

namespace myslam::camera
{
    struct MeiIntrinsics
    {
        double fx{};
        double fy{};
        double cx{};
        double cy{};

        double xi{};
        double k1{};
        double k2{};
        double p1{};
        double p2{};

        double pinholeScale{};
        int imageWidth{};
        int imageHeight{};

        double pinholeFx() const { return fx * pinholeScale; }
        double pinholeFy() const { return fy * pinholeScale; }
        double pinholeCx() const { return cx; }
        double pinholeCy() const { return cy; }

        bool initialize(double fxValue, double fyValue, double cxValue, double cyValue, double xiValue, double k1Value,
                        double k2Value, double p1Value, double p2Value, double pinholeScaleValue, int imageWidthValue,
                        int imageHeightValue) noexcept
        {
            fx = fxValue;
            fy = fyValue;
            cx = cxValue;
            cy = cyValue;
            xi = xiValue;
            k1 = k1Value;
            k2 = k2Value;
            p1 = p1Value;
            p2 = p2Value;
            pinholeScale = pinholeScaleValue;
            imageWidth = imageWidthValue;
            imageHeight = imageHeightValue;
            return isValid();
        }

        bool isValid() const noexcept
        {
            return fx > 0.0 && fy > 0.0 && pinholeScale > 0.0 && imageWidth > 0 && imageHeight > 0;
        }

        void reset() noexcept { *this = MeiIntrinsics{}; }
    };

    static_assert(constraints::Initializable<MeiIntrinsics, double, double, double, double, double, double, double,
                                             double, double, double, int, int>);
    static_assert(constraints::Resettable<MeiIntrinsics>);

} // namespace myslam::camera
