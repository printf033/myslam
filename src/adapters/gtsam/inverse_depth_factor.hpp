#pragma once

/**
 * @file inverse_depth_factor.hpp
 * @brief 锚定逆深度路标的投影因子。
 *
 * @par 为什么要另一个投影因子
 *
 * @c gtsam::GenericProjectionFactor 把路标参数化成世界系 3D 点，配一个各向同性
 * 先验。在「远、慢、小基线」的观测条件下，深度的不确定性沿视线方向拉伸成
 * 一个很长的椭球，而各向同性先验会把这个椭球硬压成球——于是点被拽到一个
 * 错误的距离上，重投影残差随之变大，门控再把它当外点剔掉。
 *
 * 逆深度参数化把「深度的不确定」变成「逆深度的一个分量」，于是：
 *   - 远点（rho → 0）在数值上是良态的，不会出现 1/z 发散；
 *   - 深度的不确定性可以独立于横向位置表达，先验不必强行拉平它；
 *   - 观测欠定时，退化方向沿视线而不是沿某个笛卡尔轴。
 *
 * @par 锚定的含义
 *
 * 路标参数存在**锚点相机的归一化平面**上：@c (u, v, rho)，其中
 * @c rho = 1/depth。锚点位姿在三角化时刻固定下来，不进入优化。代价是锚点
 * 自身的误差被固化；在固定滞后窗口（秒级）里这个代价远小于收益，而窗口
 * 之外的变量本来就会被边缘化。
 *
 * @par 为什么用数值雅可比
 *
 * 解析雅可比要穿过「逆深度 → 锚点相机系 → 世界系 → 当前相机系 → 像素」
 * 四层复合，还要乘上 bodyPSensor 外参。符号写错不会报错，只会让优化在
 * 错误的方向上收敛。数值微分的代价是每个观测多算 18 次投影，在几十个
 * 观测的量级上可以接受。
 */

#include <algorithm>
#include <functional>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <memory>

namespace myslam::adapters
{

    /**
     * @brief 锚定逆深度投影因子：X(pose) × L(u, v, rho) -> 像素
     */
    class InverseDepthProjectionFactor final : public ::gtsam::NoiseModelFactor2<::gtsam::Pose3, ::gtsam::Vector3>
    {
    public:
        using Base = ::gtsam::NoiseModelFactor2<::gtsam::Pose3, ::gtsam::Vector3>;

        /**
         * @param poseKey 关键帧位姿（World ← Body）
         * @param landmarkKey 路标（u, v, rho）
         * @param measured 像素观测
         * @param model 观测噪声（可含鲁棒核）
         * @param bodyPSensor 相机相对机身的固定外参
         * @param anchorPose 锚点时刻的相机位姿（World ← Camera），固定不入优化
         * @param K 针孔标定
         */
        InverseDepthProjectionFactor(::gtsam::Key poseKey, ::gtsam::Key landmarkKey, const ::gtsam::Point2 &measured,
                                     const ::gtsam::SharedNoiseModel &model, const ::gtsam::Pose3 &bodyPSensor,
                                     const ::gtsam::Pose3 &anchorPose, ::gtsam::Cal3_S2::shared_ptr K)
            : Base(model, poseKey, landmarkKey),
              bodyPSensor_(bodyPSensor),
              anchorPose_(anchorPose),
              K_(std::move(K)),
              measured_(measured)
        {
        }

        [[nodiscard]] ::gtsam::NonlinearFactor::shared_ptr clone() const override
        {
            return std::make_shared<InverseDepthProjectionFactor>(*this);
        }

        /**
         * @brief 把逆深度参数还原成世界系点（供门控与导出使用）
         *
         * 与 evaluateError 用同一套公式，因此门控看到的点就是优化器看到的点。
         */
        [[nodiscard]] ::gtsam::Point3 worldPoint(const ::gtsam::Vector3 &inverseDepth) const
        {
            const double rho = std::max(inverseDepth(2), kMinInverseDepth);
            const ::gtsam::Point3 inAnchor(inverseDepth(0) / rho, inverseDepth(1) / rho, 1.0 / rho);
            return anchorPose_.transformFrom(inAnchor);
        }

        [[nodiscard]] ::gtsam::Vector evaluateError(const ::gtsam::Pose3 &tWb, const ::gtsam::Vector3 &inverseDepth,
                                                    ::gtsam::OptionalMatrixType H1 = nullptr,
                                                    ::gtsam::OptionalMatrixType H2 = nullptr) const override
        {
            const std::function<::gtsam::Vector(const ::gtsam::Pose3 &, const ::gtsam::Vector3 &)> errorFunction =
                [this](const ::gtsam::Pose3 &pose, const ::gtsam::Vector3 &landmark) -> ::gtsam::Vector
            { return this->project(pose, landmark); };

            if (H1)
            {
                *H1 = ::gtsam::numericalDerivative21(errorFunction, tWb, inverseDepth);
            }
            if (H2)
            {
                *H2 = ::gtsam::numericalDerivative22(errorFunction, tWb, inverseDepth);
            }
            return project(tWb, inverseDepth);
        }

        /// 逆深度下界：保证点在相机前方，同时让梯度在边界处保持有界
        static constexpr double kMinInverseDepth = 1e-6;

    private:
        [[nodiscard]] ::gtsam::Vector project(const ::gtsam::Pose3 &tWb, const ::gtsam::Vector3 &inverseDepth) const
        {
            const ::gtsam::Point3 ptW = worldPoint(inverseDepth);
            const ::gtsam::Pose3 tWc = tWb.compose(bodyPSensor_);
            const ::gtsam::Point3 ptC = tWc.transformTo(ptW);

            // 用 clamp 而不是提前返回大残差：大残差在数值微分下会变成常数，
            // 雅可比退化成零，优化器在这一步拿到的是「无信息」，于是原地
            // 停住而不是被推回可行域。
            const double z = std::max(ptC.z(), 1e-6);
            const ::gtsam::Vector2 predicted(K_->fx() * ptC.x() / z + K_->px(), K_->fy() * ptC.y() / z + K_->py());
            return predicted - measured_;
        }

        ::gtsam::Pose3 bodyPSensor_;
        ::gtsam::Pose3 anchorPose_;
        ::gtsam::Cal3_S2::shared_ptr K_;
        ::gtsam::Point2 measured_;
    };

} // namespace myslam::adapters
