#pragma once

/**
 * @file camera_model.hpp
 * @brief 相机投影模型端口：像素 ↔ 相机系射线的唯一权威实现。
 *
 * @par 为什么合并成一个对象
 *
 * 重构前同一套相机数学散在三处：@c camera_model.hpp 负责焦距缩放，
 * @c mei_tracker.hpp 负责 MEI 反投影，@c projection_model.hpp 负责 gtsam
 * 标定。三者各自假设「后端是针孔」，任何一处改动都要在三处同步。
 *
 * 现在：模型自己回答投影与反投影，前端用源模型把原始像素变成射线、再用
 * 目标模型把射线变成观测像素；后端只拿到像素与它信任的模型。谁也不再
 * 假设视场缩放常数。
 *
 * @par 与 gtsam 的关系
 *
 * 本层完全不认识 gtsam。适配器通过能力接口 @c IPinholeCalibration 向模型
 * 索取针孔参数（再由适配器自己构造 @c Cal3_S2）；不提供该能力的模型
 * （例如真实 MEI）会在装配期被明确拒绝，而不是在运行期产生错误的雅可比。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/types.hpp"
#include <cmath>
#include <memory>
#include <optional>
#include <string_view>

namespace myslam::domain
{

    /**
     * @brief 相机投影模型
     *
     * 所有坐标都在相机坐标系内，与机身/世界系无关：外参由调用方先用
     * @c Pose::transformTo 处理，模型不持有任何位姿。
     */
    class ICameraModel
    {
    public:
        virtual ~ICameraModel() = default;

        /// 模型名（诊断日志与配置回显用）
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;

        /**
         * @brief 相机系点 -> 像素
         *
         * @return 像素坐标；点在相机后方或投影不可逆时返回 @c std::nullopt。
         *         用 @c std::optional 而不是抛异常：门控逻辑本来就要把
         *         「点跑到相机后面」当成常态分支处理。
         */
        [[nodiscard]] virtual std::optional<Vec2> project(const Vec3 &pointCamera) const = 0;

        /// 像素 -> 相机系归一化射线（方向向量，长度不保证为 1）
        [[nodiscard]] virtual std::optional<Vec3> unproject(const Vec2 &uv) const = 0;

        /// 等效焦距（像素/弧度）：用于把像素阈值换算成角度阈值
        [[nodiscard]] virtual double focalLengthPx() const noexcept = 0;

        [[nodiscard]] virtual Vec2 principalPoint() const noexcept = 0;
        [[nodiscard]] virtual int imageWidth() const noexcept = 0;
        [[nodiscard]] virtual int imageHeight() const noexcept = 0;
    };

    /**
     * @brief 可选能力：向需要针孔标定的适配器（gtsam @c Cal3_S2）暴露参数
     *
     * 能力查询而不是具体类型：适配器 @c dynamic_cast 到本接口即可，不必认识
     * @c PinholeCameraModel 这个具体类。真实 MEI 模型不实现它，于是「把 MEI
     * 直接喂给只认针孔的优化器」会在装配期被明确拒绝。
     */
    class IPinholeCalibration
    {
    public:
        virtual ~IPinholeCalibration() = default;

        [[nodiscard]] virtual double fx() const noexcept = 0;
        [[nodiscard]] virtual double fy() const noexcept = 0;
        [[nodiscard]] virtual Vec2 principalPoint() const noexcept = 0;
    };

    /**
     * @brief 针孔模型（也用于 MEI 去畸变后的等效虚拟针孔）
     *
     * @warning **本模型不实现 radtan 畸变，要求输入图像已预矫正。**
     * @c CameraIntrinsics 里的 @c k1 / @c k2 / @c p1 / @c p2 在这里**不参与
     * 任何几何计算**，@c project 与 @c unproject 都是纯针孔映射。
     *
     * 带畸变的针孔相机必须先矫正（例如 TUM 路径的 @c initUndistortRectifyMap
     * + @c remap），再交给本模型；否则图像边缘的重投影误差可达几十像素，
     * 而且不会报任何错——前端会把畸变的像素当成理想针孔像素使用。
     *
     * 需要直接消费未矫正图像并自己处理畸变时，用 @c MeiCameraModel。
     */
    class PinholeCameraModel final : public ICameraModel, public IPinholeCalibration
    {
    public:
        explicit PinholeCameraModel(const camera::CameraIntrinsics &intrinsics)
            : fx_(intrinsics.pinholeFx()),
              fy_(intrinsics.pinholeFy()),
              cx_(intrinsics.pinholeCx()),
              cy_(intrinsics.pinholeCy()),
              width_(intrinsics.imageWidth),
              height_(intrinsics.imageHeight),
              virtual_(intrinsics.isMei())
        {
        }

        [[nodiscard]] std::string_view name() const noexcept override
        {
            return virtual_ ? "pinhole(virtual, de-warped)" : "pinhole";
        }

        [[nodiscard]] std::optional<Vec2> project(const Vec3 &pointCamera) const override
        {
            const double z = pointCamera.z();
            if (!(z > 1e-9) || !pointCamera.allFinite())
            {
                return std::nullopt;
            }
            const Vec2 uv(fx_ * pointCamera.x() / z + cx_, fy_ * pointCamera.y() / z + cy_);
            if (!uv.allFinite())
            {
                return std::nullopt;
            }
            return uv;
        }

        [[nodiscard]] std::optional<Vec3> unproject(const Vec2 &uv) const override
        {
            // 与 MeiCameraModel 对称：契约说「投影不可逆时返回 nullopt」，
            // 而 fx/fy 为 0 或像素含 NaN 时这里会产出 inf/NaN，且 has_value()
            // 仍为真——调用方（如前端的 mapPixel）只判 !ray，毒值会继续传播。
            if (!uv.allFinite() || !(fx_ > 0.0) || !(fy_ > 0.0))
            {
                return std::nullopt;
            }
            const Vec3 ray((uv.x() - cx_) / fx_, (uv.y() - cy_) / fy_, 1.0);
            if (!ray.allFinite())
            {
                return std::nullopt;
            }
            return ray;
        }

        [[nodiscard]] double focalLengthPx() const noexcept override { return 0.5 * (fx_ + fy_); }
        [[nodiscard]] Vec2 principalPoint() const noexcept override { return Vec2(cx_, cy_); }
        [[nodiscard]] int imageWidth() const noexcept override { return width_; }
        [[nodiscard]] int imageHeight() const noexcept override { return height_; }

        [[nodiscard]] double fx() const noexcept override { return fx_; }
        [[nodiscard]] double fy() const noexcept override { return fy_; }

    private:
        double fx_;
        double fy_;
        double cx_;
        double cy_;
        int width_;
        int height_;
        bool virtual_;
    };

    /**
     * @brief MEI（Mei & Rives）全向模型，含径向/切向畸变
     *
     * 正向：射线 -> 球面归一化 -> radtan 畸变 -> 像素。
     * 反向：像素 -> 迭代去畸变 -> 球面归一化 -> 射线。
     *
     * 供直接支持全向重投影的后端使用。只接受针孔标定的适配器（例如 gtsam
     * 的 @c Cal3_S2 因子）不会接受本模型，装配期即可发现。
     */
    class MeiCameraModel final : public ICameraModel
    {
    public:
        explicit MeiCameraModel(const camera::CameraIntrinsics &intrinsics)
            : fx_(intrinsics.fx),
              fy_(intrinsics.fy),
              cx_(intrinsics.cx),
              cy_(intrinsics.cy),
              xi_(intrinsics.xi),
              k1_(intrinsics.k1),
              k2_(intrinsics.k2),
              p1_(intrinsics.p1),
              p2_(intrinsics.p2),
              width_(intrinsics.imageWidth),
              height_(intrinsics.imageHeight)
        {
        }

        [[nodiscard]] std::string_view name() const noexcept override { return "mei"; }

        [[nodiscard]] std::optional<Vec2> project(const Vec3 &pointCamera) const override
        {
            // 无穷远输入会让 norm 与 denom 同时变成 inf，x/denom 给 0，
            // 于是返回 (cx, cy) 一个「合法」像素——把「点不可投影」的信息
            // 完全抹掉。退化三角化的深度很容易到 inf，必须在这里拦住。
            if (!pointCamera.allFinite())
            {
                return std::nullopt;
            }
            const double norm = pointCamera.norm();
            if (!(norm > 1e-9))
            {
                return std::nullopt;
            }
            // 球面投影：denom = xi * |P| + z。denom <= 0 表示射线超出视场。
            const double denom = xi_ * norm + pointCamera.z();
            if (!(denom > 1e-9))
            {
                return std::nullopt;
            }
            double mx = pointCamera.x() / denom;
            double my = pointCamera.y() / denom;

            const double r2 = mx * mx + my * my;
            const double radial = 1.0 + k1_ * r2 + k2_ * r2 * r2;
            const double dx = 2.0 * p1_ * mx * my + p2_ * (r2 + 2.0 * mx * mx);
            const double dy = p1_ * (r2 + 2.0 * my * my) + 2.0 * p2_ * mx * my;
            mx = mx * radial + dx;
            my = my * radial + dy;

            const Vec2 uv(fx_ * mx + cx_, fy_ * my + cy_);
            if (!std::isfinite(uv.x()) || !std::isfinite(uv.y()))
            {
                return std::nullopt;
            }
            return uv;
        }

        [[nodiscard]] std::optional<Vec3> unproject(const Vec2 &uv) const override
        {
            const double xd = (uv.x() - cx_) / fx_;
            const double yd = (uv.y() - cy_) / fy_;
            if (!std::isfinite(xd) || !std::isfinite(yd))
            {
                return std::nullopt;
            }

            double x = xd;
            double y = yd;
            for (int iteration = 0; iteration < 10; ++iteration)
            {
                const double r2 = x * x + y * y;
                const double radial = 1.0 + k1_ * r2 + k2_ * r2 * r2;
                const double dx = 2.0 * p1_ * x * y + p2_ * (r2 + 2.0 * x * x);
                const double dy = p1_ * (r2 + 2.0 * y * y) + 2.0 * p2_ * x * y;
                const double ex = x * radial + dx - xd;
                const double ey = y * radial + dy - yd;

                // 2x2 雅可比（radtan 导数）
                const double j00 = radial + 2.0 * k1_ * x * x + 4.0 * k2_ * r2 * x * x + 2.0 * p1_ * y + 6.0 * p2_ * x;
                const double j01 = 2.0 * k1_ * x * y + 4.0 * k2_ * r2 * x * y + 2.0 * p1_ * x + 2.0 * p2_ * y;
                const double j10 = j01;
                const double j11 = radial + 2.0 * k1_ * y * y + 4.0 * k2_ * r2 * y * y + 6.0 * p1_ * y + 2.0 * p2_ * x;

                const double det = j00 * j11 - j01 * j10;
                if (std::abs(det) < 1e-12)
                {
                    break;
                }
                x -= (j11 * ex - j01 * ey) / det;
                y -= (-j10 * ex + j00 * ey) / det;
                if (std::abs(ex) + std::abs(ey) < 1e-12)
                {
                    break;
                }
            }

            const double r2 = x * x + y * y;
            const double d = std::sqrt(std::max(0.0, 1.0 + (1.0 - xi_ * xi_) * r2));
            const double scale = (xi_ + d) / (1.0 + r2);
            const Vec3 ray(scale * x, scale * y, scale - xi_);
            if (!ray.allFinite())
            {
                return std::nullopt;
            }
            return ray;
        }

        [[nodiscard]] double focalLengthPx() const noexcept override { return 0.5 * (fx_ + fy_); }
        [[nodiscard]] Vec2 principalPoint() const noexcept override { return Vec2(cx_, cy_); }
        [[nodiscard]] int imageWidth() const noexcept override { return width_; }
        [[nodiscard]] int imageHeight() const noexcept override { return height_; }

    private:
        double fx_;
        double fy_;
        double cx_;
        double cy_;
        double xi_;
        double k1_;
        double k2_;
        double p1_;
        double p2_;
        int width_;
        int height_;
    };

    /**
     * @brief 由内参构造与相机种类一致的模型
     *
     * 这是领域层唯一按相机种类分支的地方。前端与后端都只拿接口。
     */
    [[nodiscard]] inline std::shared_ptr<const ICameraModel> makeCameraModel(const camera::CameraIntrinsics &intrinsics)
    {
        if (intrinsics.isMei())
        {
            return std::make_shared<MeiCameraModel>(intrinsics);
        }
        return std::make_shared<PinholeCameraModel>(intrinsics);
    }

    /**
     * @brief 构造 MEI 去畸变后的等效虚拟针孔模型
     *
     * 供只接受针孔重投影的后端使用；也用于前端把观测落到虚拟针孔像素。
     * 对针孔内参它直接返回普通针孔模型。
     */
    [[nodiscard]] inline std::shared_ptr<const ICameraModel>
    makeVirtualPinholeModel(const camera::CameraIntrinsics &intrinsics)
    {
        return std::make_shared<PinholeCameraModel>(intrinsics);
    }

} // namespace myslam::domain
