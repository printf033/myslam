#pragma once

/**
 * @file camera_intrinsics.hpp
 * @brief 统一的相机内参描述：同一套数据既能表达针孔相机，也能表达 MEI 鱼眼相机。
 *
 * 全项目只有这一份相机内参定义。它属于领域层：配置解析、数据集读取、
 * 前端去畸变与后端重投影共用同一份结构，避免「后端针孔内参」与「前端
 * 标定」各存一份、靠调用点手工同步。
 *
 * @par 为什么同时保留投影参数与虚拟针孔参数
 *
 * MEI 相机可用两种模型消费：
 *   - @c MeiCameraModel：真实 MEI 投影（全向视场，供支持它的后端）；
 *   - @c VirtualPinholeModel：把视场收窄到等效针孔（@c pinholeScale），
 *     供只接受针孔重投影的后端（如 gtsam 的 @c Cal3_S2 因子）。
 *
 * 因此本结构同时描述这两组量，由装配层按估计器能力选择模型。调用方不再
 * 自己乘 @c pinholeScale —— 那是模型的选择，而不是内参的算术。
 *
 * 约束：本文件只依赖标准库。
 */

#include <cstdint>
#include <optional>
#include <string_view>

namespace myslam::camera
{

    /**
     * @brief 支持的相机投影模型
     */
    enum class CameraModel : std::uint8_t
    {
        PINHOLE, ///< 针孔模型：只用 fx / fy / cx / cy
        MEI,     ///< Unified (MEI) 全向模型：额外用 xi、径向/切向畸变与虚拟针孔缩放
    };

    /**
     * @brief 相机内参（针孔 / MEI 共用）
     *
     * 默认值是 640x480、fx=fy=460 的针孔仿真内参。真实标定必须显式给出，
     * 不要依赖默认值。
     */
    struct CameraIntrinsics
    {
        CameraModel model{CameraModel::PINHOLE};

        /// 投影参数：针孔下即 fx / fy / cx / cy；MEI 下为球面投影参数
        double fx{460.0};
        double fy{460.0};
        double cx{320.0};
        double cy{240.0};

        /// MEI 专有：镜面参数与径向 (k1, k2)、切向 (p1, p2) 畸变；针孔模型下忽略
        double xi{0.0};
        double k1{0.0};
        double k2{0.0};
        double p1{0.0};
        double p2{0.0};

        /// MEI 虚拟针孔焦距缩放；针孔模型下恒按 1.0 处理
        double pinholeScale{1.0};

        int imageWidth{640};
        int imageHeight{480};

        [[nodiscard]] bool isMei() const noexcept { return model == CameraModel::MEI; }

        /// 等效虚拟针孔缩放：针孔模型为 1，MEI 模型为 pinholeScale
        [[nodiscard]] double virtualPinholeScale() const noexcept { return isMei() ? pinholeScale : 1.0; }

        [[nodiscard]] double pinholeFx() const noexcept { return fx * virtualPinholeScale(); }
        [[nodiscard]] double pinholeFy() const noexcept { return fy * virtualPinholeScale(); }
        [[nodiscard]] double pinholeCx() const noexcept { return cx; }
        [[nodiscard]] double pinholeCy() const noexcept { return cy; }

        /**
         * @brief 几何有效性检查
         *
         * 只校验「必须为正」的量：焦距、画幅、以及 MEI 的虚拟针孔缩放。
         * cx/cy 允许落在画幅之外（弱标定与裁剪后的序列都可能如此），
         * 畸变系数可以是任意实数，因此不在这里判。
         */
        [[nodiscard]] bool isValid() const noexcept
        {
            return fx > 0.0 && fy > 0.0 && imageWidth > 0 && imageHeight > 0 && (!isMei() || pinholeScale > 0.0);
        }

        /**
         * @brief 把与模型相悖的参数归一化，使调用方无需按模型分支
         *
         * 针孔模型下 @c xi 必须为 0，否则 MEI 公式会把边缘像素当作鱼眼反解。
         * @c k1 / @c k2 / @c p1 / @c p2 不清零：对「带径向/切向畸变的针孔相机」
         * 它们是合法参数。
         *
         * @return true 表示确实改动了取值（调用方可据此提示使用者）
         */
        [[nodiscard]] bool normalizeForModel() noexcept
        {
            if (isMei() || xi == 0.0)
            {
                return false;
            }
            xi = 0.0;
            return true;
        }

        /**
         * @brief SubT Handheld1 的 MEI 标定（与 intrinsics_subt_handheld.yaml 一致）
         *
         * 单独做成工厂函数而不是结构体默认值：默认值属于「某个数据集」，
         * 不适合成为全项目通用的内参默认。
         */
        [[nodiscard]] static CameraIntrinsics subtHandheld1Mei() noexcept
        {
            CameraIntrinsics intrinsics{};
            intrinsics.model = CameraModel::MEI;
            intrinsics.fx = 758.3153257832925;
            intrinsics.fy = 676.6492212772476;
            intrinsics.cx = 318.27111164892506;
            intrinsics.cy = 239.79816832491474;
            intrinsics.xi = 1.583106303248484;
            intrinsics.k1 = -0.059098218173967695;
            intrinsics.k2 = 0.1793477408661115;
            intrinsics.p1 = 0.0016819528105368057;
            intrinsics.p2 = -0.0005887999624264534;
            intrinsics.pinholeScale = 0.4;
            intrinsics.imageWidth = 640;
            intrinsics.imageHeight = 480;
            return intrinsics;
        }
    };

    /// 模型名（配置文件与诊断日志共用）
    [[nodiscard]] constexpr std::string_view cameraModelName(CameraModel model) noexcept
    {
        return model == CameraModel::MEI ? "mei" : "pinhole";
    }

    /// 由配置文本解析模型名；大小写不敏感，未知取值返回 @c std::nullopt
    [[nodiscard]] constexpr std::optional<CameraModel> parseCameraModel(std::string_view text) noexcept
    {
        auto equalsIgnoreCase = [](std::string_view lhs, std::string_view rhs) constexpr
        {
            if (lhs.size() != rhs.size())
            {
                return false;
            }
            for (std::size_t i = 0; i < lhs.size(); ++i)
            {
                const char c = lhs[i];
                const char lower = (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
                if (lower != rhs[i])
                {
                    return false;
                }
            }
            return true;
        };
        if (equalsIgnoreCase(text, "pinhole"))
        {
            return CameraModel::PINHOLE;
        }
        if (equalsIgnoreCase(text, "mei"))
        {
            return CameraModel::MEI;
        }
        return std::nullopt;
    }

} // namespace myslam::camera
