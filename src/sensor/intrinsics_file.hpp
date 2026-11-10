#pragma once

/**
 * @file intrinsics_file.hpp
 * @brief 读取由外部工具（如 MASt3R）估计出的相机内参文件。
 *
 * @c 2_subt_dataset 的内参默认由 yaml/配置/命令行给出。没有标定数据的序列
 * 可以改由 MASt3R 从图像估计，结果写成下面这种 `key = value` 文本后被本文件
 * 解析。格式刻意保持极简：每行一个字段，`#` 起始为注释，未知键被忽略并计入
 * 告警，缺少必需字段直接失败——宁可报错，也不要静默用错内参跑完整条序列。
 *
 * @code
 * # MASt3R pinhole intrinsics
 * model = pinhole
 * source = mast3r
 * focal = 408.02
 * width = 640
 * height = 480
 * @endcode
 *
 * @par focal 与 fx 的区别
 *
 * 这里区分两个容易混淆的量：
 *
 * - @c focal 是**后端虚拟针孔焦距**，即最终写入相机配置的
 *   `pinholeFx() = fx * pinholeScale`。MASt3R 这类工具估计出的正是这个量，
 *   在针孔假设下从点图直接拟合得到。
 * - `fx`/`fy` 是 **MEI 投影参数**，只在反解畸变时使用，随 `xi` 一起定义鱼眼
 *   的投影曲线。
 *
 * 两者不是一回事：Handheld1 的 `fx = 758.3`、`xi = 1.583`，其近轴等效针孔
 * 焦距只有 `fx / (1 + xi) ≈ 294`。把估计值直接当成 `fx` 会同时破坏畸变反解，
 * 让整幅鱼眼图被当作针孔图处理——这正是早期实现把特征点堆到镜头圆形边框上
 * 的原因。
 *
 * 因此本文件只接受 @c focal，并通过 `pinholeScale = focal / fx` 把它落到
 * 虚拟针孔上；`xi`、`k1`、`k2`、`p1`、`p2` 默认从调用方传入的基线标定继承，
 * 仅在文件显式给出时才覆盖。这样"换一个没标定的序列"只需要估焦距，
 * 鱼眼几何仍由标定值保证。
 *
 * 只依赖标准库，与 @c common/dataset_config.hpp 一致：本文件不引入 OpenCV。
 */

#include "common/dataset_config.hpp"
#include <charconv>
#include <cmath>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 一次内参估计的结果
     */
    struct IntrinsicsEstimate
    {
        MeiIntrinsics intrinsics;          ///< 可直接交给回放流水线的内参
        std::string source;                ///< 数据来源标记（例如 "mast3r"）
        std::string model;                 ///< 相机模型（pinhole / mei）
        double focalPx{0.0};               ///< 文件给出的虚拟针孔焦距
        bool inheritedDistortion{false};   ///< 畸变参数是否继承自基线标定
        std::vector<std::string> warnings; ///< 解析过程中的非致命问题
    };

    namespace detail
    {

        /// 去除首尾空白
        inline std::string_view trimView(std::string_view text) noexcept
        {
            const auto first = text.find_first_not_of(" \t\r\n");
            if (first == std::string_view::npos)
            {
                return {};
            }
            const auto last = text.find_last_not_of(" \t\r\n");
            return text.substr(first, last - first + 1);
        }

        /// 解析 double；失败返回 false
        inline bool parseNumber(std::string_view text, double &out) noexcept
        {
            const char *begin = text.data();
            const char *end = begin + text.size();
            const auto result = std::from_chars(begin, end, out);
            return result.ec == std::errc{} && result.ptr == end;
        }

        /// 解析 int；失败返回 false
        inline bool parseInteger(std::string_view text, int &out) noexcept
        {
            const char *begin = text.data();
            const char *end = begin + text.size();
            const auto result = std::from_chars(begin, end, out);
            return result.ec == std::errc{} && result.ptr == end;
        }

    } // namespace detail

    /**
     * @brief 解析内参估计文件
     *
     * 必需字段：`focal`。可选字段：`width`、`height`、`cx`、`cy`、`xi`、
     * `k1`、`k2`、`p1`、`p2`、`pinholeScale`、`model`、`source`。
     *
     * 未出现的字段一律保留 @p baseline 的值，因此"某个新序列的镜头不是鱼眼"
     * 这类差异应当通过配置文件的 `dataset.intrinsics` 表达，而不是在这里猜。
     *
     * @param path 估计文件路径
     * @param baseline 基线标定（通常来自配置解析结果）；其 `xi` 与畸变系数
     *                 在文件未给出时被继承
     * @return 成功时给出内参与来源标记；失败时给出面向使用者的说明
     */
    inline std::expected<IntrinsicsEstimate, std::string>
    loadIntrinsicsFile(const std::filesystem::path &path, const MeiIntrinsics &baseline = subtHandheld1Intrinsics())
    {
        std::ifstream file(path);
        if (!file)
        {
            return std::unexpected(std::format("无法打开内参文件: {}", path.string()));
        }

        IntrinsicsEstimate estimate{};

        double focal = std::nan("");
        double cx = std::nan("");
        double cy = std::nan("");
        double xi = std::nan("");
        double k1 = std::nan("");
        double k2 = std::nan("");
        double p1 = std::nan("");
        double p2 = std::nan("");
        double scale = std::nan("");
        int width = 0;
        int height = 0;
        bool sawModel = false;

        std::string line;
        std::size_t lineNumber = 0;
        while (std::getline(file, line))
        {
            ++lineNumber;
            std::string_view view = detail::trimView(line);
            if (view.empty() || view.front() == '#')
            {
                continue;
            }
            const auto separator = view.find('=');
            if (separator == std::string_view::npos)
            {
                estimate.warnings.push_back(std::format("第 {} 行缺少 '='，已跳过", lineNumber));
                continue;
            }
            const std::string_view key = detail::trimView(view.substr(0, separator));
            const std::string_view value = detail::trimView(view.substr(separator + 1));

            auto readDouble = [&](double &slot) -> bool
            {
                double parsed = 0.0;
                if (!detail::parseNumber(value, parsed) || !std::isfinite(parsed))
                {
                    return false;
                }
                slot = parsed;
                return true;
            };

            if (key == "fx" || key == "fy")
            {
                // focal 是可移植的量，fx/fy 属于相机模型的投影参数，随 xi 变化；
                // 把它当焦距混用会让鱼眼被当成针孔处理（早期实现就踩过这个坑），
                // 因此两种模型下都只接受 focal。
                return std::unexpected(std::format(
                    "第 {} 行使用了 {}：本文件只接受虚拟针孔焦距 focal（{} 是相机投影参数，不是估计工具的输出）",
                    lineNumber, key, key));
            }
            if (key == "focal" || key == "cx" || key == "cy" || key == "xi" || key == "k1" || key == "k2" ||
                key == "p1" || key == "p2" || key == "pinholeScale")
            {
                double parsed = 0.0;
                if (!detail::parseNumber(value, parsed) || !std::isfinite(parsed))
                {
                    return std::unexpected(
                        std::format("第 {} 行字段 {} 的取值非法: {}", lineNumber, key, std::string(value)));
                }
                if (key == "focal")
                {
                    focal = parsed;
                }
                else if (key == "cx")
                {
                    cx = parsed;
                }
                else if (key == "cy")
                {
                    cy = parsed;
                }
                else if (key == "xi")
                {
                    xi = parsed;
                }
                else if (key == "k1")
                {
                    k1 = parsed;
                }
                else if (key == "k2")
                {
                    k2 = parsed;
                }
                else if (key == "p1")
                {
                    p1 = parsed;
                }
                else if (key == "p2")
                {
                    p2 = parsed;
                }
                else
                {
                    scale = parsed;
                }
            }
            else if (key == "width" || key == "height")
            {
                int parsed = 0;
                if (!detail::parseInteger(value, parsed) || parsed <= 0)
                {
                    return std::unexpected(
                        std::format("第 {} 行字段 {} 的取值非法: {}", lineNumber, key, std::string(value)));
                }
                if (key == "width")
                {
                    width = parsed;
                }
                else
                {
                    height = parsed;
                }
            }
            else if (key == "model")
            {
                sawModel = true;
                estimate.model = std::string(value);
            }
            else if (key == "source")
            {
                estimate.source = std::string(value);
            }
            else
            {
                // 估计工具可能追加新字段（样本数、耗时等），未知键不致命
                estimate.warnings.push_back(std::format("第 {} 行未知字段 {}，已忽略", lineNumber, key));
            }
        }

        if (!(focal > 0.0))
        {
            return std::unexpected(std::format("内参文件缺少有效的 focal（虚拟针孔焦距）: {}", path.string()));
        }
        if (!std::isfinite(scale) && !std::isfinite(baseline.fx))
        {
            return std::unexpected(std::format("既未给出 pinholeScale，基线也没有有效 fx: {}", path.string()));
        }
        // 模型名统一交给 camera_model 解析，避免本文件与 Lua 配置对同一
        // 词汇表（pinhole / mei）给出不同的接受范围
        std::optional<camera::CameraModel> fileModel;
        if (sawModel)
        {
            fileModel = camera::parseCameraModel(estimate.model);
            if (!fileModel)
            {
                return std::unexpected(
                    std::format("内参文件的相机模型 '{}' 不受支持（pinhole / mei）", estimate.model));
            }
        }

        // 从基线出发，文件里出现的字段逐项覆盖
        MeiIntrinsics intrinsics = baseline;
        if (fileModel)
        {
            // 文件里的 model 描述的是「估计工具用的模型」（focal 是针孔假设下的
            // 焦距），与数据集的相机模型不是一个层面：基线是 MEI 时仍按 MEI 反解
            // 畸变，focal 只用于换算虚拟针孔缩放。因此这里只规范化记录，
            // 不改写 intrinsics.model——否则鱼眼图会被当成针孔处理。
            estimate.model = std::string(camera::cameraModelName(*fileModel));
        }
        if (width > 0)
        {
            intrinsics.imageWidth = width;
        }
        if (height > 0)
        {
            intrinsics.imageHeight = height;
        }
        estimate.inheritedDistortion =
            !std::isfinite(xi) && !std::isfinite(k1) && !std::isfinite(k2) && !std::isfinite(p1) && !std::isfinite(p2);
        if (std::isfinite(xi))
        {
            intrinsics.xi = xi;
        }
        if (std::isfinite(k1))
        {
            intrinsics.k1 = k1;
        }
        if (std::isfinite(k2))
        {
            intrinsics.k2 = k2;
        }
        if (std::isfinite(p1))
        {
            intrinsics.p1 = p1;
        }
        if (std::isfinite(p2))
        {
            intrinsics.p2 = p2;
        }

        if (!std::isfinite(cx))
        {
            cx = static_cast<double>(intrinsics.imageWidth) / 2.0;
            estimate.warnings.push_back("内参文件未给出 cx，按图像中心取值");
        }
        if (!std::isfinite(cy))
        {
            cy = static_cast<double>(intrinsics.imageHeight) / 2.0;
            estimate.warnings.push_back("内参文件未给出 cy，按图像中心取值");
        }
        if (cx <= 0.0 || cx >= static_cast<double>(intrinsics.imageWidth) || cy <= 0.0 ||
            cy >= static_cast<double>(intrinsics.imageHeight))
        {
            return std::unexpected(std::format("主点 ({:.1f}, {:.1f}) 落在画幅 {}x{} 之外", cx, cy,
                                               intrinsics.imageWidth, intrinsics.imageHeight));
        }
        intrinsics.cx = cx;
        intrinsics.cy = cy;

        // focal 落到虚拟针孔：MEI 下 `pinholeFx() == focal`，换算成缩放比；
        // 针孔模型没有独立的投影焦距，focal 直接就是 fx/fy。若文件显式给了
        // pinholeScale，它与 focal 在数学上互相覆盖，这里以 focal 为准并提示用户。
        if (std::isfinite(scale))
        {
            estimate.warnings.push_back(intrinsics.isMei()
                                            ? "内参文件同时给出 pinholeScale 与 focal，已按 focal 计算缩放"
                                            : "内参文件给出 pinholeScale，但相机模型为 pinhole，该字段被忽略");
        }
        if (intrinsics.isMei())
        {
            if (!(intrinsics.fx > 0.0) || !(intrinsics.fy > 0.0))
            {
                return std::unexpected(
                    std::format("基线内参的 fx/fy 无效，无法把 focal 换算成虚拟针孔: {}", path.string()));
            }
            intrinsics.pinholeScale = focal / intrinsics.fx;
        }
        else
        {
            // 针孔下没有「投影焦距 vs 虚拟针孔焦距」之分，focal 就是 fx。
            // 保留基线的 fy/fx 比例：非方形像素的标定不该被一个单值估计
            // 抹成各向同性。
            const double aspect = intrinsics.fx > 0.0 ? intrinsics.fy / intrinsics.fx : 1.0;
            intrinsics.fx = focal;
            intrinsics.fy = focal * aspect;
            // 针孔模型不经过 MEI 球面反投影，xi 必须为 0，否则前端会把
            // 针孔图当作鱼眼反解（同 lua_config：靠 xi = 0 退化，不做模型分支）。
            // 文件里显式给出 xi 时如实告知它不生效，不做静默丢弃。
            if (intrinsics.normalizeForModel() && std::isfinite(xi))
            {
                estimate.warnings.push_back("内参文件给出 xi，但相机模型为 pinhole，该字段不生效");
            }
            intrinsics.pinholeScale = 1.0;
        }

        if (!intrinsics.isValid())
        {
            return std::unexpected(std::format("内参文件解析结果未通过校验: {}", path.string()));
        }
        estimate.intrinsics = intrinsics;
        estimate.focalPx = focal;
        if (estimate.source.empty())
        {
            estimate.source = "file";
        }
        if (!sawModel)
        {
            estimate.model = "pinhole";
        }
        return estimate;
    }

} // namespace myslam::dataset
