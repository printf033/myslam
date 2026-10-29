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
 * # MASt3R pinhole intrinsics (image centre as principal point)
 * model = pinhole
 * source = mast3r
 * fx = 383.163223
 * fy = 383.163223
 * cx = 320.000000
 * cy = 240.000000
 * width = 640
 * height = 480
 * @endcode
 *
 * 只依赖标准库，与 @c dataset/configHolder.hpp 一致：本文件不引入 OpenCV。
 */

#include "dataset/configHolder.hpp"
#include <charconv>
#include <cmath>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
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
        std::string model;                 ///< 相机模型（当前只支持 "pinhole"）
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
            // std::from_chars 的浮点支持在 GCC 11+ 可用，这里直接使用
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
     * 支持字段：`fx`、`fy`、`cx`、`cy`、`width`、`height`（必需）、
     * `model`、`source`（可选）。`model` 只接受 `pinhole`：MASt3R 类工具输出
     * 的是针孔模型，本函数据此把 MEI 的 xi 与畸变系数置零、针孔缩放置一，
     * 使后续去畸变路径退化为恒等映射。
     *
     * @param path 估计文件路径
     * @return 成功时给出内参与来源标记；失败时给出面向使用者的说明
     */
    inline std::expected<IntrinsicsEstimate, std::string> loadIntrinsicsFile(const std::filesystem::path &path)
    {
        std::ifstream file(path);
        if (!file)
        {
            return std::unexpected(std::format("无法打开内参文件: {}", path.string()));
        }

        IntrinsicsEstimate estimate{};
        double fx = std::nan("");
        double fy = std::nan("");
        double cx = std::nan("");
        double cy = std::nan("");
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

            if (key == "fx" || key == "fy" || key == "cx" || key == "cy")
            {
                double parsed = 0.0;
                if (!detail::parseNumber(value, parsed) || !std::isfinite(parsed))
                {
                    return std::unexpected(
                        std::format("第 {} 行字段 {} 的取值非法: {}", lineNumber, key, std::string(value)));
                }
                if (key == "fx")
                {
                    fx = parsed;
                }
                else if (key == "fy")
                {
                    fy = parsed;
                }
                else if (key == "cx")
                {
                    cx = parsed;
                }
                else
                {
                    cy = parsed;
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

        if (!(fx > 0.0) || !(fy > 0.0))
        {
            return std::unexpected(std::format("内参文件缺少有效的 fx/fy: {}", path.string()));
        }
        if (width <= 0 || height <= 0)
        {
            return std::unexpected(std::format("内参文件缺少有效的 width/height: {}", path.string()));
        }
        if (sawModel && estimate.model != "pinhole")
        {
            return std::unexpected(std::format("内参文件的相机模型 '{}' 不受支持（仅 pinhole）", estimate.model));
        }
        if (!std::isfinite(cx))
        {
            cx = static_cast<double>(width) / 2.0;
            estimate.warnings.push_back("内参文件未给出 cx，按图像中心取值");
        }
        if (!std::isfinite(cy))
        {
            cy = static_cast<double>(height) / 2.0;
            estimate.warnings.push_back("内参文件未给出 cy，按图像中心取值");
        }
        if (cx <= 0.0 || cx >= static_cast<double>(width) || cy <= 0.0 || cy >= static_cast<double>(height))
        {
            return std::unexpected(std::format("主点 ({:.1f}, {:.1f}) 落在画幅 {}x{} 之外", cx, cy, width, height));
        }
        if (estimate.source.empty())
        {
            estimate.source = "file";
        }

        // 针孔估计：MEI 的 xi 与畸变系数置零，缩放置一，去畸变路径退化为恒等
        MeiIntrinsics intrinsics{};
        intrinsics.fx = fx;
        intrinsics.fy = fy;
        intrinsics.cx = cx;
        intrinsics.cy = cy;
        intrinsics.xi = 0.0;
        intrinsics.k1 = 0.0;
        intrinsics.k2 = 0.0;
        intrinsics.p1 = 0.0;
        intrinsics.p2 = 0.0;
        intrinsics.pinholeScale = 1.0;
        intrinsics.imageWidth = width;
        intrinsics.imageHeight = height;
        if (!intrinsics.isValid())
        {
            return std::unexpected(std::format("内参文件解析结果未通过校验: {}", path.string()));
        }
        estimate.intrinsics = intrinsics;
        if (!sawModel)
        {
            estimate.model = "pinhole";
        }
        return estimate;
    }

} // namespace myslam::dataset
