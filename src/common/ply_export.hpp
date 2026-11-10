#pragma once

/**
 * @file ply_export.hpp
 * @brief 把路标点与轨迹导出为 PLY 点云（纯标准库 + 领域几何类型，不依赖 OpenCV/求解器）。
 *
 * 放在 @c common/ 而不是 @c viz/：导出是数据落地，不是渲染。回放器需要它
 * 却不应为了写一个点云文件把 OpenCV 可视化模块一起拉进编译。
 */

#include "domain/types.hpp"
#include <cstddef>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <string>
#include <vector>

namespace myslam::io
{

    /**
     * @brief PLY 导出失败的具体原因
     */
    struct PlyError
    {
        std::string message; ///< 面向使用者的说明
    };

    /**
     * @brief 写入 PLY 点云（ASCII）
     *
     * @param path      输出路径
     * @param landmarks 路标点（青色）
     * @param trajectory 轨迹关键帧位置（黄色）
     * @return 成功时无值；无法打开文件或写入中途失败时返回具体原因
     */
    inline std::expected<void, PlyError> writePly(const std::filesystem::path &path,
                                                  const std::vector<domain::Vec3> &landmarks,
                                                  const std::vector<domain::Vec3> &trajectory)
    {
        std::ofstream out(path);
        if (!out)
        {
            return std::unexpected(PlyError{std::format("无法写入 PLY 文件: {}", path.string())});
        }

        const std::size_t total = landmarks.size() + trajectory.size();
        out << "ply\n"
            << "format ascii 1.0\n"
            << "comment myslam VIO replay export\n"
            << "element vertex " << total << "\n"
            << "property float x\n"
            << "property float y\n"
            << "property float z\n"
            << "property uchar red\n"
            << "property uchar green\n"
            << "property uchar blue\n"
            << "end_header\n";

        for (const auto &p : landmarks)
        {
            out << p.x() << " " << p.y() << " " << p.z() << " 0 220 220\n";
        }
        for (const auto &p : trajectory)
        {
            out << p.x() << " " << p.y() << " " << p.z() << " 255 200 0\n";
        }
        if (!out)
        {
            return std::unexpected(PlyError{std::format("写入 PLY 文件时发生错误: {}", path.string())});
        }
        return {};
    }

} // namespace myslam::io
