#pragma once

/**
 * @file ply_export.hpp
 * @brief 把回放得到的路标点与轨迹导出为 PLY 点云，便于 MeshLab / CloudCompare / Open3D 查看。
 */

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <gtsam/geometry/Point3.h>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 写入 PLY 点云（ASCII）
     *
     * @param path      输出路径
     * @param landmarks 路标点（青色）
     * @param trajectory 轨迹关键帧位置（黄色）
     * @return 文件成功打开并写完时为 true
     */
    inline bool writePly(const std::filesystem::path &path, const std::vector<gtsam::Point3> &landmarks,
                         const std::vector<gtsam::Point3> &trajectory)
    {
        std::ofstream out(path);
        if (!out)
        {
            return false;
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
        return static_cast<bool>(out);
    }

} // namespace myslam::dataset
