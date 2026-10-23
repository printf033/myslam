#pragma once

/**
 * @file subt_dataset.hpp
 * @brief SubT-MRS 数据集读取：IMU CSV、相机时间戳与图像索引。
 *
 * 数据布局（与 MASt3R-Fusion 的 Handheld1_Folder 一致）：
 * @code
 * Handheld1_Folder/
 *   cam_0/
 *     0.png, 1.png, ...
 *     timestamps.txt   # 每行一个纳秒时间戳
 *   imu/
 *     imu_data.csv     # timestamp, q_x, q_y, q_z, q_w, ang_vel_{x,y,z}, lin_acc_{x,y,z}
 * @endcode
 *
 * 所有时间戳在内部统一换算为相对序列首帧的秒数，并先在 int64 域内相减再转
 * double，避免纳秒级绝对时间戳在 double 下丢失有效位。
 */

#include "logger.hpp"
#include "vio/types.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 一条 IMU 原始记录（时间已相对化）
     */
    struct ImuRecord
    {
        double timestamp{0.0};         // 相对秒
        gtsam::Vector3 gyro{0, 0, 0};  // 角速度 (rad/s)
        gtsam::Vector3 accel{0, 0, 0}; // 比力 (m/s^2)
    };

    /**
     * @brief SubT 序列读取器：负责加载时间戳与 IMU，并按序号定位图像
     */
    class SubtSequence
    {
    public:
        SubtSequence(std::filesystem::path cameraDir, std::filesystem::path imuCsv, std::filesystem::path stampFile)
            : cameraDir_(std::move(cameraDir)),
              imuCsv_(std::move(imuCsv)),
              stampFile_(std::move(stampFile))
        {
        }

        /**
         * @brief 加载全部元数据
         * @return 目录/文件缺失或格式错误时返回 false
         */
        bool load()
        {
            if (!std::filesystem::is_directory(cameraDir_))
            {
                mylog::error("相机图像目录不存在: {}", cameraDir_.string());
                return false;
            }
            if (!loadStamps() || !loadImu())
            {
                return false;
            }
            if (stampsNs_.empty() || imuNs_.empty())
            {
                mylog::error("时间戳或 IMU 数据为空: stamps={}, imu={}", stampsNs_.size(), imuNs_.size());
                return false;
            }

            // 相对化：int64 域先减原点，再换算到秒，避免 1.68e18 ns 的精度损失
            originNs_ = std::min(stampsNs_.front(), imuNs_.front());
            stamps_.reserve(stampsNs_.size());
            for (const std::int64_t ns : stampsNs_)
            {
                stamps_.push_back(static_cast<double>(ns - originNs_) * 1e-9);
            }
            for (std::size_t i = 0; i < imuNs_.size(); ++i)
            {
                imu_[i].timestamp = static_cast<double>(imuNs_[i] - originNs_) * 1e-9;
            }

            mylog::info("[数据加载] 图像 {} 帧, IMU {} 条, 时长 {:.2f} s", stamps_.size(), imu_.size(),
                        stamps_.back() - stamps_.front());
            mylog::info("[数据加载] IMU 平均频率 {:.1f} Hz, 图像平均频率 {:.1f} Hz", estimateImuRate(),
                        estimateCameraRate());
            if (imu_[0].timestamp > stamps_.front())
            {
                mylog::warn("[数据加载] 首条 IMU 晚于首帧图像 {:.3f} s，初始化阶段可能缺少惯性数据",
                            imu_[0].timestamp - stamps_.front());
            }
            return true;
        }

        std::size_t size() const { return stamps_.size(); }
        const std::vector<double> &stamps() const { return stamps_; }
        const std::vector<ImuRecord> &imu() const { return imu_; }

        /**
         * @brief 第 index 帧图像的路径（cam_0/<index>.png）
         */
        std::string imagePath(std::size_t index) const
        {
            return (cameraDir_ / (std::to_string(index) + ".png")).string();
        }

        double estimateImuRate() const
        {
            if (imu_.size() < 2)
            {
                return 0.0;
            }
            const double span = imu_.back().timestamp - imu_.front().timestamp;
            return span > 0.0 ? static_cast<double>(imu_.size() - 1) / span : 0.0;
        }

        double estimateCameraRate() const
        {
            if (stamps_.size() < 2)
            {
                return 0.0;
            }
            const double span = stamps_.back() - stamps_.front();
            return span > 0.0 ? static_cast<double>(stamps_.size() - 1) / span : 0.0;
        }

    private:
        bool loadStamps()
        {
            std::ifstream in(stampFile_);
            if (!in)
            {
                mylog::error("无法打开时间戳文件: {}", stampFile_.string());
                return false;
            }
            std::string line;
            while (std::getline(in, line))
            {
                if (line.empty())
                {
                    continue;
                }
                try
                {
                    stampsNs_.push_back(std::stoll(line));
                }
                catch (const std::exception &)
                {
                    mylog::error("时间戳格式错误: '{}'", line);
                    return false;
                }
            }
            return !stampsNs_.empty();
        }

        bool loadImu()
        {
            std::ifstream in(imuCsv_);
            if (!in)
            {
                mylog::error("无法打开 IMU 文件: {}", imuCsv_.string());
                return false;
            }
            std::string line;
            if (!std::getline(in, line))
            {
                mylog::error("IMU 文件为空: {}", imuCsv_.string());
                return false;
            }
            imu_.reserve(200000);
            imuNs_.reserve(200000);
            while (std::getline(in, line))
            {
                if (line.empty())
                {
                    continue;
                }
                // 列: timestamp, qx, qy, qz, qw, wx, wy, wz, ax, ay, az
                double fields[11] = {};
                if (!parseCsv(line, fields, 11))
                {
                    continue; // 跳过坏行，避免单行异常中断整段序列
                }
                imuNs_.push_back(static_cast<std::int64_t>(fields[0]));
                ImuRecord record;
                record.gyro = gtsam::Vector3(fields[5], fields[6], fields[7]);
                record.accel = gtsam::Vector3(fields[8], fields[9], fields[10]);
                imu_.push_back(record);
            }
            return !imu_.empty();
        }

        /**
         * @brief 解析固定列数的逗号分隔行
         * @return 字段数不足时返回 false
         */
        static bool parseCsv(const std::string &line, double *out, int expected)
        {
            std::size_t begin = 0;
            int count = 0;
            while (count < expected)
            {
                const std::size_t end = line.find(',', begin);
                const std::string token =
                    line.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
                try
                {
                    out[count] = std::stod(token);
                }
                catch (const std::exception &)
                {
                    return false;
                }
                ++count;
                if (end == std::string::npos)
                {
                    break;
                }
                begin = end + 1;
            }
            return count == expected;
        }

        std::filesystem::path cameraDir_;
        std::filesystem::path imuCsv_;
        std::filesystem::path stampFile_;

        std::vector<std::int64_t> stampsNs_;
        std::vector<std::int64_t> imuNs_;
        std::vector<double> stamps_;
        std::vector<ImuRecord> imu_;
        std::int64_t originNs_{0};
    };

} // namespace myslam::dataset
