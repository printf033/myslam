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
#include "vio/params.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
     * @brief 一条 IMU 原始记录（时间已相对化）
     */
    struct ImuRecord
    {
        double timestamp{0.0};         // 相对秒
        gtsam::Vector3 gyro{0, 0, 0};  // 角速度 (rad/s)
        gtsam::Vector3 accel{0, 0, 0}; // 比力 (m/s^2)
    };

    /**
     * @brief 序列加载失败的具体原因
     */
    struct DatasetError
    {
        std::string message; ///< 面向使用者的说明
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
         * @return 成功时无值；目录/文件缺失或格式错误时返回具体原因
         */
        std::expected<void, DatasetError> load()
        {
            if (!std::filesystem::is_directory(cameraDir_))
            {
                const std::string message = std::format("相机图像目录不存在: {}", cameraDir_.string());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
            }
            if (const auto stamps = loadStamps(); !stamps)
            {
                return std::unexpected(stamps.error());
            }
            if (const auto imu = loadImu(); !imu)
            {
                return std::unexpected(imu.error());
            }
            if (stampsNs_.empty() || imuNs_.empty())
            {
                const std::string message =
                    std::format("时间戳或 IMU 数据为空: stamps={}, imu={}", stampsNs_.size(), imuNs_.size());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
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
            return {};
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
        std::expected<void, DatasetError> loadStamps()
        {
            std::ifstream in(stampFile_);
            if (!in)
            {
                const std::string message = std::format("无法打开时间戳文件: {}", stampFile_.string());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
            }
            std::string line;
            while (std::getline(in, line))
            {
                if (line.empty())
                {
                    continue;
                }
                // 与 IMU CSV 不同，时间戳是序列的时间基准，坏行不能静默跳过
                std::int64_t value = 0;
                const auto [parsed, ec] = std::from_chars(line.data(), line.data() + line.size(), value);
                if (ec != std::errc{} || parsed != line.data() + line.size())
                {
                    const std::string message = std::format("时间戳格式错误: '{}'", line);
                    mylog::error("{}", message);
                    return std::unexpected(DatasetError{message});
                }
                stampsNs_.push_back(value);
            }
            if (stampsNs_.empty())
            {
                const std::string message = std::format("时间戳文件为空: {}", stampFile_.string());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
            }
            return {};
        }

        std::expected<void, DatasetError> loadImu()
        {
            std::ifstream in(imuCsv_);
            if (!in)
            {
                const std::string message = std::format("无法打开 IMU 文件: {}", imuCsv_.string());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
            }
            std::string line;
            if (!std::getline(in, line))
            {
                const std::string message = std::format("IMU 文件为空: {}", imuCsv_.string());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
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
            if (imu_.empty())
            {
                const std::string message = std::format("IMU 文件无有效数据行: {}", imuCsv_.string());
                mylog::error("{}", message);
                return std::unexpected(DatasetError{message});
            }
            return {};
        }

        /**
         * @brief 解析固定列数的逗号分隔行
         *
         * 使用 std::from_chars 而非 std::stod：后者以异常报告解析失败，
         * 会把"跳过一行坏数据"这种常规分支变成异常路径。from_chars 不分配、
         * 不抛异常，且不依赖 locale（stod 受全局 locale 影响）。
         *
         * @param line     待解析的整行
         * @param out      输出数组，长度至少 expected
         * @param expected 期望字段数
         * @return 字段数不足或有字段无法解析为有限数值时返回 false
         */
        static bool parseCsv(std::string_view line, double *out, int expected)
        {
            std::size_t begin = 0;
            int count = 0;
            while (count < expected)
            {
                const std::size_t end = line.find(',', begin);
                const std::size_t length = (end == std::string_view::npos ? line.size() : end) - begin;
                const char *first = line.data() + begin;
                const char *last = first + length;

                double value = 0.0;
                const auto [parsed, ec] = std::from_chars(first, last, value);
                if (ec != std::errc{} || parsed != last || !std::isfinite(value))
                {
                    return false;
                }
                out[count] = value;
                ++count;

                if (end == std::string_view::npos)
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
