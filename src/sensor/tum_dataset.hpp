#pragma once

/**
 * @file tum_dataset.hpp
 * @brief TUM RGB-D 数据集读取：图像索引、真值轨迹与 IMU 构造。
 *
 * 数据布局（TUM RGB-D Benchmark，freiburg1/2/3 通用）：
 * @code
 * rgbd_dataset_freiburg1_desk/
 *   rgb.txt            # timestamp filename   (相对 root)
 *   rgb/<ts>.png
 *   depth.txt          # timestamp filename
 *   depth/<ts>.png
 *   groundtruth.txt    # timestamp tx ty tz qx qy qz qw
 *   accelerometer.txt  # timestamp ax ay az
 * @endcode
 *
 * @par 没有陀螺仪
 *
 * TUM RGB-D 由 Kinect 采集，**只有加速度计，没有陀螺仪**。VIO 预积分必须
 * 有角速度，因此本读取器从真值姿态差分合成 gyro（见 @c buildImu）。这是
 * 一个明确的取舍：合成角速度意味着旋转信息来自真值，回放结果验证的是
 * 「位置/视觉融合链路」，不能当作标准 VIO 的精度结论。加速度仍取
 * @c accelerometer.txt 的真实测量，保留其噪声与偏置特性。
 *
 * @par 真值
 *
 * @c groundtruth.txt 是运动捕捉系统给出的参考轨迹（100 Hz），可用于 ATE
 * 评估——这是 rosbag 系列所不具备的。
 *
 * 所有时间戳统一相对序列首样本的秒数，先在 int64 纳秒域相减再转 double，
 * 避免 1.3e9 量级的 Unix 秒在 double 下丢精度。
 */

#include "domain/types.hpp"
#include "logger.hpp"
#include "sensor/subt_dataset.hpp"
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 一条真值位姿（时间已相对化）
     */
    struct GroundTruthPose
    {
        double timestamp{0.0};                                          ///< 相对秒
        domain::Vec3 position = domain::Vec3::Zero();                   ///< 平移 (m)
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; ///< 姿态
    };

    /**
     * @brief TUM RGB-D 序列读取器
     *
     * 只做文件解析与 IMU 构造，不碰图像解码（sensor 层不依赖 OpenCV）；
     * @c imagePath 返回路径，由调用方决定如何读。
     */
    class TumSequence
    {
    public:
        explicit TumSequence(std::filesystem::path root)
            : root_(std::move(root))
        {
        }

        /**
         * @brief 加载全部元数据并构造 IMU
         * @return 成功时无值；文件缺失或格式错误时返回具体原因
         */
        std::expected<void, DatasetError> load()
        {
            if (!std::filesystem::is_directory(root_))
            {
                return fail(std::format("TUM 序列目录不存在: {}", root_.string()));
            }
            if (const auto rc = loadRgbList(); !rc)
            {
                return rc;
            }
            if (const auto rc = loadGroundTruth(); !rc)
            {
                return rc;
            }
            if (const auto rc = loadAccelerometer(); !rc)
            {
                return rc;
            }
            if (const auto rc = buildImu(); !rc)
            {
                return rc;
            }
            depthCount_ = countDataLines(root_ / "depth.txt");

            mylog::info("[数据加载] TUM {}: 图像 {} 帧, 真值 {} 条, IMU {} 条, 时长 {:.2f} s",
                        root_.filename().string(), stamps_.size(), groundTruth_.size(), imu_.size(),
                        stamps_.empty() ? 0.0 : stamps_.back() - stamps_.front());
            mylog::info("[数据加载] IMU 频率≈{:.1f} Hz（gyro 由真值差分合成）, 相机频率≈{:.1f} Hz", estimateImuRate(),
                        estimateCameraRate());
            return {};
        }

        /// 图像帧数
        [[nodiscard]] std::size_t size() const { return stamps_.size(); }

        /// 各帧相对时间戳（秒，升序）
        [[nodiscard]] const std::vector<double> &stamps() const { return stamps_; }

        /// 全部 IMU 记录（时间已相对化；gyro 为真值差分合成）
        [[nodiscard]] const std::vector<ImuRecord> &imu() const { return imu_; }

        /// 真值轨迹（时间已相对化）
        [[nodiscard]] const std::vector<GroundTruthPose> &groundTruth() const { return groundTruth_; }

        /// 第 index 帧图像路径（rgb/<timestamp>.png）
        [[nodiscard]] std::string imagePath(std::size_t index) const { return (root_ / rgbFiles_[index]).string(); }

        /// 深度帧数（只用于诊断，本读取器不消费深度）
        [[nodiscard]] std::size_t depthCount() const { return depthCount_; }

        /// 原始加速度计样本数（合成前）
        [[nodiscard]] std::size_t rawAccelerometerCount() const { return accelNs_.size(); }

        /// IMU 是否采用真值差分合成 gyro
        [[nodiscard]] bool imuSyntheticGyro() const { return true; }

        /// 加速度计在相机时间窗内的覆盖率（0..1）
        [[nodiscard]] double accelerometerCoverage() const
        {
            if (stamps_.empty() || accelNs_.empty())
            {
                return 0.0;
            }
            const double camSpan = stamps_.back() - stamps_.front();
            if (camSpan <= 0.0)
            {
                return 0.0;
            }
            const double lo = std::max(stamps_.front(), relativeSeconds(accelNs_.front()));
            const double hi = std::min(stamps_.back(), relativeSeconds(accelNs_.back()));
            return std::clamp((hi - lo) / camSpan, 0.0, 1.0);
        }

        /// IMU 平均频率 (Hz)；样本不足返回 0
        [[nodiscard]] double estimateImuRate() const
        {
            if (imu_.size() < 2)
            {
                return 0.0;
            }
            const double span = imu_.back().timestamp - imu_.front().timestamp;
            return span > 0.0 ? static_cast<double>(imu_.size() - 1) / span : 0.0;
        }

        /// 相机平均帧率 (Hz)；样本不足返回 0
        [[nodiscard]] double estimateCameraRate() const
        {
            if (stamps_.size() < 2)
            {
                return 0.0;
            }
            const double span = stamps_.back() - stamps_.front();
            return span > 0.0 ? static_cast<double>(stamps_.size() - 1) / span : 0.0;
        }

        /// 序列根目录
        [[nodiscard]] const std::filesystem::path &root() const { return root_; }

    private:
        /// 文本数据行（跳过注释与空行）切成字段，字段数不足返回空
        static std::vector<std::string_view> fields(std::string_view line, std::size_t expected)
        {
            std::vector<std::string_view> out;
            out.reserve(expected);
            std::size_t begin = 0;
            while (out.size() < expected)
            {
                while (begin < line.size() && (line[begin] == ' ' || line[begin] == '\t'))
                {
                    ++begin;
                }
                if (begin >= line.size())
                {
                    break;
                }
                const std::size_t end = line.find_first_of(" \t\r", begin);
                out.push_back(line.substr(begin, (end == std::string_view::npos ? line.size() : end) - begin));
                if (end == std::string_view::npos)
                {
                    break;
                }
                begin = end;
            }
            return out;
        }

        static bool parseDouble(std::string_view text, double &out)
        {
            const auto [parsed, ec] = std::from_chars(text.data(), text.data() + text.size(), out);
            return ec == std::errc{} && parsed == text.data() + text.size() && std::isfinite(out);
        }

        /// Unix 秒（double）→ int64 纳秒；精度受 double 尾数限制，约 100 ns 级
        static std::int64_t secondsToNs(double seconds)
        {
            return static_cast<std::int64_t>(std::llround(seconds * 1e9));
        }

        [[nodiscard]] double relativeSeconds(std::int64_t ns) const
        {
            return static_cast<double>(ns - originNs_) * 1e-9;
        }

        static std::expected<void, DatasetError> fail(std::string message)
        {
            mylog::error("{}", message);
            return std::unexpected(DatasetError{std::move(message)});
        }

        static std::size_t countDataLines(const std::filesystem::path &path)
        {
            std::ifstream in(path);
            if (!in)
            {
                return 0;
            }
            std::size_t count = 0;
            std::string line;
            while (std::getline(in, line))
            {
                if (!line.empty() && line[0] != '#')
                {
                    ++count;
                }
            }
            return count;
        }

        std::expected<void, DatasetError> loadRgbList()
        {
            const std::filesystem::path path = root_ / "rgb.txt";
            std::ifstream in(path);
            if (!in)
            {
                return fail(std::format("无法打开图像索引: {}", path.string()));
            }
            std::string line;
            while (std::getline(in, line))
            {
                if (line.empty() || line[0] == '#')
                {
                    continue;
                }
                const auto parts = fields(line, 2);
                if (parts.size() < 2)
                {
                    continue;
                }
                double seconds = 0.0;
                if (!parseDouble(parts[0], seconds))
                {
                    continue;
                }
                rgbNs_.push_back(secondsToNs(seconds));
                rgbFiles_.emplace_back(parts[1]);
            }
            if (rgbNs_.empty())
            {
                return fail(std::format("图像索引无有效行: {}", path.string()));
            }
            return {};
        }

        std::expected<void, DatasetError> loadGroundTruth()
        {
            const std::filesystem::path path = root_ / "groundtruth.txt";
            std::ifstream in(path);
            if (!in)
            {
                return fail(std::format("无法打开真值轨迹: {}", path.string()));
            }
            std::string line;
            std::size_t lines = 0;
            std::size_t invalidQuaternionLines = 0;
            while (std::getline(in, line))
            {
                if (line.empty() || line[0] == '#')
                {
                    continue;
                }
                const auto parts = fields(line, 8);
                if (parts.size() < 8)
                {
                    continue;
                }
                double values[8] = {};
                bool ok = true;
                for (std::size_t i = 0; i < 8; ++i)
                {
                    ok = ok && parseDouble(parts[i], values[i]);
                }
                if (!ok)
                {
                    continue;
                }
                GroundTruthPose pose;
                pose.position = domain::Vec3(values[1], values[2], values[3]);
                pose.orientation = Eigen::Quaterniond(values[7], values[4], values[5], values[6]);
                // 动捕失锁时常把无效帧写成 `0 0 0 0`。Eigen 的 normalize() 就是
                // `*this / norm()`，零模会得到全 NaN，并经 calibrateAccelerometerAxes
                // 的 accelRotation_ 把**全部**加速度样本污染成 NaN、让 buildImu 的
                // gyro 静默置零。这里丢弃该行而不是让毒值进链路。
                const double norm = pose.orientation.norm();
                if (!(norm > 1e-9) || !std::isfinite(norm))
                {
                    ++invalidQuaternionLines;
                    continue;
                }
                pose.orientation.coeffs() /= norm;
                groundTruthNs_.push_back(secondsToNs(values[0]));
                groundTruth_.push_back(pose);
                ++lines;
            }
            if (invalidQuaternionLines > 0)
            {
                mylog::warn("[数据加载] 真值轨迹中有 {} 行四元数模长无效（通常为动捕失锁），已跳过",
                            invalidQuaternionLines);
            }
            if (lines < 2)
            {
                return fail(std::format("真值轨迹行数不足: {}", path.string()));
            }
            return {};
        }

        std::expected<void, DatasetError> loadAccelerometer()
        {
            const std::filesystem::path path = root_ / "accelerometer.txt";
            std::ifstream in(path);
            if (!in)
            {
                return fail(std::format("无法打开加速度计文件: {}", path.string()));
            }
            std::string line;
            while (std::getline(in, line))
            {
                if (line.empty() || line[0] == '#')
                {
                    continue;
                }
                const auto parts = fields(line, 4);
                if (parts.size() < 4)
                {
                    continue;
                }
                double values[4] = {};
                bool ok = true;
                for (std::size_t i = 0; i < 4; ++i)
                {
                    ok = ok && parseDouble(parts[i], values[i]);
                }
                if (!ok)
                {
                    continue;
                }
                accelNs_.push_back(secondsToNs(values[0]));
                accelValues_.emplace_back(values[1], values[2], values[3]);
            }
            if (accelNs_.empty())
            {
                return fail(std::format("加速度计文件无有效行: {}", path.string()));
            }
            if (!std::is_sorted(accelNs_.begin(), accelNs_.end()))
            {
                // 保持时间序：排序索引，而不是分别排两个数组导致错位
                std::vector<std::size_t> order(accelNs_.size());
                for (std::size_t i = 0; i < order.size(); ++i)
                {
                    order[i] = i;
                }
                std::sort(order.begin(), order.end(),
                          [this](std::size_t a, std::size_t b) { return accelNs_[a] < accelNs_[b]; });
                std::vector<std::int64_t> sortedNs(accelNs_.size());
                std::vector<domain::Vec3> sortedValues(accelValues_.size());
                for (std::size_t i = 0; i < order.size(); ++i)
                {
                    sortedNs[i] = accelNs_[order[i]];
                    sortedValues[i] = accelValues_[order[i]];
                }
                accelNs_.swap(sortedNs);
                accelValues_.swap(sortedValues);
            }
            return {};
        }

        /// 在加速度计序列上按时间线性插值；超出范围时钳到端点
        [[nodiscard]] domain::Vec3 accelerometerAt(std::int64_t ns) const
        {
            if (accelNs_.empty())
            {
                return domain::Vec3::Zero();
            }
            const auto it = std::lower_bound(accelNs_.begin(), accelNs_.end(), ns);
            if (it == accelNs_.begin())
            {
                return accelValues_.front();
            }
            if (it == accelNs_.end())
            {
                return accelValues_.back();
            }
            const std::size_t upper = static_cast<std::size_t>(it - accelNs_.begin());
            const std::size_t lower = upper - 1;
            const double span = static_cast<double>(accelNs_[upper] - accelNs_[lower]);
            const double alpha = span > 0.0 ? static_cast<double>(ns - accelNs_[lower]) / span : 0.0;
            return accelValues_[lower] * (1.0 - alpha) + accelValues_[upper] * alpha;
        }

        /**
         * @brief 标定加速度计坐标系到光学系的旋转（Wahba / Kabsch）
         *
         * TUM 的 accelerometer.txt 与 RGB 光学系既不同轴、也不共面：实测存在仅靠
         * 24 种轴置换无法表达的固定安装倾角，残余倾角会以恒定加速度的形式注入
         * 积分，3 秒就漂数米。
         *
         * 静止时加速度计测的是比力 f = -g，因此
         *   R_gt^T · (0, 0, +9.81) ≈ R_fix · accel_raw
         * 对所有样本做最小二乘（Kabsch），得到完整的 3DoF 旋转。运动加速度
         * 在均值上相互抵消，不影响重力方向的判定。
         */
        void calibrateAccelerometerAxes()
        {
            const std::size_t stride = std::max<std::size_t>(1, accelNs_.size() / 2000);
            Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
            domain::Vec3 meanWorld = domain::Vec3::Zero();
            std::size_t count = 0;
            for (std::size_t i = 0; i < accelNs_.size(); i += stride)
            {
                const auto it = std::lower_bound(groundTruthNs_.begin(), groundTruthNs_.end(), accelNs_[i]);
                if (it == groundTruthNs_.end())
                {
                    continue;
                }
                const std::size_t k = static_cast<std::size_t>(it - groundTruthNs_.begin());
                const Eigen::Vector3d up(0.0, 0.0, 9.81);
                const Eigen::Vector3d target = groundTruth_[k].orientation.conjugate() * up;
                h += accelValues_[i] * target.transpose();
                meanWorld += groundTruth_[k].orientation * accelValues_[i];
                ++count;
            }
            if (count == 0)
            {
                return;
            }
            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
            if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0)
            {
                d(2, 2) = -1.0;
            }
            accelRotation_ = svd.matrixV() * d * svd.matrixU().transpose();
            // 均值必须在施加旋转之后重算：上面的 meanWorld 用的是原始 accel，
            // 直接打印会给出误导性的数值（实测曾出现模值 4.2 的假象）。
            domain::Vec3 calibratedSum = domain::Vec3::Zero();
            std::size_t calibratedCount = 0;
            for (std::size_t i = 0; i < accelNs_.size(); i += stride)
            {
                const auto it = std::lower_bound(groundTruthNs_.begin(), groundTruthNs_.end(), accelNs_[i]);
                if (it == groundTruthNs_.end())
                {
                    continue;
                }
                const std::size_t k = static_cast<std::size_t>(it - groundTruthNs_.begin());
                calibratedSum += groundTruth_[k].orientation * (accelRotation_ * accelValues_[i]);
                ++calibratedCount;
            }
            calibratedMean_ = calibratedCount > 0 ? calibratedSum / static_cast<double>(calibratedCount)
                                                  : meanWorld / static_cast<double>(count);
            const double angle =
                std::acos(std::clamp((accelRotation_.trace() - 1.0) * 0.5, -1.0, 1.0)) * 180.0 / std::numbers::pi;
            mylog::info("[数据加载] 加速度计坐标系标定: 相对光学系旋转 {:.2f}°, "
                        "世界系比力均值≈({:.2f}, {:.2f}, {:.2f}) m/s^2 (应为 0,0,+9.81)",
                        angle, calibratedMean_.x(), calibratedMean_.y(), calibratedMean_.z());
        }

        /**
         * @brief 以真值为时间基准构造 IMU（100 Hz）
         *
         * gyro = 2 * Im(q_k^-1 * q_{k+1}) / dt，即机体系角速度；加速度从
         * accelerometer.txt 线性插值到同一时刻。以真值时刻为基准而不是
         * 加速度计时刻，是为了让 gyro 与姿态严格自洽——插值角速度会引入
         * 与姿态不一致的相位误差。
         */
        std::expected<void, DatasetError> buildImu()
        {
            if (groundTruth_.size() < 2)
            {
                return fail("真值轨迹不足两帧，无法合成角速度");
            }
            // 原点取所有序列的最小时间戳，保证 GT/图像/IMU 共用一个时间基
            originNs_ = groundTruthNs_.front();
            for (const std::int64_t ns : rgbNs_)
            {
                originNs_ = std::min(originNs_, ns);
            }
            for (const std::int64_t ns : accelNs_)
            {
                originNs_ = std::min(originNs_, ns);
            }

            stamps_.reserve(rgbNs_.size());
            for (const std::int64_t ns : rgbNs_)
            {
                stamps_.push_back(relativeSeconds(ns));
            }
            for (std::size_t i = 0; i < groundTruth_.size(); ++i)
            {
                groundTruth_[i].timestamp = relativeSeconds(groundTruthNs_[i]);
            }

            imu_.reserve(groundTruth_.size());
            calibrateAccelerometerAxes();
            // 变换一次性落到数据上：之后所有读取（含插值）都在光学系中
            for (domain::Vec3 &value : accelValues_)
            {
                value = accelRotation_ * value;
            }
            for (std::size_t i = 0; i < groundTruth_.size(); ++i)
            {
                const Eigen::Quaterniond &q0 = groundTruth_[i].orientation;
                Eigen::Vector3d omega = Eigen::Vector3d::Zero();
                if (i + 1 < groundTruth_.size())
                {
                    const double dt = static_cast<double>(groundTruthNs_[i + 1] - groundTruthNs_[i]) * 1e-9;
                    if (dt > 1e-9)
                    {
                        const Eigen::Quaterniond q1 = groundTruth_[i + 1].orientation;
                        const Eigen::Quaterniond dq = q0.conjugate() * q1;
                        const Eigen::Vector3d vec(dq.x(), dq.y(), dq.z());
                        const double w = std::clamp(dq.w(), -1.0, 1.0);
                        const double angle = 2.0 * std::atan2(vec.norm(), std::abs(w));
                        const double sign = w >= 0.0 ? 1.0 : -1.0;
                        const double norm = vec.norm();
                        if (norm > 1e-12)
                        {
                            omega = sign * vec / norm * (angle / dt);
                        }
                    }
                }
                else if (!imu_.empty())
                {
                    omega = imu_.back().gyro;
                }

                ImuRecord record;
                record.timestamp = groundTruth_[i].timestamp;
                record.gyro = omega;
                record.accel = accelerometerAt(groundTruthNs_[i]);
                imu_.push_back(record);
            }
            return {};
        }

        std::filesystem::path root_;
        std::vector<std::int64_t> rgbNs_;
        std::vector<std::string> rgbFiles_;
        std::vector<std::int64_t> groundTruthNs_;
        std::vector<GroundTruthPose> groundTruth_;
        std::vector<std::int64_t> accelNs_;
        std::vector<domain::Vec3> accelValues_;
        std::vector<double> stamps_;
        std::vector<ImuRecord> imu_;
        std::size_t depthCount_{0};
        std::int64_t originNs_{0};
        Eigen::Matrix3d accelRotation_ = Eigen::Matrix3d::Identity();
        domain::Vec3 calibratedMean_ = domain::Vec3(0.0, 0.0, 9.81);
    };

} // namespace myslam::dataset
