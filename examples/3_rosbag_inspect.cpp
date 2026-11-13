/**
 * @file 3_rosbag_inspect.cpp
 * @brief 检查 rosbag2（.db3）数据集：topic 盘点、IMU/相机/参考轨迹质量评估。
 *
 * 用法：
 * @code
 * 3_rosbag_inspect <bag.db3> [<bag.db3> ...]
 * @endcode
 *
 * 只依赖 sensor 层的 RosBagSource（SQLite + CDR 解码），不链接 OpenCV：
 * 图像只做「能否解出合法头部」的校验，不做像素级处理。输出用于回答两个
 * 问题：这份数据有没有参考真值、各传感器质量是否足以跑 VIO。
 */

#include "logger.hpp"
#include "sensor/rosbag2_source.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <format>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace
{

    using myslam::dataset::ImageFrame;
    using myslam::dataset::PoseRecord;
    using myslam::dataset::RosBagSource;
    using myslam::dataset::TopicInfo;

    double median(std::vector<double> values)
    {
        if (values.empty())
        {
            return 0.0;
        }
        const std::size_t mid = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
        return values[mid];
    }

    double percentile(std::vector<double> values, double p)
    {
        if (values.empty())
        {
            return 0.0;
        }
        const std::size_t index = std::min(values.size() - 1, static_cast<std::size_t>(p * values.size()));
        std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
        return values[index];
    }

    /**
     * @brief 从 JPEG 头部解析宽高（SOF0/SOF1/SOF2 段），不解码像素
     * @return {width, height}；解析失败返回 {0, 0}
     */
    std::pair<int, int> jpegSize(const std::vector<std::uint8_t> &data)
    {
        if (data.size() < 4 || data[0] != 0xFF || data[1] != 0xD8)
        {
            return {0, 0};
        }
        std::size_t i = 2;
        while (i + 3 < data.size())
        {
            if (data[i] != 0xFF)
            {
                ++i;
                continue;
            }
            const std::uint8_t marker = data[i + 1];
            if (marker == 0xD8 || marker == 0xD9)
            {
                i += 2;
                continue;
            }
            const std::size_t len = (static_cast<std::size_t>(data[i + 2]) << 8) | data[i + 3];
            if (len < 2 || i + 2 + len > data.size())
            {
                break;
            }
            // SOF0..SOF3, SOF5..SOF7, SOF9..SOF11, SOF13..SOF15（排除 DHT=C4、JPG=C8、DAC=CC）
            const bool isSof = marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
            if (isSof && len >= 7)
            {
                const int height = (static_cast<int>(data[i + 5]) << 8) | data[i + 6];
                const int width = (static_cast<int>(data[i + 7]) << 8) | data[i + 8];
                return {width, height};
            }
            i += 2 + len;
        }
        return {0, 0};
    }

    void printTopics(const RosBagSource &src)
    {
        std::printf("  %-45s %-40s %10s\n", "topic", "type", "count");
        std::printf("  %s\n", std::string(97, '-').c_str());
        for (const TopicInfo &t : src.topics())
        {
            std::printf("  %-45s %-40s %10llu\n", t.name.c_str(), t.type.c_str(),
                        static_cast<unsigned long long>(t.count));
        }
    }

    void analyzeImu(const RosBagSource &src)
    {
        const auto &imu = src.imu();
        if (imu.empty())
        {
            std::printf("\n[IMU] 无 ins_data 消息\n");
            return;
        }
        const double span = imu.back().timestamp - imu.front().timestamp;
        std::vector<double> dt;
        dt.reserve(imu.size());
        double accelNormSum = 0.0;
        double accelNormMin = std::numeric_limits<double>::max();
        double accelNormMax = std::numeric_limits<double>::lowest();
        double gyroNormSum = 0.0;
        double gyroNormMax = 0.0;
        std::size_t nonFinite = 0;
        for (std::size_t i = 0; i < imu.size(); ++i)
        {
            const double a = imu[i].accel.norm();
            const double g = imu[i].gyro.norm();
            if (!std::isfinite(a) || !std::isfinite(g))
            {
                ++nonFinite;
                continue;
            }
            accelNormSum += a;
            accelNormMin = std::min(accelNormMin, a);
            accelNormMax = std::max(accelNormMax, a);
            gyroNormSum += g;
            gyroNormMax = std::max(gyroNormMax, g);
            if (i > 0)
            {
                dt.push_back(imu[i].timestamp - imu[i - 1].timestamp);
            }
        }
        const double n = static_cast<double>(imu.size());
        std::printf("\n[IMU] topic=%s  n=%zu  时长=%.1f s  频率≈%.1f Hz  跳过=%llu\n", src.imuTopicName().c_str(),
                    imu.size(), span, src.estimateImuRate(), static_cast<unsigned long long>(src.skippedImu()));
        std::printf("  |accel| mean=%.3f m/s^2  min=%.3f  max=%.3f   (静止时应≈9.8)\n", accelNormSum / n, accelNormMin,
                    accelNormMax);
        std::printf("  |gyro|  mean=%.4f rad/s  max=%.4f\n", gyroNormSum / n, gyroNormMax);
        std::printf("  dt      median=%.3f ms  p99=%.3f ms  max=%.3f ms  (n_dt=%zu, 非有限=%zu)\n", median(dt) * 1e3,
                    percentile(dt, 0.99) * 1e3, dt.empty() ? 0.0 : *std::max_element(dt.begin(), dt.end()) * 1e3,
                    dt.size(), nonFinite);
    }

    void analyzeImages(const RosBagSource &src)
    {
        const auto topics = src.imageTopics();
        if (topics.empty())
        {
            std::printf("\n[相机] 无图像 topic\n");
            return;
        }
        for (const std::string &topic : topics)
        {
            const std::size_t count = src.imageCount(topic);
            if (count == 0)
            {
                std::printf("\n[相机] topic=%s  n=0（无消息，跳过）\n", topic.c_str());
                continue;
            }
            const std::vector<double> stamps = src.imageStamps(topic);
            std::vector<double> dt;
            dt.reserve(stamps.size());
            for (std::size_t i = 1; i < stamps.size(); ++i)
            {
                dt.push_back(stamps[i] - stamps[i - 1]);
            }
            const double span = stamps.size() >= 2 ? stamps.back() - stamps.front() : 0.0;
            const double rate = span > 0.0 ? static_cast<double>(stamps.size() - 1) / span : 0.0;
            const double dtMedian = median(dt);
            const double maxGap = dt.empty() ? 0.0 : *std::max_element(dt.begin(), dt.end());

            std::printf("\n[相机] topic=%s  n=%zu  时长=%.1f s  频率≈%.2f Hz\n", topic.c_str(), count, span, rate);
            std::printf("  dt median=%.3f s  p99=%.3f s  max=%.3f s\n", dtMedian, percentile(dt, 0.99), maxGap);

            // 解码前若干帧，验证 CDR 布局与像素格式
            std::size_t verified = 0;
            for (std::size_t i = 0; i < std::min<std::size_t>(count, 3); ++i)
            {
                const auto frame = src.imageAt(topic, i);
                if (!frame)
                {
                    std::printf("  帧 %zu 解码失败: %s\n", i, frame.error().message.c_str());
                    continue;
                }
                const ImageFrame &f = *frame;
                if (f.compressed)
                {
                    const bool jpeg = f.data.size() >= 2 && f.data[0] == 0xFF && f.data[1] == 0xD8;
                    const auto [w, h] = jpegSize(f.data);
                    std::printf("  帧 %zu: 压缩 format=%s  bytes=%zu  %dx%d  jpeg_magic=%s\n", i, f.encoding.c_str(),
                                f.data.size(), w, h, jpeg ? "FFD8 ok" : "BAD");
                }
                else
                {
                    const std::size_t expect = static_cast<std::size_t>(f.height) * static_cast<std::size_t>(f.step);
                    std::printf("  帧 %zu: %dx%d enc=%s step=%d bytes=%zu%s\n", i, f.width, f.height,
                                f.encoding.c_str(), f.step, f.data.size(),
                                f.data.size() == expect ? "  ok" : "  SIZE-MISMATCH");
                }
                ++verified;
            }
            if (verified == 0)
            {
                std::printf("  ! 所有抽样帧都无法解码\n");
            }
        }
    }

    void analyzePoses(const RosBagSource &src)
    {
        const auto &poses = src.referencePoses();
        if (poses.empty())
        {
            std::printf("\n[参考轨迹] 无 PoseStamped（ins_pose / dvl/pose）\n");
            return;
        }
        const double span = poses.back().timestamp - poses.front().timestamp;
        double pathLength = 0.0;
        std::vector<double> speed;
        std::vector<double> stepX;
        std::vector<double> stepY;
        std::size_t jumps = 0;
        std::size_t nonFinite = 0;
        std::size_t tinyDt = 0;
        std::size_t originResets = 0;
        double maxJump = 0.0;
        for (std::size_t i = 0; i < poses.size(); ++i)
        {
            if (!poses[i].position.allFinite())
            {
                ++nonFinite;
                continue;
            }
            if (poses[i].position.squaredNorm() < 1e-18)
            {
                ++originResets;
            }
            if (i == 0)
            {
                continue;
            }
            const double dt = poses[i].timestamp - poses[i - 1].timestamp;
            const double dist = (poses[i].position - poses[i - 1].position).norm();
            if (!std::isfinite(dist) || !std::isfinite(dt) || dt <= 0.0)
            {
                continue;
            }
            pathLength += dist;
            if (dt < 1e-3)
            {
                ++tinyDt; // INS 输出时间戳抖动，不参与速度估计
                continue;
            }
            speed.push_back(dist / dt);
            stepX.push_back(std::abs(poses[i].position.x() - poses[i - 1].position.x()));
            stepY.push_back(std::abs(poses[i].position.y() - poses[i - 1].position.y()));
            if (dist > 0.3)
            {
                ++jumps;
                maxJump = std::max(maxJump, dist);
            }
        }
        const double rate = span > 0.0 ? static_cast<double>(poses.size() - 1) / span : 0.0;

        // 量化检测：非零步长的中位数即位置输出栅格（最小值会被浮点低位抖动污染）
        auto medianNonZero = [](std::vector<double> &v)
        {
            v.erase(std::remove_if(v.begin(), v.end(), [](double x) { return !(x > 1e-6); }), v.end());
            return median(v);
        };
        const double qx = medianNonZero(stepX);
        const double qy = medianNonZero(stepY);

        std::printf("\n[参考轨迹] topic=%s  n=%zu  时长=%.1f s  频率≈%.1f Hz  跳过=%llu\n", src.poseTopicName().c_str(),
                    poses.size(), span, rate, static_cast<unsigned long long>(src.skippedPose()));
        std::printf("  路径长度=%.2f m  速度 median=%.4f m/s  p99=%.4f m/s  max=%.4f m/s\n", pathLength, median(speed),
                    percentile(speed, 0.99), speed.empty() ? 0.0 : *std::max_element(speed.begin(), speed.end()));
        // 速度长尾主要来自位置栅格（~mm 级）落在短采样间隔上，并非真实运动
        std::printf(
            "  x/y 非零步长中位数=%.4f / %.4f mm (位置输出栅格)  跳变(>0.3m)=%zu  max_jump=%.3f m  非有限=%zu\n",
            qx * 1e3, qy * 1e3, jumps, maxJump, nonFinite);
        std::printf("  dt<1ms 样本=%zu (%.1f%%，时间戳抖动，未计入速度)  原点(0,0,0)重置=%zu\n", tinyDt,
                    poses.size() > 1 ? 100.0 * static_cast<double>(tinyDt) / static_cast<double>(poses.size() - 1)
                                     : 0.0,
                    originResets);
        std::printf("  首帧=%.3f 末帧=%.3f (相对秒)\n", poses.front().timestamp, poses.back().timestamp);
    }

    void analyzeCameraInfo(const RosBagSource &src)
    {
        const auto topics = src.cameraInfoTopics();
        if (topics.empty())
        {
            std::printf("\n[内参] 无 CameraInfo topic（跑视觉需外部标定）\n");
            return;
        }
        for (const std::string &topic : topics)
        {
            const auto info = src.cameraInfo(topic);
            if (!info)
            {
                std::printf("\n[内参] %s 解码失败\n", topic.c_str());
                continue;
            }
            std::printf("\n[内参] %s  %dx%d  model=%s  fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n", topic.c_str(), info->width,
                        info->height, info->distortionModel.c_str(), info->k[0], info->k[4], info->k[2], info->k[5]);
            std::printf("  D=[");
            for (std::size_t i = 0; i < info->distortion.size(); ++i)
            {
                std::printf("%s%.6g", i == 0 ? "" : ", ", info->distortion[i]);
            }
            std::printf("]  P[3]=%.4f (基线相关)\n", info->p[3]);
        }
    }

    void inspect(const std::string &path)
    {
        std::printf("\n================================================================================\n");
        std::printf("数据集: %s\n", path.c_str());
        std::printf("================================================================================\n");
        RosBagSource src(path);
        if (const auto rc = src.load(); !rc)
        {
            std::printf("加载失败: %s\n", rc.error().message.c_str());
            return;
        }
        std::printf("消息总数=%llu  topic 数=%zu\n\n", static_cast<unsigned long long>(src.messageCount()),
                    src.topics().size());
        printTopics(src);
        analyzeImu(src);
        analyzeImages(src);
        analyzePoses(src);
        analyzeCameraInfo(src);
    }

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "用法: 3_rosbag_inspect <bag.db3> [<bag.db3> ...]\n");
        return 1;
    }
    for (int i = 1; i < argc; ++i)
    {
        inspect(argv[i]);
    }
    return 0;
}
