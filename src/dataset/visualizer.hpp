#pragma once

/**
 * @file visualizer.hpp
 * @brief 数据集回放的实时可视化：特征跟踪画面、轨迹小地图与状态叠加。
 *
 * 两种输出可同时使用：
 *   - `showWindow`：OpenCV 窗口实时显示（需要 DISPLAY，按 q/ESC 退出）；
 *   - `videoPath`：把同样画面编码写入视频文件，便于回看与分享。
 *
 * 画面布局：上方为原图叠加特征点（绿色=跨帧跟踪成功，橙色=本帧新增），
 * 下方为状态栏：左侧是轨迹小地图（世界系 XY），右侧是运行状态文字。
 */

#include "dataset/types.hpp"
#include "vio/types.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

namespace myslam::dataset
{

    /**
     * @brief 回放可视化器
     */
    class DatasetVisualizer
    {
    public:
        explicit DatasetVisualizer(const VisualizationConfig &config)
            : config_(config)
        {
            active_ = config_.showWindow || !config_.videoPath.empty();
            if (!active_)
            {
                return;
            }
            if (config_.showWindow)
            {
                try
                {
                    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
                    cv::resizeWindow(kWindowName, 720, 720);
                }
                catch (const cv::Exception &error)
                {
                    // 无 DISPLAY（SSH/容器）时窗口后端不可用：降级为纯视频/无输出
                    std::fprintf(stderr, "[viz] 无法创建窗口（%s），已关闭实时显示\n", error.what());
                    config_.showWindow = false;
                    active_ = !config_.videoPath.empty();
                }
            }
            if (!config_.videoPath.empty() && config_.videoPath.has_parent_path())
            {
                std::error_code ec;
                std::filesystem::create_directories(config_.videoPath.parent_path(), ec);
            }
        }

        ~DatasetVisualizer()
        {
            if (writer_.isOpened())
            {
                writer_.release();
            }
            if (config_.showWindow)
            {
                cv::destroyWindow(kWindowName);
            }
        }

        DatasetVisualizer(const DatasetVisualizer &) = delete;
        DatasetVisualizer &operator=(const DatasetVisualizer &) = delete;

        bool active() const { return active_; }
        bool shouldQuit() const { return quit_; }

        /**
         * @brief 渲染一帧
         *
         * @param gray 原始灰度图
         * @param frame 该帧的特征观测
         * @param trail 世界系 XY 轨迹点（关键帧位姿）
         * @param statusLines 状态栏文字（英文，OpenCV 不支持中文渲染）
         * @param cameraTime 该帧的相机时间戳（秒），用于按真实节奏节流
         */
        void update(const cv::Mat &gray, const vio::CameraFrame &frame, const std::vector<gtsam::Point2> &trail,
                    const std::vector<std::string> &statusLines, double cameraTime)
        {
            if (!active_ || quit_)
            {
                return;
            }
            if (skipCounter_++ % std::max(1, config_.displayStride) != 0)
            {
                return;
            }

            cv::Mat canvas = buildCanvas(gray, frame, trail, statusLines);
            if (canvas.empty())
            {
                return;
            }

            if (!config_.videoPath.empty())
            {
                if (!writerTried_)
                {
                    writerTried_ = true;
                    openWriter(cv::Size(canvas.cols, canvas.rows));
                }
                if (writer_.isOpened())
                {
                    writer_.write(canvas);
                }
            }

            if (config_.showWindow)
            {
                cv::imshow(kWindowName, canvas);
                const int key = cv::waitKey(1);
                if (key == 27 || key == 'q' || key == 'Q')
                {
                    quit_ = true;
                }

                // 按相机时间戳节流：playbackRate <= 0 表示不限速（离线全速）。
                // 处理慢于目标速率时不做任何等待，天然退化为全速播放。
                if (config_.playbackRate > 0.0)
                {
                    const auto now = std::chrono::steady_clock::now();
                    if (!timingStarted_)
                    {
                        timingStarted_ = true;
                        baseCameraTime_ = cameraTime;
                        baseWallTime_ = now;
                    }
                    else
                    {
                        const double targetSeconds = (cameraTime - baseCameraTime_) / config_.playbackRate;
                        const double elapsedSeconds = std::chrono::duration<double>(now - baseWallTime_).count();
                        if (targetSeconds > elapsedSeconds)
                        {
                            std::this_thread::sleep_for(std::chrono::duration<double>(targetSeconds - elapsedSeconds));
                        }
                    }
                }
                // 不依赖 WND_PROP_VISIBLE：部分 GTK 后端在首帧尚未映射窗口时返回 0，
                // 会误判为用户关闭窗口而立即退出。仅通过按键或 Ctrl-C 结束。
            }
        }

    private:
        inline static constexpr const char *kWindowName = "myslam VIO replay";

        /**
         * @brief 按可用编码器依次尝试打开视频文件
         *
         * 不同 OpenCV 构建启用的后端不同：mp4v 在部分发行版上不可用，
         * 因此回退到 AVI/MJPG（无损帧序列，文件较大但兼容性最好）。
         */
        void openWriter(const cv::Size &size)
        {
            const std::string path = config_.videoPath.string();
            const std::string ext = config_.videoPath.extension().string();
            const double fps = static_cast<double>(std::max(1, config_.videoFps));

            std::vector<std::pair<std::string, int>> candidates;
            if (ext == ".avi")
            {
                candidates.emplace_back(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            }
            else
            {
                candidates.emplace_back(path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
                candidates.emplace_back(path, cv::VideoWriter::fourcc('a', 'v', 'c', '1'));
                candidates.emplace_back(config_.videoPath.replace_extension(".avi").string(),
                                        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            }

            // 编码器探测会产生大量 OpenCV 内部报错，临时静音
            const auto previousLogLevel = cv::utils::logging::getLogLevel();
            cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
            for (const auto &[file, codec] : candidates)
            {
                if (writer_.open(file, codec, fps, size))
                {
                    cv::utils::logging::setLogLevel(previousLogLevel);
                    if (file != path)
                    {
                        std::fprintf(stderr, "[viz] %s 不可用，已回退写入 %s\n", path.c_str(), file.c_str());
                    }
                    return;
                }
                writer_.release();
            }
            cv::utils::logging::setLogLevel(previousLogLevel);
            std::fprintf(stderr, "[viz] 无法打开视频文件 %s：当前 OpenCV 构建没有可用的编码器\n", path.c_str());
        }

        cv::Mat buildCanvas(const cv::Mat &gray, const vio::CameraFrame &frame, const std::vector<gtsam::Point2> &trail,
                            const std::vector<std::string> &statusLines)
        {
            if (gray.empty())
            {
                return {};
            }
            cv::Mat bgr;
            if (gray.channels() == 1)
            {
                cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            }
            else
            {
                bgr = gray.clone();
            }

            for (const auto &feat : frame.features)
            {
                const cv::Point pt(cvRound(feat.uv.x()), cvRound(feat.uv.y()));
                // 跟踪成功=绿色，新增=橙色
                const cv::Scalar color = feat.isNew ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0);
                cv::circle(bgr, pt, 3, color, 1, cv::LINE_AA);
            }

            cv::Mat panel = cv::Mat::zeros(config_.panelHeight, bgr.cols, CV_8UC3);
            drawTrail(panel, trail);
            drawStatus(panel, statusLines);

            cv::Mat canvas;
            cv::vconcat(bgr, panel, canvas);
            return canvas;
        }

        void drawTrail(cv::Mat &panel, const std::vector<gtsam::Point2> &trail) const
        {
            const int size = std::min(panel.rows, panel.cols);
            const cv::Rect roi(0, 0, size, size);
            cv::rectangle(panel, roi, cv::Scalar(40, 40, 40), 1);

            if (trail.empty())
            {
                cv::putText(panel, "no trajectory yet", cv::Point(10, size / 2), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
                return;
            }

            double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0;
            for (const auto &p : trail)
            {
                minX = std::min(minX, p.x());
                maxX = std::max(maxX, p.x());
                minY = std::min(minY, p.y());
                maxY = std::max(maxY, p.y());
            }
            const double spanX = std::max(maxX - minX, 1.0);
            const double spanY = std::max(maxY - minY, 1.0);
            const double span = std::max(spanX, spanY) * 1.1;
            const double cx = (minX + maxX) * 0.5;
            const double cy = (minY + maxY) * 0.5;
            const int margin = 12;

            auto project = [&](const gtsam::Point2 &p)
            {
                const double u = (p.x() - cx) / span + 0.5;
                const double v = (p.y() - cy) / span + 0.5;
                return cv::Point(margin + static_cast<int>(u * (size - 2 * margin)),
                                 size - margin - static_cast<int>(v * (size - 2 * margin)));
            };

            for (std::size_t i = 1; i < trail.size(); ++i)
            {
                cv::line(panel, project(trail[i - 1]), project(trail[i]), cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            }
            cv::circle(panel, project(trail.front()), 3, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
            cv::circle(panel, project(trail.back()), 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);

            char scale[64];
            std::snprintf(scale, sizeof(scale), "%.1f m", span);
            cv::putText(panel, scale, cv::Point(6, size - 6), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(160, 160, 160),
                        1, cv::LINE_AA);
        }

        void drawStatus(cv::Mat &panel, const std::vector<std::string> &statusLines) const
        {
            const int size = std::min(panel.rows, panel.cols);
            int y = 22;
            for (const auto &line : statusLines)
            {
                cv::putText(panel, line, cv::Point(size + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
                y += 20;
                if (y > panel.rows - 6)
                {
                    break;
                }
            }
            cv::putText(panel, "green: tracked   orange: new", cv::Point(size + 10, panel.rows - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
        }

        VisualizationConfig config_;
        bool active_{false};
        bool quit_{false};
        std::size_t skipCounter_{0};
        bool writerTried_{false};
        bool timingStarted_{false};
        double baseCameraTime_{0.0};
        std::chrono::steady_clock::time_point baseWallTime_{};
        cv::VideoWriter writer_;
    };

} // namespace myslam::dataset
