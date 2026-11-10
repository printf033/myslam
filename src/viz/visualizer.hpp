#pragma once

/**
 * @file visualizer.hpp
 * @brief 数据集回放的实时可视化：特征跟踪画面、轨迹小地图与状态叠加。
 *
 * 两种输出可同时使用：
 *   - `showWindow`：OpenCV 窗口实时显示（需要 DISPLAY，按 q/ESC 退出）；
 *   - `videoPath`：把同样画面编码写入视频文件，便于回看与分享。
 *
 * 画面布局：上方左半是原始鱼眼图（叠加原始像素坐标的特征点与 MEI 有效视场
 * 轮廓），右半是同一帧经 MEI 去畸变后的虚拟针孔图像（叠加跟踪器实际使用的
 * 针孔像素坐标）；下方为状态栏：左侧是轨迹小地图（世界系 XY），右侧是运行
 * 状态文字。颜色约定：绿色=跨帧跟踪成功，橙色=本帧新增。
 *
 * 两侧必须使用各自的像素坐标：跟踪器输出的 uv 是虚拟针孔坐标，直接画在
 * 鱼眼原图上会系统性偏移，看起来像跟踪错乱。
 */

#include "common/dataset_config.hpp"
#include "domain/ports.hpp"
#include "domain/types.hpp"
#include "logger.hpp"
#include <algorithm>
#include <chrono>
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
// std::this_thread::sleep_for 用于按时间戳节流；不可依赖 OpenCV 头间接引入。
#include <thread>
#include <utility>
#include <vector>

namespace myslam::viz
{

    /**
     * @brief 回放可视化器
     */
    class DatasetVisualizer
    {
    public:
        explicit DatasetVisualizer(const dataset::VisualizationConfig &config)
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
                    // 无 DISPLAY（SSH/容器）时窗口后端不可用：降级为纯视频/无输出。
                    // 错误信息统一走 mylog（项目规范）：此处已进入回放阶段，
                    // 日志系统早已初始化，不存在“日志不可用”的情况。
                    mylog::warn("[可视化] 无法创建窗口（{}），已关闭实时显示", error.what());
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
         * @param frame 该帧的特征观测（uv 为目标模型像素）
         * @param trail 世界系 XY 轨迹点（关键帧位姿）
         * @param statusLines 状态栏文字（英文，OpenCV 不支持中文渲染）
         * @param cameraTime 该帧的相机时间戳（秒），用于按真实节奏节流
         * @param undistorted 可选：同一帧的去畸变图像（目标针孔画幅）
         * @param sourceMask 可选：原图有效视场掩码（CV_8UC1），描出真正参与约束的像素范围
         */
        void update(const cv::Mat &gray, const domain::Frame &frame, const std::vector<domain::Vec2> &trail,
                    const std::vector<std::string> &statusLines, double cameraTime,
                    const cv::Mat *undistorted = nullptr, const cv::Mat *sourceMask = nullptr)
        {
            if (!active_ || quit_)
            {
                return;
            }
            if (skipCounter_++ % std::max(1, config_.displayStride) != 0)
            {
                return;
            }

            cv::Mat canvas = buildCanvas(gray, frame, trail, statusLines, undistorted, sourceMask);
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
                // 并排布局后画幅变宽，构造时的固定窗口尺寸会把画面压扁。
                // 首次拿到真实画幅后按比例缩放到目标宽度（不超过 1600 px）。
                if (!windowSized_)
                {
                    windowSized_ = true;
                    const int targetWidth = std::min(1600, canvas.cols);
                    const double scale = static_cast<double>(targetWidth) / static_cast<double>(canvas.cols);
                    cv::resizeWindow(kWindowName, targetWidth,
                                     static_cast<int>(std::lround(static_cast<double>(canvas.rows) * scale)));
                }
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
                // 注意：不能对 config_.videoPath 调 replace_extension——
                // 那是破坏性修改成员，回退一次之后配置里的路径就被永久改成 .avi，
                // 后续日志与报告都会指向错误文件名。
                std::filesystem::path fallback = config_.videoPath;
                fallback.replace_extension(".avi");
                candidates.emplace_back(fallback.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
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
                        mylog::warn("[可视化] {} 不可用，已回退写入 {}", path, file);
                    }
                    return;
                }
                writer_.release();
            }
            cv::utils::logging::setLogLevel(previousLogLevel);
            mylog::error("[可视化] 无法打开视频文件 {}：当前 OpenCV 构建没有可用的编码器", path);
        }

        cv::Mat buildCanvas(const cv::Mat &gray, const domain::Frame &frame, const std::vector<domain::Vec2> &trail,
                            const std::vector<std::string> &statusLines, const cv::Mat *undistorted,
                            const cv::Mat *sourceMask)
        {
            if (gray.empty())
            {
                return {};
            }
            cv::Mat left = toBgr(gray);
            if (sourceMask != nullptr && !sourceMask->empty())
            {
                drawMaskOutline(left, *sourceMask);
            }
            // 原图这一侧用原始鱼眼像素，坐标才对得上
            drawFeatures(left, frame, /*useSourcePixel=*/true);
            drawCaption(left, "raw fisheye");

            cv::Mat top = left;
            if (undistorted != nullptr && !undistorted->empty())
            {
                cv::Mat right = toBgr(*undistorted);
                if (right.size() != left.size())
                {
                    cv::resize(right, right, left.size(), 0.0, 0.0, cv::INTER_LINEAR);
                }
                // 矫正图这一侧用虚拟针孔像素（即跟踪器输出的 uv）
                drawFeatures(right, frame, /*useSourcePixel=*/false);
                drawCaption(right, "MEI undistorted (virtual pinhole)");
                cv::hconcat(left, right, top);
                // 中缝分隔线
                cv::line(top, cv::Point(left.cols, 0), cv::Point(left.cols, top.rows - 1), cv::Scalar(70, 70, 70), 2);
            }

            // panelHeight 为 0 时 panel 是空 Mat，后续 rectangle/vconcat 会断言失败。
            // 配置层已把下界提到 1，这里对直接构造 VisualizationConfig 的调用方兜底。
            const int panelHeight = std::max(1, config_.panelHeight);
            cv::Mat panel = cv::Mat::zeros(panelHeight, top.cols, CV_8UC3);
            drawTrail(panel, trail);
            drawStatus(panel, statusLines);

            cv::Mat canvas;
            cv::vconcat(top, panel, canvas);
            return canvas;
        }

        static cv::Mat toBgr(const cv::Mat &image)
        {
            if (image.channels() == 1)
            {
                cv::Mat bgr;
                cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
                return bgr;
            }
            return image.clone();
        }

        /// 画特征点：原图侧用 @c sourcePixel，去畸变侧用 @c uv
        void drawFeatures(cv::Mat &image, const domain::Frame &frame, bool useSourcePixel) const
        {
            for (const domain::Observation &observation : frame.observations)
            {
                // 跟踪成功=绿色，新增=橙色
                const cv::Scalar color =
                    observation.kind == domain::ObservationKind::NEW ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0);
                const domain::Vec2 point =
                    useSourcePixel && observation.sourcePixel ? *observation.sourcePixel : observation.uv;
                cv::circle(image, cv::Point(cvRound(point.x()), cvRound(point.y())), 3, color, 1, cv::LINE_AA);
            }
        }

        /**
         * @brief 描出原图上能映射进目标画幅的区域
         *
         * 鱼眼视场远大于虚拟针孔画幅，外围像素不参与任何后端约束。掩码在一次
         * 回放里恒定，轮廓只算一次；每帧 findContours 纯属浪费。
         */
        void drawMaskOutline(cv::Mat &image, const cv::Mat &mask)
        {
            if (mask.size() != image.size() || mask.type() != CV_8UC1)
            {
                return;
            }
            if (!maskContoursReady_ || maskContoursSize_ != mask.size())
            {
                maskContours_.clear();
                cv::findContours(mask.clone(), maskContours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                maskContoursSize_ = mask.size();
                maskContoursReady_ = true;
            }
            cv::drawContours(image, maskContours_, -1, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
        }

        static void drawCaption(cv::Mat &image, const std::string &text)
        {
            // 先黑底描边再写白字：图像亮暗都能看清，且不遮住下方特征点
            const cv::Point origin(8, 20);
            cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
            cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
        }

        void drawTrail(cv::Mat &panel, const std::vector<domain::Vec2> &trail) const
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

            auto project = [&](const domain::Vec2 &p)
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

        dataset::VisualizationConfig config_;
        bool active_{false};
        bool quit_{false};
        std::size_t skipCounter_{0};
        bool writerTried_{false};
        bool windowSized_{false};
        bool timingStarted_{false};
        double baseCameraTime_{0.0};
        std::chrono::steady_clock::time_point baseWallTime_{};
        cv::VideoWriter writer_;
        /// 有效视场轮廓缓存：掩码在整段回放内恒定，只需算一次
        bool maskContoursReady_{false};
        cv::Size maskContoursSize_{};
        std::vector<std::vector<cv::Point>> maskContours_;
    };

} // namespace myslam::viz
