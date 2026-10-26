#pragma once

/**
 * @file types.hpp
 * @brief 数据集回放的纯数据配置与统计结构（不含 OpenCV 依赖）。
 *
 * 这些结构之所以从各自实现文件中抽离，是为了让不依赖 OpenCV 的代码
 * （例如 @c src/lua_config.hpp 的配置解析）能够引用它们而不把 OpenCV
 * 变成必选依赖：@c MYSLAM_BUILD_DATASET_TOOLS 默认为 OFF，此时项目根本
 * 不会 @c find_package(OpenCV)。
 *
 * 约束：本文件只允许依赖标准库，任何 @c cv:: 类型都必须留在实现头中。
 */

#include <cstddef>
#include <filesystem>

namespace myslam::dataset
{

    /**
     * @brief SubT Handheld1 的 MEI 相机参数（与 intrinsics_subt_handheld.yaml 一致）
     */
    struct MeiIntrinsics
    {
        // 原始 MEI 内参（yaml 中 calibration 前四项）
        double fx{758.3153257832925};
        double fy{676.6492212772476};
        double cx{318.27111164892506};
        double cy{239.79816832491474};
        // MEI 模型参数与畸变系数
        double xi{1.583106303248484};
        double k1{-0.059098218173967695};
        double k2{0.1793477408661115};
        double p1{0.0016819528105368057};
        double p2{-0.0005887999624264534};
        // 虚拟针孔焦距缩放（与 MASt3R-Fusion 的 K_opt = K * scale 一致）
        double pinholeScale{0.4};
        int imageWidth{640};
        int imageHeight{480};

        double pinholeFx() const { return fx * pinholeScale; }
        double pinholeFy() const { return fy * pinholeScale; }
        double pinholeCx() const { return cx; }
        double pinholeCy() const { return cy; }

        bool isValid() const { return fx > 0.0 && fy > 0.0 && pinholeScale > 0.0 && imageWidth > 0 && imageHeight > 0; }
    };

    /**
     * @brief 可视化配置
     */
    struct VisualizationConfig
    {
        bool showWindow{false};          ///< 打开实时窗口
        std::filesystem::path videoPath; ///< 视频输出路径，空则不写
        int displayStride{1};            ///< 每多少帧渲染一次（1=每帧）
        int panelHeight{180};            ///< 底部状态栏高度（像素）
        int trailMaxPoints{20000};       ///< 轨迹小地图保留的最大点数
        int videoFps{20};                ///< 视频帧率
        /// 播放倍速：0 = 不限速（离线全速，约 100+ 帧/秒）；
        /// 1 = 按相机时间戳实时播放；2 = 两倍速。仅作用于实时窗口。
        double playbackRate{0.0};
    };

    /**
     * @brief 数据集回放配置
     */
    struct DatasetRunConfig
    {
        std::filesystem::path outputTum; ///< 关键帧轨迹输出（TUM 格式），空则不写
        int startFrame{0};               ///< 从第几帧开始回放
        int maxFrames{0};                ///< 最多处理的图像帧数，0 表示全部
        int stride{1};                   ///< 每隔多少帧取一帧
        int maxFeatures{200};            ///< 每帧特征数上限
        double maxDuration{0.0};         ///< 最长回放时长 (s)，0 表示不限
        std::filesystem::path plyPath;   ///< 路标/轨迹点云导出路径，空则不写
        VisualizationConfig viz;         ///< 实时可视化配置（窗口与视频）
    };

    /**
     * @brief 回放结果统计
     */
    struct DatasetRunReport
    {
        std::size_t framesProcessed{0};
        std::size_t keyframes{0};
        std::size_t imuSamples{0};
        std::size_t resets{0};
        bool initialized{false};
        double elapsedSeconds{0.0};
        double durationSeconds{0.0};
    };

} // namespace myslam::dataset
