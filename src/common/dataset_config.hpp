#pragma once

/**
 * @file dataset_config.hpp
 * @brief 数据集回放的纯数据配置与统计结构（不含 OpenCV 依赖）。
 *
 * 这些结构之所以从各自实现文件中抽离，是为了让不依赖 OpenCV 的代码
 * （例如 @c src/lua_config.hpp 的配置解析）能够引用它们而不把 OpenCV
 * 变成必选依赖：@c MYSLAM_BUILD_DATASET_TOOLS 默认为 OFF，此时项目根本
 * 不会 @c find_package(OpenCV)。
 *
 * 约束：本文件只允许依赖标准库，任何 @c cv:: 类型都必须留在实现头中。
 */

#include "domain/camera_intrinsics.hpp"
#include "domain/types.hpp"
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

namespace myslam::dataset
{

    /**
     * @brief 数据集相机内参
     *
     * 类型统一为 @c myslam::camera::CameraIntrinsics（见 @c camera_model.hpp），
     * 针孔与 MEI 由 @c model 字段区分。保留 @c MeiIntrinsics 这个名字，因为回放
     * 流水线、配置解析与测试都按它命名，改名没有收益。
     */
    using MeiIntrinsics = camera::CameraIntrinsics;

    /**
     * @brief 内参的来源
     *
     * 三者互斥：@c CONFIG 直接用 @c dataset.intrinsics 的数值，@c FILE 从外部
     * 标定文件读，@c MAST3R 从图像估计。把「从哪来」与「值是什么」分成两个
     * 字段，回放器就不必再猜用户是不是忘了填 @c --fx。
     */
    enum class IntrinsicsSource : std::uint8_t
    {
        CONFIG, ///< 使用 @c dataset.intrinsics 里的数值
        FILE,   ///< 从 @c dataset.intrinsics.file 指定的标定文件读取
        MAST3R, ///< 调用 MASt3R 从图像估计（结果带缓存）
    };

    [[nodiscard]] inline std::string_view intrinsicsSourceName(IntrinsicsSource source) noexcept
    {
        switch (source)
        {
        case IntrinsicsSource::CONFIG:
            return "config";
        case IntrinsicsSource::FILE:
            return "file";
        case IntrinsicsSource::MAST3R:
            return "mast3r";
        }
        return "unknown";
    }

    /**
     * @brief 回放序列的位置
     *
     * @c root 是序列根目录，其余三个是**相对 root 的路径**。相对路径统一由
     * @c resolve* 拼接，调用点不再各自做字符串拼接——那是漏掉某一条路径的
     * 经典来源。写成绝对路径时原样使用，便于把配置指向别处的序列。
     */
    struct SequenceConfig
    {
        std::filesystem::path root;
        std::filesystem::path cameraDir{"cam_0"};                ///< 帧图像目录
        std::filesystem::path imuFile{"imu/imu_data.csv"};       ///< IMU CSV
        std::filesystem::path stampFile{"cam_0/timestamps.txt"}; ///< 相机时间戳

        [[nodiscard]] std::filesystem::path resolveCameraDir() const
        {
            return cameraDir.is_absolute() ? cameraDir : root / cameraDir;
        }
        [[nodiscard]] std::filesystem::path resolveImuFile() const
        {
            return imuFile.is_absolute() ? imuFile : root / imuFile;
        }
        [[nodiscard]] std::filesystem::path resolveStampFile() const
        {
            return stampFile.is_absolute() ? stampFile : root / stampFile;
        }
        [[nodiscard]] bool configured() const noexcept { return !root.empty(); }
    };

    /**
     * @brief MASt3R 内参估计选项
     *
     * 整组参数只在 @c dataset.intrinsics.source = "mast3r" 时被读取；其它来源
     * 下配置了也不报错，避免切换来源时被迫删字段。
     */
    struct Mast3rConfig
    {
        std::string python;          ///< Python 解释器，空则 MYSLAM_PYTHON 或 python3
        std::string weights;         ///< 权重路径，空则用脚本内置候选
        int pairs{6};                ///< 参与估计的图像对数
        std::string device{"auto"};  ///< auto / cpu / cuda
        std::filesystem::path cache; ///< 缓存文件，空则 <序列根>/mast3r_intrinsics.txt
        bool force{false};           ///< 忽略已有缓存，强制重算
    };

    /**
     * @brief SubT Handheld1 的 MEI 基线标定（与 intrinsics_subt_handheld.yaml 一致）
     *
     * 它以前是 @c MeiIntrinsics 的结构体默认值。统一内参类型后，默认值属于
     * 通用模型（针孔仿真），数据集专属标定改为显式工厂：新建一条序列时
     * 不会再静默继承别的数据集的标定。
     */
    [[nodiscard]] inline MeiIntrinsics subtHandheld1Intrinsics() noexcept
    {
        return camera::CameraIntrinsics::subtHandheld1Mei();
    }

    /**
     * @brief 可视化配置
     */
    struct VisualizationConfig
    {
        bool showWindow{false};          ///< 打开实时窗口
        std::filesystem::path videoPath; ///< 视频输出路径，空则不写
        /// 是否在画面右侧并排显示 MEI 去畸变后的虚拟针孔图像。
        /// 该图与跟踪器输出的特征像素坐标同源，能直观看出鱼眼畸变被拉直的效果。
        bool showUndistorted{true};
        int displayStride{1};      ///< 每多少帧渲染一次（1=每帧）
        int panelHeight{180};      ///< 底部状态栏高度（像素）
        int trailMaxPoints{20000}; ///< 轨迹小地图保留的最大点数
        /// 视频帧率。0 = 跟随序列的实际相机频率（回放时从时间戳估算），
        /// 非 0 = 使用该值。以前这里恒为 20 并被回放器无条件覆盖，配置永远不生效。
        int videoFps{0};
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
        /// 实时可视化配置（窗口与视频）。回放器本身不读它：编排层据此构造
        /// @c viz/ReplayObserver 并通过 @c IRunObserver 注入，回放器不含 viz 类型。
        VisualizationConfig viz;
        /// IMU 断档判定倍数（Lua 键 @c dataset.replay.maxImuGapFactor）。
        ///
        /// 这是历史配置入口：解析后会被搬到 `domain::ImuConfig::maxGapFactor`，
        /// 由 `TimeSynchronizer` 统一消费，保证阈值只有一个真值来源。
        double maxImuGapFactor{10.0};
    };

    /**
     * @brief 数据集回放的完整输入：序列位置、内参来源、估计选项
     *
     * 与 @c DatasetRunConfig 分开：前者回答「数据与标定从哪来」，后者回答
     * 「怎么跑、写到哪里」。回放器只消费 @c DatasetRunConfig，数据加载与
     * 内参解析发生在编排层，两者不必共享一个结构。
     */
    struct DatasetInputConfig
    {
        SequenceConfig sequence{};
        IntrinsicsSource intrinsicsSource{IntrinsicsSource::CONFIG};
        std::filesystem::path intrinsicsFile; ///< source == FILE 时使用
        Mast3rConfig mast3r{};
        /// 回放专用的相机-IMU 外参。未配置时沿用 @c vio.extrinsic；
        /// 配置了则覆盖它。每条序列的安装方式不同，把外参写进流水线配置
        /// 会逼着换序列时改算法参数，所以给它一个独立的回放入口。
        bool hasExtrinsic{false};
        domain::Pose extrinsic{};
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
        /// 后端在求解失败后完成局部恢复（重建平滑器但保留坐标系）的关键帧数。
        /// 这些帧不产出新位姿，但轨迹坐标保持连续 —— 与 @c resets 有本质区别：
        /// @c resets 会让坐标系归零，本字段代表的失败不会。
        std::size_t recoveredFrames{0};
        bool initialized{false};
        double elapsedSeconds{0.0};
        /// 实际回放到的时间跨度 (s)。中途退出时是已处理进度，不是序列总长。
        double durationSeconds{0.0};
        /// 视觉观测中断（图像变暗/遮挡/无纹理）期间的关键帧数。这些帧不重建
        /// 流水线，坐标系保持连续，仅靠 IMU 维持轨迹 —— 与 @c resets 无关。
        std::size_t visualOutageFrames{0};
        /// 视觉中断告警是否已上报（只在首次中断时打一条，避免刷屏）
        bool visualOutageReported{false};
        /// 处理过的流水线段数。每段因 IMU 断档或优化失败而重建，各自从局部
        /// 原点重新初始化，坐标系互不相同，轨迹不可跨段直接拼接。
        std::size_t segments{0};
        /// 各段关键帧数之和与跨段累积的关键帧列表不一致时，以本字段为准
        std::size_t trajectorySamples{0};
    };

} // namespace myslam::dataset
