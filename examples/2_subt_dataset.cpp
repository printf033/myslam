/**
 * @file 2_subt_dataset.cpp
 * @brief 在 SubT-MRS 数据集上回放 myslam VIO，并输出关键帧轨迹。
 *
 * 用法：
 * @code
 * 2_subt_dataset                        # 读 config/subt_handheld1.lua，按里面的配置跑
 * 2_subt_dataset --config abl.lua       # 用另一份配置（消融实验）
 * @endcode
 *
 * 所有运行时参数都在 Lua 里：序列路径、内参来源、MASt3R 选项、起止帧、
 * 输出路径、可视化开关。本程序只接受 @c --config 与 @c --help：保留一个
 * 「用哪份配置」的入口，而不是继续维护一套与 Lua 字段一一对应、却总是
 * 忘记同步的命令行参数。消融实验请写独立的 lua 文件。
 */

#include "app/replay.hpp"
#include "config/lua_config.hpp"
#include "domain/config.hpp"
#include "domain/types.hpp"
#include "log_config.hpp"
#include "logger.hpp"
#include "sensor/intrinsics_file.hpp"
#include "sensor/subt_dataset.hpp"
#include "viz/replay_observer.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <spawn.h>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

#if defined(__SANITIZE_ADDRESS__) || (defined(__has_feature) && __has_feature(address_sanitizer))
extern "C" const char *__lsan_default_suppressions()
{
    return "leak:gtsam::guardedIsDebug\n";
}
#endif

// environ 必须声明在全局作用域：放进匿名命名空间会得到内部链接，
// posix_spawn 拿到的就不是 libc 的环境表了。
extern char **environ;

namespace
{

    void printUsage()
    {
        std::fprintf(stderr, "用法: 2_subt_dataset [--config FILE] [--help]\n"
                             "\n"
                             "所有运行时配置都在 Lua 文件里，不再接受命令行参数：\n"
                             "  dataset.sequence.*    序列目录与三个数据文件\n"
                             "  dataset.intrinsics.*  内参数值与来源（config / file / mast3r）\n"
                             "  dataset.extrinsic     回放专用相机-IMU 外参\n"
                             "  dataset.mast3r.*      MASt3R 估计选项\n"
                             "  dataset.replay.*      起止帧、步长、特征数、轨迹/点云输出\n"
                             "  dataset.viz.*         窗口、录像、播放倍速\n"
                             "  vio.* / system.*      算法与时序参数\n"
                             "\n"
                             "--config 不指定时读取当前目录的 config/subt_handheld1.lua；该文件不存在时\n"
                             "使用内置默认值（此时没有序列路径，回放会直接报错）。\n");
    }

    /// 用 posix_spawn 直接执行，不经过 shell：路径里的空格与引号不会被解释
    int runProcess(const std::vector<std::string> &args)
    {
        if (args.empty())
        {
            return -1;
        }
        std::vector<char *> argv;
        argv.reserve(args.size() + 1);
        for (const auto &arg : args)
        {
            argv.push_back(const_cast<char *>(arg.c_str()));
        }
        argv.push_back(nullptr);

        pid_t pid = 0;
        const int spawnError = posix_spawn(&pid, argv[0], nullptr, nullptr, argv.data(), environ);
        if (spawnError != 0)
        {
            return -1;
        }
        int status = 0;
        if (waitpid(pid, &status, 0) < 0)
        {
            return -1;
        }
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }

    /// 解析 Python 解释器：配置 > MYSLAM_PYTHON > python3
    std::string resolvePython(const myslam::dataset::Mast3rConfig &options)
    {
        if (!options.python.empty())
        {
            return options.python;
        }
        if (const char *env = std::getenv("MYSLAM_PYTHON"); env != nullptr && *env != '\0')
        {
            return env;
        }
        return "python3";
    }

    /**
     * @brief 用 MASt3R 估计内参
     *
     * 优先复用缓存文件：同一序列重复回放不必重跑 ViT-Large 推理。缓存缺失或
     * 用 `--force-estimate` 指定重算时调用 scripts/mast3r_intrinsics.py。
     *
     * @param dataDir 帧图像目录（cam_0）
     * @param options 估计参数
     * @param baseline 基线标定；其 xi 与畸变系数会被继承
     * @return 估计结果；失败时给出可操作的说明
     */
    std::expected<myslam::dataset::IntrinsicsEstimate, std::string>
    estimateIntrinsicsWithMast3r(const std::string &dataDir, const myslam::dataset::Mast3rConfig &options,
                                 const myslam::dataset::MeiIntrinsics &baseline)
    {
        const std::filesystem::path frames(dataDir);
        std::filesystem::path cache = options.cache;
        if (cache.empty())
        {
            cache = frames.parent_path() / "mast3r_intrinsics.txt";
        }

        if (!options.force && std::filesystem::is_regular_file(cache))
        {
            auto cached = myslam::dataset::loadIntrinsicsFile(cache, baseline);
            if (cached)
            {
                std::fprintf(stderr, "[info] 复用 MASt3R 内参缓存: %s（dataset.mast3r.force 可重算）\n",
                             cache.string().c_str());
                return cached;
            }
            std::fprintf(stderr, "[warning] 缓存 %s 无法解析（%s），重新估计\n", cache.string().c_str(),
                         cached.error().c_str());
        }

#ifdef MYSLAM_MAST3R_SCRIPT
        const std::string script = MYSLAM_MAST3R_SCRIPT;
#else
        const std::string script = "scripts/mast3r_intrinsics.py";
#endif
        if (!std::filesystem::is_regular_file(script))
        {
            return std::unexpected(std::format(
                "找不到 MASt3R 估计脚本: {}（可用 dataset.intrinsics.source = \"file\" 指定已有结果）", script));
        }

        const std::string python = resolvePython(options);
        std::vector<std::string> args{
            python,     script,         "--images", dataDir,       "--pairs", std::to_string(options.pairs),
            "--device", options.device, "--output", cache.string()};
        if (!options.weights.empty())
        {
            args.insert(args.end(), {"--weights", options.weights});
        }

        mylog::info("用 MASt3R 估计内参: {}（pairs={}, device={}）", python, options.pairs, options.device);
        const int exitCode = runProcess(args);
        if (exitCode != 0)
        {
            return std::unexpected(
                std::format("MASt3R 估计失败（退出码 {}）。请确认：\n"
                            "  1) Python 环境装了 torch/dust3r（可在 dataset.mast3r.python 或 MYSLAM_PYTHON 里"
                            "指向它，例如 conda 的 mast3r 环境）；\n"
                            "  2) 权重可用（dataset.mast3r.weights 或 MAST3R_WEIGHTS）；\n"
                            "  3) thirdparty/mast3r 已含嵌套子模块："
                            "git submodule update --init --recursive thirdparty/mast3r",
                            exitCode));
        }
        return myslam::dataset::loadIntrinsicsFile(cache, baseline);
    }

} // namespace

int main(int argc, char **argv)
{
    // 唯一的命令行参数：选哪份 Lua 配置。其余全部来自配置本身，
    // 这样「配置里写的」与「实际运行的」不会因为漏传参数而分叉。
    std::filesystem::path configPath = "config/subt_handheld1.lua";
    bool explicitConfig = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h")
        {
            printUsage();
            return 0;
        }
        if (arg == "--config")
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "--config 缺少文件路径\n");
                printUsage();
                return 2;
            }
            configPath = argv[++i];
            explicitConfig = true;
            continue;
        }
        std::fprintf(stderr, "未知参数: %s（所有运行时配置都在 Lua 文件里）\n", arg.c_str());
        printUsage();
        return 2;
    }

    // MYSLAM_CONSOLE_LOG=1 时保留 mylog 默认的 stdout 输出，便于调试崩溃路径
    // （文件输出带缓冲，异常终止可能丢失尾部日志）；此时也不让配置文件的
    // logger 节改写输出后端。
    const bool consoleLog = std::getenv("MYSLAM_CONSOLE_LOG") != nullptr;

    myslam::config::ProjectConfig project;
    const bool configFileExists = std::filesystem::is_regular_file(configPath);
    const myslam::config::LoadResult loaded =
        configFileExists ? myslam::config::load(configPath, project, /*applyLogging=*/!consoleLog)
                         : myslam::config::LoadResult(std::unexpected(
                               myslam::config::ConfigError{myslam::config::ConfigErrorKind::FILE_NOT_FOUND,
                                                           std::format("配置文件不存在: {}", configPath.string())}));

    if (!loaded && explicitConfig)
    {
        std::fprintf(stderr, "[error] %s (%s)\n", loaded.error().message.c_str(), loaded.error().kindName().data());
        return 2;
    }

    // 未从配置得到日志后端时回退到代码默认（当前工作目录的单文件输出）
    const bool loggingApplied = loaded.has_value() && loaded->logging;
    if (!loggingApplied && !consoleLog)
    {
        myslam::LogConfig logConfig;
        if (const char *level = std::getenv("MYSLAM_LOG_LEVEL"))
        {
            logConfig.level = level;
        }
        if (const auto result = myslam::configureLogging(logConfig); !result)
        {
            std::fprintf(stderr, "[warning] %s，沿用当前日志配置\n", result.error().message.c_str());
        }
    }
    if (!loaded)
    {
        std::fprintf(stderr, "[info] %s，使用内置默认配置\n", loaded.error().message.c_str());
    }

    // ---------------------------------------------------------------------
    // 序列位置：全部来自 dataset.sequence
    // ---------------------------------------------------------------------
    const myslam::dataset::DatasetInputConfig &input = project.input;
    if (!input.sequence.configured())
    {
        std::fprintf(stderr,
                     "[error] 未配置 dataset.sequence.root（配置文件: %s）\n"
                     "        在 Lua 里写 dataset.sequence.root = \"<序列目录>\"，\n"
                     "        该目录下应含 cam_0/ 与 imu/imu_data.csv。\n",
                     configPath.c_str());
        return 2;
    }
    const std::filesystem::path cameraDir = input.sequence.resolveCameraDir();
    const std::filesystem::path imuFile = input.sequence.resolveImuFile();
    const std::filesystem::path stampFile = input.sequence.resolveStampFile();

    int missing = 0;
    const std::pair<const char *, std::filesystem::path> required[] = {
        {"相机图像目录", cameraDir}, {"IMU 文件", imuFile}, {"时间戳文件", stampFile}};
    for (const auto &[what, path] : required)
    {
        if (!std::filesystem::exists(path))
        {
            std::fprintf(stderr, "❌ 缺少%s: %s\n", what, path.string().c_str());
            ++missing;
        }
    }
    if (missing != 0)
    {
        std::fprintf(stderr, "   请检查 %s 的 dataset.sequence（当前 root = %s）\n", configPath.c_str(),
                     input.sequence.root.string().c_str());
        return 1;
    }

    // ---------------------------------------------------------------------
    // 内参：dataset.intrinsics.source 决定下面的数值字段是否生效
    // ---------------------------------------------------------------------
    myslam::dataset::MeiIntrinsics intrinsics = project.intrinsics;
    {
        std::optional<myslam::dataset::IntrinsicsEstimate> estimate;
        switch (input.intrinsicsSource)
        {
        case myslam::dataset::IntrinsicsSource::CONFIG:
            break;
        case myslam::dataset::IntrinsicsSource::FILE:
        {
            auto loadedIntrinsics = myslam::dataset::loadIntrinsicsFile(input.intrinsicsFile, project.intrinsics);
            if (!loadedIntrinsics)
            {
                std::fprintf(stderr, "[error] 读取内参文件 %s 失败: %s\n", input.intrinsicsFile.string().c_str(),
                             loadedIntrinsics.error().c_str());
                return 2;
            }
            estimate = std::move(*loadedIntrinsics);
            break;
        }
        case myslam::dataset::IntrinsicsSource::MAST3R:
        {
            auto estimated = estimateIntrinsicsWithMast3r(cameraDir.string(), input.mast3r, project.intrinsics);
            if (!estimated)
            {
                std::fprintf(stderr, "[error] %s\n", estimated.error().c_str());
                return 2;
            }
            estimate = std::move(*estimated);
            break;
        }
        }

        if (estimate)
        {
            for (const std::string &warning : estimate->warnings)
            {
                std::fprintf(stderr, "[warning] %s\n", warning.c_str());
            }
            intrinsics = estimate->intrinsics;
            // 两个 "model" 不是一回事，日志里分开写：estimate->model 是估计工具
            // 假设的投影模型（只用于把 focal 换算成焦距），intrinsics.model 才是
            // 回放前端实际使用的相机模型。
            mylog::info("内参来源: {}（估计假设 {} / 相机模型 {}），虚拟针孔焦距 {:.2f} px，主点 ({:.2f}, {:.2f})，"
                        "画幅 {}x{}",
                        estimate->source, estimate->model, myslam::camera::cameraModelName(intrinsics.model),
                        intrinsics.pinholeFx(), intrinsics.cx, intrinsics.cy, intrinsics.imageWidth,
                        intrinsics.imageHeight);
            if (estimate->inheritedDistortion)
            {
                mylog::info("畸变参数继承自配置: xi={:.4f}, k1={:.5f}, k2={:.5f}, 虚拟针孔缩放={:.4f}", intrinsics.xi,
                            intrinsics.k1, intrinsics.k2, intrinsics.pinholeScale);
            }
            else
            {
                mylog::info("畸变参数取自估计文件: xi={:.4f}, k1={:.5f}, k2={:.5f}, 虚拟针孔缩放={:.4f}", intrinsics.xi,
                            intrinsics.k1, intrinsics.k2, intrinsics.pinholeScale);
            }
            mylog::info("注意: 自动估计只覆盖焦距；相机-IMU 外参取 dataset.extrinsic 或 vio.extrinsic");
        }
        else
        {
            mylog::info("内参来源: 配置 dataset.intrinsics（相机模型 {}），fx={:.2f}, fy={:.2f}, 画幅 {}x{}",
                        myslam::camera::cameraModelName(intrinsics.model), intrinsics.fx, intrinsics.fy,
                        intrinsics.imageWidth, intrinsics.imageHeight);
        }
    }

    if (!intrinsics.isValid())
    {
        std::fprintf(stderr, "相机内参非法: fx=%f fy=%f size=%dx%d\n", intrinsics.fx, intrinsics.fy,
                     intrinsics.imageWidth, intrinsics.imageHeight);
        return 2;
    }

    // 回放配置整份来自 dataset.replay / dataset.viz；轨迹的父目录由程序创建，
    // 否则「写了 outputTum 却静默没生成文件」会在回放结束后才被发现。
    myslam::dataset::DatasetRunConfig runConfig = project.replay;
    if (!runConfig.outputTum.empty())
    {
        const std::filesystem::path parent = runConfig.outputTum.parent_path();
        if (!parent.empty())
        {
            std::error_code ec;
            std::filesystem::create_directories(parent, ec);
            if (ec)
            {
                std::fprintf(stderr, "[warning] 无法创建轨迹输出目录 %s: %s\n", parent.string().c_str(),
                             ec.message().c_str());
            }
        }
    }

    myslam::dataset::SubtSequence sequence(cameraDir, imuFile, stampFile);
    if (const auto loadedSequence = sequence.load(); !loadedSequence)
    {
        // 具体原因已由 SubtSequence 记录到日志，这里只给出面向终端的摘要
        std::fprintf(stderr, "[error] %s\n", loadedSequence.error().message.c_str());
        return 1;
    }

    // 最终归一化：针孔模型下不允许残留 MEI 的球面参数。
    // 配置解析与内参文件两条路径已各自归一化，但 --xi / --pinhole-scale 直接
    // 写字段，绕过了它们；不在这里再收一次，
    // `--config <pinhole 配置> --xi 1.5` 就会静默恢复鱼眼反解。
    if (intrinsics.normalizeForModel())
    {
        std::fprintf(stderr, "[warning] 相机模型为 pinhole，xi 已置 0（针孔模型没有球面反投影）\n");
    }

    // 观测像素的语义由估计器声明的观测模型决定（见 EstimatorBundle）：
    // gtsam_fixed_lag 只做针孔重投影，因此它会选折算后的虚拟针孔模型
    // （MEI 自动乘 pinholeScale，针孔直接使用）。回放只需把标定交给配置，
    // 由装配层与适配器决定怎么消费。
    // 以配置文件的 vio 节为基线再覆盖——原先直接 default 构造配置，
    // 导致 config/subt_handheld1.lua 里所有 vio.* 参数（门限、窗口、噪声、Huber…）
    // 在这条回放路径上全部静默失效。
    myslam::domain::PipelineConfig vioConfig = project.vio;
    vioConfig.camera.intrinsics = intrinsics;
    // IMU 采样率以序列为准，这是物理约束而不是配置覆盖：它同时决定噪声
    // 离散化与断档判定周期，写死配置值会让时间同步按错误的周期判断层。
    vioConfig.imu.rateHz = sequence.estimateImuRate() > 0.0 ? sequence.estimateImuRate() : 200.0;
    // 回放专用外参：dataset.extrinsic 优先，未配置则沿用 vio.extrinsic。
    // 以前这段是硬编码在这里的，导致配置文件的 vio.extrinsic 在回放路径
    // 上被静默覆盖——配置改了却不生效，比没有配置更糟。
    if (input.hasExtrinsic)
    {
        vioConfig.camera.bodyPSensor = input.extrinsic;
    }

    // 视频帧率：配置里写 0 表示跟随序列（SubT Handheld 约 24.5 Hz），否则用
    // 配置值。以前这段无条件覆盖 videoFps，配置里写什么都无效；录制时长
    // 与真实序列一致仍然由「0 = 跟随」保证。
    const double cameraRate = sequence.estimateCameraRate();
    if (runConfig.viz.videoFps <= 0 && cameraRate > 0.0)
    {
        runConfig.viz.videoFps = static_cast<int>(std::lround(cameraRate));
    }

    // IMU 实际采样率以序列为准：它同时决定噪声离散化与断档判定周期。
    // 回放由序列时间戳驱动，不读 SystemConfig 的 cameraDt / totalDuration /
    // rmseThreshold——那些描述仿真与打分口径。初始化超时属于
    // vio.initialization.timeoutSeconds（Lua 键 system.initTimeout）。

    mylog::info("[回放配置] 外点门限 软/硬/预优化 = {:.1f}/{:.1f}/{:.1f} px, 关键帧视差 {:.1f} px, "
                "固定滞后窗口 {:.2f}s, Huber delta {:.3f}",
                vioConfig.vision.maxReprojectionError, vioConfig.vision.maxReprojectionErrorHard,
                vioConfig.vision.maxReprojectionErrorPreopt, vioConfig.vision.minParallaxPx,
                vioConfig.estimator.slidingWindowLag, vioConfig.vision.huberLossDelta);
    if (!vioConfig.initialization.gravityAlignment)
    {
        mylog::warn("[回放配置] 重力对齐已关闭（消融实验）：不估初始姿态与零偏，初始姿态取单位阵、零偏置零");
        if (vioConfig.initialization.initialTiltDeg != 0.0)
        {
            mylog::warn("[回放配置] 故意注入初始倾斜 {:.2f}°（方位角 {:.1f}°），零偏置零；"
                        "后端需要自己把姿态拉回来",
                        vioConfig.initialization.initialTiltDeg, vioConfig.initialization.initialTiltAzimuthDeg);
        }
    }
    else if (vioConfig.initialization.initialTiltDeg != 0.0)
    {
        mylog::warn("[回放配置] vio.initialTiltDeg = {:.2f}° 已忽略：重力对齐开启时会估出真实姿态并覆盖它"
                    "（需同时把 vio.gravityAlignment 设为 false）",
                    vioConfig.initialization.initialTiltDeg);
    }
    mylog::info("[回放配置] 静止检测超时 {:.2f}s（0 = 不超时，严格等静止）", project.vio.initialization.timeoutSeconds);

    mylog::info("[回放配置] 相机-IMU 外参来源: {}", input.hasExtrinsic ? "dataset.extrinsic" : "vio.extrinsic");

    // 初始化超时属于 vio.initialization（Lua 键 system.initTimeout），已随
    // vioConfig 传入；回放器本身不读 system 节的时序与验收口径。
    // 显示与录像由观察者实现；未开窗口也未指定视频时不构造它，
    // 回放器因此不会产生任何 cv::imshow / VideoWriter 调用。
    std::unique_ptr<myslam::vio::IRunObserver> observer;
    if (runConfig.viz.showWindow || !runConfig.viz.videoPath.empty())
    {
        // 观测模型必须与流水线一致：先按配置构造一个流水线取得估计器声明的模型
        myslam::vio::VioPipeline probe(vioConfig);
        observer = std::make_unique<myslam::viz::ReplayObserver>(runConfig.viz, intrinsics, probe.observationModel());
    }
    // system.timingCsv 之前没有消费者：配置里写了路径、解析进了
    // SystemConfig，但回放器拿不到它，分段耗时永远只存在于一条日志里。
    // 这类「写了但静默不生效」的字段比不存在的字段更难排查。
    // system.timingCsv 之前没有消费者：配置里写了路径、也解析进了
    // SystemConfig，但回放器拿不到它，分段耗时永远只存在于一条日志里。
    // 这类「写了但静默不生效」的字段比不存在的字段更难排查。
    myslam::dataset::ReplayRunner runner(vioConfig, intrinsics, runConfig, std::move(observer),
                                         project.system.timingCsvPath);
    const myslam::dataset::DatasetRunReport report = runner.run(sequence);

    if (!runConfig.outputTum.empty())
    {
        mylog::info("关键帧轨迹已写入: {}", runConfig.outputTum.string());
    }
    return report.initialized ? 0 : 1;
}
