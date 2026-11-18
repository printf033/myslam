-- myslam 运行期配置：TUM RGB-D（freiburg1）数据源专用
--
-- 由 myslam::config::load("config/tum_vi.lua", cfg) 载入：
--   * vio 节 -> 针孔内参（freiburg1 官方标定）、单位阵外参与 TUM 传感器噪声
--   * system 节 -> myslam::vio::SystemConfig
--   * dataset.intrinsics -> 回放入口读取的相机内参（与 vio 节保持一致）
--   * logger / fileManager 两节 -> 交给子库 mylog 应用
--
-- 本文件是「数据源 ↔ 配置文件」一一对应约定下的 TUM 份：freiburg1 的内参、
-- 外参与 IMU 噪声都写在这里，C++ 源码中不再保留任何该数据集的默认值。
-- TUM RGB-D 没有陀螺仪，gyro 由真值姿态差分合成（见 4_tum_vio.cpp），因此
-- 外参必须是单位阵（body 系与相机系重合），IMU 噪声按 Kinect 实测谱密度给。
--
-- 安全说明：脚本在未注册标准库的裸 lua_State 中执行，io / os / package /
-- debug 均不可用，配置脚本无法读写文件或启动进程。字段类型不符会被忽略，
-- 数值越界会被钳位，拼错的字段名会在 stderr 上点名。
--
-- 全部字段都可省略：省略即沿用代码内置默认值，因此本文件同时充当默认值清单。

-- =============================================================================
-- 视觉惯性参数
-- =============================================================================
vio = {
    -- 估计器（后端）注册名：选哪个实现由这里决定，流水线一行不改。
    --   gtsam_fixed_lag  紧耦合固定滞后平滑（参考实现，只接受针孔重投影）
    --   imu_only         纯 IMU 航位推算（可替换性验证与 A/B 基线）
    -- 未知名字在装配期报错，不会静默回退到默认实现。
    estimator = "gtsam_fixed_lag",

    -- 相机模型：pinhole（针孔）或 mei（鱼眼）。gtsam_fixed_lag 折算为虚拟针孔，
    -- 支持全向重投影的估计器可直接使用 mei；
    -- mei 时下面的 xi / k1 / k2 / p1 / p2 / pinholeScale 才会生效。
    model = "pinhole",
    -- 相机内参（TUM freiburg1 官方标定，针孔模型）
    fx = 517.306408,
    fy = 516.469215,
    cx = 318.643040,
    cy = 255.313989,
    imageWidth = 640,
    imageHeight = 480,
    -- freiburg1 的径向畸变很大（k1≈0.26, k2≈-0.95），回放时由
    -- TumReplaySource 预计算重映射表矫正到无畸变图；不矫正的话图像边缘的
    -- 重投影误差可达几十像素。
    xi = 0.0,
    k1 = 0.262383,
    k2 = -0.953104,
    p1 = -0.005358,
    p2 = 0.002628,
    pinholeScale = 1.0,

    -- 相机相对 IMU 的外参 tBc（Camera in Body frame）
    -- TUM RGB-D 的 gyro 由真值相机姿态差分合成，body 系与相机系重合，
    -- 所以外参必须是单位阵；继承别的安装参数会让每帧重投影偏几十像素。
    extrinsic = {
        quaternion = { w = 1.0, x = 0.0, y = 0.0, z = 0.0 },
        translation = { x = 0.0, y = 0.0, z = 0.0 },
    },

    -- IMU 噪声特性（连续时间功率谱密度，200 Hz 采样下样本标准差 = sigma / sqrt(dt)）
    -- TUM 的传感器特性与常规手持 MEMS 差别很大：gyro 由真值差分，物理上无零偏、
    -- 噪声极低；accel 是 Kinect 加速度计，实测白噪声谱密度约 0.003 m/s^2/sqrt(Hz)。
    accelNoiseSigma = 0.0030, -- 加速度计白噪声 (m/s^2 / sqrt(Hz))
    gyroNoiseSigma = 1.0e-5,  -- 陀螺仪白噪声（真值差分，近似无噪声）
    accelBiasRw = 0.001,      -- 加速度计零偏随机游走 (m/s^3 / sqrt(Hz))
    gyroBiasRw = 1.0e-5,      -- 陀螺仪零偏随机游走（真值差分，近似无零偏）
    integrationNoise = 1e-8,  -- 数值积分不确定度
    imuRateHz = 200.0,        -- IMU 采样率 (Hz)：采样步长由它派生，不再单独配置

    -- 重力常数
    gravityMagnitude = 9.81, -- 重力加速度模长 (m/s^2)

    -- 相机-IMU 时间偏移 (s)：正值表示相机时间戳比 IMU 晚。由回放的时间同步器
    -- 统一扣除；目前是固定常数，不做在线估计。
    timeOffsetSeconds = 0.0,

    -- 曝光时长 (s)：图像内容对应曝光中点（触发时刻 + exposure/2）。数据集
    -- timestamps.txt 只记录触发时刻且无曝光元数据，保持 0 即不修正。
    exposureSeconds = 0.0,

    -- 初始化阶段是否做静止检测与重力粗对齐。设为 false 即消融实验：不估初始
    -- 姿态与零偏（初始姿态取单位阵、零偏置零），世界系 z 轴不再与真实重力对
    -- 齐，轨迹会整体倾斜，用于量化“对齐这一步”的贡献。与 --gravity-alignment off 等价。
    gravityAlignment = true,

    -- 消融用：人为把初始姿态弄歪，测量后端能不能拉回来（需 gravityAlignment = false）。
    -- 倾斜量绕世界系水平轴旋转，方位角决定轴的方向（0 = 绕 +X，即抬头/低头；
    -- 90 = 绕 +Y，即左右侧倾）。与 --initial-tilt / --initial-tilt-azimuth 等价。
    -- 注意：开启重力对齐时本项被忽略，因为对齐会估出真实姿态并覆盖它。
    initialTiltDeg = 0.0,
    initialTiltAzimuthDeg = 0.0,

    -- 视觉观测噪声与鲁棒核
    pixelNoiseSigma = 1.0,             -- 特征点重投影高斯噪声 (像素)
    huberLossDelta = 1.345,            -- Huber 损失阈值
    maxReprojectionError = 8.0,        -- 优化后外点软门限 (像素)
    maxReprojectionErrorHard = 15.0,   -- 优化后外点硬门限 (像素)，应不严于软门限
    maxReprojectionErrorPreopt = 50.0, -- 预优化预门控宽门限 (像素)，应最宽松
    outlierStreakThreshold = 3,        -- 软门限连续超限多少帧后标记为外点
    postOptOutlierMarking = true,      -- 诊断开关：优化后是否永久标记外点
    preoptTemporaryRejection = true,   -- 诊断开关：预优化是否临时拒绝超限观测

    -- 关键帧选取策略
    minParallaxPx = 20.0,              -- 触发关键帧的平均视差阈值 (像素)
    minTrackingRatio = 0.65,           -- 触发关键帧的特征留存率下限，取值 [0, 1]
    minKeyframeInterval = 0.15,        -- 两次关键帧的最小时间间隔 (s)
    maxKeyframeInterval = 0.45,        -- 强制插入关键帧的最大时间间隔 (s)
    minTriangulationParallaxPx = 3.0,  -- 三角化所需最小视差 (像素)
    landmarkGateWarmupObs = 4,         -- 新路标门控保护期（观测次数）
    -- 有效路标的相机系深度区间 (m)。超出该区间的路标投影因子雅可比随 1/z 发散，
    -- 单条观测即可让线性系统病态（iSAM2 抛 IndeterminateSystemException）。
    minLandmarkDepth = 0.3,            -- 深度下限，低于此值（含相机后方）视为几何无效
    maxLandmarkDepth = 20.0,           -- 深度上限
    minVisualFactorsForHealth = 5,     -- 视觉因子低于此值视为视觉失去约束能力
    zuptWindowSeconds = 0.5,           -- 零速更新的静止检测窗口 (s)
    zuptVelocitySigma = 0.02,          -- 零速更新的速度先验标准差 (m/s)
    extraSmootherIterations = 1,       -- 每个关键帧的额外平滑迭代次数

    -- 滑动窗口
    maxSlidingWindowSize = 8, -- 滑动窗口内最大关键帧数
    slidingWindowLag = 1.6,   -- 固定滞后平滑窗口时长 (s)

    -- 初始化判据：原为硬编码（40 样本 / 2 秒窗口），现可做消融
    initializerMinSamples = 40,     -- 判定所需最少 IMU 样本数
    initializerWindowSeconds = 2.0, -- 静止采样窗口时长 (s)，超出丢弃最旧样本
    minInitObservations = 15,       -- 首关键帧退化门控：有效观测数下限
    minInitSpreadDeg = 3.0,         -- 首关键帧退化门控：射线最大夹角下限 (deg)

    -- 健康监控阈值：原为硬编码，现可随平台/噪声量级调整
    healthMaxAccelBiasNorm = 1.0,      -- 加速度计零偏模长上限 (m/s^2)
    healthMaxGyroBiasNorm = 0.15,      -- 陀螺仪零偏模长上限 (rad/s)
    healthMaxSpeed = 20.0,             -- 系统速度上限 (m/s)
    healthMinActiveFeatures = 5,       -- 视觉观测数下限
    healthAlertStreakThreshold = 3,    -- 连续异常多少帧才降级

    -- 先验强度（标准差，m）：
    -- landmarkPriorSigma 是两视图三角化的软正则项，过强会压制多视图观测；
    -- recovery* 是局部恢复时以锚点重建平滑器的弱约束。
    initialPoseSigma = 0.05,
    initialVelSigma = 0.1,
    initialBiasSigma = 0.05,
    landmarkPriorSigma = 1.0,
    recoveryPoseSigma = 0.1,
    recoveryVelSigma = 0.2,
    recoveryBiasSigma = 0.1,
}

-- =============================================================================
-- 运行时序与验收门限
-- 注意：IMU 采样步长 = 1 / vio.imuRateHz，不是可配项。
-- =============================================================================
system = {
    cameraDt = 0.050,     -- 相机采样步长 (s)，对应 20 Hz，须不小于 IMU 采样步长
    totalDuration = 5.0,  -- 仿真/运行时长 (s)
    rmseThreshold = 0.20, -- 平移 RMSE 验收门限 (m)
    initTimeout = 2.0,    -- 初始化超时 (s)：超时后放宽静止检测，避免从运动状态开始时卡死
    -- 初始化写入的世界系高度 (m)：世界系原点取初始 IMU 位置，水平面由重力对齐确定，
    -- 高度是自由平移量，需要初值才能与仿真真值逐帧比较。
    initialHeightMeters = 1.0,
}

-- =============================================================================
-- 数据集回放（仅 2_subt_dataset 使用；本节为纯数据，不需要 OpenCV 即可解析）
-- =============================================================================
dataset = {
    -- -------------------------------------------------------------------------
    -- 序列位置（相对仓库根目录；也可以写绝对路径）
    --
    -- 约定在仓库根目录运行，相对路径都按当前工作目录解析。
    -- 换一条序列只改 root（或再调 replay.startFrame）。
    -- -------------------------------------------------------------------------
    sequence = {
        -- TUM 入口（4_tum_vio）用位置参数传序列目录，不读本节；留空避免误导。
        root = "",
    },

    -- -------------------------------------------------------------------------
    -- 相机内参
    --
    -- source 决定下面的数值字段是否真的生效：
    --   "config"  直接使用本节数值（默认）
    --   "file"    从 file 指定的标定文件读取（本节数值作为缺失字段的基线）
    --   "mast3r"  用 MASt3R 从图像估计焦距（缓存复用，见 mast3r 节）
    -- -------------------------------------------------------------------------
    intrinsics = {
        source = "config",
        file = "",

        model = "pinhole",
        fx = 517.306408,
        fy = 516.469215,
        cx = 318.643040,
        cy = 255.313989,
        xi = 0.0,
        k1 = 0.262383,
        k2 = -0.953104,
        p1 = -0.005358,
        p2 = 0.002628,
        pinholeScale = 1.0,
        imageWidth = 640,
        imageHeight = 480,
    },

    -- -------------------------------------------------------------------------
    -- MASt3R 内参估计选项（仅 intrinsics.source = "mast3r" 时使用）
    -- TUM 自带官方标定，正常不需要这一节；保留字段以便按需开启。
    -- -------------------------------------------------------------------------
    mast3r = {
        python = "",     -- 空 = 环境变量 MYSLAM_PYTHON，再退回 python3
        weights = "",    -- 空 = 脚本内置候选或环境变量 MAST3R_WEIGHTS
        pairs = 6,       -- 参与估计的图像对数 [1, 128]
        device = "auto", -- auto / cpu / cuda
        cache = "",      -- 空 = <sequence.root>/mast3r_intrinsics.txt
        force = false,   -- true = 忽略已有缓存重新估计
    },

    -- -------------------------------------------------------------------------
    -- 回放范围与输出
    -- -------------------------------------------------------------------------
    replay = {
        outputTum = "", -- 关键帧轨迹输出（TUM 格式），空串 = 由入口程序按序列名生成
        plyPath = "",   -- 路标/轨迹点云导出路径，空串 = 不导出
        startFrame = 0, -- TUM 序列从运动开始，无 SubT 那样的前段空洞问题
        maxFrames = 0,     -- 最多处理的图像帧数，0 = 全部
        stride = 1,        -- 每隔多少帧取一帧，最小 1
        maxFeatures = 200, -- 每帧特征数上限
        maxDuration = 0.0, -- 最长回放时长 (s)，0 = 不限
        -- IMU 断档判定倍数：相邻间隔超过 maxImuGapFactor / vio.imuRateHz 即视为断层。
        -- 200 Hz 下默认 10 倍 = 50 ms；断档处不跨缺口积分，并重建流水线重新初始化。
        maxImuGapFactor = 10.0,
    },

    -- 实时可视化（需要 OpenCV highgui；两者都关闭时不做渲染）
    viz = {
        showWindow = true, -- 打开实时窗口（需要 DISPLAY）
        videoPath = "",     -- 视频输出路径，空则不写
        -- 画面右侧并排显示 MEI 去畸变后的虚拟针孔图像（与特征点坐标系一致），
        -- 关掉可省掉每帧一次 remap，只看原始鱼眼图
        showUndistorted = true,
        displayStride = 1,  -- 每多少帧渲染一次
        panelHeight = 180,  -- 底部状态栏高度（像素）
        trailMaxPoints = 20000, -- 轨迹小地图保留的最大点数
        videoFps = 0,           -- 0 = 跟随序列的相机频率；非 0 = 固定帧率
        playbackRate = 0.0,     -- 0 = 不限速；1 = 实时；2 = 两倍速
    },
}

-- =============================================================================
-- 日志（由 mylog 解析，本程序不重复实现）
-- =============================================================================
logger = {
    -- 低于该级别的日志被直接丢弃
    -- Options: "trace" | "debug" | "info" | "warn" | "error" | "fatal"
    suppressLevel = "info",

    -- true 时级别标签带 ANSI 颜色；只适合终端，写文件请用 false
    stdoutVivid = false,

    -- 是否转义控制字符（含 ESC / \x7f），阻断日志注入伪造 ANSI 序列
    escapeControlChars = true,

    -- 时间戳用 UTC（true）还是本地时间（false）
    timestampUtc = false,

    -- 单条日志字节上限，合法区间 [64 B, 64 MiB]
    maxLogLength = 1024 * 1024, -- 1 MiB

    -- FATAL 触发后的系统动作
    -- "none" 适合常驻服务与测试；"exit" 正常退出；"abort" 崩溃转储
    fatalAction = "none",
}

fileManager = {
    -- 每线程私有写缓冲（字节），0 = 关闭并退回「全局锁 + 共享缓冲」
    threadBufferCapacity = 64 * 1024, -- 64 KiB

    -- 共享写缓冲（字节）
    fileBufferCapacity = 128 * 1024, -- 128 KiB

    -- 单个日志文件上限（字节），超过即滚动，合法区间 [1 KiB, 64 GiB]
    fileCapacity = 100 * 1024 * 1024, -- 100 MiB

    -- 距上次写盘超过该毫秒数就 flush，合法区间 [0, 24h]
    intervalFlushFile_ms = 30 * 1000, -- 30s

    -- 保留的日志文件数上限，0 = 不清理
    maxFiles = 5,
}
