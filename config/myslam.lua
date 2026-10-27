-- myslam 运行期配置
--
-- 由 myslam::config::load("config/myslam.lua", cfg) 载入：
--   * vio / system 两节 -> myslam::vio::VioConfig / SystemConfig
--   * logger / fileManager 两节 -> 交给子库 mylog 应用
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
    -- 相机内参（针孔模型）
    fx = 460.0,
    fy = 460.0,
    cx = 320.0,
    cy = 240.0,
    imageWidth = 640.0,
    imageHeight = 480.0,

    -- 相机相对 IMU 的外参 tBc（Camera in Body frame）
    -- 工业标准定义：机身前向 (+X_b) 对应相机光轴 (+Z_c)，机身右向 (-Y_b) 对应
    -- 相机横轴 (+X_c)，机身下向 (-Z_b) 对应相机纵轴 (+Y_c)。
    -- 四元数按单位化处理，无需自己保证模长为 1；两个子表可独立省略。
    extrinsic = {
        quaternion = { w = -0.5, x = 0.5, y = -0.5, z = 0.5 },
        translation = { x = 0.10, y = 0.0, z = 0.05 },
    },

    -- IMU 噪声特性（连续时间功率谱密度，200 Hz 采样下样本标准差 = sigma / sqrt(dt)）
    accelNoiseSigma = 0.0057, -- 加速度计白噪声 (m/s^2 / sqrt(Hz))
    gyroNoiseSigma = 3.6e-4,  -- 陀螺仪白噪声 (rad/s / sqrt(Hz))
    accelBiasRw = 3e-4,       -- 加速度计零偏随机游走 (m/s^3 / sqrt(Hz))
    gyroBiasRw = 3e-5,        -- 陀螺仪零偏随机游走 (rad/s^2 / sqrt(Hz))
    integrationNoise = 1e-8,  -- 数值积分不确定度
    imuRateHz = 200.0,        -- IMU 采样率 (Hz)：采样步长由它派生，不再单独配置

    -- 重力常数
    gravityMagnitude = 9.81, -- 重力加速度模长 (m/s^2)

    -- 视觉观测噪声与鲁棒核
    pixelNoiseSigma = 1.0,             -- 特征点重投影高斯噪声 (像素)
    huberLossDelta = 1.345,            -- Huber 损失阈值
    maxReprojectionError = 8.0,        -- 优化后外点软门限 (像素)
    maxReprojectionErrorHard = 15.0,   -- 优化后外点硬门限 (像素)，应不严于软门限
    maxReprojectionErrorPreopt = 50.0, -- 预优化预门控宽门限 (像素)，应最宽松
    postOptOutlierMarking = true,      -- 诊断开关：优化后是否永久标记外点
    preoptTemporaryRejection = true,   -- 诊断开关：预优化是否临时拒绝超限观测

    -- 关键帧选取策略
    minParallaxPx = 20.0,              -- 触发关键帧的平均视差阈值 (像素)
    minTrackingRatio = 0.65,           -- 触发关键帧的特征留存率下限，取值 [0, 1]
    minKeyframeInterval = 0.15,        -- 两次关键帧的最小时间间隔 (s)
    maxKeyframeInterval = 0.45,        -- 强制插入关键帧的最大时间间隔 (s)
    minTriangulationParallaxPx = 3.0,  -- 三角化所需最小视差 (像素)
    landmarkGateWarmupObs = 4,         -- 新路标门控保护期（观测次数）
    extraSmootherIterations = 1,       -- 每个关键帧的额外平滑迭代次数

    -- 滑动窗口
    maxSlidingWindowSize = 8, -- 滑动窗口内最大关键帧数
    slidingWindowLag = 1.6,   -- 固定滞后平滑窗口时长 (s)
}

-- =============================================================================
-- 运行时序与验收门限
-- 注意：IMU 采样步长 = 1 / vio.imuRateHz，不是可配项。
-- =============================================================================
system = {
    cameraDt = 0.050,     -- 相机采样步长 (s)，对应 20 Hz，须不小于 IMU 采样步长
    totalDuration = 5.0,  -- 仿真/运行时长 (s)
    rmseThreshold = 0.20, -- 平移 RMSE 验收门限 (m)
}

-- =============================================================================
-- 数据集回放（仅 2_subt_dataset 使用；本节为纯数据，不需要 OpenCV 即可解析）
-- =============================================================================
dataset = {
    -- MEI 相机内参（默认值与 intrinsics_subt_handheld.yaml 一致）
    intrinsics = {
        fx = 758.3153257832925,
        fy = 676.6492212772476,
        cx = 318.27111164892506,
        cy = 239.79816832491474,
        xi = 1.583106303248484,  -- MEI 模型参数，视场 > 180° 时可达 1.5+
        k1 = -0.059098218173967695,
        k2 = 0.1793477408661115,
        p1 = 0.0016819528105368057,
        p2 = -0.0005887999624264534,
        pinholeScale = 0.4, -- 虚拟针孔焦距缩放（K_opt = K * scale）
        imageWidth = 640,
        imageHeight = 480,
    },

    -- 回放范围与输出；路径写空串表示不输出
    replay = {
        outputTum = "",  -- 关键帧轨迹输出（TUM 格式）
        plyPath = "",    -- 路标/轨迹点云导出路径
        startFrame = 0,   -- 从第几帧开始回放
        maxFrames = 0,    -- 最多处理的图像帧数，0 = 全部
        stride = 1,       -- 每隔多少帧取一帧，最小 1
        maxFeatures = 200, -- 每帧特征数上限
        maxDuration = 0.0, -- 最长回放时长 (s)，0 = 不限
        -- IMU 断档判定倍数：相邻间隔超过 maxImuGapFactor / vio.imuRateHz 即视为断层。
        -- 200 Hz 下默认 10 倍 = 50 ms；断档处不跨缺口积分，并重建流水线重新初始化。
        maxImuGapFactor = 10.0,
    },

    -- 实时可视化（需要 OpenCV highgui；两者都关闭时不做渲染）
    viz = {
        showWindow = false, -- 打开实时窗口（需要 DISPLAY）
        videoPath = "",     -- 视频输出路径，空则不写
        displayStride = 1,  -- 每多少帧渲染一次
        panelHeight = 180,  -- 底部状态栏高度（像素）
        trailMaxPoints = 20000, -- 轨迹小地图保留的最大点数
        videoFps = 20,          -- 视频帧率
        playbackRate = 0.0,      -- 0 = 不限速；1 = 实时；2 = 两倍速
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
