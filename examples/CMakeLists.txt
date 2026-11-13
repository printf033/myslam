function(gen_bin name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE myslam::myslam myslam::myslam_sanitized)
    # Lua 配置支持可选：关闭 MYSLAM_ENABLE_LUA 时 myslam_lua 目标不存在
    if(TARGET myslam::myslam_lua)
        target_link_libraries(${name} PRIVATE myslam::myslam_lua)
    endif()
endfunction()

gen_bin(1_visual_inertial_odometry)

# rosbag2 数据集检查：只依赖 sensor 层（SQLite + CDR 解码），不链接 OpenCV，
# 因此不受 MYSLAM_BUILD_DATASET_TOOLS 门控。
gen_bin(3_rosbag_inspect)

# SubT 数据集回放依赖 OpenCV（图像解码、角点检测与光流），默认不构建，
# 需要时以 -DMYSLAM_BUILD_DATASET_TOOLS=ON 显式开启。OpenCV 的两种来源
# （系统包 / thirdparty/opencv 子模块）由 thirdparty 的 myslam::opencv 统一封装。
if(MYSLAM_BUILD_DATASET_TOOLS)
    add_executable(2_subt_dataset 2_subt_dataset.cpp)
    target_link_libraries(2_subt_dataset PRIVATE
        myslam::myslam
        myslam::replay
        myslam::frontend_cv
        myslam::viz
        myslam::myslam_sanitized
    )
    # 可选 Lua 配置支持（与 gen_bin 生成的示例一致）
    if(TARGET myslam::myslam_lua)
        target_link_libraries(2_subt_dataset PRIVATE myslam::myslam_lua)
    endif()

    # --init mast3r 需要调用仓库内的内参估计脚本。把绝对路径编进去，
    # 避免二进制在其它工作目录下运行时找不到 scripts/。
    target_compile_definitions(2_subt_dataset PRIVATE
        MYSLAM_MAST3R_SCRIPT="${PROJECT_SOURCE_DIR}/scripts/mast3r_intrinsics.py")

    # rosbag2 数据集跑 VIO：需要 OpenCV 解码 JPEG / 原始像素，走 ReplayRunner
    # 的同一条回放管道（数据源实现 IReplaySource）。
    add_executable(3_rosbag_vio 3_rosbag_vio.cpp)
    target_link_libraries(3_rosbag_vio PRIVATE
        myslam::myslam
        myslam::replay
        myslam::frontend_cv
        myslam::viz
        myslam::myslam_sanitized
    )
    if(TARGET myslam::myslam_lua)
        target_link_libraries(3_rosbag_vio PRIVATE myslam::myslam_lua)
    endif()

    # TUM RGB-D 数据集：有运动捕捉真值，回放后可直接算 ATE。
    # 需要 OpenCV 读图，且 gyro 由真值差分合成（数据集无陀螺仪）。
    add_executable(4_tum_vio 4_tum_vio.cpp)
    target_link_libraries(4_tum_vio PRIVATE
        myslam::myslam
        myslam::replay
        myslam::frontend_cv
        myslam::viz
        myslam::myslam_sanitized
    )
    if(TARGET myslam::myslam_lua)
        target_link_libraries(4_tum_vio PRIVATE myslam::myslam_lua)
    endif()
endif()
