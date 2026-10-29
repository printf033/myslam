function(gen_bin name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE myslam::myslam myslam::myslam_sanitized)
    # Lua 配置支持可选：关闭 MYSLAM_ENABLE_LUA 时 myslam_lua 目标不存在
    if(TARGET myslam::myslam_lua)
        target_link_libraries(${name} PRIVATE myslam::myslam_lua)
    endif()
endfunction()

gen_bin(1_visual_inertial_odometry)

# SubT 数据集回放依赖 OpenCV（图像解码、角点检测与光流），默认不构建，
# 需要时以 -DMYSLAM_BUILD_DATASET_TOOLS=ON 显式开启。OpenCV 的两种来源
# （系统包 / thirdparty/opencv 子模块）由 thirdparty 的 myslam::opencv 统一封装。
if(MYSLAM_BUILD_DATASET_TOOLS)
    add_executable(2_subt_dataset 2_subt_dataset.cpp)
    target_link_libraries(2_subt_dataset PRIVATE
        myslam::myslam
        myslam::myslam_sanitized
        myslam::opencv
    )
    # 可选 Lua 配置支持（与 gen_bin 生成的示例一致）
    if(TARGET myslam::myslam_lua)
        target_link_libraries(2_subt_dataset PRIVATE myslam::myslam_lua)
    endif()
endif()
