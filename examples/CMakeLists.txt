function(gen_bin name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE myslam::myslam myslam::myslam_sanitized)
endfunction()

gen_bin(1_visual_inertial_odometry)

# SubT 数据集回放依赖 OpenCV（图像解码、角点检测与光流），默认不构建，
# 需要时以 -DMYSLAM_BUILD_DATASET_TOOLS=ON 显式开启。
if(MYSLAM_BUILD_DATASET_TOOLS)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d video calib3d highgui videoio)
    add_executable(2_subt_dataset 2_subt_dataset.cpp)
    target_link_libraries(2_subt_dataset PRIVATE
        myslam::myslam
        myslam::myslam_sanitized
        ${OpenCV_LIBS}
    )
    target_include_directories(2_subt_dataset PRIVATE ${OpenCV_INCLUDE_DIRS})

    # OpenCV highgui 通过 pango 间接依赖 fontconfig；部分发行版只装了运行时库
    # (libfontconfig.so.1) 而没有开发符号链接，ld 会报未定义符号，这里显式补上。
    find_library(FONTCONFIG_LIBRARY NAMES fontconfig libfontconfig.so.1)
    if(FONTCONFIG_LIBRARY)
        target_link_libraries(2_subt_dataset PRIVATE ${FONTCONFIG_LIBRARY})
    endif()
endif()
