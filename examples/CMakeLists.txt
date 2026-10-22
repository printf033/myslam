function(gen_bin name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE myslam::myslam myslam::myslam_sanitized)
endfunction()

gen_bin(1_visual_inertial_odometry)
