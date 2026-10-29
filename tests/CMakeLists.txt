# myslam_test 只依赖 VIO 核心，任何配置下都会构建。
add_executable(myslam_test test_vio.cpp)
target_link_libraries(myslam_test PRIVATE
    myslam::myslam
    myslam::myslam_sanitized
)
add_test(NAME myslam_test COMMAND myslam_test)

# 内参估计文件的解析不依赖 Lua 与 OpenCV，任何配置下都可构建。
add_executable(myslam_intrinsics_test test_intrinsics_file.cpp)
target_link_libraries(myslam_intrinsics_test PRIVATE
    myslam::myslam
    myslam::myslam_sanitized
)
add_test(NAME myslam_intrinsics_test COMMAND myslam_intrinsics_test)

# myslam_config_test 需要真实的 Lua 运行时：MYSLAM_ENABLE_LUA 为 OFF 时
# lua_config.hpp 会退化为桩实现，load() 恒返回失败，任何断言都无意义。
# 因此这里不注册该测试，而不是让它以失败的形式出现在 CI 里。
if(TARGET myslam::myslam_lua)
    add_executable(myslam_config_test test_lua_config.cpp)
    target_link_libraries(myslam_config_test PRIVATE
        myslam::myslam
        myslam::myslam_sanitized
        myslam::myslam_lua
    )
    add_test(NAME myslam_config_test COMMAND myslam_config_test)
else()
    message(STATUS "myslam: MYSLAM_ENABLE_LUA is OFF, skipping myslam_config_test")
endif()
