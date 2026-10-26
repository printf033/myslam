set(TEST_TARGETS
    myslam_test
    myslam_config_test
)

set(myslam_test_SOURCES test_vio.cpp)
set(myslam_config_test_SOURCES test_lua_config.cpp)

foreach(target IN LISTS TEST_TARGETS)
    add_executable(${target} ${${target}_SOURCES})

    # Include paths and dependencies are propagated by the public myslam targets
    target_link_libraries(${target} PRIVATE
        myslam::myslam
        myslam::myslam_sanitized
    )

    # Lua 配置支持可选：关闭 MYSLAM_ENABLE_LUA 时 myslam_lua 目标不存在，
    # 此时 lua_config.hpp 退化为桩实现，配置测试会自行失败并说明原因。
    if(TARGET myslam::myslam_lua)
        target_link_libraries(${target} PRIVATE myslam::myslam_lua)
    endif()

    add_test(NAME ${target} COMMAND ${target})
endforeach()
