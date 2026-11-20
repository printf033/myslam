# myslam_test 只依赖 VIO 核心，任何配置下都会构建。
add_executable(myslam_test test_vio.cpp)
target_link_libraries(myslam_test PRIVATE
    myslam::myslam
    myslam::myslam_sanitized
)
add_test(NAME myslam_test COMMAND myslam_test)

# 内参估计文件的解析不依赖 Lua 与 OpenCV，任何配置下都可构建。
# 有 Lua 时额外链接 myslam_lua：基线标定从 config/<数据源>.lua 读取，
# 测试源码里不再内联任何数据源数值（MYSLAM_HAS_LUA=0 时退回代码默认值）。
add_executable(myslam_intrinsics_test test_intrinsics_file.cpp)
target_link_libraries(myslam_intrinsics_test PRIVATE
    myslam::myslam
    myslam::myslam_sanitized
)
if(TARGET myslam::myslam_lua)
    target_link_libraries(myslam_intrinsics_test PRIVATE myslam::myslam_lua)
endif()
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

# 分层依赖检查纳入 ctest：越层 include 在测试阶段直接暴露，
# 而不是等到某次“为什么 common 头能看见 backend”的排查才被发现。
if(Python3_Interpreter_FOUND)
    add_test(NAME myslam_layer_check
             COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/check_layers.py ${PROJECT_SOURCE_DIR})
endif()

# 端到端精度门禁：从 config/config.lua 加载真实配置跑完整仿真，
# 用 isAcceptable + isConsistent 决定退出码。与 myslam_test 的区别是参数
# 来源：这里走完整的「文件 -> 解析 -> 交叉校验 -> 装配」链路，因此配置
# 本身写坏（或默认值叠加后精度掉出门限）会被这条门禁拦住。
#
# 需要 Lua：没有 Lua 时本程序打印跳过并返回 0，不值得单独注册一个像成功的
# 空测试，因此与 myslam_config_test 共用同一个条件。
if(TARGET myslam::myslam_lua)
    add_executable(myslam_sim_gate test_sim_gate.cpp)
    target_link_libraries(myslam_sim_gate PRIVATE
        myslam::myslam
        myslam::myslam_sanitized
        myslam::myslam_lua
    )
    add_test(NAME myslam_sim_gate COMMAND myslam_sim_gate)
    # 仿真含 gtsam 求解，Debug + ASan 下比纯逻辑测试慢；给足余量而不是
    # 让它在慢机器上偶发超时。超时时间只影响判定，不影响单次运行耗时。
    # WORKING_DIRECTORY 必须是构建根：程序按相对路径 config/config.lua
    # 找配置，默认的构建目录 tests/ 下没有这份副本。
    set_tests_properties(myslam_sim_gate PROPERTIES
        TIMEOUT 900
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()

# 位姿图：跨窗口的长期一致性。它必须 include gtsam 适配器，而 myslam_test
# 刻意只依赖领域层，因此单独成为一条测试。
add_executable(myslam_pose_graph_test test_pose_graph.cpp)
target_link_libraries(myslam_pose_graph_test PRIVATE
    myslam::myslam
    myslam::myslam_sanitized
)
add_test(NAME myslam_pose_graph_test COMMAND myslam_pose_graph_test)

# 动态初始化需要视觉几何（本质矩阵、PnP），因此必须链接 frontend_cv。
# 与 myslam_test 分开的理由是它把 OpenCV 拉进测试依赖：核心层可以在没有
# OpenCV 的环境里编译通过，本条测试不该破坏那个性质。
# 只有启用了数据集工具（即 OpenCV 确实可用）时才注册：否则 frontend_cv
# 没有可用的 include 路径，测试会在编译期失败，把「缺少 OpenCV」伪装成
# 「测试失败」。``myslam::frontend_cv`` 是 INTERFACE 库，无条件存在，
# 因此不能用它做判据。
if(MYSLAM_BUILD_DATASET_TOOLS)
    add_executable(myslam_dynamic_init_test test_dynamic_init.cpp)
    target_link_libraries(myslam_dynamic_init_test PRIVATE
        myslam::replay
        myslam::myslam_sanitized
    )
    add_test(NAME myslam_dynamic_init_test COMMAND myslam_dynamic_init_test)
endif()
