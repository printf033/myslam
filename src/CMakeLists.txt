# -----------------------------------------------------------------------------
# Generated headers
#
# 版本号与项目名由顶层 CMakeLists.txt 的 project() 唯一决定，这里生成
# <build>/src/generated/myslam.hpp，避免 C++ 代码中再手工维护一份。
# -----------------------------------------------------------------------------
set(MYSLAM_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/myslam.hpp.in"
    "${MYSLAM_GENERATED_INCLUDE_DIR}/myslam.hpp"
    @ONLY
)

# -----------------------------------------------------------------------------
# Layered interface targets
#
#   domain                     纯契约与值类型（Eigen + 日志，不含求解器/视觉库）
#   common/config/sensor       领域之上：数据配置、Lua 解析、传感器实现
#   frontend                   domain + OpenCV（观测提取）
#   adapters                   domain + gtsam（估计器实现）
#   pipeline                   domain（编排核心，不认识任何适配器）
#   app                        pipeline + sensor + adapters + frontend
#   viz                        app + OpenCV（显示）
#
# 关键不变式：pipeline 不链接 gtsam，也不链接 OpenCV；换估计器只改
# adapters 与配置名，流水线与编排层不动。
# -----------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(myslam_domain INTERFACE)
add_library(myslam::domain ALIAS myslam_domain)

target_include_directories(myslam_domain INTERFACE
    $<BUILD_INTERFACE:${MYSLAM_GENERATED_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(myslam_domain INTERFACE cxx_std_26)

target_compile_options(myslam_domain INTERFACE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -fmacro-prefix-map=${PROJECT_SOURCE_DIR}/=>
)

find_package(Threads REQUIRED)
target_link_libraries(myslam_domain INTERFACE
    Eigen3::Eigen
    Threads::Threads
    myslam::mylog
)

# 非算法数据：点云导出与数据集回放配置。只依赖 domain。
add_library(myslam_common INTERFACE)
add_library(myslam::common ALIAS myslam_common)
target_link_libraries(myslam_common INTERFACE myslam::domain)

# 配置解析：只依赖 domain。
add_library(myslam_config INTERFACE)
add_library(myslam::config ALIAS myslam_config)
target_link_libraries(myslam_config INTERFACE myslam_common)

# 数据来源：仿真器、数据集读取、内参文件。不链接 gtsam。
# rosbag2 源需要 SQLite（.db3 存储）；nlohmann/json 走系统头文件，
# 只用于解析 mfosn INS 的 JSON 字符串消息。
add_library(myslam_sensor INTERFACE)
add_library(myslam::sensor ALIAS myslam_sensor)
target_link_libraries(myslam_sensor INTERFACE myslam_common myslam::sqlite)

# 前端：相机模型与观测提取接口在 domain；具体光流实现需要 OpenCV。
add_library(myslam_frontend INTERFACE)
add_library(myslam::frontend ALIAS myslam_frontend)
target_link_libraries(myslam_frontend INTERFACE myslam::domain)

add_library(myslam_frontend_cv INTERFACE)
add_library(myslam::frontend_cv ALIAS myslam_frontend_cv)
target_link_libraries(myslam_frontend_cv INTERFACE
    myslam_frontend
    myslam::opencv
)

# 估计器适配器：唯一允许出现 gtsam 的地方。
add_library(myslam_adapters INTERFACE)
add_library(myslam::adapters ALIAS myslam_adapters)
target_link_libraries(myslam_adapters INTERFACE
    myslam::domain
    gtsam::gtsam
)

# 流水线：只依赖 domain。刻意不链接 adapters / gtsam / OpenCV。
add_library(myslam_pipeline INTERFACE)
add_library(myslam::pipeline ALIAS myslam_pipeline)
target_link_libraries(myslam_pipeline INTERFACE myslam::domain)

# 编排层：仿真基准与回放。回放需要 OpenCV 读图，单列为 myslam_replay。
add_library(myslam_app INTERFACE)
add_library(myslam::app ALIAS myslam_app)
target_link_libraries(myslam_app INTERFACE
    myslam_pipeline
    myslam_sensor
    myslam_adapters
    myslam::domain
)

add_library(myslam_replay INTERFACE)
add_library(myslam::replay ALIAS myslam_replay)
target_link_libraries(myslam_replay INTERFACE
    myslam_app
    myslam_frontend_cv
    myslam::opencv
)

# 显示与导出：只依赖公共类型 + OpenCV。
add_library(myslam_viz INTERFACE)
add_library(myslam::viz ALIAS myslam_viz)
target_link_libraries(myslam_viz INTERFACE
    myslam_common
    myslam_replay
    myslam::opencv
)

# 聚合目标：示例与测试链接 myslam::myslam。
# 刻意不含 viz：viz 需要 OpenCV，而核心与仿真示例必须能在无 OpenCV 环境构建。
add_library(myslam INTERFACE)
add_library(myslam::myslam ALIAS myslam)
target_link_libraries(myslam INTERFACE
    myslam_app
    myslam_pipeline
    myslam_sensor
    myslam_config
    myslam_common
    myslam::domain
)

option(MYSLAM_SANITIZE "Enable ASan/UBSan for Debug builds" ${MYSLAM_IS_TOP_LEVEL})

add_library(myslam_sanitized INTERFACE)
add_library(myslam::myslam_sanitized ALIAS myslam_sanitized)

if(MYSLAM_SANITIZE)
    target_compile_options(myslam_sanitized INTERFACE
        $<$<AND:$<CONFIG:Debug>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-fsanitize=address,undefined -fno-omit-frame-pointer>
    )
    target_link_options(myslam_sanitized INTERFACE
        $<$<AND:$<CONFIG:Debug>,$<NOT:$<CXX_COMPILER_ID:MSVC>>>:-fsanitize=address,undefined>
    )
endif()

# -----------------------------------------------------------------------------
# Lua configuration support
# -----------------------------------------------------------------------------
option(MYSLAM_ENABLE_LUA "Enable Lua (LuaJIT) configuration support" ON)

if(MYSLAM_ENABLE_LUA)
    find_package(PkgConfig QUIET)

    set(MYSLAM_LUAJIT_FOUND OFF)
    if(PkgConfig_FOUND)
        pkg_check_modules(LUAJIT IMPORTED_TARGET luajit)
        if(TARGET PkgConfig::LUAJIT)
            set(MYSLAM_LUAJIT_FOUND ON)
        endif()
    endif()

    if(MYSLAM_LUAJIT_FOUND)
        add_library(myslam_lua INTERFACE)
        add_library(myslam::myslam_lua ALIAS myslam_lua)
        target_link_libraries(myslam_lua INTERFACE myslam::config PkgConfig::LUAJIT)
        target_compile_definitions(myslam_lua INTERFACE MYSLAM_HAS_LUA=1)
        message(STATUS "myslam: Lua configuration enabled (LuaJIT ${LUAJIT_VERSION})")
    else()
        set(MYSLAM_ENABLE_LUA OFF CACHE BOOL
            "Enable Lua (LuaJIT) configuration support" FORCE)
        message(WARNING
            "myslam: LuaJIT not found (pkg-config module 'luajit'), "
            "so MYSLAM_ENABLE_LUA has been turned OFF. "
            "Install it to enable Lua configuration: "
            "Debian/Ubuntu 'libluajit-5.1-dev', Arch 'luajit', macOS 'luajit'.")
    endif()
endif()

# -----------------------------------------------------------------------------
# 分层依赖静态检查
# -----------------------------------------------------------------------------
if(Python3_Interpreter_FOUND)
    add_custom_target(check-layers
        COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/check_layers.py ${PROJECT_SOURCE_DIR}
        COMMENT "Checking layer include boundaries"
        VERBATIM)
endif()
