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

add_library(myslam INTERFACE)
add_library(myslam::myslam ALIAS myslam)

target_include_directories(myslam INTERFACE
    $<BUILD_INTERFACE:${MYSLAM_GENERATED_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(myslam INTERFACE
    cxx_std_26
)

target_compile_options(myslam INTERFACE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-Wall -Wextra -fmacro-prefix-map=${PROJECT_SOURCE_DIR}/=>
)

find_package(Threads REQUIRED)
target_link_libraries(myslam INTERFACE
    Threads::Threads
    gtsam::gtsam
    myslam::mylog
)

option(MYSLAM_SANITIZE "Enable ASan/UBSan for Debug builds" ${MYSLAM_IS_TOP_LEVEL})

# Sanitizer helper target (optional, used in tests and examples)
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
#
# 与子库 mylog 的 MYLOG_ENABLE_LUA 同一策略：默认开启，未链接
# myslam::myslam_lua 时 src/lua_config.hpp 退化为桩实现（load() 恒返回失败），
# 项目仍可正常构建与运行。
# -----------------------------------------------------------------------------
option(MYSLAM_ENABLE_LUA "Enable Lua (LuaJIT) configuration support" ON)

if(MYSLAM_ENABLE_LUA)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LUAJIT REQUIRED IMPORTED_TARGET luajit)

    add_library(myslam_lua INTERFACE)
    add_library(myslam::myslam_lua ALIAS myslam_lua)
    target_link_libraries(myslam_lua INTERFACE myslam::myslam PkgConfig::LUAJIT)
    target_compile_definitions(myslam_lua INTERFACE MYSLAM_HAS_LUA=1)
endif()
