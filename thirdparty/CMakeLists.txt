# Third-party dependencies managed as git submodules.
#
# gtsam: factor-graph optimization and state estimation.
# Only configured here; its own tests/examples are disabled so that myslam
# builds just the library it consumes.

set(GTSAM_BUILD_TESTS           OFF CACHE BOOL "Do not build gtsam tests"     FORCE)
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "Do not build gtsam examples"  FORCE)
set(GTSAM_BUILD_UNSTABLE        OFF CACHE BOOL "Do not build gtsam unstable"  FORCE)
set(GTSAM_USE_SYSTEM_EIGEN      ON  CACHE BOOL "Use system Eigen for gtsam"   FORCE)

add_subdirectory(gtsam EXCLUDE_FROM_ALL)

# Modern namespaced alias so consumers link gtsam::gtsam instead of the raw
# target name.
if(TARGET gtsam AND NOT TARGET gtsam::gtsam)
    add_library(gtsam::gtsam ALIAS gtsam)
endif()

# mylog is consumed as headers only.
#
# It is deliberately not added with add_subdirectory(): its INTERFACE target
# propagates -freflection (unsupported by the current GCC 15 toolchain) and its
# optional Lua configuration layer requires LuaJIT. Exposing the include path
# through an INTERFACE target keeps dependants free of raw include paths and
# lets a future toolchain switch replace this wrapper with mylog::mylog.
add_library(myslam_mylog INTERFACE)
add_library(myslam::mylog ALIAS myslam_mylog)

target_include_directories(myslam_mylog INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mylog/src>
)
