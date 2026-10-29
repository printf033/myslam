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
#
# Both thirdparty/ itself and mylog's src/ directory are on the include path:
# mylog headers include each other by bare file name ("fileManager.hpp"),
# while consumers spell mylog-owned headers with the `mylog/` prefix
# (#include <mylog/src/lua_config.hpp>) so they cannot be shadowed by
# same-named myslam headers such as src/lua_config.hpp.
add_library(myslam_mylog INTERFACE)
add_library(myslam::mylog ALIAS myslam_mylog)

target_include_directories(myslam_mylog INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mylog/src>
)

# -----------------------------------------------------------------------------
# OpenCV
#
# `myslam::opencv` is the single entry point for consumers. It has two
# mutually exclusive backends:
#
#   * default            -> system OpenCV 4 via find_package (fast, distro-managed);
#   * MYSLAM_USE_BUNDLED_OPENCV=ON -> build the modules myslam needs from the
#     thirdparty/opencv submodule, so the pinned version evolves with this repo
#     instead of drifting with the host distribution.
#
# The interface carries no dependencies unless MYSLAM_BUILD_DATASET_TOOLS is ON,
# so ordinary builds never even look for OpenCV.
# -----------------------------------------------------------------------------
add_library(myslam_opencv INTERFACE)
add_library(myslam::opencv ALIAS myslam_opencv)

if(MYSLAM_BUILD_DATASET_TOOLS)
    if(MYSLAM_USE_BUNDLED_OPENCV)
        # Only the modules the dataset tools actually use: image decode, feature
        # detection, optical flow, camera geometry and the optional viewer.
        set(BUILD_LIST "core,imgproc,imgcodecs,features2d,video,calib3d,highgui,videoio"
            CACHE STRING "OpenCV modules built from thirdparty/opencv" FORCE)
        set(BUILD_TESTS           OFF CACHE BOOL "Do not build OpenCV tests"    FORCE)
        set(BUILD_PERF_TESTS      OFF CACHE BOOL "Do not build OpenCV perf tests" FORCE)
        set(BUILD_EXAMPLES        OFF CACHE BOOL "Do not build OpenCV examples" FORCE)
        set(BUILD_opencv_apps     OFF CACHE BOOL "Do not build OpenCV apps"     FORCE)
        set(BUILD_DOCS            OFF CACHE BOOL "Do not build OpenCV docs"     FORCE)
        set(BUILD_JAVA            OFF CACHE BOOL "Do not build OpenCV Java bindings" FORCE)
        set(BUILD_opencv_python3  OFF CACHE BOOL "Do not build OpenCV Python bindings" FORCE)
        set(WITH_IPP              OFF CACHE BOOL "Do not vendor Intel IPP"      FORCE)
        set(WITH_OPENCL           OFF CACHE BOOL "Do not enable OpenCL runtime" FORCE)
        # The replay tools decode JPEG/PNG only; JPEG2000 support would pull in
        # a host openjpeg whose CMake package is often broken (conda ships
        # targets pointing at deleted helper binaries), failing configure.
        set(WITH_OPENJPEG         OFF CACHE BOOL "Do not link a host OpenJPEG" FORCE)
        # Same reasoning for the other optional codecs: skip them entirely so
        # the build never links a host library that disagrees with its headers
        # (a conda libtiff newer than the system one breaks the imgcodecs link).
        set(WITH_TIFF             OFF CACHE BOOL "Do not link a host libtiff"  FORCE)
        set(WITH_JASPER           OFF CACHE BOOL "Do not link a host Jasper"   FORCE)
        set(WITH_OPENEXR          OFF CACHE BOOL "Do not link a host OpenEXR"  FORCE)
        set(WITH_GDAL             OFF CACHE BOOL "Do not link a host GDAL"     FORCE)
        set(WITH_GPHOTO2          OFF CACHE BOOL "Do not link gPhoto2"         FORCE)

        add_subdirectory(opencv EXCLUDE_FROM_ALL)

        # OpenCV publishes its include directories as PRIVATE on each module
        # target, so nothing propagates through target_link_libraries. Spell the
        # paths out for the modules myslam links; the generated headers
        # (opencv2/opencv_modules.hpp, cvconfig.h) land under CMAKE_BINARY_DIR.
        target_include_directories(myslam_opencv INTERFACE
            $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>)
        foreach(_myslam_opencv_module
                core imgproc imgcodecs features2d flann video calib3d highgui videoio)
            target_include_directories(myslam_opencv INTERFACE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opencv/modules/${_myslam_opencv_module}/include>)
        endforeach()

        target_link_libraries(myslam_opencv INTERFACE
            opencv_core
            opencv_imgproc
            opencv_imgcodecs
            opencv_features2d
            opencv_video
            opencv_calib3d
            opencv_highgui
            opencv_videoio
        )
        message(STATUS "myslam: OpenCV built from thirdparty/opencv submodule")
    else()
        find_package(OpenCV REQUIRED
            COMPONENTS core imgproc imgcodecs features2d video calib3d highgui videoio)
        target_link_libraries(myslam_opencv INTERFACE ${OpenCV_LIBS})
        target_include_directories(myslam_opencv INTERFACE ${OpenCV_INCLUDE_DIRS})

        message(STATUS "myslam: using system OpenCV ${OpenCV_VERSION}")
    endif()

    # OpenCV highgui reaches fontconfig through pango; some distributions ship
    # only the runtime library (libfontconfig.so.1) without the development
    # symlink, or a stale copy first on the linker path (conda's fontconfig is
    # older than the system pango and lacks FcConfigSetDefaultSubstitute).
    # Resolving it explicitly keeps both backends linkable.
    find_library(FONTCONFIG_LIBRARY NAMES fontconfig libfontconfig.so.1)
    if(FONTCONFIG_LIBRARY)
        target_link_libraries(myslam_opencv INTERFACE ${FONTCONFIG_LIBRARY})
    endif()
endif()

# -----------------------------------------------------------------------------
# MASt3R (https://github.com/naver/mast3r)
#
# Pinned as a submodule so the reconstruction toolchain behind the SubT
# sequences evolves together with this repository, but it deliberately takes no
# part in the C++ build: it is a Python/PyTorch research stack with no CMake
# target and no headers. myslam consumes its *output* — the Handheld1_Folder
# layout (images + IMU + ground truth) that scripts/run_subt.sh replays — and
# mirrors MASt3R-Fusion's MEI intrinsics / pinhole-scale preprocessing.
#
# Track upstream with:
#   git submodule update --remote thirdparty/mast3r
# -----------------------------------------------------------------------------
