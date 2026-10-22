add_library(myslam INTERFACE)
add_library(myslam::myslam ALIAS myslam)

target_include_directories(myslam INTERFACE
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
