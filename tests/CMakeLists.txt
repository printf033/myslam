set(TEST_SOURCES
    test_vio.cpp
)

add_executable(myslam_test ${TEST_SOURCES})

# Include paths and dependencies are propagated by the public myslam targets
target_link_libraries(myslam_test PRIVATE
    myslam::myslam
    myslam::myslam_sanitized
)

add_test(NAME myslam_test COMMAND myslam_test)
