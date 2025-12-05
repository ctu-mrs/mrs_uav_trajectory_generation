get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_executable(test_${TEST_NAME}
  test.cpp
)

target_link_libraries(test_${TEST_NAME}
  PRIVATE
    rclcpp::rclcpp
    mrs_lib::mrs_lib
    mrs_uav_testing::mrs_uav_testing
    ${mrs_msgs_TARGETS}
)

install(
  TARGETS test_${TEST_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_ros_isolated_launch_test(test.py TIMEOUT 300)
