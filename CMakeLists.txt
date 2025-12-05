cmake_minimum_required(VERSION 3.28)
project(mrs_uav_trajectory_generation)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  std_msgs
  std_srvs
  mrs_uav_testing
  backward_ros
  nlopt
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavTrajectoryGeneration_EthTrajectoryGeneration
  MrsUavTrajectoryGeneration_MrsTrajectoryGeneration
  MrsUavTrajectoryGeneration_PathRandomFlier
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Internal common library

add_library(MrsUavTrajectoryGeneration_InternalCommon INTERFACE)
add_library(mrs_uav_trajectory_generation::internal_common ALIAS
  MrsUavTrajectoryGeneration_InternalCommon
)

target_link_libraries(MrsUavTrajectoryGeneration_InternalCommon
  INTERFACE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    nlopt::nlopt
    ${mrs_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
)

# ETH Trajectory Generation

add_library(MrsUavTrajectoryGeneration_EthTrajectoryGeneration SHARED)
add_library(mrs_uav_trajectory_generation::eth_trajectory_generation ALIAS
  MrsUavTrajectoryGeneration_EthTrajectoryGeneration
)

target_sources(MrsUavTrajectoryGeneration_EthTrajectoryGeneration
  PRIVATE
    "src/eth_trajectory_generation/motion_defines.cpp"
    "src/eth_trajectory_generation/polynomial.cpp"
    "src/eth_trajectory_generation/segment.cpp"
    "src/eth_trajectory_generation/timing.cpp"
    "src/eth_trajectory_generation/trajectory.cpp"
    "src/eth_trajectory_generation/trajectory_sampling.cpp"
    "src/eth_trajectory_generation/vertex.cpp"
    "src/eth_trajectory_generation/rpoly/rpoly_ak1.cpp"
)

# Headers are not installed so there is no install interface
target_include_directories(MrsUavTrajectoryGeneration_EthTrajectoryGeneration
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(MrsUavTrajectoryGeneration_EthTrajectoryGeneration
  PUBLIC
    mrs_uav_trajectory_generation::internal_common
)

# MRS Trajectory Generation

add_library(MrsUavTrajectoryGeneration_MrsTrajectoryGeneration SHARED)

target_sources(MrsUavTrajectoryGeneration_MrsTrajectoryGeneration
  PRIVATE
    "src/mrs_trajectory_generation.cpp"
)

target_link_libraries(MrsUavTrajectoryGeneration_MrsTrajectoryGeneration
  PUBLIC
    mrs_uav_trajectory_generation::internal_common
    mrs_uav_trajectory_generation::eth_trajectory_generation
)

target_compile_definitions(MrsUavTrajectoryGeneration_MrsTrajectoryGeneration PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

rclcpp_components_register_nodes(MrsUavTrajectoryGeneration_MrsTrajectoryGeneration
  "mrs_uav_trajectory_generation::MrsTrajectoryGeneration"
)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## | -------------------- path random flier ------------------- |

add_library(MrsUavTrajectoryGeneration_PathRandomFlier SHARED)

target_sources(MrsUavTrajectoryGeneration_PathRandomFlier
  PRIVATE
    "src/path_random_flier.cpp"
)

target_link_libraries(MrsUavTrajectoryGeneration_PathRandomFlier
  PUBLIC
    mrs_uav_trajectory_generation::internal_common
)

target_compile_definitions(MrsUavTrajectoryGeneration_PathRandomFlier PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

rclcpp_components_register_nodes(MrsUavTrajectoryGeneration_PathRandomFlier
  "mrs_uav_trajectory_generation::PathRandomFlier"
)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
    MrsUavTrajectoryGeneration_InternalCommon
    ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
