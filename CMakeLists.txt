cmake_minimum_required(VERSION 3.0.2)
project(mrs_uav_trajectory_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  std_msgs
  mrs_msgs
  mrs_lib
  nlopt_ros
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

generate_dynamic_reconfigure_options(
  config/drs.cfg
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES EthTrajectoryGeneration MrsTrajectoryGeneration
  CATKIN_DEPENDS roscpp std_msgs mrs_lib mrs_msgs dynamic_reconfigure nlopt_ros
  DEPENDS Eigen
  )

add_library(EthTrajectoryGeneration
  src/eth_trajectory_generation/motion_defines.cpp
  src/eth_trajectory_generation/polynomial.cpp
  src/eth_trajectory_generation/segment.cpp
  src/eth_trajectory_generation/timing.cpp
  src/eth_trajectory_generation/trajectory.cpp
  src/eth_trajectory_generation/trajectory_sampling.cpp
  src/eth_trajectory_generation/vertex.cpp
  src/eth_trajectory_generation/rpoly/rpoly_ak1.cpp
  )

add_library(MrsTrajectoryGeneration
  src/mrs_trajectory_generation.cpp
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

target_link_libraries(MrsTrajectoryGeneration
  EthTrajectoryGeneration
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
