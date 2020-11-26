cmake_minimum_required(VERSION 3.0.2)
project(mrs_uav_trajectory_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  mav_msgs
  nlopt
  )

generate_dynamic_reconfigure_options(
  config/drs.cfg
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES TrajectoryGeneration TrajectoryGenerationNodelet
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs mav_msgs dynamic_reconfigure nlopt
  DEPENDS Eigen
  )

add_library(TrajectoryGeneration
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/rpoly/rpoly_ak1.cpp
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_library(TrajectoryGenerationNodelet src/trajectory_generation.cpp)

target_link_libraries(TrajectoryGenerationNodelet
  TrajectoryGeneration
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
