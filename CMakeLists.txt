cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  cfg/trajectory_generation.cfg
  )

#Find Eigen libraries
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES TrajectoryGeneration TrajectoryGenerationNodelet
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs mav_msgs dynamic_reconfigure nlopt
  DEPENDS Eigen
  )

###########
## Build ##
###########

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

## Declare a C++ library
add_library(TrajectoryGenerationNodelet src/trajectory_generation.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(TrajectoryGenerationNodelet
  TrajectoryGeneration
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
