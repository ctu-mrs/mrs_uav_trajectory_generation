cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

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
  catkin_simple
  glog_catkin
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
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs mav_msgs nlopt catkin_simple glog_catkin dynamic_reconfigure
  DEPENDS Eigen
  )

# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

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
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)

# Link against yaml-cpp.
target_link_libraries(TrajectoryGeneration
  ${YamlCpp_LIBRARIES}
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
