cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_trajectory_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  std_msgs
  mrs_msgs
  mrs_lib
  nlopt
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# nlopt is too loud to have this hear normally
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

generate_dynamic_reconfigure_options(
  config/drs.cfg
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(LIBRARIES
  EthTrajectoryGeneration MrsTrajectoryGeneration
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS roscpp std_msgs mrs_lib mrs_msgs dynamic_reconfigure nlopt
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

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
