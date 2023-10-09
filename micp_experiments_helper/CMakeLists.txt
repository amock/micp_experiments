cmake_minimum_required(VERSION 3.0.2)
project(micp_experiments_helper)

set(ROS_DEPS
  roscpp
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(rmagine 2.2.1...
COMPONENTS
  core
)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS 
    include
#  LIBRARIES amock_tf_tools
  CATKIN_DEPENDS
    ${ROS_DEPS}
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(leaf_odom_to_root src/leaf_odom_to_root.cpp)
target_link_libraries(leaf_odom_to_root
  ${catkin_LIBRARIES}
  rmagine::core
)

add_executable(change_imu_frame src/change_imu_frame.cpp)
target_link_libraries(change_imu_frame
  ${catkin_LIBRARIES}
  rmagine::core
)

add_executable(fuse_gps_imu src/fuse_gps_imu.cpp)
target_link_libraries(fuse_gps_imu
  ${catkin_LIBRARIES}
  rmagine::core
  Eigen3::Eigen
)

