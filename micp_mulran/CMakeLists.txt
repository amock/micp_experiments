cmake_minimum_required(VERSION 3.2)
project(micp_mulran)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    dynamic_reconfigure
    mesh_msgs
    image_transport
    cv_bridge
    actionlib_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package()