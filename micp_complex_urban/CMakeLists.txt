cmake_minimum_required(VERSION 3.2)
project(micp_complex_urban)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

message(STATUS "CMake Version: ${CMAKE_VERSION}")
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.9)
    message(STATUS ">= 3.9 - Enabling Link Time Optimization")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

set(ROS_DEPS
    roscpp
    rmcl
    tf2_ros
    sensor_msgs
    geometry_msgs
    irp_sen_msgs
)

find_package(catkin REQUIRED COMPONENTS
    ${ROS_DEPS}
)

find_package(rmagine 2.1.1 REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    # include
#  LIBRARIES amock_tf_tools
  CATKIN_DEPENDS
    ${ROS_DEPS}
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${rmagine_INCLUDE_DIRS}    
)

add_executable(car_static_transforms src/car_static_transforms.cpp)
add_dependencies(car_static_transforms
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(car_static_transforms
    ${catkin_LIBRARIES}
    ${rmagine_LIBRARIES}
    Eigen3::Eigen
)

add_executable(compute_odometry src/compute_odometry.cpp)
add_dependencies(compute_odometry
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(compute_odometry
    ${catkin_LIBRARIES}
    ${rmagine_LIBRARIES}
    Eigen3::Eigen
)

