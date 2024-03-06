cmake_minimum_required(VERSION 3.12)
project(micp_mulran2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmcl REQUIRED)
find_package(robot_localization REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmagine REQUIRED COMPONENTS
  core
)

find_package(Boost COMPONENTS system filesystem REQUIRED)


# BUILD LIB
add_library(micp_mulran2
  src/dataset_loading.cpp
)

target_include_directories(micp_mulran2 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(micp_mulran2
  rclcpp
  sensor_msgs
)

target_link_libraries(micp_mulran2
  rmagine::core
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

install(TARGETS micp_mulran2
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# BUILD NODE
add_executable(micp_mulran2_node
  src/micp_mulran2_node.cpp
)

target_include_directories(micp_mulran2_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(micp_mulran2_node
  rclcpp
  std_msgs
)

target_link_libraries(micp_mulran2_node
  micp_mulran2
)

install(TARGETS micp_mulran2_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY include
    DESTINATION share/${PROJECT_NAME}
)

ament_package()