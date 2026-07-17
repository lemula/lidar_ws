cmake_minimum_required(VERSION 3.8)
project(robot_body_filter)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

add_executable(robot_body_filter_node src/robot_body_filter_node.cpp)
target_compile_features(robot_body_filter_node PUBLIC cxx_std_17)
target_link_libraries(robot_body_filter_node Eigen3::Eigen)
ament_target_dependencies(robot_body_filter_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  urdf
)

install(TARGETS robot_body_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
