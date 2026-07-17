cmake_minimum_required(VERSION 3.8)
project(excavator_dig_point)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EntryPoint.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(excavator_dig_point_node src/dig_point_node.cpp)
target_compile_features(excavator_dig_point_node PUBLIC cxx_std_17)
target_include_directories(excavator_dig_point_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(excavator_dig_point_node
  geometry_msgs
  grid_map_core
  grid_map_msgs
  nav_msgs
  rclcpp
  std_msgs
  visualization_msgs
)
target_link_libraries(excavator_dig_point_node ${OpenCV_LIBS})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(excavator_dig_point_node "${cpp_typesupport_target}")

install(TARGETS excavator_dig_point_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES GRID_MAP_ENTRY_POINT_RULES.zh.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
