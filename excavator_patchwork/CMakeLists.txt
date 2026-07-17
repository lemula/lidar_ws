cmake_minimum_required(VERSION 3.8)
project(excavator_patchwork)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(patchworkpp_core STATIC
  src/patchworkpp.cpp
  src/plane_fit.cpp
)
target_compile_features(patchworkpp_core PUBLIC cxx_std_17)
target_include_directories(patchworkpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(patchworkpp_core PUBLIC Eigen3::Eigen)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Keep the vendored upstream core warning-clean without modifying its code.
  target_compile_options(patchworkpp_core PRIVATE
    -Wno-sign-compare
    -Wno-unused-variable
    -Wno-unused-but-set-variable
  )
endif()

add_executable(excavator_patchwork_node src/ground_segmentation_node.cpp)
target_compile_features(excavator_patchwork_node PUBLIC cxx_std_17)
target_link_libraries(excavator_patchwork_node patchworkpp_core Eigen3::Eigen)
ament_target_dependencies(excavator_patchwork_node rclcpp sensor_msgs)

install(TARGETS excavator_patchwork_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
