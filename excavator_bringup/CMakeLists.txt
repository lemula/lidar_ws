cmake_minimum_required(VERSION 3.8)
project(excavator_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES README.zh.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
