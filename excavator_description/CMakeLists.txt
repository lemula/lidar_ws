cmake_minimum_required(VERSION 3.8)
project(excavator_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/bucket_tip_pose_publisher.py
    scripts/waji_joint_controls.py
    scripts/waji_joint_slider.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
