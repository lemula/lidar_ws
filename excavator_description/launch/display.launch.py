import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("excavator_description")
    urdf_path = os.path.join(package_dir, "urdf", "excavator.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    use_tip_pose_publisher = LaunchConfiguration("use_tip_pose_publisher")
    use_sim_time = LaunchConfiguration("use_sim_time")
    reference_frame = LaunchConfiguration("reference_frame")
    pose_topic = LaunchConfiguration("pose_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_tip_pose_publisher", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("reference_frame", default_value="world"),
            DeclareLaunchArgument(
                "pose_topic", default_value="/bucket_tip_pose_world"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="excavator_description",
                executable="bucket_tip_pose_publisher.py",
                name="bucket_tip_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "reference_frame": reference_frame,
                        "pose_topic": pose_topic,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(use_tip_pose_publisher),
            ),
        ]
    )
