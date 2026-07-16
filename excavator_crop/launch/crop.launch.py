from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("excavator_crop"))
        / "config"
        / "crop_box.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument(
                "output_topic", default_value="/perception/points_roi"
            ),
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="autoware_crop_box_filter",
                executable="crop_box_filter",
                name="crop_box_filter",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
                remappings=[
                    ("input", LaunchConfiguration("input_topic")),
                    ("output", LaunchConfiguration("output_topic")),
                ],
            ),
        ]
    )

