from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("excavator_height_map"))
        / "config"
        / "height_map.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ground_topic", default_value="/terrain/ground_points"
            ),
            DeclareLaunchArgument(
                "nonground_topic", default_value="/terrain/nonground_points"
            ),
            DeclareLaunchArgument(
                "grid_map_topic", default_value="/terrain/grid_map"
            ),
            Node(
                package="excavator_height_map",
                executable="excavator_height_map_node",
                name="excavator_height_map",
                output="screen",
                parameters=[
                    config,
                    {
                        "ground_topic": LaunchConfiguration("ground_topic"),
                        "nonground_topic": LaunchConfiguration("nonground_topic"),
                        "grid_map_topic": LaunchConfiguration("grid_map_topic"),
                    },
                ],
            ),
        ]
    )
