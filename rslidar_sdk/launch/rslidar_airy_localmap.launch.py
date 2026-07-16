"""Start RSAIRY and publish point clouds directly in world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("rslidar_sdk"))
    default_config_path = package_share / "config" / "config_localmap.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=str(default_config_path),
                description="Absolute path to the rslidar_sdk localmap YAML config",
            ),
            Node(
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                name="rslidar_sdk_node",
                output="screen",
                parameters=[{"config_path": LaunchConfiguration("config_path")}],
            )
        ]
    )
