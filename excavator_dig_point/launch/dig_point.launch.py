from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("excavator_dig_point"))
        / "config"
        / "dig_point.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="excavator_dig_point",
                executable="dig_point_scaffold",
                name="dig_point_scaffold",
                output="screen",
                parameters=[config],
                remappings=[
                    ("grid_map", "/terrain/grid_map"),
                    ("dig_target", "/digging/target"),
                ],
            )
        ]
    )

