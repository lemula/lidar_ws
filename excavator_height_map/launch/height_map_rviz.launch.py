from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("excavator_height_map"))
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="height_map_rviz",
                output="screen",
                arguments=["-d", str(share / "rviz" / "height_map.rviz")],
            )
        ]
    )
