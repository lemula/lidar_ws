from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("excavator_dig_point"))
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="dig_point_rviz",
                output="screen",
                arguments=["-d", str(share / "rviz" / "dig_point.rviz")],
            )
        ]
    )
