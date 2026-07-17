from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = str(
        Path(get_package_share_directory("excavator_patchwork"))
        / "rviz"
        / "segmentation.rviz"
    )
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="segmentation_rviz",
                output="screen",
                arguments=["-d", rviz_config],
            )
        ]
    )
