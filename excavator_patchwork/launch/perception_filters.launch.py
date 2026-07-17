from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    body_filter_share = Path(get_package_share_directory("robot_body_filter"))
    patchwork_share = Path(get_package_share_directory("excavator_patchwork"))
    patchwork_config = str(patchwork_share / "config" / "patchworkpp.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(body_filter_share / "launch" / "filters_only.launch.py")
                )
            ),
            Node(
                package="excavator_patchwork",
                executable="excavator_patchwork_node",
                name="excavator_patchwork",
                output="screen",
                parameters=[patchwork_config],
            ),
        ]
    )
