from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    body_filter_share = Path(get_package_share_directory("robot_body_filter"))
    patchwork_share = Path(get_package_share_directory("excavator_patchwork"))
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        body_filter_share
                        / "launch"
                        / "excavator_body_filter.launch.py"
                    )
                ),
                launch_arguments={"start_rviz": "false"}.items(),
            ),
            Node(
                package="excavator_patchwork",
                executable="excavator_patchwork_node",
                name="excavator_patchwork",
                output="screen",
                parameters=[
                    str(patchwork_share / "config" / "patchworkpp.yaml")
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="segmentation_rviz",
                output="screen",
                arguments=[
                    "-d",
                    str(patchwork_share / "rviz" / "segmentation.rviz"),
                ],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
