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
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")
    enable_body_filter = LaunchConfiguration("enable_body_filter")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("crop_min_x", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_x", default_value="2.0"),
            DeclareLaunchArgument("crop_min_y", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_y", default_value="2.0"),
            DeclareLaunchArgument("crop_min_z", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_z", default_value="5.0"),
            DeclareLaunchArgument("enable_body_filter", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        body_filter_share
                        / "launch"
                        / "excavator_body_filter.launch.py"
                    )
                ),
                launch_arguments={
                    "start_rviz": "false",
                    "crop_min_x": crop_min_x,
                    "crop_max_x": crop_max_x,
                    "crop_min_y": crop_min_y,
                    "crop_max_y": crop_max_y,
                    "crop_min_z": crop_min_z,
                    "crop_max_z": crop_max_z,
                    "enable_body_filter": enable_body_filter,
                }.items(),
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
