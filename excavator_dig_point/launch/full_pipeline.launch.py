from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    height_map_share = Path(get_package_share_directory("excavator_height_map"))
    dig_point_share = Path(get_package_share_directory("excavator_dig_point"))
    start_rviz = LaunchConfiguration("start_rviz")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("crop_min_x", default_value="-0.5"),
            DeclareLaunchArgument("crop_max_x", default_value="1.0"),
            DeclareLaunchArgument("crop_min_y", default_value="0.0"),
            DeclareLaunchArgument("crop_max_y", default_value="1.5"),
            DeclareLaunchArgument("crop_min_z", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_z", default_value="2.0"),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(height_map_share / "launch" / "full_pipeline.launch.py")
                        ),
                        launch_arguments={
                            "start_rviz": "false",
                            "crop_min_x": crop_min_x,
                            "crop_max_x": crop_max_x,
                            "crop_min_y": crop_min_y,
                            "crop_max_y": crop_max_y,
                            "crop_min_z": crop_min_z,
                            "crop_max_z": crop_max_z,
                        }.items(),
                    )
                ],
            ),
            Node(
                package="excavator_dig_point",
                executable="excavator_dig_point_node",
                name="excavator_dig_point",
                output="screen",
                parameters=[str(dig_point_share / "config" / "dig_point.yaml")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="dig_point_rviz",
                output="screen",
                arguments=["-d", str(dig_point_share / "rviz" / "dig_point.rviz")],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
