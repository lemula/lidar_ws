from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    filter_share = Path(get_package_share_directory("robot_body_filter"))
    rviz = LaunchConfiguration("rviz")
    argument_names = (
        "input_topic",
        "cropped_topic",
        "output_topic",
        "crop_min_x",
        "crop_max_x",
        "crop_min_y",
        "crop_max_y",
        "crop_min_z",
        "crop_max_z",
        "body_padding",
        "body_scale",
        "use_sim_time",
    )
    arguments = {name: LaunchConfiguration(name) for name in argument_names}

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "input_topic", default_value="/localmap/world_points"
            ),
            DeclareLaunchArgument(
                "cropped_topic", default_value="/localmap/cropped_points"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/localmap/body_filtered_points"
            ),
            DeclareLaunchArgument("crop_min_x", default_value="-0.5"),
            DeclareLaunchArgument("crop_max_x", default_value="1.0"),
            DeclareLaunchArgument("crop_min_y", default_value="0.0"),
            DeclareLaunchArgument("crop_max_y", default_value="1.5"),
            DeclareLaunchArgument("crop_min_z", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_z", default_value="2.0"),
            DeclareLaunchArgument("body_padding", default_value="0.02"),
            DeclareLaunchArgument("body_scale", default_value="1.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(filter_share / "launch" / "filters_only.launch.py")
                ),
                launch_arguments=arguments.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="body_filter_rviz",
                output="screen",
                arguments=["-d", str(filter_share / "rviz" / "body_filter.rviz")],
                condition=IfCondition(rviz),
            ),
        ]
    )
