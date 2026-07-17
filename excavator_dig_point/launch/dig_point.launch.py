from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("excavator_dig_point"))
        / "config"
        / "dig_point.yaml"
    )
    input_topic = LaunchConfiguration("input_topic")
    start_rviz = LaunchConfiguration("start_rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/terrain/grid_map"
            ),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            Node(
                package="excavator_dig_point",
                executable="excavator_dig_point_node",
                name="excavator_dig_point",
                output="screen",
                parameters=[
                    config,
                    {"input_topic": input_topic},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="dig_point_rviz",
                output="screen",
                arguments=[
                    "-d",
                    str(
                        Path(get_package_share_directory("excavator_dig_point"))
                        / "rviz"
                        / "dig_point.rviz"
                    ),
                ],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
