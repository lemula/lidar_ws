from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    patchwork_share = Path(get_package_share_directory("excavator_patchwork"))
    rviz = LaunchConfiguration("rviz")
    input_topic = LaunchConfiguration("input_topic")
    ground_topic = LaunchConfiguration("ground_topic")
    nonground_topic = LaunchConfiguration("nonground_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "input_topic", default_value="/localmap/body_filtered_points"
            ),
            DeclareLaunchArgument(
                "ground_topic", default_value="/terrain/ground_points"
            ),
            DeclareLaunchArgument(
                "nonground_topic", default_value="/terrain/nonground_points"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(patchwork_share / "launch" / "ground.launch.py")
                ),
                launch_arguments={
                    "input_topic": input_topic,
                    "ground_topic": ground_topic,
                    "nonground_topic": nonground_topic,
                }.items(),
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
                condition=IfCondition(rviz),
            ),
        ]
    )
