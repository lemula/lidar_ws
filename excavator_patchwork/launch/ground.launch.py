from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("excavator_patchwork"))
        / "config"
        / "patchworkpp.yaml"
    )

    input_topic = LaunchConfiguration("input_topic")
    ground_topic = LaunchConfiguration("ground_topic")
    nonground_topic = LaunchConfiguration("nonground_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/localmap/body_filtered_points"
            ),
            DeclareLaunchArgument(
                "ground_topic", default_value="/terrain/ground_points"
            ),
            DeclareLaunchArgument(
                "nonground_topic", default_value="/terrain/nonground_points"
            ),
            Node(
                package="excavator_patchwork",
                executable="excavator_patchwork_node",
                name="excavator_patchwork",
                output="screen",
                parameters=[
                    config,
                    {
                        "input_topic": input_topic,
                        "ground_topic": ground_topic,
                        "nonground_topic": nonground_topic,
                    },
                ],
            ),
        ]
    )
