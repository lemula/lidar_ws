from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    patchwork_share = Path(get_package_share_directory("excavator_patchwork"))
    height_map_share = Path(get_package_share_directory("excavator_height_map"))
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            # Scope the nested start_rviz=false setting so that it cannot
            # overwrite this launch file's start_rviz argument.
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                patchwork_share
                                / "launch"
                                / "full_pipeline.launch.py"
                            )
                        ),
                        launch_arguments={"start_rviz": "false"}.items(),
                    )
                ],
            ),
            Node(
                package="excavator_height_map",
                executable="excavator_height_map_node",
                name="excavator_height_map",
                output="screen",
                parameters=[
                    str(height_map_share / "config" / "height_map.yaml")
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="height_map_rviz",
                output="screen",
                arguments=["-d", str(height_map_share / "rviz" / "height_map.rviz")],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
