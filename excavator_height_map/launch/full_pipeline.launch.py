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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    patchwork_share = Path(get_package_share_directory("excavator_patchwork"))
    height_map_share = Path(get_package_share_directory("excavator_height_map"))
    start_rviz = LaunchConfiguration("start_rviz")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")
    enable_body_filter = LaunchConfiguration("enable_body_filter")

    map_length_x = ParameterValue(
        PythonExpression(
            ["float('", crop_max_x, "') - float('", crop_min_x, "')"]
        ),
        value_type=float,
    )
    map_length_y = ParameterValue(
        PythonExpression(
            ["float('", crop_max_y, "') - float('", crop_min_y, "')"]
        ),
        value_type=float,
    )
    map_center_x = ParameterValue(
        PythonExpression(
            ["(float('", crop_max_x, "') + float('", crop_min_x, "')) / 2.0"]
        ),
        value_type=float,
    )
    map_center_y = ParameterValue(
        PythonExpression(
            ["(float('", crop_max_y, "') + float('", crop_min_y, "')) / 2.0"]
        ),
        value_type=float,
    )

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
                    )
                ],
            ),
            Node(
                package="excavator_height_map",
                executable="excavator_height_map_node",
                name="excavator_height_map",
                output="screen",
                parameters=[
                    str(height_map_share / "config" / "height_map.yaml"),
                    {
                        "length_x": map_length_x,
                        "length_y": map_length_y,
                        "center_x": map_center_x,
                        "center_y": map_center_y,
                    },
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
