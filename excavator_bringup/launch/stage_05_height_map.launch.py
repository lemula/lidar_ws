from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def map_value(expression):
    return ParameterValue(PythonExpression(expression), value_type=float)


def generate_launch_description():
    height_map_share = Path(get_package_share_directory("excavator_height_map"))
    rviz = LaunchConfiguration("rviz")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")

    length_x = map_value(
        ["float('", crop_max_x, "') - float('", crop_min_x, "')"]
    )
    length_y = map_value(
        ["float('", crop_max_y, "') - float('", crop_min_y, "')"]
    )
    center_x = map_value(
        ["(float('", crop_max_x, "') + float('", crop_min_x, "')) / 2.0"]
    )
    center_y = map_value(
        ["(float('", crop_max_y, "') + float('", crop_min_y, "')) / 2.0"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "ground_topic", default_value="/terrain/ground_points"
            ),
            DeclareLaunchArgument(
                "nonground_topic", default_value="/terrain/nonground_points"
            ),
            DeclareLaunchArgument(
                "grid_map_topic", default_value="/terrain/grid_map"
            ),
            DeclareLaunchArgument("crop_min_x", default_value="-0.5"),
            DeclareLaunchArgument("crop_max_x", default_value="1.0"),
            DeclareLaunchArgument("crop_min_y", default_value="0.0"),
            DeclareLaunchArgument("crop_max_y", default_value="1.5"),
            Node(
                package="excavator_height_map",
                executable="excavator_height_map_node",
                name="excavator_height_map",
                output="screen",
                parameters=[
                    str(height_map_share / "config" / "height_map.yaml"),
                    {
                        "ground_topic": LaunchConfiguration("ground_topic"),
                        "nonground_topic": LaunchConfiguration("nonground_topic"),
                        "grid_map_topic": LaunchConfiguration("grid_map_topic"),
                        "length_x": length_x,
                        "length_y": length_y,
                        "center_x": center_x,
                        "center_y": center_y,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="height_map_rviz",
                output="screen",
                arguments=[
                    "-d",
                    str(height_map_share / "rviz" / "height_map.rviz"),
                ],
                condition=IfCondition(rviz),
            ),
        ]
    )
