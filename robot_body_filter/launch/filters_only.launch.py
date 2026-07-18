import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    filter_share = get_package_share_directory("robot_body_filter")
    description_share = get_package_share_directory("excavator_description")

    urdf_path = os.path.join(description_share, "urdf", "excavator.urdf")
    filter_config_path = os.path.join(
        filter_share, "config", "excavator_body_filter.yaml"
    )

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    input_topic = LaunchConfiguration("input_topic")
    cropped_topic = LaunchConfiguration("cropped_topic")
    output_topic = LaunchConfiguration("output_topic")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")
    body_padding = LaunchConfiguration("body_padding")
    body_scale = LaunchConfiguration("body_scale")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_body_filter = LaunchConfiguration("enable_body_filter")
    crop_output_topic = IfElseSubstitution(
        enable_body_filter, cropped_topic, output_topic
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/localmap/world_points"
            ),
            DeclareLaunchArgument(
                "cropped_topic", default_value="/localmap/cropped_points"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/localmap/body_filtered_points"
            ),
            DeclareLaunchArgument("crop_min_x", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_x", default_value="2.0"),
            DeclareLaunchArgument("crop_min_y", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_y", default_value="2.0"),
            DeclareLaunchArgument("crop_min_z", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_z", default_value="5.0"),
            DeclareLaunchArgument("body_padding", default_value="0.02"),
            DeclareLaunchArgument("body_scale", default_value="1.0"),
            DeclareLaunchArgument("enable_body_filter", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="pcl_ros",
                executable="filter_crop_box_node",
                name="pointcloud_crop",
                output="screen",
                remappings=[
                    ("input", input_topic),
                    ("output", crop_output_topic),
                ],
                parameters=[
                    {
                        "min_x": ParameterValue(crop_min_x, value_type=float),
                        "max_x": ParameterValue(crop_max_x, value_type=float),
                        "min_y": ParameterValue(crop_min_y, value_type=float),
                        "max_y": ParameterValue(crop_max_y, value_type=float),
                        "min_z": ParameterValue(crop_min_z, value_type=float),
                        "max_z": ParameterValue(crop_max_z, value_type=float),
                        "negative": False,
                        "keep_organized": False,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="robot_body_filter",
                executable="robot_body_filter_node",
                name="robot_body_filter",
                output="screen",
                parameters=[
                    filter_config_path,
                    {
                        "robot_description": robot_description,
                        "input_topic": cropped_topic,
                        "output_topic": output_topic,
                        "padding": ParameterValue(body_padding, value_type=float),
                        "scale": ParameterValue(body_scale, value_type=float),
                        "use_sim_time": use_sim_time,
                    },
                ],
                condition=IfCondition(enable_body_filter),
            ),
        ]
    )
