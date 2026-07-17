import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    filter_share = get_package_share_directory("robot_body_filter")
    description_share = get_package_share_directory("excavator_description")
    lidar_share = get_package_share_directory("rslidar_sdk")

    urdf_path = os.path.join(description_share, "urdf", "excavator.urdf")
    config_path = os.path.join(
        filter_share, "config", "excavator_body_filter.yaml"
    )

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    start_lidar = LaunchConfiguration("start_lidar")
    start_crop_box = LaunchConfiguration("start_crop_box")
    start_description = LaunchConfiguration("start_description")
    start_default_joint_states = LaunchConfiguration(
        "start_default_joint_states"
    )
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_crop_box", default_value="true"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument(
                "start_default_joint_states", default_value="true"
            ),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("crop_min_x", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_x", default_value="2.0"),
            DeclareLaunchArgument("crop_min_y", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_y", default_value="2.0"),
            DeclareLaunchArgument("crop_min_z", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_z", default_value="5.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        lidar_share,
                        "launch",
                        "rslidar_airy_localmap.launch.py",
                    )
                ),
                condition=IfCondition(start_lidar),
            ),
            Node(
                package="pcl_ros",
                executable="filter_crop_box_node",
                name="pointcloud_crop",
                output="screen",
                remappings=[
                    ("input", "/localmap/world_points"),
                    ("output", "/localmap/cropped_points"),
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
                condition=IfCondition(start_crop_box),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(description_share, "launch", "display.launch.py")
                ),
                launch_arguments={
                    "use_tip_pose_publisher": "false",
                    "use_sim_time": use_sim_time,
                }.items(),
                condition=IfCondition(start_description),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="body_filter_default_joint_states",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                        "rate": 20,
                    }
                ],
                condition=IfCondition(start_default_joint_states),
            ),
            Node(
                package="robot_body_filter",
                executable="robot_body_filter_node",
                name="robot_body_filter",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="body_filter_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(filter_share, "rviz", "body_filter.rviz"),
                ],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
