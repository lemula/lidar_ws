from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def stage(file_name, condition, arguments):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("excavator_bringup"), "launch", file_name]
                    )
                ),
                condition=condition,
                launch_arguments=arguments.items(),
            )
        ],
    )


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    start_lidar = LaunchConfiguration("start_lidar")
    start_urdf = LaunchConfiguration("start_urdf")
    start_filters = LaunchConfiguration("start_filters")
    start_ground = LaunchConfiguration("start_ground")
    start_height_map = LaunchConfiguration("start_height_map")
    start_dig_point = LaunchConfiguration("start_dig_point")
    enable_body_filter = LaunchConfiguration("enable_body_filter")
    crop_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "crop_min_x",
            "crop_max_x",
            "crop_min_y",
            "crop_max_y",
            "crop_min_z",
            "crop_max_z",
        )
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_urdf", default_value="true"),
            DeclareLaunchArgument("start_filters", default_value="true"),
            DeclareLaunchArgument("start_ground", default_value="true"),
            DeclareLaunchArgument("start_height_map", default_value="true"),
            DeclareLaunchArgument("start_dig_point", default_value="true"),
            DeclareLaunchArgument("enable_body_filter", default_value="true"),
            DeclareLaunchArgument("crop_min_x", default_value="-0.5"),
            DeclareLaunchArgument("crop_max_x", default_value="1.0"),
            DeclareLaunchArgument("crop_min_y", default_value="0.0"),
            DeclareLaunchArgument("crop_max_y", default_value="1.5"),
            DeclareLaunchArgument("crop_min_z", default_value="-2.0"),
            DeclareLaunchArgument("crop_max_z", default_value="2.0"),
            stage(
                "stage_01_lidar.launch.py",
                IfCondition(start_lidar),
                {"rviz": "false"},
            ),
            stage(
                "stage_02_urdf.launch.py",
                IfCondition(start_urdf),
                {"rviz": "false"},
            ),
            stage(
                "stage_03_filters.launch.py",
                IfCondition(start_filters),
                {
                    "rviz": "false",
                    "enable_body_filter": enable_body_filter,
                    **crop_arguments,
                },
            ),
            stage(
                "stage_04_ground.launch.py",
                IfCondition(start_ground),
                {"rviz": "false"},
            ),
            stage(
                "stage_05_height_map.launch.py",
                IfCondition(start_height_map),
                {
                    "rviz": "false",
                    "crop_min_x": crop_arguments["crop_min_x"],
                    "crop_max_x": crop_arguments["crop_max_x"],
                    "crop_min_y": crop_arguments["crop_min_y"],
                    "crop_max_y": crop_arguments["crop_max_y"],
                },
            ),
            stage(
                "stage_06_dig_point.launch.py",
                IfCondition(start_dig_point),
                {"rviz": rviz},
            ),
        ]
    )
