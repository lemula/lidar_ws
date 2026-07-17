from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = Path(get_package_share_directory("rslidar_sdk"))
    rviz = LaunchConfiguration("rviz")
    config_path = LaunchConfiguration("config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "config_path",
                default_value=str(lidar_share / "config" / "config_localmap.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(lidar_share / "launch" / "rslidar_airy_localmap.launch.py")
                ),
                launch_arguments={"config_path": config_path}.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="lidar_rviz",
                output="screen",
                arguments=["-d", str(lidar_share / "rviz" / "rviz2.rviz")],
                condition=IfCondition(rviz),
            ),
        ]
    )
