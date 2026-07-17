from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dig_point_share = Path(get_package_share_directory("excavator_dig_point"))
    rviz = LaunchConfiguration("rviz")
    input_topic = LaunchConfiguration("input_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("input_topic", default_value="/terrain/grid_map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(dig_point_share / "launch" / "dig_point.launch.py")
                ),
                launch_arguments={
                    "input_topic": input_topic,
                    "start_rviz": rviz,
                }.items(),
            ),
        ]
    )
