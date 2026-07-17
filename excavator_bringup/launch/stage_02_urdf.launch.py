from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = Path(get_package_share_directory("excavator_description"))
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_tip_pose_publisher = LaunchConfiguration("use_tip_pose_publisher")
    reference_frame = LaunchConfiguration("reference_frame")
    pose_topic = LaunchConfiguration("pose_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_tip_pose_publisher", default_value="true"),
            DeclareLaunchArgument("reference_frame", default_value="world"),
            DeclareLaunchArgument(
                "pose_topic", default_value="/bucket_tip_pose_world"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "display.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_tip_pose_publisher": use_tip_pose_publisher,
                    "reference_frame": reference_frame,
                    "pose_topic": pose_topic,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "slider.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "rviz.launch.py")
                ),
                condition=IfCondition(rviz),
            ),
        ]
    )
