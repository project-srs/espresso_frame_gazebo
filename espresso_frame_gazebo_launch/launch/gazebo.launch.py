"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world", default_value="cafe.world", description="world file name"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("gazebo_ros"),
                        "/launch/gzserver.launch.py",
                    ]
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "verbose": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("gazebo_ros"),
                        "/launch/gzclient.launch.py",
                    ]
                ),
                launch_arguments={"verbose": "true"}.items(),
            ),
        ]
    )
