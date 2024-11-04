"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world", default_value="cafe.world", description="world file name"
            ),
            DeclareLaunchArgument(
                'sample_app', default_value='false'
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
            Node(
                package="joy",
                executable="joy_node",
                remappings=[("joy", "/device/mavros/rc_joy/joy")],
                output="screen",
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': ParameterValue(Command(['xacro ', str(get_package_share_path('espresso_frame_gazebo_launch')) + '/urdf/all.urdf']))}],
            ),
            Node(
                package="espresso_frame_gazebo_launch",
                executable="sample_app.py",
                output="screen",
                namespace="/app",
                condition=IfCondition(LaunchConfiguration('sample_app')),
            ),
        ]
    )
