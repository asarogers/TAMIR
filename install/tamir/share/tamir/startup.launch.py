import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    """Generate launch description with nodes and service call."""
    return LaunchDescription([
        # Include the RealSense camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            launch_arguments={
                'depth_module.profile': '1280x720x30',
                'rgb_camera.profile': '1280x720x30',
                'enable_sync': 'true',
            }.items()
        ),

        # Launch vision node
        Node(
            package='tamir',
            executable='vision',
            output='screen',
        ),

        # Launch Tamir interface node
        Node(
            package='tamir',
            executable='tamir_interface',
            output='screen',
        ),

        # Call the /pair_bluetooth service
        ExecuteProcess(
            cmd=[
                FindExecutable(name="ros2"),
                "service call",
                "/pair_bluetooth",
                "std_srvs/srv/Empty"
            ],
            shell=True
        ),
    ])
