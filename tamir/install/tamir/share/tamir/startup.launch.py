import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    return LaunchDescription([
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
            Node(
                package='tamir',
                executable='vision',
                output='screen',
            ),
            Node(
                package='tamir',
                executable='tamir_interface',
                output='screen',
            ),
        ]
    )
