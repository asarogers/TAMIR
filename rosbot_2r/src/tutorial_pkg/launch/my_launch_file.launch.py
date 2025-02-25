from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tutorial_pkg',
            executable='tracker',
            name='tracker_node',
            output='screen',
            parameters=[],
            remappings=[('/image', '/camera/image_raw')]
        )
    ])
