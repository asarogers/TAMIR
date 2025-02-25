from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbot',
            executable='tracker',
            name='tracker_node',
            output='screen',
            parameters=[],
            # remappings=[('/image', '/camera/image_raw')]
        ),
        Node(
            package='rosbot',
            executable='depth_node',
            name='depth_node',
            output='screen',
            parameters=[],
        ),
        # Node(
        #     package='rosbot',
        #     executable='image_node',
        #     name='image_node',
        #     output='screen',
        #     parameters=[],
        #     # remappings=[('/image', '/camera/image_raw')]
        # )

    ])
