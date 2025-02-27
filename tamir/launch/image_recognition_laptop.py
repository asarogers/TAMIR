from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tamir',
            executable='tracker',
            name='tracker_node',
            output='screen',
            parameters=[],
            # remappings=[('/image', '/camera/image_raw')]
        ),
        # Node(
        #     package='rosbot',
        #     executable='depth_node',
        #     name='depth_node',
        #     output='screen',
        #     parameters=[],
        # ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/depth/camera_info')
            ],
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('tamir'), 'tags.yaml']),
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),
    ])
