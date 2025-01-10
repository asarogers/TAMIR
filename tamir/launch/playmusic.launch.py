"""Launch necessary nodes to make toast."""
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    """TODO."""
    os.environ["PYTHONPATH"] += ":/path/to/myenv/lib/python3.12/site-packages"
    return LaunchDescription(
        [
            Node(package='tamir', executable='music_node'),
            # Node(package='toast', executable='transform_auditor')
        ]
    )
