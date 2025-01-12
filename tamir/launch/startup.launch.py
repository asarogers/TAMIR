import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    venv_path = os.path.expanduser('~/repo/winter/project/TAMIR/myenv')

    # Command to source the virtual environment
    source_venv = ExecuteProcess(
        cmd=['bash', '-c', f'source {venv_path}/bin/activate && exec "$@"', 'bash'],
        output='screen'
    )
    
    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     'demo',
            #     default_value='False',
            #     description='Demo Only? (True will launch Franka demo.launch.py)',
            # ),
            # IncludeLaunchDescription(
            #     PathJoinSubstitution(
            #         [
            #             FindPackageShare('franka_fer_moveit_config'),
            #             'launch',
            #             'demo.launch.py',
            #         ]
            #     ),
            #     condition=IfCondition(
            #         EqualsSubstitution(LaunchConfiguration('demo'), 'True')
            #     ),
            # ),
            # IncludeLaunchDescription(
            #     PathJoinSubstitution(
            #         [
            #             FindPackageShare('realsense'), 'camera.launch.py',
            #         ]
            #     ),
            #     condition=IfCondition(
            #         EqualsSubstitution(LaunchConfiguration('demo'), 'False')
            #     ),
            # ),
            # Node(
            #     package='rviz2',
            #     executable='rviz2',
            #     arguments=[
            #         '-d', PathJoinSubstitution(
            #             [FindPackageShare('toast'), 'toast_view.rviz']
            #         ),
            #         '--ros-args', '--log-level', 'WARN'
            #     ],
            #     condition=IfCondition(
            #         EqualsSubstitution(LaunchConfiguration('demo'), 'False')
            #     ),
            # ),
            Node(
            package='tamir',
            executable='tamir_interface',
            output='screen',
            ),
            # Node(package='toast', executable='transform_auditor')
        ]
    )
