from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='action',
            executable='action_server',
            name='action_server',
            output='screen'
        ),
        Node(
            package='action',
            executable='action_client',
            name='action_client',
            output='screen'
        ),
        Node(
            package='topic',
            executable='data_pull',
            name='data_pull',
            output='screen'
        ),       
        Node(
            package='topic',
            executable='data_storage',
            name='data_storage',
            output='screen'
        ),     
        Node(
            package='topic',
            executable='strategy1',
            name='strategy1',
            output='screen'
        ),
        Node(
            package='topic',
            executable='strategy2',
            name='strategy2',
            output='screen'
        )                     
    ])