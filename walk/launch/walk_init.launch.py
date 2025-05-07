from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch nao_ik
        Node(
            package='nao_ik',
            executable='nao_ik',
            name='nao_ik_node',
            output='screen',
        ),
        # Launch nao_phase_provider
        Node(
            package='nao_phase_provider',
            executable='nao_phase_provider',
            name='nao_phase_provider_node',
            output='screen',
        ),
        # Launch walk node
        Node(
            package='walk',
            executable='walk',
            name='walk_node',
            output='screen',
        ),
        # Launch init_walk_node
        Node(
            package='walk',
            executable='init_walk_node',
            name='init_walk_node',
            output='screen',
        ),
    ])