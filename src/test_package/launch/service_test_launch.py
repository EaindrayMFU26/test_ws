from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='test_package',
                executable='node_a',
                name='node_a',
                output='screen'
            ),
            Node(
                package='test_package',
                executable='node_b',
                name='node_b',
                output='screen'
            )
        ]
    )