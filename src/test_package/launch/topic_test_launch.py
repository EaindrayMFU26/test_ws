from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='test_package',
                executable='talker',
                name='talker',
                output='screen',
                arguments= ['34']
            ),
            Node(
                package='test_package',
                executable='listener',
                name='listener',
                output='screen'
            )
        ]
    )