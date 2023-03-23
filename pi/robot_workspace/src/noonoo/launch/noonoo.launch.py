from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # example node
        Node(package='name_of_package',
             executable='name_of_executable',
             name='name_of_node'),
    ])