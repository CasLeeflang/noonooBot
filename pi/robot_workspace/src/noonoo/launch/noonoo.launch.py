from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # example node
        Node(package='noonoo',
             executable='service_arduino_connectivity',
             name='service_arduino_connectivity'),
    ])