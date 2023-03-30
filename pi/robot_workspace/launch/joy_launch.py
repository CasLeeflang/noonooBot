from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy',
             node_executable='joy_node',
             name='joy_node',
             parameters=[{
                 'dev': '/dev/input/js0',
                 'deadzone': 0.3,
                 'autorepeat_rate': 20.0,
             }]),
        Node(
            package='teleop_twist_joy',
            node_executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                '/home/pi/noonooBot/pi/robot_workspace/config/xbox-noonoo.yaml'
            ])
    ])