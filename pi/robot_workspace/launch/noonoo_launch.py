from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(package='cv2serial',
        #      executable='node',
        #      output='screen',
        #      name='node'),
        Node(package='serial_bridge',
             executable='serial_bridge',
             output='screen',
             name='serial_bridge'),
        Node(package='tfbroadcaster',
             executable='tfbroadcaster',
             output='screen',
             name='tfbroadcaster'
             ),
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])