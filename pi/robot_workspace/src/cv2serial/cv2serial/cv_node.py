import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct


class cvNode(Node):

    def __init__(self):
        super().__init__('cv_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel',
                                                     self.listener_callback,
                                                     10)

        self.get_logger().info('Starting cv2serial node')
        self.subscription
        self.ser = serial.Serial('/dev/ttyACM0')
        self.ser.baudrate = 115200

    def listener_callback(self, msg):

        sending = bytearray()

        sending += struct.pack('f', msg.angular.z)
        sending += struct.pack('f', msg.linear.x)
        sending.append(0x0d)
        sending.append(0x0a)

        self.ser.write(sending)

        # received_message = self.ser.read_until(b'\r\n')
        # self.get_logger().info('Received: "%s"' % received_message)

        self.ser.reset_input_buffer()

        self.get_logger().info('Linear: "%s"' % msg.linear.x)
        self.get_logger().info('Angular: "%s"' % msg.angular.z)


def main(args=None):
    rclpy.init(args=args)
    cv_node = cvNode()
    rclpy.spin(cv_node)
    cvNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()