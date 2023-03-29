import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import ByteMultiArray, Byte
import serial
import struct


class serialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        # ROS topic --[data]--> Serial device
        self.subscription = self.create_subscription(Twist, 'cmd_vel',
                                                     self.write_serial, 10)

        # Serial device --[data]--> ROS topic
        self.publisher = self.create_publisher(Byte, 'serial_read', 10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.read_serial)

        self.get_logger().info('Starting serial bridge node')
        self.subscription
        self.ser = serial.Serial('/dev/ttyACM0')
        self.ser.baudrate = 115200

        self.ser.reset_input_buffer()

    def write_serial(self, msg):

        sending = bytearray()

        sending += struct.pack('f', (msg.angular.z * 1.5))
        sending += struct.pack('f', (msg.linear.x * 0.4))
        sending.append(0x0d)
        sending.append(0x0a)

        self.ser.write(sending)

        # received_message = self.ser.read_until(b'\r\n')
        # self.get_logger().info('Received: "%s"' % received_message)

        self.get_logger().info('Linear: "%s"' % msg.linear.x)
        self.get_logger().info('Angular: "%s"' % msg.angular.z)
        # self.read_serial()

    def read_serial(self):
        while (self.ser.in_waiting >= 14):
            received_message = self.ser.read_until(b'\r\n')
            self.publisher.publish(received_message)
            self.get_logger().info('Received: "%s"' % received_message)
            return


def main(args=None):
    rclpy.init(args=args)
    serial_bridge_node = serialBridgeNode()
    rclpy.spin(serial_bridge_node)
    serialBridgeNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()