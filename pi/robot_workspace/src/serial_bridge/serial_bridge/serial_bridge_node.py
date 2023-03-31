import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct

class serialBridgeNode(Node):

  

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.logSerialWrite = False
        self.logSerialRead = True
        self.serial_port = '/dev/ttyACM0'
        self.serial_msg_length = 14
        self.serial_com_time = 0.05
        self.baudrate = 115200

        self.subscription = self.create_subscription(Twist, 'cmd_vel',
                                                     self.write_serial, 
                                                     10)

        self.publisher = self.create_publisher(Odometry, 
                                               'odom', 
                                               10)

        self.timer = self.create_timer(self.serial_com_time, 
                                       self.read_serial)
        
        self.publisher
        self.subscription

        self.get_logger().info('Initializing serial bridge node')
        self.ser = serial.Serial(self.serial_port)
        self.ser.baudrate = self.baudrate
        self.ser.reset_input_buffer()

    def unpack_serial_msg(message):
        odometry_msg = Odometry()

        odometry_msg.pose.pose.position.x = struct.unpack(
            '<f', message[0:4])[0]
        odometry_msg.pose.pose.position.y = struct.unpack(
            '<f', message[4:8])[0]
        odometry_msg.pose.pose.orientation.z = struct.unpack(
            '<f', message[8:12])[0]
        return odometry_msg
    
    def pack_serial_msg(message):
        sending = bytearray()
        sending += struct.pack('<f', message.linear.x)
        sending += struct.pack('<f', message.angular.z)
        sending.append(0x0d)
        sending.append(0x0a)
        return sending

    def write_serial(self, msg):
        self.ser.write(self.pack_serial_msg(msg))

        if self.logSerialWrite:
            self.get_logger().info('Linear: "%s"' % msg.linear.x)
            self.get_logger().info('Angular: "%s"' % msg.angular.z)

    def read_serial(self):
        while (self.ser.in_waiting >= self.serial_msg_length):
            received_message = self.ser.read_until(b'\r\n')

            odometry_msg = self.unpack_serial_msg(received_message)

            self.publisher.publish(odometry_msg)

            if self.logSerialRead:
                self.get_logger().info(
                    f'Current position: \n X: {str(odometry_msg.pose.pose.position.x)[:3]} Y: {str(odometry_msg.pose.pose.position.y)[:3]} Z-orientation: {str(odometry_msg.pose.pose.orientation.z)[:3]}'
                )
            return


def main(args=None):
    rclpy.init(args=args)
    serial_bridge_node = serialBridgeNode()
    rclpy.spin(serial_bridge_node)
    serialBridgeNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()