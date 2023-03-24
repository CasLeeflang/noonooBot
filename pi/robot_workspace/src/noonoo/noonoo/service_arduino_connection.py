#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

last_data = ""
started = False
pub = rospy.Publisher('/odom', Pose, queue_size=1000)


def write_to_serial(data):
    arduino.write(bytes(data, 'utf-8'))


def read_from_serial():
    data = arduino.readline()
    print(data)
    return data


def get_data_from_cmd_vel(input: Twist):
    linear = input.linear
    angular = input.angular
    data_to_send = "linear: " + str(linear.x) + " " + str(
        linear.y) + " " + str(linear.z) + " angular: " + str(
            angular.x) + " " + str(angular.y) + " " + str(angular.z)
    print(data_to_send)
    return write_to_serial(data_to_send)


def callback(data):
    print("New message received")
    global started, last_data
    last_data = data
    if (not started):
        started = True


def timer_callback(event):
    global started, pub, last_data
    if (started):
        pub.publish(last_data)
        print("Last message published")


def listener():

    rospy.init_node('arduino_connection', anonymous=True)

    cmd_vel_data = rospy.Subscriber('cmd_vel', String, callback)
    timer = rospy.Timer(rospy.Duration(0.5), timer_callback)

    rospy.spin()
    timer.shutdown()


if __name__ == '__main__':
    print("Running")
    listener()