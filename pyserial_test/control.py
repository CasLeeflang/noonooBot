import serial
import time
import numpy as np
import struct

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

angular_z = float(0.0)

linear_x = float(0.0)

increment = float(0.1)


def write_to_serial(x):
    if (x):
        arduino.write(bytes(x, 'utf-8'))
        time.sleep(0.05)
        data = arduino.readline()
        return data


def convert_input(input):
    global angular_z
    global linear_x
    if input == 'w':
        linear_x += increment
    elif input == 's':
        linear_x -= increment
    elif input == 'a':
        angular_z -= increment
    elif input == 'd':
        angular_z += increment
    elif input == 'q':
        linear_x = 0.0
        angular_z = 0.0
    else:
        print('Invalid input')
    return 'Linear velocity (x): ' + str(
        linear_x) + ' Angular velocity (z): ' + str(angular_z)


def float_to_hex(f):
    hex_conversion = hex(struct.unpack('<I', struct.pack('<f', f))[0])
    hex_conversion = hex_conversion.split('x')[1]
    zfill = hex_conversion.zfill(8)
    return zfill


while True:
    message = convert_input(input())  # Taking input from user
    print(message)

    twist = float_to_hex(linear_x) + float_to_hex(angular_z) + '0x0a'

    print(twist)
    write_to_serial(twist)
