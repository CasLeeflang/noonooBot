import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)


def write_read():
    # time.sleep(1)
    data = arduino.readline()
    return data


while True:
    value = write_read()
    print(value)  # printing the value
