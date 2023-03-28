# Importing Libraries
import serial
import time
import struct

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

while True:
    incomingBytes = arduino.readline()
    #print(list(incomingBytes)) # len(incomingBytes)

    if len(incomingBytes) == 14:
        number = (struct.unpack('<f', incomingBytes[0:4]))
        linearNum = struct.unpack('<f', incomingBytes[4:8])
        rotNum = struct.unpack('<f', incomingBytes[8:12])
        print('x-pos = {:<30f} y-pos = {:<30f}  z-rot = {:<30f}'.format(number[0], linearNum[0], rotNum[0])) # printing the value

