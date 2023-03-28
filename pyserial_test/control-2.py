import serial
import struct

arduino = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=0)

cmdLinVel = 3.0
cmdAngVel = -2.34

while True:

    cmdLinVel = float(input("Enter the linear velocity: ")) # Taking input from user
    cmdAngVel = float(input("Enter the angular velocity: "))

    sending = bytearray()
    sending += struct.pack("f", cmdLinVel)
    sending += struct.pack("f", cmdAngVel)
    sending.append(0x0d)
    sending.append(0x0a)



    print(list(sending)) # printing the value
    arduino.write(sending)

    incomingBytes = arduino.readline()
    #print(list(incomingBytes)) # len(incomingBytes)

    if len(incomingBytes) == 14:
        number = (struct.unpack('<f', incomingBytes[0:4]))
        linearNum = struct.unpack('<f', incomingBytes[4:8])
        rotNum = struct.unpack('<f', incomingBytes[8:12])
        print('x-pos = {:<30f} y-pos = {:<30f}  z-rot = {:<30f}'.format(number[0], linearNum[0], rotNum[0])) # printing the value