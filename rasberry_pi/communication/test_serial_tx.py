# Importing Libraries
import serial
import struct
import time
import pygame




arduino = serial.Serial(port='COM7', baudrate=115200, timeout=0)

cmdLinVel = 3.0
cmdAngVel = -2.34

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print(joysticks)


while True:
   
    
        

    if False:

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                print(pygame.joystick.Joystick(0).get_axis)

        cmdLinVel = float(input("Enter the linear velocity: ")) # Taking input from user
        cmdAngVel = float(input("Enter the angular velocity: "))

        sending = bytearray()
        sending += struct.pack("f", cmdLinVel)
        sending += struct.pack("f", cmdAngVel)
        sending.append(0x0d)
        sending.append(0x0a)

    

        print(list(sending)) # printing the value
        arduino.write(sending)
    