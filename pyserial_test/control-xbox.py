from __future__ import print_function
import xbox
import serial
import struct


arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0)

xpos = ''
ypos = ''
zrot = ''

# Format floating point number to string format -x.xxx
def fmtFloat(n):
    return '{:6.3f}'.format(n)

# Print one or more values without a line feed
def show(*args):
    for arg in args:
        print(arg, end="")

# Print true or false value based on a boolean, without linefeed
def showIf(boolean, ifTrue, ifFalse=" "):
    if boolean:
        show(ifTrue)
    else:
        show(ifFalse)

# Instantiate the controller
joy = xbox.Joystick()

def float_to_hex(f):
    hex_conversion = hex(struct.unpack('<I', struct.pack('<f', f))[0])
    hex_conversion = hex_conversion.split('x')[1]
    zfill = hex_conversion.zfill(8)
    return zfill

def construct_message(linear, angular):
     twist = float_to_hex(linear) + float_to_hex(angular) + '0x0d' + '0x0a'
     return twist

def write_to_serial(x):
    if (x):
        arduino.write(bytes(x, 'utf-8'))
        # time.sleep(0.05)
        # show(read_from_serial())

def read_from_serial():
    incomingBytes = arduino.readline()
    #print(list(incomingBytes)) # len(incomingBytes)

    if len(incomingBytes) == 14:
        number = (struct.unpack('<f', incomingBytes[0:4]))
        linearNum = struct.unpack('<f', incomingBytes[4:8])
        rotNum = struct.unpack('<f', incomingBytes[8:12])
        # show('x-pos = {:<25f} y-pos = {:<25f}  z-rot = {:<25f}'.format(number[0], linearNum[0], rotNum[0])) # printing the value
        xpos = 'x-pos = {:<25f}'.format(number[0])
        ypos = 'y-pos = {:<25f}'.format(linearNum[0])
        zrot = 'z-rot = {:<25f}'.format(rotNum[0])


# Show various axis and button states until Back button is pressed
print("Xbox controller sample: Press Back button to exit")
while not joy.Back():
    # Show connection status
    show("Connected:")
    showIf(joy.connected(), "Y", "N")
    # Left analog stick
    show("  Left X/Y:", fmtFloat(joy.leftX()), "/", fmtFloat(joy.leftY()))

    # transmit serial command
    write_to_serial(construct_message(joy.leftY(), joy.leftX()))

    # Right trigger
    # show("  RightTrg:", fmtFloat(joy.rightTrigger()))
    # # A/B/X/Y buttons
    # show("  Buttons:")
    # showIf(joy.A(), "A")
    # showIf(joy.B(), "B")
    # showIf(joy.X(), "X")
    # showIf(joy.Y(), "Y")
    # # Dpad U/D/L/R
    # show("  Dpad:")
    # showIf(joy.dpadUp(),    "U")
    # showIf(joy.dpadDown(),  "D")
    # showIf(joy.dpadLeft(),  "L")
    # showIf(joy.dpadRight(), "R")

    # read serial
    # show(read_from_serial())
    read_from_serial()
    show(xpos)
    show(ypos)
    show(zrot)

    # Move cursor back to start of line
    show(chr(13))
# Close out when done
joy.close()
