import serial
import time
# import keyboard

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

angular_z = float(0.0)

linear_x = float(0.0)


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
        linear_x += 0.1
    elif input == 's':
        linear_x -= 0.1
    elif input == 'a':
        angular_z -= 0.1
    elif input == 'd':
        angular_z += 0.1
    elif input == 'q':
        linear_x = 0.0
        angular_z = 0.0
    else:
        print('Invalid input')
    return 'Linear velocity (x): ' + str(
        linear_x) + ' Angular velocity (z): ' + str(angular_z)


# keyboard.add_hotkey('enter', lambda: function_x())

# def function_x():
#     print('You pressed enter!')

while True:
    message = convert_input(input())  # Taking input from user
    print(message)
    value = write_to_serial(message)
    print(value)  # printing the value
