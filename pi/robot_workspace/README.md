# Noonoo bot command reference

## Commands

### Normal operation

`ros2 launch launch/noonoo_launch.py`

### Testing

#### run only serial node

`ros2 run cv2serial node`

### Control

#### Keyboard control

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

#### Joystick (xbox) control

`ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'`

### Misc.

`source install/setup.bash`

`colcon build --symlink-install`