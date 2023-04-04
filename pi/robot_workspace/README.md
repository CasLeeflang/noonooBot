# Noonoo bot command reference

## Commands

### Build

`noonoobuild`

### Launch

`noonoolaunch`

aka

`ros2 launch launch/noonoo_launch.py`

#### SLAM

`noonooslam`

aka

`ros2 launch slam_toolbox online_sync_launch.py`

`ros2 run slam_toolbox sync_slam_toolbox_node use_sim_time:=false max_laser_range:=12`

#### Navigation

`noonoonav`

aka

`ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false`


### Control

#### **Keyboard control**

`noonookeyboard`

aka

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

#### **Joystick (xbox) control**

`noonoojoy`

aka

`ros2 launch launch/joy_launch.py`


### Misc.

`source install/setup.bash`

`colcon build --symlink-install`