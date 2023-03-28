1. Clone the package into <workspace>/src
2. navigate <workspace> directory
3. run 'colcon build --symlink-install'
4. run 'source install/setup.bash'

You can now run the node.
Use teleop_keyboard_twist to publish to cmd_vel
Make sure an arduino is connectd to ttyACM0, if its connected to a different port navigate to the package folder/cs2serial/cv2serial/cv_node.py and change the port
