Steps to run the package:
1: unzip the cv2serial zip
2. add the package folder to your workspace /src file
3. navigate to workspace directory
4. Use Colcon to buil the workspace using 'colcon build --symlink-install'
5. run 'source install/setup.bash'
6. run the teleop_twist_keyboard node
7. run cv2serial node

Make sure the arduino is connected to USB dev/ttyACM0, otherwise you'll need to change the port in cv2serial/cv2serial/cv_node.py
