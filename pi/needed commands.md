# commands

`source install/setup.bash`

## Lidar

`ros2 launch rplidar_ros view_rplidar.launch.py`
`ros2 topic echo /scan`

## Teleop

## Serial bridge

<!-- `ros2 run serial_driver serial_bridge --ros-args --params-file ./src/transport_drivers/serial_driver/params/example.params.yaml` -->

`ros2 launch serial_driver serial_driver_bridge_node.launch.py`

`ros2 topic pub /serial_write std_msgs/String "data: hello world" -t 1`

`ros2 topic pub /serial_write std_msgs/float32 "data: 97" -t 1`

# topics

## Lidar

`/scan`

## Teleop

`/cmd_vel`

## serial bridge

connected to `/dev/ttyACM0`
baud_rate: 115200

### Topics

`serial_read`
`serial_write`