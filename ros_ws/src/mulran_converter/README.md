# README
A short guidancebook to convert MulRan dataset to ROS2 bag.

Base Environment: Ubuntu 20.04, ROS2 Foxy. (If you are using different environment, it may not work.)

Refs: [Humble ROS2 MulRan Converter](https://github.com/ASIG-X/ros2bag_MulRan)

## Make Ros2Bag From MulRan Dataset
### mytest.cpp
Build and Run:
```bash
cd ~/Fast-LIO2_SC-SLAM/ros_ws
colcon build --packages-select mulran_converter --symlink-install
source install/setup.bash
# ros2 run mulran_converter mulran_converter
# wait for the bag file to be generated
# run (recommended)
ros2 run mulran_converter mulran_converter --ros-args -p input_folder:=/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA -p output_bag_prefix:=/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub
# launch
ros2 launch mulran_converter convert.launch.py input_folder:=/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA output_bag_prefix:=/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub
```
Check and Play:
```bash
# Check bag file
ros2 bag info /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub/my_test.bag
# Play bag file
ros2 bag play /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub/my_test.bag --loop
# another terminal
ros2 topic echo /livox/imu /gps/fix
