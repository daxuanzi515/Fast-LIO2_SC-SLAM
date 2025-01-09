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


ros2 run mulran_converter mulran_converter --ros-args -p input_folder:=/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/DCC02_DATA -p output_bag_prefix:=/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub

ros2 run mulran_converter mulran_converter --ros-args -p input_folder:=/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/DCC03_DATA -p output_bag_prefix:=/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub

ros2 run mulran_converter mulran_converter --ros-args -p input_folder:=/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/Riverside02_DATA-p output_bag_prefix:=/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub

# launch
ros2 launch mulran_converter convert.launch.py input_folder:=/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA output_bag_prefix:=/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub
```
Put DATA in your input_folder:
```bash 
/input_folder
    xsens_imu.csv
    gps.csv
    data_stamp.csv
    Ouster/
        XXXX.bin
```
Edit your bag name:
```cpp
class CsvToBag : public rclcpp::Node {
public:
    CsvToBag() : Node("csv_to_bag") {
        this->declare_parameter<std::string>("input_folder", "/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA");
        this->declare_parameter<std::string>("output_bag_prefix", "/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub");
        
        input_folder_ = this->get_parameter("input_folder").as_string();
        output_bag_prefix_ = this->get_parameter("output_bag_prefix").as_string();
        imu_csv_path_ = input_folder_ + "/xsens_imu.csv";
        gps_csv_path_ = input_folder_ + "/gps.csv";
        data_stamp_csv_path_ = input_folder_ + "/data_stamp.csv";
        ouster_folder_path_ = input_folder_ + "/Ouster";
        output_bag_path_ = output_bag_prefix_ + "/my_test.bag"; // edit your name or only input your complete bag path
        // ignore
    }};
```

Check and Play:
```bash
# Check bag file
ros2 bag info /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub/my_test.bag
# Play bag file
ros2 bag play /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub/my_test.bag --loop
# another terminal
ros2 topic echo /livox/imu /gps/fix                                      
```