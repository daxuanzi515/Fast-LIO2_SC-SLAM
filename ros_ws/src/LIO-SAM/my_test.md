# Guide to Build LIO-SAM
github: https://github.com/TixiaoShan/LIO-SAM

It recommends using 4.1 release GTSAM but I use 4.2.0.
```bash
cd ~/Fast-LIO2_SC-SLAM/ros_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd LIO-SAM
git checkout ros2
cd ..
cd ..
colcon build --packages-select lio_sam --symlink-install
source install/setup.bash
```

## Fix GTSAM and Eigen Version Conflict
This error occurs again when GTSAM and Eigen version conflict.
```bash
In file included from /usr/local/include/gtsam/base/Matrix.h:28,
                 from /usr/local/include/gtsam/base/Manifold.h:22,
                 from /usr/local/include/gtsam/base/Lie.h:26,
                 from /usr/local/include/gtsam/base/VectorSpace.h:11,
                 from /usr/local/include/gtsam/geometry/Point2.h:20,
                 from /usr/local/include/gtsam/geometry/Unit3.h:23,
                 from /usr/local/include/gtsam/geometry/Rot3.h:26,
                 from /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/src/LIO-SAM/src/imuPreintegration.cpp:3:
/usr/local/include/gtsam/base/Vector.h:75:52: error: static assertion failed: Error: GTSAM was built against a different version of Eigen
   75 |     GTSAM_EIGEN_VERSION_WORLD==EIGEN_WORLD_VERSION &&
      |                                                    ^
In file included from /usr/local/include/gtsam/base/Matrix.h:28,
                 from /usr/local/include/gtsam/base/Manifold.h:22,
                 from /usr/local/include/gtsam/base/Lie.h:26,
                 from /usr/local/include/gtsam/base/VectorSpace.h:11,
                 from /usr/local/include/gtsam/geometry/Point2.h:20,
                 from /usr/local/include/gtsam/geometry/Unit3.h:23,
                 from /usr/local/include/gtsam/geometry/Rot3.h:26,
                 from /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/src/LIO-SAM/src/mapOptmization.cpp:4:
/usr/local/include/gtsam/base/Vector.h:75:52: error: static assertion failed: Error: GTSAM was built against a different version of Eigen
   75 |     GTSAM_EIGEN_VERSION_WORLD==EIGEN_WORLD_VERSION &&
      |                                                    ^
make[2]: *** [CMakeFiles/lio_sam_imuPreintegration.dir/build.make:63：CMakeFiles/lio_sam_imuPreintegration.dir/src/imuPreintegration.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:158：CMakeFiles/lio_sam_imuPreintegration.dir/all] 错误 2
make[1]: *** 正在等待未完成的任务....
make[2]: *** [CMakeFiles/lio_sam_mapOptimization.dir/build.make:63：CMakeFiles/lio_sam_mapOptimization.dir/src/mapOptmization.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:130：CMakeFiles/lio_sam_mapOptimization.dir/all] 错误 2
make: *** [Makefile:141：all] 错误 2
---
Failed   <<< lio_sam [34.9s, exited with code 2]

Summary: 0 packages finished [35.1s]
  1 package failed: lio_sam
  1 package had stderr output: lio_sam
```
Find your GTSAM and Eigen install path:
My 4.2.0 GTSAM install path is `/usr/local/include/gtsam`, and my GTSAM Eigen install path is `/usr/local/include/eigen3`.
Find this file and change eigen version to default version 3.3.7 like this:
```bash
cd /usr/local/include/eigen3/Eigen/src/Core/util
sudo gedit Macros.h
# fix original is 3.4.0
#define EIGEN_WORLD_VERSION 3
#define EIGEN_MAJOR_VERSION 3
#define EIGEN_MINOR_VERSION 7
```
Then, try to build again.
```bash
rm -rf build/lio_sam install/lio_sam log/lio_sam
colcon build --packages-select lio_sam --symlink-install
source install/setup.bash
# success!
Finished <<< lio_sam [42.7s]
Summary: 1 package finished [42.9s]
  1 package had stderr output: lio_sam
```
## Subscribed Topics
Check info in `LIO-SAM/config/params.yaml` and `LIO-SAM/launch/run.launch.py` to see what topics are subscribed and published.
In yaml file:
```yaml 
ros__parameters:
    # Topics
    pointCloudTopic: "/points"                   # Point cloud data
    imuTopic: "/imu/data"                        # IMU data
    odomTopic: "odometry/imu"                    # IMU pre-preintegration odometry, same frequency as IMU
    gpsTopic: "odometry/gpsz"                    # GPS odometry topic from navsat, see module_navsat.launch file
```
From ROS2 bag subscription:
- lidar -> imageProjection.cpp
- imu -> imuPreintegration.cpp, imageProjection.cpp
- gps -> mapOptimization.cpp

From node publication:
- imu odometry -> imuPreintegration.cpp
- cloud_info -> imageProjection.cpp, featureExtraction.cpp
- lidar odometry -> mapOptimization.cpp

Check details in `include/lio_sam/utility.hpp`:
```cpp
ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
{
    declare_parameter("pointCloudTopic", "points");
    get_parameter("pointCloudTopic", pointCloudTopic);
    declare_parameter("imuTopic", "imu/data");
    get_parameter("imuTopic", imuTopic);
    declare_parameter("odomTopic", "lio_sam/odometry/imu");
    get_parameter("odomTopic", odomTopic);
    declare_parameter("gpsTopic", "lio_sam/odometry/gps");
    get_parameter("gpsTopic", gpsTopic);
}

// change OUTPUT FOLDER
declare_parameter("savePCDDirectory", "/home/cxx/Fast-LIO2_SC-SLAM/output/lio_sam");
```
lidarTopic: `points` TYPE: `sensor_msgs::msg::PointCloud2`
imuTopic: `imu/data` TYPE: `sensor_msgs::msg::Imu`
gpsTopic: `lio_sam/odometry/gps` TYPE: `nav_msgs::msg::Odometry`

Use MulRan Dataset link: https://github.com/kimdaebeom/lio-sam-for-mulran/tree/main

Fix subscribed topics in `LIO-SAM/config/params.yaml`:
```yaml 
lio_sam:

  # Topics
  pointCloudTopic: "/raw_points"
  imuTopic: "/livox/imu" 
  odomTopic: "odometry/imu" 
  gpsTopic: "/tcpfix"     

  # Sensor Settings
  sensor: ouster   
  N_SCAN: 64   
  Horizon_SCAN: 1024 
  downsampleRate: 1   
  lidarMinRange: 1.0
  lidarMaxRange: 1000.0

  # IMU Settings
  imuAccNoise: 0.009939570888238808e-03
  imuGyrNoise: 0.005636343949698187e-03
  imuAccBiasN: 0.64356659353532566e-03
  imuGyrBiasN: 0.35640318696367613e-03
  imuGravity: 9.80511
  imuRPYWeight: 0.01

  # Extrinsics (lidar -> IMU)
  extrinsicTrans: [1.77, -0.00, -0.05]
  extrinsicRot: [-1.0, 0.0, 0.0,
                  0.0, -1.0, 0.0,
                  0.0, 0.0, 1.0]
  extrinsicRPY: [-1.0,  0.0, 0.0,
                 0.0, -1.0, 0.0,
                  0.0, 0.0, 1.0]
```


Play your bag:
```bash
ros2 bag play DCC01.bag --topics /raw_points /livox/imu --rate 0.2 --read-ahead-queue-size 300000 -l
```


Some problems:
```bash
[lio_sam_imageProjection-4] [INFO] [1736505343.228905521] [lio_sam_imageProjection]: Waiting for IMU data ...
[lio_sam_imageProjection-4] [INFO] [1736505343.611208889] [lio_sam_imageProjection]: Waiting for IMU data ...
[lio_sam_imageProjection-4] [INFO] [1736505343.614393248] [lio_sam_imageProjection]: Waiting for IMU data ...
[lio_sam_imageProjection-4] [INFO] [1736505343.615985533] [lio_sam_imageProjection]: Waiting for IMU data ...
[lio_sam_imageProjection-4] [INFO] [1736505343.617590325] [lio_sam_imageProjection]: Waiting for IMU data ...
[lio_sam_imageProjection-4] [INFO] [1736505343.977713102] [lio_sam_imageProjection]: Waiting for IMU data ...
[lio_sam_imuPreintegration-3] terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
[lio_sam_imuPreintegration-3]   what():  
[lio_sam_imuPreintegration-3] Indeterminant linear system detected while working near variable
[lio_sam_imuPreintegration-3] 8646911284551352328 (Symbol: x8).
[lio_sam_imuPreintegration-3] 
[lio_sam_imuPreintegration-3] Thrown when a linear system is ill-posed.  The most common cause for this
[lio_sam_imuPreintegration-3] error is having underconstrained variables.  Mathematically, the system is
[lio_sam_imuPreintegration-3] underdetermined.  See the GTSAM Doxygen documentation at
[lio_sam_imuPreintegration-3] http://borg.cc.gatech.edu/ on gtsam::IndeterminantLinearSystemException for
[lio_sam_imuPreintegration-3] more information.
[ERROR] [lio_sam_imuPreintegration-3]: process has died [pid 4026664, exit code -6, cmd '/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/install/lio_sam/lib/lio_sam/lio_sam_imuPreintegration --ros-args -r __node:=lio_sam_imuPreintegration --params-file /home/cxx/Fast-LIO2_SC-SLAM/ros_ws/install/lio_sam/share/lio_sam/config/mine.yaml'].
```
Potential solutions: https://github.com/TixiaoShan/LIO-SAM/issues/110


```bash
ros2 launch lio_sam run.launch.py

cd ~/Fast-LIO2_SC-SLAM/ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch fast_lio mapping.launch.py

source install/setup.bash
ros2 launch aloam_velodyne launcher.py

ros2 bag play DCC01.bag --read-ahead-queue-size 90000

ros2 bag play DCC01.bag --rate 0.5 --read-ahead-queue-size 90000 --topics /livox/lidar /livox/imu /gps/fix  


ros2 daemon stop
ros2 daemon start
# then
ros2 node list
ros2 topic list
```
