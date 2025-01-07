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
colcon build --packages-select lio_sam --symlink-instal
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
declare_parameter("savePCDDirectory", "/home/cxx/Fast-LIO2_SC-SLAM/output/LOAM");
```
lidarTopic: `points` TYPE: `sensor_msgs::msg::PointCloud2`
imuTopic: `imu/data` TYPE: `sensor_msgs::msg::Imu`
gpsTopic: `lio_sam/odometry/gps` TYPE: `nav_msgs::msg::Odometry`
