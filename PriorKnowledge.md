## Deployment
### Base Environment
20.04 Ubuntu
ROS2 Foxy

### FrontEnd - Fast-LIO2
ROS2: https://github.com/Ericsii/FAST_LIO_ROS2

#### `src/laserMapping.cpp`
1. 订阅的 ROS 话题：

    1.1 /livox/lidar   点云数据：

    话题名称：/livox/lidar
    话题类型：livox_ros_driver2::msg::CustomMsg
    描述：根据雷达类型选择不同的回调函数进行订阅。
    如果雷达类型是 AVIA，则订阅 livox_pcl_cbk 回调，否则订阅 standard_pcl_cbk 回调。
    
    1.2 /livox/imu   IMU 数据：

    话题名称：/livox/imu
    话题类型：sensor_msgs::msg::Imu
    描述：订阅 IMU 数据，用于获取加速度、角速度、线性加速度等。


2. 发布的 ROS 话题：Can use as Input to BackEnd

    2.1 /cloud_registered
    话题类型：sensor_msgs::msg::PointCloud2
    
    描述：发布注册后的点云数据，通常用于将点云数据经过配准或变换后生成的结果。
    
    2.2 /cloud_registered_body
    话题类型：sensor_msgs::msg::PointCloud2
    
    描述：发布与机器人身体坐标系对齐的注册后的点云数据。
    
    2.3 /cloud_registered_lidar
    话题类型：sensor_msgs::msg::PointCloud2
    
    描述：发布与激光雷达坐标系对齐的注册后的点云数据。

    2.4 /cloud_effected
    话题类型：sensor_msgs::msg::PointCloud2
    
    描述：发布经过效应处理的点云数据，如滤波、去噪、特征提取等。
    
    2.5 /Laser_map：
    话题类型：sensor_msgs::msg::PointCloud2
    
    描述：发布激光雷达生成的地图，通常是通过地图构建算法（如 SLAM）生成的点云地图。
    
    2.6 /Odometry：
    话题类型：nav_msgs::msg::Odometry
    
    描述：发布机器人的里程计数据，通常包括机器人的位置、速度和姿态（四元数），用于定位和导航。
    
    2.7 /path：
    话题类型：nav_msgs::msg::Path
    
    描述：发布机器人的路径信息，记录机器人的位姿轨迹。





### BackEnd - ScanContext++ & A-LOAM  `SC-PGO`

#### `src/kittiHelper`:
Collect image, lidar and groud-truth Data from KITT Datasets and then publish them as ROS messages and store them in ROS bags.

从KITTI数据集读取图像、激光雷达（LIDAR）点云和地面真实轨迹数据，并将它们发布为ROS消息，同时将这些数据存储到ROS bag文件中, 主要功能包括图像和点云的读取、转换和发布，地面真实轨迹的处理，以及数据存储.

1. 读取配置参数
代码首先通过ROS参数服务器获取一些必要的配置参数：
dataset_folder：数据集所在的文件夹路径。
sequence_number：数据序列的编号。
to_bag：是否将数据保存为ROS bag文件。
output_bag_file：保存的bag文件名。
publish_delay：发布数据时的延迟。

2. 初始化ROS节点和发布者
    
    初始化ROS节点 Kitti_Helper:
    
    创建多个ROS话题发布者：
    /velodyne_points：发布LIDAR点云数据。
    /image_left：发布左侧图像。
    /image_right：发布右侧图像。
    /odometry_gt：发布地面真实轨迹数据。
    /path_gt：发布地面真实轨迹路径。

3. 读取时间戳文件和地面真实轨迹
读取一个时间戳文件 (times.txt)，该文件中每一行包含一个时间戳，指示一帧数据的时间。
读取地面真实轨迹文件 (sequence_number.txt)，该文件每一行包含机器人在该时间戳下的位姿（包括旋转矩阵和平移向量）。

4. 处理每一帧数据
程序逐帧读取数据并进行如下处理：
图像数据：根据时间戳构造左图和右图的文件路径，读取并发布这两张图像。
地面真实轨迹：根据每一帧的位姿信息，计算机器人的四元数旋转和位移，并发布为/odometry_gt和/path_gt消息。
LIDAR数据：根据时间戳构造LIDAR数据的文件路径，读取LIDAR点云数据并将其转换为ROS支持的PointCloud2消息，发布到/velodyne_points。

5. 数据转换和处理
旋转变换：地面真实轨迹中的旋转矩阵会经过一个转换（R_transform）来适应ROS坐标系。
LIDAR点云：LIDAR数据以二进制格式存储，每个点包含x, y, z坐标和强度值，程序将其转换为pcl::PointCloud <pcl::PointXYZI> 类型，并发布为ROS消息。

6. 保存到ROS bag文件
如果to_bag参数设置为true，所有的数据（图像、点云、路径、里程计）会被写入一个ROS bag文件中，方便后续的离线分析和处理。

7. 循环处理和发布数据
程序通过ROS的循环速率（ros::Rate）控制发布频率，并通过时间戳同步图像、点云和位姿数据。

8. 结束时关闭ROS bag文件
当处理完所有帧数据后，程序会关闭ROS bag文件并输出"Done"

#### `src/scanRegistration.cpp`
1. 订阅了1话题

    /velodyne_points

    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：这是来自激光雷达传感器的原始点云数据，通常是由激光雷达扫描获取的环境中每个激光束反射点的空间坐标。该数据是3D空间中的点云数据，可以包含大量的空间信息。
    
    处理：laserCloudHandler 处理该数据，通常是对激光雷达的原始数据进行进一步的处理（例如，去噪、分割、转换等）。

2. 发布了6话题
    
    2.1 /velodyne_cloud_2
    
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：发布经过处理后的激光雷达点云数据，通常是一些原始点云数据进行降采样、过滤或者转换后的结果。
    
    用途：可以用于后续的点云处理、地图生成或显示等。

    2.2 /laser_cloud_sharp
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：发布锐角点（Sharp Corner Points），通常这些是通过激光雷达的点云数据计算得出的具有明显变化的角点，用于特征提取、匹配等。
    
    用途：这些点在激光雷达的数据处理中是有重要作用的，通常用于定位、建图等应用中。

    2.3 /laser_cloud_less_sharp
    
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：发布不那么锐角的点（Less Sharp Corner Points），这些点与上述锐角点不同，通常是更平缓的区域。
    
    用途：这些点可以用于场景中更平坦区域的特征提取。

    2.4 /laser_cloud_flat
    
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：发布平坦区域的点（Flat Surface Points），这些是通过计算点云数据中平坦的区域得到的，通常这些点在定位和地图构建中起着重要的作用。
    
    用途：平坦点云可以用于构建地图、路径规划等。

    2.5 /laser_cloud_less_flat
    
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：发布不那么平坦的点（Less Flat Surface Points），这些点通常来自点云中不太平坦的区域。
    
    用途：和上面类似，但它们代表了点云中更不平的区域。

    2.6 /laser_remove_points
    
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：发布被移除的点（Removed Points）。这可能是那些经过预处理、去噪或者过滤后的不需要的点。
    
    用途：可以用来进一步清理点云数据，去除无效点。

    2.7（可选） /laser_scanid_{i}（i 为扫描线编号）
    
    数据类型：sensor_msgs::msg::PointCloud2
    
    描述：如果设置了 PUB_EACH_LINE 为真，则发布每一条扫描线的点云数据。激光雷达通常会生成多个扫描线的数据，每条扫描线的点云数据会被分别发布。
    
    用途：可以单独查看每条扫描线的数据，通常用于调试或更加细致的点云分析.

#### `src/laseOdometry.cpp`
1. 订阅的 ROS 话题：

    1.1 /laser_cloud_sharp：订阅锐利的角点（corner points）点云数据。

    功能：用于获取激光点云中的锐利角点，作为特征进行优化。

    1.2 /laser_cloud_less_sharp：订阅非锐利的角点数据。

    功能：获取激光点云中的非锐利角点，作为特征进行优化。
    
    1.3 /laser_cloud_flat：订阅平坦面特征点数据。

    功能：用于获取激光点云中的平坦面特征点。
    
    1.4 /laser_cloud_less_flat：订阅非平坦面的特征点数据。

    功能：获取激光点云中的非平坦面特征点，作为特征进行优化。
    
    1.5 /velodyne_cloud_2：订阅全分辨率的激光点云数据。

    功能：提供完整的激光点云数据，供后续的优化过程使用。


2. 发布的 ROS 话题：

    2.1 /laser_cloud_corner_last：发布最后一帧的角点特征点云。

    功能：将最后一帧的锐利角点数据发布，用于其他系统进行进一步处理。
    
    2.2 /laser_cloud_surf_last：发布最后一帧的平面特征点云。

    功能：将最后一帧的平面特征点数据发布，用于其他系统进行进一步处理。

    2.3 /velodyne_cloud_3：发布处理过的全分辨率激光点云数据。

    功能：发布优化后的激光点云数据，供其他系统使用。
    
    2.4 /laser_odom_to_init：发布激光里程计计算的位姿（Odometry）信息。

    功能：发布机器人当前位置和姿态（包括位置和四元数旋转）信息，用于导航或跟踪。
    
    2.5 /laser_odom_path：发布激光里程计计算的路径信息（路径序列）。

    功能：发布机器人经过的路径，用于可视化和分析。

3. 主要功能：

    3.1 特征提取：从激光点云数据中提取角点（corner points）和平面（surface points）特征。

    3.2 数据同步与优化：使用Ceres优化库进行数据的优化处理，通过最小化误差（如点云匹配误差）来优化机器人的姿态。

    3.3 发布里程计与路径：通过计算当前机器人的位姿（位置和姿态），发布里程计信息，同时发布机器人的路径信息，用于路径跟踪和可视化。

    3.4 点云转换：将特征点从当前帧转换到参考坐标系，以便进行进一步的匹配和优化。

    3.5 点云和里程计数据的频率控制：通过skipFrameNum控制数据处理的频率和发布的频率。


#### `src/laserMapping.cpp`

1. 订阅的内容：代码订阅了四个话题，用于接收来自激光雷达和里程计的数据

    1.1 激光雷达角点云数据：
    话题：/laser_cloud_corner_last
    类型：sensor_msgs::msg::PointCloud2
    订阅目的：接收来自激光雷达的角点数据，这些数据表示的是激光雷达扫描中提取出来的特征点，通常用于识别环境中的边缘或者特征区域。

    1.2 激光雷达平面点云数据：
    话题：/laser_cloud_surf_last
    类型：sensor_msgs::msg::PointCloud2
    订阅目的：接收来自激光雷达的平面点云数据，这些数据通常代表地面或者其他平坦的表面，用于地图构建和环境建模。
    
    1.3 激光雷达里程计数据：
    话题：/laser_odom_to_init
    类型：nav_msgs::msg::Odometry
    订阅目的：接收激光雷达里程计（或机器人定位）的数据，包含机器人当前的位置和姿态。这个数据用于定位和地图优化。

    1.4 全分辨率原始点云数据：
    话题：/velodyne_cloud_3
    类型：sensor_msgs::msg::PointCloud2
    订阅目的：接收来自激光雷达的原始点云数据，通常这些数据不经过滤波和下采样，包含了所有扫描到的点。

2. 发布的内容：代码发布了六个话题，用于将处理后的数据和状态信息提供给其他系统

    2.1 **周围环境点云数据**：
    话题：/laser_cloud_surround
    类型：sensor_msgs::msg::PointCloud2
    发布目的：发布机器人周围环境的点云数据，用于其他模块分析和使用。

    2.2 **生成的地图点云数据**：
    话题：/laser_cloud_map
    类型：sensor_msgs::msg::PointCloud2
    发布目的：发布地图生成后的点云数据，通常是经过多次扫描融合和优化后的环境地图数据。

    2.3 **全分辨率点云数据**：
    话题：/velodyne_cloud_registered
    类型：sensor_msgs::msg::PointCloud2
    发布目的：发布与激光雷达相关的注册后的全分辨率点云数据，用于后续的地图构建和可视化。

    2.4 **局部区域点云数据**：
    话题：/velodyne_cloud_registered_local
    类型：sensor_msgs::msg::PointCloud2
    发布目的：发布局部区域的点云数据，通常是激光雷达扫描时当前扫描到的区域，以供更精细的分析或后续处理。

    2.5 **调整后的里程计数据**：
    话题：/aft_mapped_to_init
    类型：nav_msgs::msg::Odometry
    发布目的：发布经过地图优化后的里程计数据，这个数据通常比原始的里程计数据更加精确，反映了更高质量的位置和姿态信息。

    2.6 **高频率调整后的里程计数据**：
    话题：/aft_mapped_to_init_high_frec
    类型：nav_msgs::msg::Odometry
    发布目的：发布经过高频率优化后的里程计数据，通常用于精细的定位和控制，具有更高的时间分辨率。

    2.7 调整后的路径信息：
    话题：/aft_mapped_path
    类型：nav_msgs::msg::Path
    发布目的：发布调整后的路径数据，用于表示机器人从起点到终点的路径轨迹，通常用于后续的路径跟踪或者可视化展示。

3. 主要功能：
    
    3.1 数据订阅与处理：
    从多个话题订阅激光雷达的点云数据（角点和面点）以及里程计数据。激光雷达数据用于环境建模，里程计数据用于定位。对接收到的点云数据进行处理，通常包括下采样、滤波等操作，以便后续的地图生成和优化。

    3.2 数据发布：
    将处理后的点云数据和优化后的里程计数据发布到对应话题，以供其他模块或系统使用。

    3.3 地图构建和定位优化：
    根据激光雷达数据构建地图，同时优化定位，减少传感器误差对定位的影响。

    3.4 并行处理：
    启动一个独立线程来处理映射过程 (mapping_process)，实现数据的异步处理，确保系统高效运行。

#### **`src/laserPosegraphOptimization.cpp`**
If it does not add Fast-LIO2, it subscribes the point cloud topic and odometry topic from `src/laserMapping.cpp`.
In this case, it remaps the subscribed topics from Fast-LIO2:
```yaml
<remap from="/aft_mapped_to_init" to="/Odometry"/>
<remap from="/velodyne_cloud_registered_local" to="/cloud_registered_body"/>
<remap from="/cloud_for_scancontext" to="/cloud_registered_lidar"/> 
```

1. 订阅的三个话题：
    
    1.1 激光雷达点云数据：
    话题：/velodyne_cloud_registered_local
    类型：sensor_msgs::msg::PointCloud2
    功能：接收激光雷达的点云数据，用于地图构建、定位、回环检测等。
    
    1.2 里程计数据：
    话题：/aft_mapped_to_init
    类型：nav_msgs::msg::Odometry
    功能：接收机器人当前的位姿信息（位置和姿态），通常是经过优化的里程计数据。
    

    1.3 GPS数据：
    话题：/gps/fix
    类型：sensor_msgs::msg::NavSatFix
    功能：接收来自GPS的定位信息，通常用于全局定位和漂移校正。
    
    1.4 处理过程：
    基于这三个订阅的话题数据，节点会进行如下计算和处理：
    - iSAM2 优化：使用 iSAM2 算法进行增量式位姿优化。
    - 回环检测：检测机器人是否回到已知的位置，通过匹配点云来识别回环并更新位姿图。
    - 地图更新：基于优化后的位姿数据和点云信息，不断更新和优化地图。
    - 路径跟踪：计算并优化机器人的运动路径。

2. 发布的六个话题：

    2.1 优化后的里程计数据：
    话题：/aft_pgo_odom
    类型：nav_msgs::msg::Odometry
    订阅的话题：/aft_mapped_to_init（nav_msgs::msg::Odometry）
    订阅数据：优化后的里程计数据（位置、姿态等）
    处理过程：里程计数据通过 Pose Graph Optimization (PGO) 进行优化。PGO 通过增加回环约束来纠正机器人轨迹中的偏差。

    2.2 重新发布的里程计数据：
    话题：/repub_odom
    类型：nav_msgs::msg::Odometry
    订阅的话题：/aft_mapped_to_init（nav_msgs::msg::Odometry）
    订阅数据：原始的里程计数据
    处理过程：这里的发布是原始的未经优化的里程计数据，通常是从传感器或其他模块直接获取的数据。这些数据是未经 PG（Pose Graph）优化处理的，可能存在一定误差或漂移。

    2.3 优化后的路径信息：
    话题：/aft_pgo_path
    类型：nav_msgs::msg::Path
    订阅的话题：/aft_mapped_to_init（nav_msgs::msg::Odometry）
    订阅数据：优化后的里程计数据，提供机器人的位置和姿态随时间变化的轨迹。
    间接关联：/velodyne_cloud_registered_local（sensor_msgs::msg::PointCloud2）中的点云数据，用于更新地图和计算回环。
    处理过程：基于优化后的里程计数据，系统可以构建出机器人的路径。这些路径数据反映了机器人从起始位置到当前位置的运动轨迹。

    2.4 优化后的地图数据：
    话题：/aft_pgo_map
    类型：sensor_msgs::msg::PointCloud2
    订阅的话题：/velodyne_cloud_registered_local（sensor_msgs::msg::PointCloud2）
    订阅数据：经过注册的激光雷达点云数据。
    间接关联：/aft_mapped_to_init（nav_msgs::msg::Odometry）优化后的位姿数据，用于提高地图精度。
    处理过程：地图数据通过优化后的位姿信息（/aft_mapped_to_init）与激光雷达点云数据（/velodyne_cloud_registered_local）相结合，不断更新和优化地图。随着机器人的移动，点云数据不断被添加进地图中。使用回环检测和优化算法来减少地图中因累积误差产生的漂移，确保生成的地图更精确。

    2.5 回环扫描数据：
    话题：/loop_scan_local
    类型：sensor_msgs::msg::PointCloud2
    订阅的话题：/velodyne_cloud_registered_local（sensor_msgs::msg::PointCloud2）
    订阅数据：局部的激光雷达点云数据，用于回环检测。
    间接关联：/aft_mapped_to_init（nav_msgs::msg::Odometry）提供的优化后位姿数据，用于回环约束的计算。
    处理过程：在回环检测过程中，局部扫描数据（/velodyne_cloud_registered_local）将被用于与历史数据进行匹配，识别回环发生的时刻。如果系统检测到回环，它会将回环约束添加到位姿图中，并使用该数据来优化全局地图和位姿估计。

    2.6 回环子地图数据：
    话题：/loop_submap_local
    类型：sensor_msgs::msg::PointCloud2
    订阅的话题：/velodyne_cloud_registered_local（sensor_msgs::msg::PointCloud2）
    订阅数据：局部的激光雷达点云数据。
    间接关联：/aft_mapped_to_init（nav_msgs::msg::Odometry）优化后的位姿数据，用于生成子地图。
    处理过程：在回环检测过程中，局部点云数据会被用来创建回环子地图。这些子地图是回环检测过程中局部区域的地图表示，用于优化和修正机器人在该区域的位姿。这些子地图将被发布，以便后续处理模块（如地图修正、路径规划）使用。

### Conclusion

From BackEnd, it needs subscribe these topics from Fast-LIO2:
1. 激光雷达点云数据	/velodyne_cloud_registered_local => /cloud_registered_lidar 

2. 里程计数据	/aft_mapped_to_init =>	/Odometry

3. GPS数据	/gps/fix

4. Scancontext /cloud_for_scancontext => cloud_registered_lidar

### Next Step
1. 跑通这个项目，得到optimized_poses.txt和地图数据，保存
2. evo工具使用, 用于评估跑出来的optimized_poses.txt的质量，需要一个groud-truth.txt
3. 关于launch/fastlio_ouster64.launch的修改，先跑通之后再做...
    
    ```html
    <!-- input from FASTLIO2 -->
    <remap from="/aft_mapped_to_init" to="/Odometry"/>
    <remap from="/velodyne_cloud_registered_local" to="/cloud_registered_body"/>
    <remap from="/cloud_for_scancontext" to="/cloud_registered_lidar"/>   <!-- because ScanContext requires lidar-ego-centric coordinate for the better performance -->
    ```

师姐的建议：
再看一下evo评估的estimate数据的保存 以及
要不要把imu坐标系下的点云换成lidar的试一下


## Draw 3D image or Point Cloud
Details in `src/alom_velodyne/utils/`
- `draw_3d_image.py`: save a combined 3D image of the point cloud and the optimized poses.
- `makeMergedMap.py`: use the optimized poses to generate a merged map.
- `myViewer.py`: a simple 3D viewer for the merged map and the optimized poses.
- `jet_table.npy`: a numpy array of the color map for the merged map.
- `sample_data`: a short clip of the merged map and the optimized poses in KAIST03.


