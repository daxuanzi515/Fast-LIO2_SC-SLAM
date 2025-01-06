## Related Links

### EXP PART

FAST_LIO_SLAM:

github: (source codes)

ROS1: https://github.com/gisbi-kim/FAST_LIO_SLAM
ROS2: https://github.com/rohrschacht/FAST_LIO_SLAM_ros2

CSDN:（quick start）

https://blog.csdn.net/weixin_45863010/article/details/14260274

知乎：
github: https://github.com/mengkai98/ieskf_slam

FrontEnd

BackEnd
https://zhuanlan.zhihu.com/p/666030162
https://zhuanlan.zhihu.com/p/687676064



### Cite

SC-A-LOAM: (SC-PGO + Fast-Lio2)

https://github.com/gisbi-kim/SC-A-LOAM

Fast-Lio2+SC-A-LOAM:

https://github.com/yanliang-wang/FAST_LIO_LC

FAST-LIO2 + LIO-SAM: 

模块化：https://github.com/engcang/FAST-LIO-SAM

非模块化：https://github.com/kahowang/FAST_LIO_SAM



### Better Loop Detection

FAST-LIO-SAM-QN:

https://github.com/engcang/FAST-LIO-SAM-QN


### 2024.12.30-31 presentation
1.复现fast-lio2 + scan context++ + gtsam位姿图优化
2.mulran数据集转换

### 2024.1.1-1.5 presentation
1.evo评估 时间戳对齐问题
2.MulRan 数据集 / 给定的标准数据集 转换ROS2 bag
3.看看Fast-LIO2的评估数据集

发现的问题：
Fast-Lio2 在长通道和狭长通道里面表现不好，特征点缺失，导致检测不到有效点云；这里在Riverside02里面可以看出来从300帧检测左右场景开始出现狭长通道，这时候由于特征点的缺失然后导致点云数据开始稀缺，计算的位姿开始偏移。

[Brno-Urban-Dataset](https://github.com/Robotics-BUT/Brno-Urban-Dataset)
