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


### 2024.12.30-12.31 presentation
1.复现fast-lio2 + scan context++ + gtsam位姿图优化
2.mulran数据集转换

### 2025.1.1-1.5 presentation
1.evo评估 时间戳对齐问题
2.MulRan 数据集 / 给定的标准数据集 转换ROS2 bag
3.看看Fast-LIO2的评估数据集

发现的问题：
Fast-Lio2 在长通道和狭长通道里面表现不好，特征点缺失，导致检测不到有效点云；这里在Riverside02里面可以看出来从300帧检测左右场景开始出现狭长通道，这时候由于特征点的缺失然后导致点云数据开始稀缺，计算的位姿开始偏移。

[Brno-Urban-Dataset](https://github.com/Robotics-BUT/Brno-Urban-Dataset)


### 2025.1.10-1.13 final report and presentation
- slides：
1.SC-PGO后端用了什么方法；
2.这个后端的渲染效果图；
mulran数据集上的结果，与fast-lio2的对比；
为什么没跟lio-sam对比 (如果lio-sam效果太差)。
（可选凑数）插值法的效果例子
总时长：5分钟。

- report：
EXPERIMENTS 2页：
1.fast-lio2直接跑hku的无ground truth数据集的建图+轨迹。
2.sc-a-loam的结果，跟slides内容对齐。evo结果可以图片、表格都放。插值法例子数据也可以放。
Related Work:
预计3/4页，内容取自之前的综述slides。


### 2025.1.13-1.17 report
加一个花费时间的统计，后端的部分进行计时；
lio-sam的数据时间窗口读取，这部分代码需要修改！因为lio-sam并不是实时的；
当播放包的信息过多，它会因为几个节点之间消息通讯不及时，导致计算IMU积分卡顿，后续的数据无法输出。

助教的建议一：是把数据读取的单位减小或者把数据包播放速度减慢
助教的建议二：修改Lio-sam的源代码进行数据读取优化

加跟时间有关的评估，比如跑完一整个包，后端实际耗时：
普通数据包的耗时；fast-lio2前端实时处理的耗时；sc-pgo*的后端耗时；sc-c和sc-m的后端耗时。