# Output Instructions
- odom_poses.txt: From Fast-Lio2 /Odometry topic and not handle it anymore. (original data)
- optimized_poses.txt: Use /Odometry topic message and apply gstam optimized algorithm to get a precious trajectory.
- Scans/**.pcd: In some frames(0-200), a series of pcd data from LiDAR. They can be used for creating a 3D map for SLAM. 

Optimized poses and pcds can be used to create a complete map for Localization and Mapping assignments.

But optimized poses are KITTI format, the same format as the original data without any timestamps. While the ground-truth data has timestamp attribute, our optimized poses are some key frames which are hard to tell a specific timestamp, it is a delimma between the optimized poses and the ground-truth data during evaluation. 

If we align them simply by the start frame, it can not be a effective way to evaluate the performance of the algorithm. And, we already have done the evaluation with evo by using KITTI format ground-truth, the figure shows bad performance.

Specifically, ATE image can be so large for testing data and ground-truth data, while RPE image is common.

How to solve this problem?

-----

The problem is already **Solved**, please check the `refs/MulRan_dataset` for more details.

1.First, convert the complete tum file `trajectory_tum.txt` with `times.txt` and `optimized_poses.txt`.

2.Then, use `refs/MulRan_dataset/mulran_pose/*` as ground-truth data.

3.Finally, follow each markdown file in `refs/MulRan_dataset/mytest/*` to launch evaluation using evo.
