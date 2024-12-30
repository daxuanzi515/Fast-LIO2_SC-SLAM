import os
import copy
import numpy as np
from numpy import linalg as LA
import open3d as o3d

# 加载颜色表
jet_table = np.load('/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/src/aloam_velodyne/utils/python/jet_table.npy')

# 参数设置
data_dir = "/home/cxx/Fast-LIO2_SC-SLAM/output/new_Riverside02_bag_281"  # 数据目录
scan_idx_range_to_stack = [0, 331]  # 叠加范围
node_skip = 1  # 跳过帧数
intensity_color_max = 100  # 最大强度值
is_near_removal = True
thres_near_removal = 2  # 移除近距离点阈值（米）

# 加载点云文件
scan_dir = os.path.join(data_dir, "Scans")
scan_files = sorted(os.listdir(scan_dir))

# 加载位姿文件
poses = []
with open(os.path.join(data_dir, "optimized_poses.txt"), 'r') as f:
    for line in f:
        pose_SE3 = np.asarray([float(i) for i in line.split()])
        pose_SE3 = np.vstack((np.reshape(pose_SE3, (3, 4)), np.asarray([0, 0, 0, 1])))
        poses.append(pose_SE3)

# 初始化可视化与点云存储
pcd_combined = o3d.geometry.PointCloud()  # 合并后的点云
path_points = []  # 路径点存储

for node_idx, scan_file in enumerate(scan_files):
    if node_idx < scan_idx_range_to_stack[0] or node_idx >= scan_idx_range_to_stack[1]:
        continue

    if node_idx % node_skip != 0:
        continue

    print(f"Processing frame {node_idx}")

    # 加载点云数据
    scan_path = os.path.join(scan_dir, scan_file)
    scan_pcd = o3d.io.read_point_cloud(scan_path)
    scan_xyz_local = np.asarray(scan_pcd.points)

    # 加载位姿并转换点云到全局坐标系
    scan_pose = poses[node_idx]
    scan_pcd.transform(scan_pose)
    scan_xyz_global = np.asarray(scan_pcd.points)

    # 移除近距离点
    if is_near_removal:
        scan_ranges = LA.norm(scan_xyz_local, axis=1)
        eff_idxes = np.where(scan_ranges > thres_near_removal)[0]
        scan_pcd = scan_pcd.select_by_index(eff_idxes)

    # 更新合并点云
    pcd_combined += scan_pcd

    # 保存路径点
    path_points.append(scan_pose[:3, 3])  # 提取位姿中的平移部分

# 添加路径
path_line_set = o3d.geometry.LineSet()
path_line_set.points = o3d.utility.Vector3dVector(np.array(path_points))
path_indices = [[i, i + 1] for i in range(len(path_points) - 1)]
path_line_set.lines = o3d.utility.Vector2iVector(np.array(path_indices))
path_line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(path_indices))])  # 红色路径

# 可视化结果
o3d.visualization.draw_geometries([pcd_combined, path_line_set])

# # 保存点云和路径
# map_path = os.path.join(data_dir, f"map_{scan_idx_range_to_stack[0]}_{scan_idx_range_to_stack[1]}.pcd")
# o3d.io.write_point_cloud(map_path, pcd_combined)
# print(f"地图已保存到 {map_path}")

# path_path = os.path.join(data_dir, f"path_{scan_idx_range_to_stack[0]}_{scan_idx_range_to_stack[1]}.ply")
# o3d.io.write_line_set(path_path, path_line_set)
# print(f"路径已保存到 {path_path}")
