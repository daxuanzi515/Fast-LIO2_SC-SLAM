import open3d as o3d

# 加载点云
file_path = "/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/Riverside02.pcd"
pcd = o3d.io.read_point_cloud(file_path)

# 检查点云是否加载成功
if pcd.is_empty():
    print("Failed to load point cloud!")
    exit()

# 使用体素采样减少点云大小
voxel_size = 0.05  # 设置体素大小，越大点数越少
pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)

print(f"Original points: {len(pcd.points)}, Downsampled points: {len(pcd_downsampled.points)}")

# 可视化采样后的点云
o3d.visualization.draw_geometries([pcd_downsampled])
