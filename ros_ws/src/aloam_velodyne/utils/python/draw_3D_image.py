import sys
import numpy as np
import open3d as o3d

# 加载点云数据
pcd = o3d.io.read_point_cloud("/home/cxx/Fast-LIO2_SC-SLAM/output/map_0_to_453_with_intensity.pcd")
path_file = "/home/cxx/Fast-LIO2_SC-SLAM/output/optimized_poses.txt"

poses = []
f = open(path_file, 'r')
while True:
    line = f.readline()
    if not line:
        break
    pose_SE3 = np.asarray([float(i) for i in line.split()])
    pose_SE3 = np.vstack((np.reshape(pose_SE3, (3, 4)), np.asarray([0, 0, 0, 1])))
    poses.append(pose_SE3)
f.close()


path_points = np.array([pose[:3, 3] for pose in poses])  # 提取平移部分 tx, ty, tz
path_line_set = o3d.geometry.LineSet()
path_line_set.points = o3d.utility.Vector3dVector(path_points)
path_indices = [[i, i + 1] for i in range(len(path_points) - 1)]
path_line_set.lines = o3d.utility.Vector2iVector(path_indices)
path_line_set.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0] for _ in path_indices])


# 创建一个可视化窗口
vis = o3d.visualization.Visualizer()

# 初始化窗口
vis.create_window()

# 将点云添加到窗口
vis.add_geometry(pcd)
# add path
vis.add_geometry(path_line_set)

opt = vis.get_render_option()
opt.point_size = 3  # 设置点的大小
opt.background_color = [0.0, 0.0, 0.0]  # 黑色背景

# 设置相机视角 (可选)
view_control = vis.get_view_control()
view_control.set_zoom(0.3)
view_control.set_front([0.0, 0.0, 0.0])
view_control.set_lookat([0.0, 0.0, 0.0])
view_control.set_up([0.0, 0.0, 0.0])
view_control.set_zoom(0.8)

# 渲染并保存截图
vis.poll_events()
vis.update_renderer()
vis.run()

# optional
# output_combined_file = "/home/cxx/Fast-LIO2_SC-SLAM/output/combined_map_with_path.ply"
# path_points_pcd = o3d.geometry.PointCloud()
# path_points_pcd.points = o3d.utility.Vector3dVector(path_points)
# path_points_pcd.paint_uniform_color([1.0, 0.0, 0.0]) 
# combined_pcd = pcd + path_points_pcd

# # 保存合并后的点云为最终文件
# o3d.io.write_point_cloud(output_combined_file, combined_pcd)
# print(f"点云与路径合并文件已保存到: {output_combined_file}")


# combined_pcd = o3d.io.read_point_cloud(output_combined_file)
# o3d.visualization.draw_geometries([combined_pcd])


# 提高截图的分辨率
# vis.capture_screen_image("/home/cxx/Fast-LIO2_SC-SLAM/output/map_0_to_240_with_intensity.png")
# sys.sleep(10)
# 关闭窗口
# vis.destroy_window()


