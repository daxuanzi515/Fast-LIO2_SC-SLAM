import sys
import open3d as o3d

# 加载点云数据
pcd = o3d.io.read_point_cloud("/home/cxx/Fast-LIO2_SC-SLAM/output/map_0_to_240_with_intensity.pcd")

# 创建一个可视化窗口
vis = o3d.visualization.Visualizer()

# 初始化窗口
vis.create_window()

# 将点云添加到窗口
vis.add_geometry(pcd)

opt = vis.get_render_option()
opt.point_size = 3  # 设置点的大小
opt.background_color = [0.0, 0.0, 0.0]  # 黑色背景

# 设置相机视角 (可选)
view_control = vis.get_view_control()
view_control.set_front([0.0, 0.0, -1.0])
view_control.set_lookat([0.0, 0.0, 0.0])
view_control.set_up([0.0, -1.0, 0.0])
view_control.set_zoom(0.8)

# 渲染并保存截图
vis.poll_events()
vis.update_renderer()

# 提高截图的分辨率
vis.capture_screen_image("/home/cxx/Fast-LIO2_SC-SLAM/output/map_0_to_240_with_intensity.png")
sys.sleep(10)
# 关闭窗口
vis.destroy_window()
