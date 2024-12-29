import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 预定义路径
    package_path = get_package_share_directory('aloam_velodyne')
    default_rviz_config_path = os.path.join(package_path, 'rviz_cfg', 'aloam_velodyne.rviz')

    # ALOAM Velodyne 节点
    aloam_velodyne_node = Node(
        package='aloam_velodyne',
        executable='alaserPGO',
        name='laserPosegraphOptimization',
        output='screen',
        remappings=[
            ('/aft_mapped_to_init', '/Odometry'),
            ('/velodyne_cloud_registered_local', '/cloud_registered_body'),
            ('/cloud_for_scancontext', '/cloud_registered_lidar')
        ]
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', default_rviz_config_path],
    )

    # 创建 LaunchDescription 对象并添加动作
    ld = LaunchDescription()
    ld.add_action(aloam_velodyne_node)
    ld.add_action(rviz_node)

    return ld