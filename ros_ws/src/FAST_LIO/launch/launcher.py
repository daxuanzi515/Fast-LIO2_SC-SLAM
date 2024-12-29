import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to launch RViz'),

        # Fast-LIO Mapping node
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                # You can directly define parameters here or use a YAML file for larger sets
                {'feature_extract_enable': False},
                {'point_filter_num': 3},
                {'max_iteration': 3},
                {'scan_publish_enable': True},
                {'dense_publish_enable': True},
                {'filter_size_surf': 0.5},
                {'filter_size_map': 0.5},
                {'cube_side_length': 1000},
                {'runtime_pos_log_enable': False},
                {'pcd_save_enable': False},
                # Add path to the configuration file here
                PathJoinSubstitution([get_package_share_directory('fast_lio'), 'config', 'ouster64.yaml'])
            ]
        ),

        # RViz visualization (optional, only if rviz argument is true)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([get_package_share_directory('fast_lio'), 'rviz_cfg', 'loam_livox.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])
