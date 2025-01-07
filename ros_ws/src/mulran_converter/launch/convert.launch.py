import launch
from launch import LaunchDescription
from launch.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 启动 mulran_converter 节点，并动态设置参数
        Node(
            package='mulran_converter',  # 包名
            executable='mulran_converter',  # 可执行文件名
            name='mulran_converter',  # 节点名
            output='screen',  # 控制台输出
            parameters=[{
                'input_folder': LaunchConfiguration('input_folder'),  # 从命令行参数获取 input_folder
                'output_bag_prefix': LaunchConfiguration('output_bag_prefix')  # 从命令行参数获取 output_bag_prefix
            }],
            # 启动时将命令行参数传递给节点
        )
    ])
# ros2 launch mulran_converter convert_to_bag.launch.py \
#     input_folder:="/home/cxx/Fast-LIO2_SC-SLAM/refs/examples/KAIST03_DATA" \
#     output_bag_prefix:="/home/cxx/Fast-LIO2_SC-SLAM/ros_ws/test_hub"
