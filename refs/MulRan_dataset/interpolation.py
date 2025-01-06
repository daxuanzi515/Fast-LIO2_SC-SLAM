import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
def slerp(t, q1, q2):
    """
    实现四元数的球面线性插值 (SLERP)
    :param t: 插值参数 (0 <= t <= 1)
    :param q1: 起始四元数
    :param q2: 目标四元数
    :return: 插值后的四元数
    """
    q1 = np.array(q1)
    q2 = np.array(q2)

    # 计算内积（用于确定插值方向）
    dot = np.dot(q1, q2)

    # 如果内积为负，翻转其中一个四元数，保证最短路径
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    # 如果内积接近1，则使用线性插值（避免浮点数问题）
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    # 球面线性插值公式
    theta_0 = np.arccos(dot)  # 初始角度
    theta = theta_0 * t       # 插值角度

    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)

    s1 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s2 = sin_theta / sin_theta_0

    return s1 * q1 + s2 * q2
    
    
def interpolate_to_gt(gt_file, output_file, interpolated_file):
    # 读取 Ground Truth 数据
    gt_data = []
    with open(gt_file, 'r') as f:
        for line in f:
            row = line.strip().split()
            timestamp = float(row[0])
            pose = list(map(float, row[1:]))
            gt_data.append((timestamp, pose))
    # print(f"Read {len(gt_data)} ground truth poses")
    # 读取后端输出数据
    output_data = []
    with open(output_file, 'r') as f:
        for line in f:
            row = line.strip().split()
            timestamp = float(row[0])
            pose = list(map(float, row[1:]))
            output_data.append((timestamp, pose))
    # print(f"Read {len(output_data)} optimized poses")
    # 提取时间戳
    gt_timestamps = np.array([item[0] for item in gt_data])
    # gt_poses = np.array([item[1] for item in gt_data])
    
    output_timestamps = np.array([item[0] for item in output_data])
    output_translations = np.array([item[1][:3] for item in output_data])  # 提取平移 (x, y, z)
    output_quaternions = np.array([item[1][3:] for item in output_data])  # 提取四元数 (qx, qy, qz, qw)

    #print(output_translations)
    

    # 初始化插值结果
    interpolated_results = []

    for gt_time in gt_timestamps:
        # 找到输出数据中时间戳的上下界
        if gt_time < output_timestamps[0] or gt_time > output_timestamps[-1]:
            continue  # 跳过无法插值的时间戳

        idx = np.searchsorted(output_timestamps, gt_time)
        t1, t2 = output_timestamps[idx - 1], output_timestamps[idx]
        trans1, trans2 = output_translations[idx - 1], output_translations[idx]
        quat1, quat2 = output_quaternions[idx - 1], output_quaternions[idx]

        # 计算插值比例
        t = (gt_time - t1) / (t2 - t1)
        # print(f"Interpolating between {t1:.3f} and {t2:.3f} with t={t:.3f}")

        # 平移线性插值
        interp_translation = trans1 * (1 - t) + trans2 * t

        # 四元数 SLERP 插值
        interp_quaternion = slerp(t, quat1, quat2)
        #print(interp_translation.tolist())
        # 合并结果
        interpolated_results.append([gt_time] + interp_translation.tolist() + interp_quaternion.tolist())
        


    # 保存插值结果
    with open(interpolated_file, 'w') as f:
        for result in interpolated_results:
            #print(result)
            f.write(" ".join(map(str, result)) + "\n")
    print("Interpolation done!")
            
    

# 示例用法
# gt_file = 'converted_pose.txt'  # Ground Truth 文件路径
# output_file = 'optimized_trajectory.txt'  # 后端输出文件路径
# interpolated_file = 'interpolated_output.txt'  # 补齐后输出文件路径


gt_file = '/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/KAIST02/KAIST02.txt'  # Ground Truth 文件路径
output_file = '/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/KAIST02/trajectory_tum.txt'  # 后端输出文件路径
interpolated_file = '/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/KAIST02/interpolated_poses.txt'  # 补齐后输出文件路径
interpolate_to_gt(gt_file, output_file, interpolated_file)
