import numpy as np
from scipy.spatial.transform import Rotation as R

def tum_to_kitti(tum_file, kitti_file):
    """
    Convert TUM format data to KITTI format.
    Args:
        tum_file (str): Path to TUM format file.
        kitti_file (str): Path to save KITTI format file.
    """
    with open(tum_file, 'r') as f1, open(kitti_file, 'w') as f2:
        lines = f1.readlines()
        for line in lines[0: 331]:
            data = list(map(float, line.strip().split()))
            # TUM 格式: timestamp x y z qx qy qz qw
            timestamp, x, y, z, qx, qy, qz, qw = data
            # 将四元数转换为旋转矩阵
            rotation = R.from_quat([qx, qy, qz, qw])
            R_mat = rotation.as_matrix()  # 3x3 矩阵
            # 转换为 KITTI 格式: R11 R12 R13 tx R21 R22 R23 ty R31 R32 R33 tz
            kitti_row = list(R_mat.flatten()) + [x, y, z]
            kitti_row_str = " ".join(map(str, kitti_row))
            f2.write(kitti_row_str + "\n")

ins = f"/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/Riverside02/tum_ground_truth.txt"
out = f"/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/Riverside02/kitti_ground_truth.txt"
tum_to_kitti(ins, out)
