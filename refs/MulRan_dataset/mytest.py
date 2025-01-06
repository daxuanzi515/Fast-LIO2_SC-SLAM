import numpy as np
from scipy.spatial.transform import Rotation as R

def load_kitti_poses(file_path):
    """
    Load KITTI-style poses from a text file.
    """
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            T = np.fromstring(line.strip(), sep=' ').reshape(3, 4)
            R_mat = T[:, :3]  # 3x3 rotation matrix
            t_vec = T[:, 3]   # translation vector
            poses.append((R_mat, t_vec))
    return poses

def load_timestamps(file_path):
    """
    Load timestamps from a text file and convert them to nanoseconds.
    """
    timestamps = []
    with open(file_path, 'r') as f:
        for line in f:
            timestamp_s = float(line.strip())  # Seconds
            # timestamp_ns = int(timestamp_s * 1e9)  # Convert to nanoseconds
            timestamps.append(timestamp_s)
    return timestamps

def save_tum_format(poses, timestamps, output_file):
    """
    Save poses and timestamps in TUM format (timestamps in nanoseconds).
    """
    with open(output_file, 'w') as f:
        for i, (R_mat, t_vec) in enumerate(poses):
            timestamp_ns = timestamps[i]
            # Convert rotation matrix to quaternion
            quat = R.from_matrix(R_mat).as_quat()  # [qx, qy, qz, qw]
            # Write in TUM format: timestamp x y z qx qy qz qw
            f.write(f"{timestamp_ns} {t_vec[0]:.6f} {t_vec[1]:.6f} {t_vec[2]:.6f} {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")

# Paths to input files
times_file = "/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/KAIST02/times.txt"
poses_file = "/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/KAIST02/optimized_poses.txt"
output_file = "/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/KAIST02/trajectory_tum.txt"

# Load data
timestamps = load_timestamps(times_file)
poses = load_kitti_poses(poses_file)

# Ensure timestamps and poses are the same length
assert len(timestamps) == len(poses), "Timestamps and poses count mismatch!"

# Save in TUM format
save_tum_format(poses, timestamps, output_file)
print(f"Converted to TUM format and saved to {output_file}")
