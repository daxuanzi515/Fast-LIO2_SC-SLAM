import numpy as np
from pyquaternion import Quaternion
import re

def read_transformations_file(filename):
    points = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        data_section = False
        for line in lines:
            if line.strip() == "DATA ascii":
                data_section = True
                continue
            if data_section:
                values = re.split(r'\s+', line.strip())
                if len(values) == 8:
                    x, y, z, _, roll, pitch, yaw, _ = map(float, values)
                    points.append((x, y, z, roll, pitch, yaw))
    return points

def read_new_timestamps_file(filename):
    timestamps = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            values = re.split(r'\s+', line.strip())
            if len(values) == 8:
                timestamp = float(values[0])
                timestamps.append(timestamp)
    return timestamps

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(qw, qx, qy, qz)

def save_as_tum_format(points, timestamps, output_filename):
    with open(output_filename, 'w') as tum_file:
        for i, (x, y, z, roll, pitch, yaw) in enumerate(points):
            timestamp = timestamps[i]
            q = euler_to_quaternion(roll, pitch, yaw)
            tum_file.write(f"{timestamp} {x} {y} {z} {q.x} {q.y} {q.z} {q.w}\n")

if __name__ == "__main__":
    transformations_filename = "/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/lio-sam/DCC03/transformations.txt"
    new_timestamps_filename = "/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/lio-sam/DCC03/DCC03.txt"
    tum_filename = "/home/cxx/Fast-LIO2_SC-SLAM/refs/MulRan_dataset/mytest/lio-sam/DCC03/trajectory_tum.txt"

    points = read_transformations_file(transformations_filename)
    timestamps = read_new_timestamps_file(new_timestamps_filename)
    save_as_tum_format(points, timestamps, tum_filename)
    print(f"Trajectory saved to {tum_filename}")