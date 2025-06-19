import numpy as np
import os
from scipy.spatial.transform import Rotation as R

def kitti_to_desired_format(kitti_file, output_file):
    data = np.loadtxt(kitti_file).reshape(-1, 3, 4)
    rotations = data[:, :, :3]
    translations = data[:, :, 3]
    with open(output_file, 'w') as out:
        for rot, trans in zip(rotations, translations):
            # Convert rotation matrix to quaternion
            r = R.from_matrix(rot)
            q = r.as_quat()
            # Generate a random timestamp for demonstration
            timestamp = np.random.uniform(0, 1e9)  # Replace as needed
            out.write(f"{timestamp:.6f} {trans[0]} {trans[1]} {trans[2]} {q[0]} {q[1]} {q[2]} {q[3]}\n")

def euroc_to_desired_format(euroc_file, output_file):
    with open(euroc_file, 'r') as f, open(output_file, 'w') as out:
        for line in f:
            if line.startswith("#"):
                continue  # skip comment lines
            parts = line.strip().split(',')
            # print(parts)
            timestamp = parts[0]
            tx, ty, tz = parts[1:4]
            qw, qx, qy, qz = parts[4:8]
            out.write(f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")


def holoset_to_desired_format(holoset_file, output_file):
    with open(holoset_file, 'r') as f, open(output_file, 'w') as out:
        for line in f:
            parts = line.strip().split(',')
            timestamp, tx, ty, tz, w, x, y, z = parts
            out.write(f"{timestamp} {tx} {ty} {tz} {x} {y} {z} {w}\n")

if __name__ == "__main__":
    output_folder = "/home/.../Tracking_code/Evaluation/gt"
    kitti_base = "/home/.../KITTI/data_odometry_poses/dataset/poses"
    euroc_base = "/home/.../EuroC/"
    holoset_base = "/home/.../holoset/"
    
    # Convert KITTI dataset
    folder = ['00','01','02','03','04','05','06','07','08','09','10'] 
    for i in range(len(folder)):
        kitti_file = os.path.join(kitti_base, folder[i]+".txt")
        kitti_output_file = os.path.join(output_folder,"evo_kitti_"+folder[i]+".txt")
        kitti_to_desired_format(kitti_file, kitti_output_file)

    # Convert EuRoC dataset
    folder = ["MH_01_easy", "MH_02_easy","MH_03_medium","MH_04_difficult","MH_05_difficult","V1_01_easy","V1_02_medium",
            "V1_03_difficult","V2_01_easy","V2_02_medium","V2_03_difficult"]
    for i in range(len(folder)):
        euroc_file = os.path.join(euroc_base,folder[i], 'mav0/state_groundtruth_estimate0', "data.csv")
        euroc_output_file = os.path.join(output_folder,"evo_euroc_"+folder[i]+".txt")
        euroc_to_desired_format(euroc_file, euroc_output_file)

    # Convert Holoset dataset
    folder = ["campus-center-seq1/campus-center-seq1","campus-center-seq2/campus-center-seq2","suburbs-jog-seq1/suburbs-jog-seq1", 
               "suburbs-jog-seq2/suburbs-jog-seq2","suburbs-seq1/suburbs-seq1", "suburbs-seq2/suburbs-seq2"]
    for i in range(len(folder)):
        holoset_file = os.path.join(holoset_base,folder[i], "pose_quat.txt")
        desired_string = folder[i].split('/')[0]  # splits at the '/' and takes the first part
        print(desired_string)
        holoset_output_file = os.path.join(output_folder,"evo_holoset_"+desired_string+".txt")
        holoset_to_desired_format(holoset_file, holoset_output_file)
        
   