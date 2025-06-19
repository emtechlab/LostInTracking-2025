import numpy as np
import matplotlib.pyplot as plt
import os

def load_data(filename):
    data = np.loadtxt(filename, delimiter=' ')
    # We only take the tx, ty, tz columns for simplicity in this example
    return data[:, 1:4]

def compute_optimal_scale(ground_truth, estimate):
    dot_product = np.dot(ground_truth.flatten(), estimate.flatten())
    norm = np.dot(estimate.flatten(), estimate.flatten())
    return dot_product / norm

def compute_ate(ground_truth, estimate, scale=1.0):
    estimate_scaled = estimate * scale
    ate = np.sqrt(np.sum(np.square(ground_truth - estimate_scaled)) / len(ground_truth))
    return ate

def compute_rpe(ground_truth, estimate, delta=1):
    rpe_translations = []
    for i in range(len(ground_truth) - delta):
        gt_diff = ground_truth[i + delta] - ground_truth[i]
        est_diff = estimate[i + delta] - estimate[i]
        rpe_translations.append(np.linalg.norm(gt_diff - est_diff))
    rpe = np.mean(rpe_translations)
    return rpe

def align_trajectories(ground_truth, estimate):
    """Align the estimate to the ground truth."""
    assert ground_truth.shape == estimate.shape
    # Compute the mean of the points
    mu_gt = ground_truth.mean(axis=0, keepdims=True)
    mu_est = estimate.mean(axis=0, keepdims=True)

    # Centralize the datasets
    gt_centered = ground_truth - mu_gt
    est_centered = estimate - mu_est

    # Compute the optimal rotation using Umeyama's method
    W = np.dot(gt_centered.T, est_centered)
    U, s, Vt = np.linalg.svd(W)
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        U[:, -1] = -U[:, -1]
        R = np.dot(U, Vt)

    # Compute the optimal translation
    t = mu_gt.T - np.dot(R, mu_est.T)

    # Align the estimated trajectory
    estimate_aligned = np.dot(estimate, R.T) + t.T

    return estimate_aligned

def umeyama_alignment(src, dst, with_scale=True):
    """Compute Sim(3)/SE(3) alignment using the Umeyama method."""
    assert src.shape == dst.shape
    n, dim = src.shape

    mean_src = np.mean(src, axis=0)
    mean_dst = np.mean(dst, axis=0)

    centered_src = src - mean_src
    centered_dst = dst - mean_dst

    cov_matrix = np.dot(centered_src.T, centered_dst) / n

    U, _, Vt = np.linalg.svd(cov_matrix)

    d = np.sign(np.linalg.det(np.dot(U, Vt)))
    I = np.eye(dim)
    I[-1, -1] = d
    R = np.dot(U, np.dot(I, Vt))

    if with_scale:
        scale = np.trace(np.dot(cov_matrix, R.T)) / np.sum(np.square(centered_src))
    else:
        scale = 1.0

    t = mean_dst - scale * np.dot(R, mean_src)

    return R, t, scale

def plot_trajectories(ground_truth, estimate, estimate_aligned, fig_path):
    plt.figure()
    plt.plot(ground_truth[:, 0], ground_truth[:, 1], label="Ground Truth", color="green")
    plt.plot(estimate[:, 0], estimate[:, 1], label="Estimate", color="red")
    plt.plot(estimate_aligned[:, 0], estimate_aligned[:, 1], label="Estimate Aligned", color="blue", linestyle='--')
    plt.legend()
    plt.title("Trajectories")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    print(fig_path)
    plt.savefig(fig_path+".pdf")

basepath = "/Results/"
algo = ["Deepvo",'DFVO','ORBSLAM3','SfmLearner','tartanvo', 'DROIDSLAM','KP3D']
algo = ['ORBSLAM3']
algo = ['DFVO']
# algo = ["tartanvo"]
# algo = ['DROIDSLAM']
# algo = ['SfmLearner']
algo = ['KP3D']
holoset_sequence = ["campus-center-seq1","campus-center-seq2","suburbs-jog-seq1", "suburbs-jog-seq2","suburbs-seq1", "suburbs-seq2"]
kitti_sequence = ["00","01","02","03","04","05","06","07","08","09","10"]
euroc_sequence = ["MH_01_easy", "MH_02_easy","MH_03_medium","MH_04_difficult","MH_05_difficult","V1_01_easy","V1_02_medium",
            "V1_03_difficult","V2_01_easy","V2_02_medium","V2_03_difficult"]
euroc_sequence = ["MH_01", "MH_02","MH_03","MH_04","MH_05","V_101","V_102",
            "V_103","V_201","V_202","V_203"]
euroc_sequence = ["MH01", "MH02","MH03","MH04","MH05","V101","V102","V103","V201","V202","V203"] # for ORBSLAM3
dataset_list = ["Holoset","EuRoC","KITTI"]

dataset = dataset_list[0]
if dataset == "Holoset":
    sequence = holoset_sequence
elif dataset == "EuRoC":
    sequence = euroc_sequence
else:
    sequence = kitti_sequence

sequence_type = ["mono","stereo", 'regular']
sequence_type = sequence_type[2]

for j in  range(len(algo)):
    for i in range(len(sequence)):
        print("Sequence: ",sequence[i])
        if algo[j] == "ORBSLAM3" and dataset == "EuRoC":
            sequence = ["MH01", "MH02","MH03","MH04","MH05","V101","V102","V103","V201","V202","V203"] # for ORBSLAM3
            if sequence_type == "mono":
                filename = os.path.join(dataset,"f_dataset-"+sequence[i]+"_mono")
            else: 
                # algo[i] == "ORBSLAM3" and dataset == "EuRoC" and sequence_type == "stereo":
                filename = os.path.join(dataset,"f_dataset-"+sequence[i]+"_stereoi")
        elif algo[j] == "ORBSLAM3" and dataset == "KITTI":
            if sequence_type == "mono":
                filename = "mono-seq"+sequence[i]
            else:
                filename = "stereo-seq"+sequence[i]
        else:
            filename = sequence[i]

        if algo[j] == "ORBSLAM3" and dataset == "EuRoC": # for groundtruth files
            sequence = ["MH_01_easy", "MH_02_easy","MH_03_medium","MH_04_difficult","MH_05_difficult","V1_01_easy","V1_02_medium",
            "V1_03_difficult","V2_01_easy","V2_02_medium","V2_03_difficult"]
        
        ground_truth_file = os.path.join("gt/evo_"+dataset.lower()+"_"+sequence[i]+".txt")
        
        print(ground_truth_file)
        # estimate_file = os.path.join(basepath, algo[j],dataset+sequence[i]+".txt")
        if algo[j] == 'Deepvo' and dataset == "Holoset":
            estimate_file = os.path.join(basepath, algo[j],dataset,filename+".csv") 
            print(os.path.join(basepath, algo[j],dataset,filename+".csv") )
        elif algo[j] == 'SfmLearner' and dataset == "Holoset":
            estimate_file = os.path.join(basepath, algo[j],dataset,filename+".npy")

        else:
            estimate_file = os.path.join(basepath, algo[j],dataset,filename+".txt")

        fig_path = os.path.join(basepath, algo[j],"trajectories",dataset+"_"+filename)
        print(fig_path)
        if not os.path.exists(os.path.join(basepath, algo[j],"trajectories")):
            os.makedirs(os.path.join(basepath, algo[j],"trajectories"))

        ground_truth = load_data(ground_truth_file)
        estimate = load_data(estimate_file)
        

        if ground_truth.shape != estimate.shape:
            print("Sequence did not match groundtruth ",ground_truth.shape, estimate.shape)
            ground_truth = ground_truth[:estimate.shape[0]]
            
            
        ate = compute_ate(ground_truth, estimate)
        rpe = compute_rpe(ground_truth, estimate)
        print(ate)
        # print(f"ATE: {ate}")
        # print(f"RPE: {rpe}")
        scale = compute_optimal_scale(ground_truth, estimate)
        ate_with_scale = compute_ate(ground_truth, estimate, scale)

        # print(f"Optimal Scale: {scale}")
        # print(f"ATE with Optimal Scale: {ate_with_scale}")

        # Align the estimated trajectory and compute the ATE again
        estimate_aligned = align_trajectories(ground_truth, estimate)
        ate_aligned = compute_ate(ground_truth, estimate_aligned, scale =1)
        # print(f"ATE (Aligned): {ate_aligned}")

        # Sim(3) alignment
        R_sim3, t_sim3, scale_sim3 = umeyama_alignment(estimate, ground_truth, with_scale=True)
        aligned_estimate_sim3 = scale_sim3 * np.dot(estimate, R_sim3.T) + t_sim3

        # SE(3) alignment
        R_se3, t_se3, _ = umeyama_alignment(estimate, ground_truth, with_scale=False)
        aligned_estimate_se3 = np.dot(estimate, R_se3.T) + t_se3

        # Compute ATEs for both alignments
        ate_sim3 = compute_ate(ground_truth, aligned_estimate_sim3)
        ate_se3 = compute_ate(ground_truth, aligned_estimate_se3)
        
        print(f"ATE with Sim(3) alignment: {ate_sim3}")
        print(f"ATE with SE(3) alignment: {ate_se3}")
        print(f"Scale error (from Sim(3) alignment): {scale_sim3}")

        plot_trajectories(ground_truth, estimate)
        plot_trajectories(ground_truth, estimate, aligned_estimate_sim3, fig_path)
