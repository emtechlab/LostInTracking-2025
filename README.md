# LostInTracking-2025

This repository contains the evaluation and analysis code for the paper:

📄 **Lost in Tracking Translation: A Comprehensive Analysis of Visual SLAM in Human-Centered XR and IoT Ecosystems**  
**Yasra Chandio, Khotso Selialia, Joseph DeGol, Luis Garcia, Fatima M. Anwar**  
*arXiv preprint arXiv:2411.07146, 2024*

---

## 📍 Overview

This project performs a rigorous evaluation of state-of-the-art visual SLAM tracking algorithms across three representative domains:

- Autonomous Vehicles (KITTI)
- Aerial Robotics (EuRoC)
- Mixed Reality (HoloSet)

We systematically analyze tracking performance across different levels:

- Method-level  
- Dataset-level  
- Sequence-level  
- Sample-level  

We also propose a novel taxonomy of environmental, locomotion, and algorithmic challenges and introduce practical suggestions for improving tracking robustness.

---

## 📁 Repository Structure

- `eval.py`: Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE) using evo.  
- `setting_up_gt_for_evo.py`: Utility to format ground truth trajectories compatible with evo.  
- `README.md`: You’re reading it!

---

## 📦 Setup

Install dependencies:

```bash
pip install evo numpy opencv-python

```

## 📥 Datasets and Tracking Repositories

You must download datasets and SLAM implementations manually.

### 📌 Datasets

- **[KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)** – stereo visual odometry SLAM benchmark with ground-truth poses
- **[EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)** – visual-inertial drone dataset with synchronized IMU, stereo cameras, and ground truth 
- **[HoloSet Dataset](https://dl.acm.org/doi/10.1145/3560905.3567763)** – mixed-reality visual-inertial pose estimation dataset captured with HoloLens 2

---

### 📌 SLAM Repositories Evaluated

Clone and run each of the following SLAM methods separately to generate trajectory outputs:

- **[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)**
- **[DSM (Direct Sparse Mapping)](https://github.com/jzubizarreta/dsm)**
- **[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**  
- **[DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM)**  
- **[SfMLearner](https://github.com/tinghuiz/SfMLearner)**  
- **[KP3D](https://github.com/vsislab/kp3d)**  
- **[TartanVO](https://github.com/TartanVO/TartanVO)**  
- **[DF-VO (DFVO)](https://github.com/ChiWeiHsiao/DF-VO)**  
- **[DeepVO](https://github.com/ChiWeiHsiao/Deep-VO-pytorch)**  

Once you generate predicted trajectories from each tracker, place them in a structured directory and use `eval.py` to compute metrics.

## Citation
If you use this code or build upon this analysis or use Holoset, please cite:

```bibtex
@article{chandio2024lost,
  title={Lost in Tracking Translation: A Comprehensive Analysis of Visual SLAM in Human-Centered XR and IoT Ecosystems},
  author={Chandio, Yasra and Selialia, Khotso and DeGol, Joseph and Garcia, Luis and Anwar, Fatima M},
  journal={arXiv preprint arXiv:2411.07146},
  year={2024}
}

@inproceedings{chandio2022holoset,
  title={Holoset-a dataset for visual-inertial pose estimation in extended reality: Dataset},
  author={Chandio, Yasra and Bashir, Noman and Anwar, Fatima M},
  booktitle={Proceedings of the 20th ACM Conference on Embedded Networked Sensor Systems},
  pages={1014--1019},
  year={2022}
}



