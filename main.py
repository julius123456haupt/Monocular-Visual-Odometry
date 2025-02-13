import numpy as np
import matplotlib.pyplot as plt
import os
import cv2 as cv
from FeatureTracking import FeatureTracker
from PoseEstimation import PoseEstimation
from VisualOdometry import VisualOdometry

def load_poses(filepath):
    """
    copied from Nicolai Nielse
    """
    poses = []
    try:
        with open(filepath, 'r') as f:
            for line in f.readlines():
                vals = np.fromstring(line, dtype=np.float64, sep=' ')
                poses.append(vals.reshape(3, 4))
    except FileNotFoundError:
        print(f"Error: Could not find poses file at {filepath}")
        return None
    except Exception as e:
        print(f"Error loading poses: {e}")
        return None
    return poses

def align_trajectories(est_poses, gt_poses):

    if len(est_poses) < 2 or len(gt_poses) < 2:
        return est_poses

    # Berechne die initiale Bewegungsrichtung für beide Trajektorien
    gt_initial_direction = gt_poses[1][0:3, 3] - gt_poses[0][0:3, 3]
    est_initial_direction = est_poses[1][0:3, 3] - est_poses[0][0:3, 3]

    # Normalisiere die Vektoren
    gt_initial_direction = gt_initial_direction / np.linalg.norm(gt_initial_direction)
    est_initial_direction = est_initial_direction / np.linalg.norm(est_initial_direction)

    # Berechne den Rotationswinkel zwischen den Vektoren in der X-Z-Ebene
    angle = np.arctan2(gt_initial_direction[2], gt_initial_direction[0]) - \
            np.arctan2(est_initial_direction[2], est_initial_direction[0])

    # Erstelle Rotationsmatrix
    R_align = np.array([[np.cos(angle), 0, -np.sin(angle)],
                       [0, 1, 0],
                       [np.sin(angle), 0, np.cos(angle)]])

    # Wende die Rotation auf alle geschätzten Posen an
    aligned_poses = []
    for pose in est_poses:
        aligned_pose = pose.copy()
        aligned_pose[0:3, 3] = R_align @ pose[0:3, 3]
        aligned_poses.append(aligned_pose)

    return aligned_poses

def plot_trajectories(gt_poses, estimated_poses):
    if not gt_poses or not estimated_poses:
        return
    
    aligned_poses = align_trajectories(estimated_poses, gt_poses)
    
    gt_x = [pose[0, 3] for pose in gt_poses]
    gt_z = [pose[2, 3] for pose in gt_poses]
    
    # Extract aligned estimated (x,z) translations and invert X-axis
    est_x = [-pose[0, 3] for pose in aligned_poses]  # Invert X-axis
    est_z = [pose[2, 3] for pose in aligned_poses]

    plt.figure(figsize=(10, 10))
    
    plt.plot(gt_x, gt_z, 'b-', label='Ground Truth', linewidth=2)
    plt.plot(est_x, est_z, 'r-', label='Estimated', linewidth=2)

    plt.plot(gt_x[0], gt_z[0], 'go', label='Start', markersize=10)
    plt.plot(gt_x[-1], gt_z[-1], 'ko', label='Ground Truth End', markersize=10)
    plt.plot(est_x[-1], est_z[-1], 'mo', label='Estimated End', markersize=10)

    plt.title('KITTI Trajectory Comparison', fontsize=14)
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Z (meters)', fontsize=12)
    plt.axis('equal')
    plt.grid(True)
    plt.legend(fontsize=10)
    
    all_x = gt_x + est_x
    all_z = gt_z + est_z
    padding = 10  
    plt.xlim(min(all_x) - padding, max(all_x) + padding)
    plt.ylim(min(all_z) - padding, max(all_z) + padding)
    
    plt.show()

def main():
    data_dir = "/Volumes/Festplatte hauptmann/dataset/sequences/00"
    video_path = "/Users/juliushauptmann/Desktop/HROB/CV/Projekt/kitti00_short.mp4"
    
    K = np.array([[718.856, 0, 607.1928],
                  [0, 718.856, 185.2157],
                  [0, 0, 1]])
    
    pose_estimator = PoseEstimation(data_dir, K)
    ground_truth_poses = pose_estimator.gt_poses
    vo = VisualOdometry(K)
    
    cap = cv.VideoCapture(video_path)
    estimated_path = []

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        success, est_pos = vo.processFrame(gray)
        
        if success:
            estimated_path.append(est_pos)
            print(f"Estimated position: {est_pos[0:3, 3]}")
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
    
    # Plot trajectories
    N = len(estimated_path)
    ground_truth_poses = ground_truth_poses[:N]
    plot_trajectories(ground_truth_poses, estimated_path)

if __name__ == "__main__":
    main()