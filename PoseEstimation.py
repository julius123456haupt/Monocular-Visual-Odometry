import cv2 as cv
import numpy as np
import os

class PoseEstimation():
    def __init__(self, data_dir, Kamera_matrix):
        self.K = Kamera_matrix
        self.gt_poses = self._load_poses(os.path.join(data_dir, 'poses.txt'))
        self.absolute_pose = self.gt_poses[0]
        self.curr_R = np.eye(3)
        self.curr_t = np.zeros((3,1))
        self.prev_R = np.eye(3)
        self.prev_t = np.zeros((3,1))
        self.prev_points3d = None

    def _load_calib(self, filepath):
        """
        copied from Nicolai Nielse
        """
        with open(filepath, 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P
    
    def _load_poses(self, filepath):
        """
        copied from Nicolai Nielse
        """
        poses = []
        with open(filepath, 'r') as f:
            for line in f.readlines():
                T = np.fromstring(line, dtype=np.float64, sep=' ')
                T = T.reshape(3, 4)
                T = np.vstack((T, [0, 0, 0, 1]))
                poses.append(T)
        return poses
    

                
    def estimatePose(self, prev_pts_inliers, curr_pts_inliers, E):

        if len(prev_pts_inliers) < 8 or len(prev_pts_inliers) < 8:
            print("zu wenig Punkte um Position zu berechnen")
            return None, None, None, False
    
        # decompose Essential Matrix to recover relative pose between cameras (in my case between two frames of the same camera)
        _, R, t, _ = cv.recoverPose(E, prev_pts_inliers, curr_pts_inliers, self.K)
        
        # estimate relative scale with triangualtion
        realtive_scale = self.triangulation(prev_pts_inliers, curr_pts_inliers, R, t)

        return R, t, realtive_scale, True
    
    def triangulation(self, prev_pts, curr_pts, R, t):
        # create camera projection for both frames
        P1 = self.K @ np.hstack((np.eye(3), np.zeros((3,1))))
        P2 = self.K @ np.hstack((R,t))

        # triangulate between points that match in each frame 
        points4d = cv.triangulatePoints(P1, P2, prev_pts.T, curr_pts.T)

        # convert the 4D point vector into 3D by dividing the homogenious coordinate through the 4. component
        curr_points3d = (points4d / points4d[3]).T[:,:3]

        if self.prev_points3d is None:
            self.prev_points3d = curr_points3d
            return 1.0

        # compute relative scale with 3D point cloud scale
        relative_scale = self.computeCloudScale(self.prev_points3d, curr_points3d)

        self.prev_points3d = curr_points3d

        return max(0.1, min(relative_scale, 1)) # limitate the relative scale if necessary
    
    def computeCloudScale(self, prev_points3d, curr_points3d):

        if len(prev_points3d) < 4 or len(curr_points3d) < 4:
            return 1.0
        
        dist1 = []
        dist2 = []

        for i in range(min(20, len(prev_points3d))):  # Limit to 20 points for efficiency
            for j in range(i+1, min(20, len(prev_points3d))):
                d1 = np.linalg.norm(prev_points3d[i] - prev_points3d[j])
                d2 = np.linalg.norm(curr_points3d[i] - curr_points3d[j])
                if d1 > 0 and d2 > 0:  # Avoid zero distances
                    dist1.append(d1)
                    dist2.append(d2)

        if not dist1 or not dist2:
            return 1.0
        
        ratios = np.array(dist2) / np.array(dist1)
        relative_scale = np.median(ratios)

        if relative_scale < 0.1 or relative_scale > 10:
            return 1.0
        
        return relative_scale
    
    def updatePose(self, R, t, relative_scale):
        
        self.prev_R = self.curr_R.copy()
        self.prev_t = self.curr_t.copy()

        self.curr_t = self.curr_t + relative_scale * (self.curr_R @ t)
        self.curr_R = R @ self.curr_R

        # normalize the rotation matrix
        U, _, Vt = np.linalg.svd(self.curr_R)
        self.curr_R = U @ Vt

        # apply estimated scale on translation and put Transformation matrix together
        scaled_t = t * relative_scale
        T = self.getPose(R, scaled_t)

        # calculate absolute pose Matrix
        self.absolute_pose = self.absolute_pose @ np.linalg.inv(T)
        self.last_t = scaled_t
        self.last_R = R


    def getPose(self, R, t):
        # compose Transformation Matrix out of Rotation and translation
        pose = np.eye(4)
        pose[:3,:3] = R
        pose[:3, 3:] = t

        return pose
    
    def absolutePosition(self):
        # get current absolute position Matrix
        return self.getPose(self.curr_R, self.curr_t)
    
    def currentPosition(self):
        return self.curr_R, self.curr_t
    
    def currentTrajectorypoint(self):
        return self.curr_t.flatten()
    

