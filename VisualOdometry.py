import cv2 as cv
import numpy as np 
import matplotlib.pyplot as plt
from FeatureTracking import FeatureTracker
from PoseEstimation import PoseEstimation

data_dir = "/Volumes/Festplatte hauptmann/dataset/sequences/00"

class VisualOdometry():
    def __init__(self, K):
        self.prev_frame = None
        self.prev_kps = None
        self.prev_des = None

        self.FT = FeatureTracker(K)
        self.PE = PoseEstimation(data_dir, K)

    
    def processFrame(self, frame):
        if self.prev_frame is None:
            kps, des = self.FT.extractFeatures(frame)       

            self.prev_frame = frame.copy()
            self.prev_kps  = kps
            self.prev_des = des

        
        kps, des = self.FT.extractFeatures(frame)
        prev_pts, curr_pts, E = self.FT.matchFeatures(self.prev_des, des, self.prev_kps, kps)
        self.visualize_frame(frame, curr_pts, prev_pts )

        R, t, realtive_scale, success = self.PE.estimatePose(prev_pts, curr_pts, E)
        

        # img_matches = cv.drawMatches(self.prev_frame, self.prev_kps, frame, kps, good_matches[0:30], None)
        # cv.imshow("Display", img_matches)
        if success is not True:
            return False, None, None
        
        self.PE.updatePose(R, t, realtive_scale)
        self.prev_frame = frame.copy()
        self.prev_kps = kps
        self.prev_des = des 

        return True, self.PE.absolutePosition()
    
    def visualize_frame(self, curr_frame, curr_pts, prev_pts):

        display_frame = curr_frame.copy()

        display_frame = cv.cvtColor(display_frame, cv.COLOR_GRAY2BGR)

        for pt in curr_pts:
            x, y = map(int, pt)
            cv.circle(display_frame, (x, y), 1, (0, 0, 255), -1)

        # Zeichne die vorherigen Punkte in Blau
        for pt in prev_pts:
            x, y = map(int, pt)
            cv.circle(display_frame, (x, y), 1, (255, 0, 0), -1)

        # Zeichne Linien zwischen den korrespondierenden Punkten
        for (curr_pt, prev_pt) in zip(curr_pts, prev_pts):
            curr_x, curr_y = map(int, curr_pt)
            prev_x, prev_y = map(int, prev_pt)
            cv.line(display_frame, (curr_x, curr_y), (prev_x, prev_y), (255, 0, 255), 1)

        cv.imshow("Display", display_frame)



