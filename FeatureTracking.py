import cv2 as cv 
import numpy as np

class FeatureTracker():
    def __init__(self, Kamera_matrix):
        self.K = Kamera_matrix
        self.orb = cv.ORB_create(nfeatures=3000) 
        self.bf = cv.BFMatcher(cv.NORM_HAMMING)     

    def extractFeatures(self, frame):
        # feature detection
        # corners = cv.goodFeaturesToTrack(frame,500,qualityLevel=0.03,minDistance=7, blockSize=7) # --> von George Hotz
        # kps = [cv.KeyPoint(x=f[0][0], y=f[0][1], size=31) for f in corners] 
        kps, des = self.orb.detectAndCompute(frame, None)

        if des is None or len(des) < 8 :
            print("Zu wenig Keypoints erkannt")
            return None, None

        # img_kp = cv.drawKeypoints(frame, kps, None, color=(0,255,0), flags=0)
        # cv.imshow("Display", img_kp)
        return kps, des

    def matchFeatures(self, prev_des, curr_des, prev_kps, curr_kps):
        # match descriptors using brute-force method
        matches = self.bf.knnMatch(prev_des, curr_des, k=2) 

        # ratio test as per Lowe's paper
        good_matches = [m for m,n in matches if m.distance < 0.8 * n.distance]
    
        if len(good_matches) < 8:
            print("zu wenig matches gefunden")
            return np.array([]), np.array([])

        # get x, y position of the good matched keypoints from the current and previous frame
        pts1 = np.float32([prev_kps[g.queryIdx].pt for g in good_matches])
        pts2 = np.float32([curr_kps[g.trainIdx].pt for g in good_matches])

        # compute Essential Matrix
        E, mask = cv.findEssentialMat(pts1, pts2, self.K, method=cv.RANSAC, threshold=1.0)

        if E is None:
            print("Essential Matrix konnte nicht berechnet werden")
            return np.array([]), np.array([])

        # select only the inlier points
        pts1 = pts1[mask.ravel() == 1]
        pts2 = pts2[mask.ravel() == 1]

        return pts1, pts2, E
    
    # def trackFeatures(self, prev_frame, curr_frame):
    #     # extract features, detect keypoints and compute descriptors from previous and current frame
    #     kps1, des1 = self.extractFeatures(prev_frame)
    #     kps2, des2 = self.extractFeatures(curr_frame)

    #     # calculate matched points
    #     pts1, pts2 = self.featureMatching(des1, des2, kps1, kps2)

    #     return pts1, pts2   