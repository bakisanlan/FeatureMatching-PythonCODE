import numpy as np
import cv2
from utils import rotate_image

class VisualOdometry:
    
    def __init__(self, K = np.eye(3), scale = 1, nfeatures = 1000, nlevels = 1, type = 'ORB', t = np.zeros((3,1)), R = np.eye(3)):
        
        self.K = K
        self.scale = scale
        self.prev_frame    = None  #Previous frame
        self.prev_KpDes    = None  #Prevous frame's keypoints 
        self.firstFlag = 1
        self.t = t 
        self.R = R
        self.VO_frame = None
        self.VO_matchframe = None
        self.n_match = 0
        self.type = type
        self.R_rel = np.identity(3)
        self.t_rel = np.zeros((3, 1))
        
        # Initialize detector ORB/SIFT.
        if type == 'ORB':
            self.detector = cv2.ORB_create(nfeatures=nfeatures, nlevels = nlevels, edgeThreshold = 5)

        elif type == 'SIFT':
            self.detector = cv2.SIFT_create(nfeatures=nfeatures)
            
        # Initialize matcher
        if type == 'ORB':
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            
        elif type == 'SIFT':
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks=50)   # or pass empty dictionary
            self.matcher = cv2.FlannBasedMatcher(index_params,search_params)

    def process_vo_frame(self,curr_frame, scale):
        """
        Process two consecutive frames to compute the relative camera motion.
        
        Parameters:
        curr_frame : current image frame (BGR or grayscale)
        
        Returns:
        R_rel : relative rotation matrix (3x3 numpy array)
        t_rel : relative translation vector (3x1 numpy array; up to scale)
                If not enough matches are found, returns (None, None).
        """
        
        # Convert to grayscale if needed.
        if len(curr_frame.shape) == 3:
            curr_frame = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
        else:
            curr_frame = curr_frame.copy()
            
        if self.firstFlag:
            kp0, des0 = self.detector.detectAndCompute(curr_frame, None)
            
            if des0 is None:
                print('No feature detected for first frame of VO')
                return None, None
            
            self.prev_KpDes = {'Kp' : kp0 , 'Des' : des0}
            self.prev_frame = curr_frame.copy()
            
            # self.rel_R = np.identity(3)
            # self.rel_t = np.zeros((3, 1))
            
            self.firstFlag = 0
            return None, None
        
        else:
            kp_curr, des_curr = self.detector.detectAndCompute(curr_frame, None)
            if des_curr is None:
                print('No feature detected for next frame of VO')
                return None, None
            
            kp_prev, des_prev = self.prev_KpDes['Kp'], self.prev_KpDes['Des'] 

            if self.type == 'ORB':
                # Use BFMatcher with Hamming distance.
                matches = self.matcher.match(des_prev, des_curr)
                
                # Sort matches by descriptor distance.
                matches = sorted(matches, key=lambda x: x.distance)
                self.n_match = len(matches)
            
            elif self.type == 'SIFT':
                matches = self.matcher.knnMatch(des_prev,des_curr,k=2)
                                
                # ratio test as per Lowe's paper
                matches = [m for i, (m, n) in enumerate(matches) if m.distance < 0.7 * n.distance]
                matches = sorted(matches, key=lambda x: x.distance)

                self.n_match = len(matches)
                
            if len(matches) < 8:
                print(f'No sufficient number of matches found, n matches: {len(matches)}')
                return None, None
            
            # Extract matched keypoints.
            match_pts_prev = np.float32([kp_prev[m.queryIdx].pt for m in matches])
            match_pts_curr = np.float32([kp_curr[m.trainIdx].pt for m in matches])
            
            # Compute the essential matrix using RANSAC.
            E, _ = cv2.findEssentialMat(match_pts_curr, match_pts_prev, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            E2, _ = cv2.findEssentialMat(match_pts_prev, match_pts_curr, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

            if E is None:
                print('Essential matrix is None')
                return None, None

            # Recover pose from the essential matrix.
            _, R_rel, t_rel, _ = cv2.recoverPose(E, match_pts_curr, match_pts_prev, self.K)
            _, R_rel2, t_rel2, _ = cv2.recoverPose(E2, match_pts_prev, match_pts_curr, self.K)
            self.R_rel = R_rel
            self.t_rel = t_rel

            # If the estimated scale is too small (e.g. near zero), choose a default scale.
            if scale > 0.1:                
                # Update camera pose
                self.t = self.t + scale * (self.R @ (t_rel)).flatten()
                self.R = R_rel @ (self.R)
            
            # Draw frames
            self.VO_frame = cv2.drawKeypoints(curr_frame,kp_curr,None,color= (128,0,128), flags= 0)
            # self.VO_matchframe = rotate_image(cv2.drawMatches(self.prev_frame,kp_prev,curr_frame,kp_curr,matches,outImg=None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS, matchColor= (0,255,0), singlePointColor= (0,255,0)),np.pi/2)
            self.VO_matchframe = cv2.drawMatches(self.prev_frame,kp_prev,curr_frame,kp_curr,matches,outImg=None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS, matchColor= (0,255,0), singlePointColor= (0,255,0))


            # Update previous frame as current frame
            self.prev_KpDes = {'Kp' : kp_curr , 'Des' : des_curr}
            self.prev_frame = curr_frame.copy()
            
            return self.R, self.t