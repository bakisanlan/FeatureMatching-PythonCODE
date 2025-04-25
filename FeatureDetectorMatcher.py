import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from LightGlue.lightglue import SuperPoint,LightGlue, SIFT
from LightGlue.lightglue.utils import rbd,numpy_image_to_torch

import torch
import pickle
torch.set_grad_enabled(False)

def findInlier(src_points, dst_points, transform_type="similarity", ransacReprojThreshold=5.0):
    """
    Rough Python approximation of MATLAB's 'estgeotform2d(...,"similarity")'.
    Uses OpenCV's estimateAffinePartial2D or estimateAffine2D as an analogy.
    Returns (matrix, inlierMask, _) to mimic (tform, inlierIdx, status).
    Here, inlierMask is a boolean array marking inliers.
    """
    if len(src_points) < 3 or len(dst_points) < 3:
        # Not enough points to estimate a transform
        return None, np.array([], dtype=bool), None

    # OpenCV expects points as (x, y) float32
    src_pts = np.array(src_points, dtype=np.float32).reshape(-1, 1, 2)
    dst_pts = np.array(dst_points, dtype=np.float32).reshape(-1, 1, 2)

    # For 'similarity', we can use estimateAffinePartial2D
    M, inlier_mask = cv2.estimateAffinePartial2D(src_pts, dst_pts, method=cv2.RANSAC,
                                                 ransacReprojThreshold=ransacReprojThreshold)
    if inlier_mask is None:
        return None, np.array([], dtype=bool), None

    inlier_mask = inlier_mask.ravel().astype(bool)
    return inlier_mask

class FeatureDetectorMatcher:
    
    def __init__(self, detector_opt = {'type' : 'SuperPoint', 'params' : {'max_keypoints': 2048}},
                       matcher_opt = {'type' : 'LightGlue' ,  'params' : {'depth_confidence' : 0.9, 'width_confidence' : 0.95}}):
        
        """"
        Initialize the FeatureDetectorMatcher class.
        
        Parameters:
        detector_opt (dict): Options for the feature detector. Should contain 'type' and 'params'.
            - type (str): Type of feature detector ('SuperPoint' or 'ORB').
            - params (dict): Parameters for the feature detector.
            
        matcher_opt (dict): Options for the feature matcher. Should contain 'type' and 'params'.
            - type (str): Type of feature matcher ('LightGlue' or 'KN Matcher').
            - params (dict): Parameters for the feature matcher.    
            
        Default ORB detector parameters:
        detector_opt = {'type' : 'ORB', 'params' : {'nfeatures': 1000, 'nlevels': 1, 'edgeThreshold': 5, 'firstLevel': 0, 'scoreType': cv2.ORB_HARRIS_SCORE}}
        matcher_opt = {'type' : 'KN Matcher', 'params' : {'normType': cv2.NORM_HAMMING, 'crossCheck': True}}
        
        # FLANN based matching parameters
        FLANN_INDEX_LSH = 6
        index_params= dict(algorithm = FLANN_INDEX_LSH,
                      table_number = 6, # 12
                      key_size = 12,     # 20
                      multi_probe_level = 1) #2
        search_params = dict(checks=50)   # or pass empty dictionary
        matcher_opt = {'type' : 'FLANN Matcher', 'params' : {'index_params': index_params, 'search_params': search_params}}
        """
        
        #Define the device for PyTorch
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
        self.detector_type = detector_opt['type']

        # Load the detector and matcher based on the provided options
        if self.detector_type == 'SuperPoint':
            self.Detector = SuperPoint(**detector_opt['params']).to(self.device)   
            self.Matcher  = LightGlue(**matcher_opt['params']).to(self.device) 
            
        elif self.detector_type == 'ORB':
            self.Detector = cv2.ORB_create(**detector_opt['params'])
            # self.Matcher  = cv2.BFMatcher(**matcher_opt['params'])
            self.Matcher = cv2.FlannBasedMatcher(**matcher_opt['params']['index_params'], **matcher_opt['params']['search_params'])
            
        else:
            raise ValueError("Detector type not supported. Choose 'SuperPoint' or 'ORB'.")
                
        
    def detectFeatures(self, frame):
        """
        Detect features in the given image using the specified detector.
        
        Parameters:
        image (numpy.ndarray): Input image in which to detect features.
        
        Returns:
        keypoints (tensor/opencv list)  : List of detected keypoints.
        keypoints_np (numpy.ndarray)    : Keypoints in numpy format.
        descriptors (tensor/opencv list): Descriptors for the detected keypoints.
        """
        
        if self.detector_type == 'ORB':
            # Convert image to grayscale if it is not already
            if len(frame.shape) == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            keypoints, descriptors = self.Detector.detectAndCompute(frame, None)
            keypoints_np = np.array([kp.pt for kp in keypoints])
            
        elif self.detector_type == 'SuperPoint':
            feat = self.Detector.extract(numpy_image_to_torch(frame).to(self.device))
            keypoints, descriptors = feat["keypoints"] , feat
            keypoints_np = keypoints.cpu().numpy().squeeze()

        return keypoints, keypoints_np, descriptors
    
    def matchFeatures(self,UAVKp,UAVDesc,PartKp,PartDesc,batch_mode = False):
        
        """
        Matches features between UAV and particle images.
        Returns an Nx2 or list(Nx2) array of matched feature indices
        
        Parameters:
        - UAVDesc: UAV image descriptors (numpy array or torch tensor)
        - PartDesc: particle image descriptors (numpy array or torch tensor) or list of descriptors of all particles if batch_mode is True
        - batch_mode: boolean flag for LightGlue batch mode
        
        Returns:
        - inliers: Nx2 array of matched feature indices or list of Nx2 arrays if batch_mode is True
        """
        
        # Placeholder for index pairs
        inliers = np.array([], dtype=bool) 
        
        # Batch mode for LightGlue Feature Matching
        # NOTE : NOT FINISHED YET
        if batch_mode:
            
            inliers_list = []
            
            # for construct feature list get first element of desc2 list
            UAV_feat_list  = UAVDesc
            part_feat_list = PartDesc[1]
            for i in range(len(PartDesc)):
                #add
                for key in part_feat_list.keys():
                    UAV_feat_list[key]  = torch.cat([UAV_feat_list[key] , UAVDesc[key]]    , dim=0)
                    part_feat_list[key] = torch.cat([part_feat_list[key], PartDesc[i][key]], dim=0)
                    
            with torch.inference_mode():
                matches_list = self.Matcher({'image0': UAV_feat_list, 'image1': part_feat_list})        
                
            index_pairs_list = [matches["matches"].cpu().numpy() for matches in matches_list]
            
            # Loop through each particle and find inliers
            for i in range(len(PartDesc)):
                
                src_pts = PartKp[i][index_pairs_list[i][:,1]]
                dst_pts = UAVKp[index_pairs_list[i][:,0]]

                try:
                    inliers= findInlier(src_pts, dst_pts, transform_type="similarity")
                except Exception:
                    # Emulate "ErrorHandler"
                    inliers = np.array([], dtype=bool)

                # Append inliers to the list
                inliers_list.append(inliers)
                
            return inliers_list
        
        # Single mode for OpenCV and LightGlue Feature Matching
        else:
            if UAVDesc is None or PartDesc is None:
                return np.empty((0, 2), dtype=int)

            if self.AIM.detector == 'ORB':
                matches = self.Matcher.match(UAVDesc, PartDesc)
                # Sort matches by distance
                # matches = sorted(matches, key=lambda x: x.distance)
                # Filter matches based on distance threshold (optional)
                index_pairs = np.array([[m.queryIdx, m.trainIdx]  for m,n in matches if m.distance < 0.75 * n.distance], dtype=int)
                
            elif self.AIM.detector == 'SP':
                matches = self.Matcher({"image0": UAVDesc, "image1": PartDesc})
                UAVDesc, PartDesc, matches = [rbd(x) for x in [UAVDesc, PartDesc, matches]]  # remove batch dimension
                index_pairs = matches["matches"].cpu().numpy()


            if index_pairs.shape[0] == 0:
                return np.array([], dtype=bool)

            src_pts = PartKp[index_pairs[:,1]]
            dst_pts = UAVKp[index_pairs[:,0]]

            try:
                inliers= findInlier(src_pts, dst_pts, transform_type="similarity")
                
            except Exception:
                # Emulate "ErrorHandler"
                inliers = np.array([], dtype=bool)

            return inliers