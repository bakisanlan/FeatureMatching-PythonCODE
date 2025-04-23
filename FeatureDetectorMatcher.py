import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from LightGlue.lightglue import SuperPoint,LightGlue, SIFT
from LightGlue.lightglue.utils import rbd,numpy_image_to_torch

import torch
import pickle
torch.set_grad_enabled(False)

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
            self.Matcher  = cv2.BFMatcher(**matcher_opt['params'])

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
    
    
    def matchFeatures(self, UAV_desc, part_desc, batch_mode = False):
        
        """
        Matches features between UAV and particle images.
        Returns an Nx2 or list(Nx2) array of matched feature indices
        
        Parameters:
        - UAV_desc: UAV image descriptors (numpy array or torch tensor)
        - part_desc: particle image descriptors (numpy array or torch tensor) or list of descriptors of all particles if batch_mode is True
        - batch_mode: boolean flag for LightGlue batch mode
        
        Returns:
        - index_pairs: numpy array of matched feature indices (Nx2) or list of arrays if batch_mode is True
        """
        
        # Batch mode for LightGlue Feature Matching
        # NOTE : NOT FINISHED YET
        if batch_mode:
            # for construct feature list get first element of desc2 list
            UAV_feat_list  = UAV_desc
            part_feat_list = part_desc[1]
            for i in range(len(part_desc)):
                #add
                for key in part_feat_list.keys():
                    UAV_feat_list[key]  = torch.cat([UAV_feat_list[key] , UAV_desc[key]]    , dim=0)
                    part_feat_list[key] = torch.cat([part_feat_list[key], part_desc[i][key]], dim=0)
                    
            with torch.inference_mode():
                matches_list = self.Matcher({'image0': UAV_feat_list, 'image1': part_feat_list})        
                
            index_pairs_list = [matches["matches"].cpu().numpy() for matches in matches_list]
            index_pairs = index_pairs_list
                
        # Single mode for OpenCV and LightGlue Feature Matching
        else:
            if UAV_desc is None or part_desc is None:
                return np.empty((0, 2), dtype=int)

            if self.AIM.detector == 'ORB':
                matches = self.Matcher.match(UAV_desc, part_desc)
                # Sort matches by distance
                matches = sorted(matches, key=lambda x: x.distance)
                # Filter matches based on distance threshold (optional)
                
                
                
                # Convert to an Nx2 array: [ (i_idx1, i_idx2), ... ]
                index_pairs = np.array([[m.queryIdx, m.trainIdx] for m in matches], dtype=int)

            elif self.AIM.detector == 'SP':
                matches = self.Matcher({"image0": UAV_desc, "image1": part_desc})
                UAV_desc, part_desc, matches = [rbd(x) for x in [UAV_desc, part_desc, matches]]  # remove batch dimension
                index_pairs = matches["matches"].cpu().numpy()



        return index_pairs