import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from LightGlue.lightglue import SuperPoint,LightGlue, SIFT
from LightGlue.lightglue.utils import rbd,numpy_image_to_torch

import torch
import pickle
torch.set_grad_enabled(False)

def findInlier(src_points, dst_points, ransacReprojThreshold=5.0):
    """
    Rough Python approximation of MATLAB's 'estgeotform2d(...,"similarity")'.
    Uses OpenCV's estimateAffinePartial2D or estimateAffine2D as an analogy.
    Returns (matrix, inlierMask, _) to mimic (tform, inlierIdx, status).
    Here, inlierMask is a boolean array marking inliers.
    """
    if len(src_points) < 3 or len(dst_points) < 3:
        # Not enough points to estimate a transform
        return np.array([], dtype=bool)

    # OpenCV expects points as (x, y) float32
    src_pts = np.array(src_points, dtype=np.float32).reshape(-1, 1, 2)
    dst_pts = np.array(dst_points, dtype=np.float32).reshape(-1, 1, 2)

    # For 'similarity', we can use estimateAffinePartial2D
    _, inlier_mask = cv2.estimateAffinePartial2D(src_pts, dst_pts, method=cv2.RANSAC,
                                                 ransacReprojThreshold=ransacReprojThreshold)
    if inlier_mask is None:
        return np.array([], dtype=bool)

    inlier_mask = inlier_mask.ravel().astype(bool)
    return inlier_mask

class FeatureDetectorMatcher:
    
    def __init__(self, detector_opt = {'type' : 'SP', 'params' : {'max_num_keypoints': 2048}},
                       matcher_opt  = {'type' : 'LightGlue' ,  'params' : {'depth_confidence' : 0.9, 'width_confidence' : 0.95}}):
        
        """"
        Initialize the FeatureDetectorMatcher class.
        
        Parameters:
        detector_opt (dict): Options for the feature detector. Should contain 'type' and 'params'.
            - type (str): Type of feature detector ('SP' or 'ORB').
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
        if self.detector_type == 'SP':
            self.Detector = SuperPoint(**detector_opt['params']).to(self.device)   
            self.Matcher  = LightGlue(**matcher_opt['params']).to(self.device) 
            # self.Matcher.compile(mode='reduce-overhead')
            
        elif self.detector_type == 'ORB':
            if 'params' not in detector_opt or detector_opt['params'] is None:
                
                # Default parameters for ORB detector
                detector_opt['params'] = {'nfeatures': 2048, 'nlevels': 8, 'edgeThreshold': 5}
                
                # Default parameters for matcher
                FLANN_INDEX_LSH = 6
                index_params= dict(algorithm = FLANN_INDEX_LSH,
                                    table_number = 6, # 12
                                    key_size = 12,     # 20
                                    multi_probe_level = 1) #2
                search_params = dict(checks=50)   # or pass empty dictionary
                matcher_opt = {'type' : 'FLANN Matcher', 'params' : {'index_params': index_params, 'search_params': search_params}}
                                
            self.Detector = cv2.ORB_create(**detector_opt['params'])
            matcher_opt = {'type' : 'FLANN Matcher', 'params' : {'index_params': index_params, 'search_params': search_params}}
            # self.Matcher  = cv2.BFMatcher(**matcher_opt['params'])
            self.Matcher = cv2.FlannBasedMatcher(matcher_opt['params']['index_params'], matcher_opt['params']['search_params'])
            
            self.Matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

            
        else:
            raise ValueError("Detector type not supported. Choose 'SP' or 'ORB'.")
                
        
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
            
        elif self.detector_type == 'SP':
            feat = self.Detector.extract(numpy_image_to_torch(frame).to(self.device))
            keypoints, descriptors = feat["keypoints"] , feat
            keypoints_np = keypoints.cpu().numpy().squeeze()

        return keypoints, keypoints_np, descriptors
    
    def matchFeatures(self,UAVKp,UAVDesc,ParticlesKp,ParticlesDesc,batch_mode = False):
        
        """
        Matches features between UAV and particle images.
        Returns an Nx2 or list(Nx2) array of matched feature indices
        
        Parameters:
        - UAVDesc: UAV image descriptors (numpy array or torch tensor)
        - ParticlesDesc: particle image descriptors (numpy array or torch tensor) or list of descriptors of all particles if batch_mode is True
        - batch_mode: boolean flag for LightGlue batch mode
        
        Returns:
        - inliers: Nx2 array of matched feature indices or list of Nx2 arrays if batch_mode is True
        """
        
        # Placeholder for inlierIdxList
        
        inlierIdxList = []        
        # Batch mode for LightGlue Feature Matching
        # NOTE : NOT FINISHED YET
        if batch_mode:
            
            
            # for construct feature list get first element of desc2 list
            UAV_feat_list  = UAVDesc
            part_feat_list = ParticlesDesc[1]
            for i in range(len(ParticlesDesc)):
                #add
                for key in part_feat_list.keys():
                    UAV_feat_list[key]  = torch.cat([UAV_feat_list[key] , UAVDesc[key]]    , dim=0)
                    part_feat_list[key] = torch.cat([part_feat_list[key], ParticlesDesc[i][key]], dim=0)
                    
            with torch.inference_mode():
                matches_list = self.Matcher({'image0': UAV_feat_list, 'image1': part_feat_list})        
                
            index_pairs_list = [matches["matches"].cpu().numpy() for matches in matches_list]
            
            # Loop through each particle and find inliers
            for i in range(len(ParticlesDesc)):
                
                src_pts = ParticlesKp[i][index_pairs_list[i][:,1]]
                dst_pts = UAVKp[index_pairs_list[i][:,0]]

                inliers = findInlier(src_pts, dst_pts)

                # Append inliers to the list
                inlierIdxList.append(inliers)
                
            return inlierIdxList
        
        # Single mode for OpenCV and LightGlue Feature Matching
        else:
            
            # Loop through each particle and find inliers
            for PartDesc, PartKp in zip(ParticlesDesc, ParticlesKp):
                
                if UAVDesc is None or PartDesc is None:
                    inlierIdxList.append(np.empty((0, 2), dtype=int))
                    continue
                
                # Match features using the specified matcher
                if self.detector_type == 'ORB':
                    # matches = self.Matcher.knnMatch(UAVDesc, PartDesc, k=2)
                    # # Sort matches by distance
                    # # matches = sorted(matches, key=lambda x: x.distance)
                    # # Filter matches based on distance threshold (optional)
                    # index_pairs = np.array([
                    #     [m.queryIdx, m.trainIdx]
                    #     for pair in matches if len(pair) == 2
                    #     for m, n in [pair] if m.distance < 0.95 * n.distance
                    # ], dtype=int)                    
                    
                    matches = self.Matcher.match(UAVDesc, PartDesc)
                    # Convert to an Nx2 array: [ (i_idx1, i_idx2), ... ]
                    index_pairs = np.array([[m.queryIdx, m.trainIdx] for m in matches], dtype=int)
                    
                    
                elif self.detector_type == 'SP':
                    matches = self.Matcher({"image0": UAVDesc, "image1": PartDesc})
                    _, _, matches = [rbd(x) for x in [UAVDesc, PartDesc, matches]]  # remove batch dimension
                    index_pairs = matches["matches"].cpu().numpy()

                # Check if there are any matches
                if index_pairs.shape[0] == 0:
                    inlierIdxList.append(np.empty((0, 2), dtype=int))
                    continue
                
                src_pts = PartKp[index_pairs[:,1]]
                dst_pts = UAVKp[index_pairs[:,0]]

                # Append inliers to the list
                inliers= findInlier(src_pts, dst_pts)
                inlierIdxList.append(inliers)
                    
            return inlierIdxList
        
    def MaskFeatures(self, featuresBase, keypointBase_np, image_size, mask, maxKP = None, LocalKp = None):
        """
        Mask the features based on the provided mask.
        
        Parameters:
        features (numpy.ndarray): Array of features to be masked.
        mask (numpy.ndarray): Binary mask indicating which features to keep.
        
        Returns:
        numpy.ndarray: Masked features.
        """
        
        if self.detector_type == 'ORB':
            maskedDescriptors    = featuresBase[mask]
            maskedKeypoints_np   = keypointBase_np[mask]
            
            if LocalKp is not None:
                maskedLocalKp = LocalKp[mask]  + np.array([ image_size[0] // 2, image_size[1] // 2])
                                
            if maxKP is not None:
                n_kp = maskedDescriptors.shape[0]
                n_discards = n_kp - maxKP
                if n_discards > 0:
                    perm = np.random.permutation(n_kp)  
                    keep_idx = perm[n_discards:]
                    maskedDescriptors = maskedDescriptors[keep_idx,:]
                    
                    if LocalKp is not None:
                        maskedLocalKp = maskedLocalKp[keep_idx,:]
                        
                
        elif self.detector_type == 'SP':
            keypoints, keypoint_scores, descriptors = featuresBase["keypoints"][:,mask,:] , \
                                                      featuresBase["keypoint_scores"][:,mask], \
                                                      featuresBase["descriptors"][:,mask,:]
            image_size_tensor = torch.from_numpy(np.array([image_size[0],image_size[1]])[np.newaxis, : ]).to(self.device)

            # scales, oris =  featuresBase["scales"][:,mask] , featuresBase["oris"][:,mask]
            maskedDescriptors = {"keypoints"   : keypoints,    "keypoint_scores" : keypoint_scores,
                                 "descriptors" : descriptors , "image_size"      : image_size_tensor}#, "scales" : scales, "oris" : oris}
            
            maskedKeypoints_np = keypoints.cpu().numpy().squeeze()
            
            if LocalKp is not None:
                maskedLocalKp = LocalKp[mask] + np.array([ image_size[0] // 2, image_size[1] // 2])
                
            if maxKP is not None:
                n_kp = maskedDescriptors["keypoints"].shape[1]
                n_discards = n_kp - maxKP
                if n_discards > 0:
                    perm = np.random.permutation(n_kp)  
                    keep_idx = perm[n_discards:]
                    maskedDescriptors["keypoints"]       = maskedDescriptors["keypoints"][:,keep_idx,:]
                    maskedDescriptors["keypoint_scores"] = maskedDescriptors["keypoint_scores"][:,keep_idx]
                    maskedDescriptors["descriptors"]     = maskedDescriptors["descriptors"][:,keep_idx,:]
                    
                    if LocalKp is not None:
                        maskedLocalKp = maskedLocalKp[keep_idx,:]
            
        if LocalKp is not None:
            return maskedKeypoints_np, maskedLocalKp, maskedDescriptors
        
        else:
            return maskedKeypoints_np, maskedDescriptors