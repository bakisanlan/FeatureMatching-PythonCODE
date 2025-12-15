# import os
import cv2
import numpy as np
# import matplotlib.pyplot as plt
# from LightGlue.lightglue import SuperPoint,LightGlue, SIFT
# from LightGlue.lightglue.utils import rbd,numpy_image_to_torch

import torch
from Timer import Timer
# import pickle
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
                                                 ransacReprojThreshold=ransacReprojThreshold)   # NOTE: CHECK SPEED OF RANSAC
    if inlier_mask is None:
        return np.array([], dtype=bool)

    inlier_mask = inlier_mask.ravel().astype(bool)
    return inlier_mask

class FeatureDetectorMatcher:
    
    # def __init__(self, detector_opt = {'type' : 'SP', 'params' : {'max_num_keypoints': 2048}},
    #                    matcher_opt  = {'type' : 'LightGlue' ,  'params' : {'depth_confidence' : 0.9, 'width_confidence' : 0.95}}):
    def __init__(self, detector_opt = None,
                       matcher_opt  = None):

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
        
        
        # Default XFEAT detector and matcher parameters:
        detector_opt = {'params': {'top_k': 1300, 'detection_threshold': 0.05}}
        """
        
        #Define the device for PyTorch
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
        # self.device = torch.device("cpu")
        print(f"FeatureDetectorMatcher Using device: {self.device}")
        self.detector_type = None if detector_opt is None else detector_opt['type']

        # Load the detector and matcher based on the provided options
        if self.detector_type == 'SP':
            self.Detector = SuperPoint(**detector_opt['params']).to(self.device)   
            self.Matcher  = LightGlue(**matcher_opt['params']).to(self.device) 
            # self.Matcher.compile(mode='reduce-overhead')
            
        elif self.detector_type == 'XFEAT':

            
            if 'params' not in detector_opt or detector_opt['params'] is None:
                # Default parameters for XFEAT detector
                detector_opt['params'] = {'top_k': 1400}
                
                # Default parameters for matcher
                matcher_opt = {}
                
                self.Detector = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained = True, **detector_opt['params'])
                self.Matcher  = self.Detector.match_lighterglue
            
            
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

        # Image Mathcing using ZNCC (Zero-mean Normalized Cross-Correlation) if no detector/matcher is specified
        else:
            self.TemplateMatchingFlag = True
            print(f"No detector/matcher specified. Using ZNCC for image matching.")
            

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
            
        elif self.detector_type == 'XFEAT':
            feat = self.Detector.detectAndCompute(frame)[0]                # NOTE: Also support batched mode, but here we use single image mode, no need to convert torch tensor
            feat.update({'image_size': (frame.shape[1], frame.shape[0])})  # add image size info for light glue matcher
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
                    
                    try:
                        matches = self.Matcher.match(UAVDesc, PartDesc)
                        # Convert to an Nx2 array: [ (i_idx1, i_idx2), ... ]
                        index_pairs = np.array([[m.queryIdx, m.trainIdx] for m in matches], dtype=int)
                    
                    except:
                        index_pairs = np.empty((0, 2), dtype=int)
                    
                    
                elif self.detector_type == 'SP':
                    try: 
                        matches = self.Matcher({"image0": UAVDesc, "image1": PartDesc})
                        _, _, matches = [rbd(x) for x in [UAVDesc, PartDesc, matches]]  # remove batch dimension
                        index_pairs = matches["matches"].cpu().numpy()
                    except:
                        index_pairs = np.empty((0, 2), dtype=int)
                        
                elif self.detector_type == 'XFEAT':
                    
                    # with Timer("XFEAT LightGlue Matching"):
                    _, _, matches = self.Matcher(UAVDesc, PartDesc)   # returns np.array of shape Nx2 for matches
                        # print(f"number of kps UAV and Part: {UAVDesc['keypoints'].shape[0]}, {PartDesc['keypoints'].shape[0]}")
                    index_pairs = matches
                    # except:
                    #     index_pairs = np.empty((0, 2), dtype=int)

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
            
            # Convert global keypoints to local keypoints as torch tensor
            if LocalKp is not None:
                maskedLocalKp = LocalKp[mask] + np.array([ image_size[0] // 2, image_size[1] // 2])
                keypoints = torch.from_numpy(maskedLocalKp.astype(np.float32)[np.newaxis, :, :]).to(self.device)

            # scales, oris =  featuresBase["scales"][:,mask] , featuresBase["oris"][:,mask]
            maskedDescriptors = {"keypoints"   : keypoints,    "keypoint_scores" : keypoint_scores,
                                 "descriptors" : descriptors , "image_size"      : image_size_tensor}#, "scales" : scales, "oris" : oris}
            
            maskedKeypoints_np = keypoints.cpu().numpy().squeeze()

                
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
                        
        elif self.detector_type == 'XFEAT':   # NOTE: remove batch dimension indexing, this is only difference from SP  
            keypoints, scores, descriptors = featuresBase["keypoints"][mask,:] , \
                                             featuresBase["scores"][mask], \
                                             featuresBase["descriptors"][mask,:]
            image_size = np.array([image_size[0],image_size[1]])

            # Convert global keypoints to local keypoints as torch tensor
            if LocalKp is not None:
                maskedLocalKp = LocalKp[mask] + np.array([ image_size[0] // 2, image_size[1] // 2])
                keypoints = torch.from_numpy(maskedLocalKp.astype(np.float32)).to(self.device)

            # scales, oris =  featuresBase["scales"][:,mask] , featuresBase["oris"][:,mask]
            maskedDescriptors = {"keypoints"   : keypoints,    "scores" : scores,
                                 "descriptors" : descriptors , "image_size"      : image_size}#, "scales" : scales, "oris" : oris}
            
            maskedKeypoints_np = keypoints.cpu().numpy().squeeze()
                
            if maxKP is not None:
                n_kp = maskedDescriptors["keypoints"].shape[1]
                n_discards = n_kp - maxKP
                if n_discards > 0:
                    perm = np.random.permutation(n_kp)  
                    keep_idx = perm[n_discards:]
                    maskedDescriptors["keypoints"]       = maskedDescriptors["keypoints"][keep_idx,:]
                    maskedDescriptors["scores"]          = maskedDescriptors["scores"][keep_idx]
                    maskedDescriptors["descriptors"]     = maskedDescriptors["descriptors"][keep_idx,:]
                    
                    if LocalKp is not None:
                        maskedLocalKp = maskedLocalKp[keep_idx,:]
            

        return maskedKeypoints_np, maskedDescriptors
    

    def znccMatch(self, patches: np.ndarray,
                   template: np.ndarray,
                   mask: np.ndarray,
                   eps: float = 1e-12) -> np.ndarray:
        """
        Masked ZNCC (ignores mask==0 pixels).
        
        Parameters
        ----------
        patches : np.ndarray, shape (N, h, w)
            Satellite patches (particles), grayscale
        template : np.ndarray, shape (h, w)
            UAV template (orthoprojected), grayscale
        mask : np.ndarray, shape (h, w)
            Binary mask: 1=valid, 0=invalid (black region)
        eps : float
            Small constant to avoid division by zero
            
        Returns
        -------
        scores : np.ndarray, shape (N,)
            Masked ZNCC scores in range [-1, 1]
        """
        N, h, w = patches.shape
        M = h * w
        
        # Flatten arrays
        P = patches.astype(np.float32).reshape(N, M)   # (N, M)
        T = template.astype(np.float32).ravel()        # (M,)
        W = mask.astype(np.float32).ravel()            # (M,)
        
        # Count valid pixels
        w_sum = W.sum()
        if w_sum < 1:
            raise ValueError("Mask has no valid pixels (sum(mask)==0).")
        
        w_sum = w_sum + eps
        
        # Compute weighted mean for template
        mu_T = (W * T).sum() / w_sum
        
        # Zero-mean template (only at valid pixels)
        T_centered = (T - mu_T) * W
        
        # Template standard deviation
        sigma_T = np.sqrt((T_centered * T_centered).sum()) + eps
        
        # Per-patch weighted mean (over valid pixels only)
        mu_P = (P * W).sum(axis=1) / w_sum  # (N,)
        
        # Zero-mean patches (broadcast mu_P to shape (N, M))
        P_centered = (P - mu_P[:, np.newaxis]) * W  # (N, M)
        
        # Per-patch standard deviation
        sigma_P = np.sqrt((P_centered * P_centered).sum(axis=1)) + eps  # (N,)
        
        # Compute correlation (dot product of centered values)
        numerator = P_centered @ T_centered  # (N,)
        
        # Normalize by standard deviations
        denominator = sigma_P * sigma_T
        
        scores = numerator / denominator
        
        # Clip to valid range [-1, 1] (due to numerical errors)
        scores = np.clip(scores, -1.0, 1.0)
        
        return scores
    

# NOTE: This is the most fast but rotation is not considered yet
def masked_zncc_particles_from_satellite_map_center(      
    sat_img, template, mask, cxs, cys, eps=1e-12, invalid_value=np.nan
):
    """
    Masked ZNCC for many particles, given particle CENTER coordinates.

    sat_img : (H,W) grayscale satellite image
    template: (h,w) grayscale UAV template
    mask    : (h,w) {0,1} or float weights; 1=valid, 0=invalid
    cxs,cys : arrays of particle CENTER coordinates (pixel indices), length N
              (assumed in satellite image coordinate frame)

    returns : (N,) masked ZNCC scores. Particles that would go out-of-bounds -> invalid_value.
    """
    sat = sat_img.astype(np.float32)
    T   = template.astype(np.float32)
    M   = mask.astype(np.float32)

    h, w = T.shape
    if M.shape != (h, w):
        raise ValueError(f"mask shape {M.shape} must match template shape {(h, w)}")

    Wm = float(M.sum())
    if Wm <= 0:
        raise ValueError("mask has no valid pixels (sum(mask)==0)")

    # Masked template mean and zero-mean masked template
    mu_T = (M * T).sum() / (Wm + eps)
    T0   = (T - mu_T) * M
    denom_T = np.sqrt((T0 * T0).sum()) + eps

    # Correlation maps over all possible top-left positions
    num_map = cv2.matchTemplate(sat,       T0, cv2.TM_CCORR)  # (H-h+1, W-w+1)
    S_map   = cv2.matchTemplate(sat,        M, cv2.TM_CCORR)
    S2_map  = cv2.matchTemplate(sat * sat,  M, cv2.TM_CCORR)

    cxs = np.asarray(cxs)
    cys = np.asarray(cys)

    # Convert center -> top-left (define center consistently)
    # For odd w/h: exact. For even w/h: this chooses the "left/top of the two middle pixels".
    x0 = np.rint(cxs).astype(np.int32) - (w // 2)
    y0 = np.rint(cys).astype(np.int32) - (h // 2)

    # Valid top-left ranges for matchTemplate maps
    Hm, Wm_map = num_map.shape  # H-h+1, W-w+1
    valid = (x0 >= 0) & (y0 >= 0) & (x0 < Wm_map) & (y0 < Hm)

    scores = np.full(x0.shape, invalid_value, dtype=np.float32)
    if not np.any(valid):
        return scores

    xv = x0[valid]
    yv = y0[valid]

    num = num_map[yv, xv]
    S   = S_map[yv, xv]
    S2  = S2_map[yv, xv]

    var = S2 - (S * S) / (float(M.sum()) + eps)  # masked variance
    var = np.maximum(var, eps)
    denom_I = np.sqrt(var)

    scores[valid] = num / (denom_I * denom_T)
    return scores