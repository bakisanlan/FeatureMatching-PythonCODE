import numpy as np
import torch
torch.set_grad_enabled(False)
import cv2
import os
import logging
import time
from utils import ned2px, extract_rotated_patch_optimized, drawKeypoints, rotate_image
from Timer import Timer
# from FeatureDetectorMatcher import FeatureDetectorMatcher 


logger = logging.getLogger(__name__)


def estgeotform2d(src_points, dst_points, transform_type="similarity", ransacReprojThreshold=5.0):
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
    return M, inlier_mask, None

class DatabaseScanner:
    """
    Python equivalent of the MATLAB DatabaseScanner class.

    This class is used by State Estimators (e.g., PF/MPF) for
    getting measurements (images) of UAV or particles through
    scanning an offline database (satellite image).
    """

    def __init__(self, FeatureDM = None, snapDim=(400, 400), AIM=None, 
                 showFeatures = False, showFrame = True,
                 batch_mode = True):
        """
        Constructor. In MATLAB, the class had optional arguments via varargin.
        Here we define explicit optional parameters or accept them as needed.
        """
        self.snapDim = snapDim       # Snapped image dimension [W, H]
        self.AIM = AIM               # AerialImageModel-like object (expects .I and .mp, etc.)
        self.pxRned =  np.array([
                                [ np.cos(np.pi/2), np.sin(np.pi/2), 0],
                                [-np.sin(np.pi/2), np.cos(np.pi/2), 0],
                                [ 0              , 0              , 1]
                                ])
        self.showFeatures = showFeatures
        self.showFrame = showFrame
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.outmapFrame = cv2.imread(os.path.join(script_dir, '..', 'data', 'particles_out_map.png'))
        self.outmapFrame = cv2.cvtColor(self.outmapFrame, cv2.COLOR_BGR2RGB)
        self.batch_mode = batch_mode
        self.partInfo = {'nMostKp': None , 'nMostMatchedKp': None} 

        # Throttle for occasional status logs (avoid spamming at high-rate).
        self._last_snap_log_time = 0.0
        self.snap_log_interval_s = 2.0

        # Initialize the feature detector and matcher in default mode
        self.FeatureDM = FeatureDM

    def find_likelihood(self, partImgCenterWorldPos, partYaw, UAVKp = None, UAVDesc = None, UAVFrame = None, MaskOrthography = None):
        """
        Equivalent to MATLAB find_likelihood():
        1) Snap images for each particle's hypothetical position.
        2) Compute the number of matched inlier features between UAVImage and each particle image.
        3) Return these values as likelihood array.
        """

        # Get flag of TemplateMatching with checking inputs of function
        TemplateMatchingFlag = self.FeatureDM.TemplateMatchingFlag

        # Particle kp/view extraction for Image Matching
        # ParticlesRotatedMaskOrtho = None
        with Timer('Particle kp/view extraction'):
            # Extract view of particles when template matching is used(UAVKp or UAVDesc is None)
            if TemplateMatchingFlag:
                ParticlesPatches              = self.findParticlesPatch(partImgCenterWorldPos,partYaw,MaskOrthography)   # NOTE: MaskOrthography check if it is [0,1]

                # self.FeatureDM.masked_zncc_particles_from_satellite_map_center(self.AIM.Igray, UAVFrame, MaskOrthography, partImgCenterWorldPos)  # NOTE: Try to impelement

            # Extract keypoints and descriptors of particles when feature matching is used
            else:
                # No orthoprojection mask is given
                if MaskOrthography is None:
                    ParticlesKp,ParticlesDesc = self.findParticlesKeypointDescriptors(partImgCenterWorldPos,partYaw)
                # Orthoprojection mask is given
                else:
                    ParticlesKp,ParticlesDesc = self.findParticlesKeypointDescriptorsOrtho(partImgCenterWorldPos,partYaw,MaskOrthography)   # NOTE: If orthoprojection mask is given use yaw as error yaw

        # Image Matching step
        if TemplateMatchingFlag:

            with Timer('Image Matching using ZNCC'):        

                # ZNCC matching
                UAVFrame = cv2.GaussianBlur(UAVFrame, (0, 0), sigmaX=0.8, sigmaY=0.8)  # Blur UAV frame to reduce noise effects to improve ZNCC matching
                ScoreParticles = self.FeatureDM.znccMatch(ParticlesPatches, UAVFrame, MaskOrthography)

                # Print and store info
                maxScore  = max(ScoreParticles)
                minScore  = min(ScoreParticles)
                meanScore = np.mean(ScoreParticles)
                self.partInfo = {'maxScore': maxScore , 'minScore': minScore, 'meanScore': meanScore}
                # print(f"maxScore: {maxScore}    minScore: {minScore}    meanScore: {meanScore}")

        else:

            with Timer('Image Matching using {} Features'.format(self.FeatureDM.detector_type)):
                # Feature matching
                # Get inlierIdx boolean arrays, one per particle
                inlierIdx = self.FeatureDM.matchFeatures(UAVKp,UAVDesc,ParticlesKp,ParticlesDesc,self.batch_mode)
                # Count matched features to find score for each particle
                ScoreParticles = [np.sum(x) for x in inlierIdx]

                # Print and store info
                maxScore = max(ScoreParticles)
                nMostKp = max([len(x) for x in ParticlesKp])
                self.partInfo = {'nMostKp': nMostKp , 'maxScore': maxScore} 
                # print(f"nMostKp: {nMostKp}    nMostMatchedKp: {maxScore}")

        
        # Get most likelihood(the one has most score) particle
        FramemostLikelihoodPart = None
        if self.showFrame:
            idx_mostLikelihoodPart               = np.argmax(ScoreParticles) 
            mostLikelihoodPartCenterWorldPos     = partImgCenterWorldPos[idx_mostLikelihoodPart,:]
            mostLikelihoodPartYaw                = partYaw[idx_mostLikelihoodPart]
            mostlikelihoodPartKp                 = None if TemplateMatchingFlag else ParticlesKp[idx_mostLikelihoodPart]
            # mostlikelihoodPartRotatedMaskOrtho   = None if ParticlesRotatedMaskOrtho is None else ParticlesRotatedMaskOrtho[idx_mostLikelihoodPart]
            FramemostLikelihoodPart              = self.snapPartImage(mostLikelihoodPartCenterWorldPos,mostLikelihoodPartYaw,mostlikelihoodPartKp, MaskOrthography)

        return FramemostLikelihoodPart, ScoreParticles
        

    def findParticlesKeypointDescriptors(self,particlesWorldPos,particlesYaw):
        """
        Filters keypoints and descriptors that lie within a rotated rectangle.
        
        Parameters:
        - keypoints: List of cv2.KeyPoint objects.
        - descriptors: numpy array of shape (N, D), corresponding descriptors.
        - rect_center: tuple (x_c, y_c), center of the rectangle.
        - rect_size: tuple (w, h), dimensions of the rectangle (width, height).\
        - angle: float, rotation angle of the rectangle in degrees (counterclockwise).

        Returns:
        - filtered_keypoints: List of cv2.KeyPoint objects inside the rectangle.
        - filtered_descriptors: numpy array of descriptors corresponding to those keypoints.
        """
        # Convert from NED world frame to px(u,v)
        particlesPxPos = ned2px(particlesWorldPos, self.AIM.leftupperNED, self.AIM.mp, self.pxRned)

        w, h = self.snapDim
        turn_radius = np.sqrt((w/2)**2 + (h/2)**2)
        N = particlesPxPos.shape[0]
        
        # Find min-max x,y in particles for bounding box
        min_x = particlesPxPos[:,0].min() - turn_radius
        max_x = particlesPxPos[:,0].max() + turn_radius
        min_y = particlesPxPos[:,1].min() - turn_radius
        max_y = particlesPxPos[:,1].max() + turn_radius

        # Reduce keypoints to those within the bounding box
        reduced_mask = (
            (self.AIM.keypointBase_np[:, 0] <= max_x) & (self.AIM.keypointBase_np[:, 0] >= min_x) &
            (self.AIM.keypointBase_np[:, 1] <= max_y) & (self.AIM.keypointBase_np[:, 1] >= min_y)
        )
        
        reduced_keypoints_np, reduced_descriptors = self.FeatureDM.MaskFeatures(
            self.AIM.featuresBase, self.AIM.keypointBase_np, self.snapDim, reduced_mask
        )
        
        K = reduced_keypoints_np.shape[0]  # Number of reduced keypoints
        
        # === VECTORIZED: Pre-compute all rotation matrices (N, 2, 2) ===
        cos_yaw = np.cos(particlesYaw)  # (N,)
        sin_yaw = np.sin(particlesYaw)  # (N,)
        R_all = np.stack([
            np.stack([cos_yaw, sin_yaw], axis=-1),
            np.stack([-sin_yaw, cos_yaw], axis=-1)
        ], axis=1)  # (N, 2, 2)
        
        # === VECTORIZED: Shift keypoints relative to each particle center ===
        # reduced_keypoints_np: (K, 2), particlesPxPos: (N, 2)
        # shifted: (N, K, 2)
        shifted_keypoints = reduced_keypoints_np[np.newaxis, :, :] - particlesPxPos[:, np.newaxis, :]
        
        # === VECTORIZED: Rotate keypoints using einsum ===
        # R_all: (N, 2, 2), shifted_keypoints: (N, K, 2)
        # local_keypoints: (N, K, 2)
        local_keypoints = np.einsum('nij,nkj->nki', R_all, shifted_keypoints)
        
        # === VECTORIZED: Compute inside masks for all particles ===
        # inside_masks: (N, K) boolean array
        inside_masks = (
            (np.abs(local_keypoints[:, :, 0]) <= w // 2) &
            (np.abs(local_keypoints[:, :, 1]) <= h // 2)
        )
        
        # === Loop only for MaskFeatures (variable output sizes) ===
        ParticlesKeypoints = []
        ParticlesDescriptors = []
        
        for i in range(N):
            particle_keypoint, particle_descriptor = self.FeatureDM.MaskFeatures(
                reduced_descriptors, reduced_keypoints_np, self.snapDim,
                inside_masks[i], LocalKp=local_keypoints[i]
            )
            ParticlesKeypoints.append(particle_keypoint)
            ParticlesDescriptors.append(particle_descriptor)

        return ParticlesKeypoints, ParticlesDescriptors

    def findParticlesKeypointDescriptorsOrtho(self,particlesWorldPos,particlesYawError,MaskOrthography):
        """
        Filters keypoints and descriptors that lie within a rotated rectangle with orthography mask.
        
        Parameters:
        - keypoints: List of cv2.KeyPoint objects.
        - descriptors: numpy array of shape (N, D), corresponding descriptors.
        - rect_center: tuple (x_c, y_c), center of the rectangle.
        - rect_size: tuple (w, h), dimensions of the rectangle (width, height).
        - angle: float, rotation angle of the rectangle in degrees (counterclockwise).
        - MaskOrthography: Mask matrix to filter keypoints from UAV orthoprojected view

        Returns:
        - filtered_keypoints: List of cv2.KeyPoint objects inside the rectangle.
        - filtered_descriptors: numpy array of descriptors corresponding to those keypoints.
        """
        # Convert from NED world frame to px(u,v)
        particlesPxPos = ned2px(particlesWorldPos, self.AIM.leftupperNED, self.AIM.mp, self.pxRned)

        self.snapDim = MaskOrthography.shape[::-1]  # w,h
        w, h = self.snapDim
        turn_radius = np.sqrt((w/2)**2 + (h/2)**2)
        N = particlesPxPos.shape[0]

        # Find min-max x,y in particles for bounding box
        min_x = particlesPxPos[:,0].min() - turn_radius
        max_x = particlesPxPos[:,0].max() + turn_radius
        min_y = particlesPxPos[:,1].min() - turn_radius
        max_y = particlesPxPos[:,1].max() + turn_radius

        # Reduce keypoints to those within the bounding box
        reduced_mask = (
            (self.AIM.keypointBase_np[:, 0] <= max_x) & (self.AIM.keypointBase_np[:, 0] >= min_x) &
            (self.AIM.keypointBase_np[:, 1] <= max_y) & (self.AIM.keypointBase_np[:, 1] >= min_y)
        )
        
        reduced_keypoints_np, reduced_descriptors = self.FeatureDM.MaskFeatures(
            self.AIM.featuresBase, self.AIM.keypointBase_np, self.snapDim, reduced_mask
        )
        
        K = reduced_keypoints_np.shape[0]  # Number of reduced keypoints
        
        # === VECTORIZED: Pre-compute all rotation matrices (N, 2, 2) ===
        cos_yaw = np.cos(particlesYawError)  # (N,)
        sin_yaw = np.sin(particlesYawError)  # (N,)
        R_all = np.stack([
            np.stack([cos_yaw, sin_yaw], axis=-1),
            np.stack([-sin_yaw, cos_yaw], axis=-1)
        ], axis=1)  # (N, 2, 2)
        
        # === VECTORIZED: Shift keypoints relative to each particle center ===
        shifted_keypoints = reduced_keypoints_np[np.newaxis, :, :] - particlesPxPos[:, np.newaxis, :]
        
        # === VECTORIZED: Rotate keypoints using einsum ===
        local_keypoints = np.einsum('nij,nkj->nki', R_all, shifted_keypoints)
        
        # === VECTORIZED: Convert to image frame coordinates ===
        local_keypoints_imgframe = local_keypoints + np.array([w // 2, h // 2])
        xy_all = np.floor(local_keypoints_imgframe).astype(np.int32)  # (N, K, 2)
        
        # === VECTORIZED: Compute bounds check for all particles ===
        x_all = xy_all[:, :, 0]  # (N, K)
        y_all = xy_all[:, :, 1]  # (N, K)
        bounds_mask = (x_all >= 1) & (x_all < w-1) & (y_all >= 1) & (y_all < h-1)
        
        # === Loop for mask rotation and final masking (unavoidable due to rotate_image) ===
        ParticlesKeypoints = []
        ParticlesDescriptors = []
        
        for i in range(N):
            # Rotate orthography mask for this particle
            rotated_mask_ortho = rotate_image(MaskOrthography, particlesYawError[i])
            
            # Get clipped indices for mask lookup
            x_clipped = x_all[i].clip(0, w-1)
            y_clipped = y_all[i].clip(0, h-1)
            
            # Combine bounds check with mask lookup
            inside_mask = bounds_mask[i] & rotated_mask_ortho[y_clipped, x_clipped].astype(bool)
            
            particle_keypoint, particle_descriptor = self.FeatureDM.MaskFeatures(
                reduced_descriptors, reduced_keypoints_np, self.snapDim,
                inside_mask, LocalKp=local_keypoints[i]
            )
            
            ParticlesKeypoints.append(particle_keypoint)
            ParticlesDescriptors.append(particle_descriptor)

        return ParticlesKeypoints, ParticlesDescriptors


    def snapPartImage(self, partWorldPos, yaw, partLocalKp = None, MaskOrthography=None):
        """
        Snap images for each particle’s hypothetical position from the big map,
        and rotate them as needed. Returns a list of images (one per particle).
        In MATLAB, we used cellfun to avoid for loops, but in Python a list
        comprehension is both Pythonic and efficient.
        """
        
        #Convert part pos to pixel
        PartPxPos = ned2px(partWorldPos,self.AIM.leftupperNED,self.AIM.mp, self.pxRned).squeeze() # shape 2,

        w, h = self.snapDim

        # This can be called many times per second. Keep it debug + throttled.
        now = time.time()
        if (now - self._last_snap_log_time) >= self.snap_log_interval_s:
            logger.debug("Snapped particle view size: %dx%d", w, h)
            self._last_snap_log_time = now
        
        # Return a blank frame if particles are out of the map
        if (PartPxPos[0] <= self.AIM.I.shape[1] - w//2) and (PartPxPos[1] <= self.AIM.I.shape[0] - h//2) and \
           (PartPxPos[0] >= w//2) and (PartPxPos[1] >= h//2):
            # PartFrame = rotate_image(PartFrame,yaw)
            
            PartFrame = extract_rotated_patch_optimized(
                                                        self.AIM.Igray,
                                                        tuple(PartPxPos),
                                                        tuple(self.snapDim),
                                                        np.rad2deg(yaw)
                                                        )
            
            # Add local local keypoints to particle frame if requested
            if self.showFeatures and partLocalKp is not None:
                PartFrame = drawKeypoints(PartFrame, partLocalKp)

            if MaskOrthography is not None:
                PartFrame = cv2.bitwise_and(PartFrame, PartFrame, mask=MaskOrthography)
        else:
            PartFrame = self.outmapFrame 
                        
        return PartFrame


    def findParticlesPatch(self, particlesWorldPos, particlesYaw, MaskOrthography):
        """
        Extract grayscale patches from satellite map for ZNCC matching.
        
        Parameters
        ----------
        particlesWorldPos : np.ndarray, shape (N, 2) or (N, 3)
            Particle positions in NED world frame (only x, y are used).
        particlesYaw : np.ndarray, shape (N,)
            Yaw angles for each particle in radians.
        MaskOrthography : np.ndarray, shape (H, W)
            Binary mask from orthoprojection (255=valid, 0=invalid).
            
        Returns
        -------
        ParticlesPatches : np.ndarray, shape (N, H, W)
            Grayscale patches extracted for each particle (masked).
        ParticlesRotatedMaskOrtho : list of np.ndarray
            Rotated orthography masks for each particle, each shape (H, W).
        """
        # Convert from NED world frame to pixel coordinates
        particlesPxPos = ned2px(
            particlesWorldPos,
            self.AIM.leftupperNED,
            self.AIM.mp,
            self.pxRned
        )
        
        # Update snap dimension based on mask size
        self.snapDim = MaskOrthography.shape[::-1]  # (w, h)
        w, h = self.snapDim
        
        N = particlesPxPos.shape[0]  #get number of particles
        
        # Initialize output arrays
        ParticlesPatches = np.zeros((N, h, w), dtype=np.uint8)
        # ParticlesRotatedMaskOrtho = []
        
        # Convert satellite image to grayscale if needed
        satellite_gray = self.AIM.Igray
        
        # Extract patch for each particle
        for i in range(N):
            center_px = tuple(particlesPxPos[i, :])
            yaw_rad = particlesYaw[i]
            
            # Extract rotated grayscale patch from satellite image
            patch = extract_rotated_patch_optimized(
                satellite_gray,
                center_px,
                (w, h),
                np.rad2deg(yaw_rad)
            )
            
            # Rotate the orthography mask to match particle yaw
            # rotated_mask = rotate_image(MaskOrthography, yaw_rad)
            # rotated_mask = MaskOrthography
            
            # Apply mask to the patch (zero out invalid regions)
            masked_patch = cv2.bitwise_and(patch, patch, mask=MaskOrthography)
            
            # Store results
            ParticlesPatches[i] = masked_patch
            # ParticlesRotatedMaskOrtho.append(rotated_mask)
        
        return ParticlesPatches #, ParticlesRotatedMaskOrtho
