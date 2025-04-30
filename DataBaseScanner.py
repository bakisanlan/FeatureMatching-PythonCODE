import numpy as np
import torch
torch.set_grad_enabled(False)
import cv2
from utils import *

from FeatureDetectorMatcher import FeatureDetectorMatcher 

from Timer import Timer
from LightGlue.lightglue import LightGlue
from LightGlue.lightglue.utils import rbd,numpy_image_to_torch

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

    def __init__(self, FeatureDM = FeatureDetectorMatcher(), snap_dim=(400, 400), AIM=None, 
                 showFeatures = False, showFrame = True,
                 useColorSimilarity = False, batch_mode = True):
        """
        Constructor. In MATLAB, the class had optional arguments via varargin.
        Here we define explicit optional parameters or accept them as needed.
        """
        self.snapDim = snap_dim       # Snapped image dimension [W, H]
        self.AIM = AIM               # AerialImageModel-like object (expects .I and .mp, etc.)
        self.pxRned =  np.array([
                                [ np.cos(np.pi/2), np.sin(np.pi/2), 0],
                                [-np.sin(np.pi/2), np.cos(np.pi/2), 0],
                                [ 0              , 0              , 1]
                                ])
        self.showFeatures = showFeatures
        self.showFrame = showFrame
        self.outmapFrame = cv2.imread('data/particles_out_map.png')
        self.outmapFrame = cv2.cvtColor(self.outmapFrame, cv2.COLOR_BGR2RGB)
        self.useColorSimilarity = useColorSimilarity
        self.batch_mode = batch_mode
        self.partInfo = {'nMostKp': None , 'nMostMatchedKp': None} 


        # Initialize the feature detector and matcher in default mode
        self.FeatureDM = FeatureDM

    def find_likelihood(self, UAVKp, UAVDesc, partImgCenterWorldPos, partYaw, UAVframe = None):
        """
        Equivalent to MATLAB find_likelihood():
        1) Snap images for each particle's hypothetical position.
        2) Compute the number of matched inlier features between UAVImage and each particle image.
        3) Return these values as likelihood array.
        """
        # Snap images for each particle
        # with Timer('desc'):
        if not self.useColorSimilarity:
            LocalParticlesKp, ParticlesKp,ParticlesDesc = self.findParticlesKeypointDescriptors(partImgCenterWorldPos,partYaw)

            # with Timer('match'):
            # Get inlierIdx boolean arrays, one per particle
            inlierIdx = self.FeatureDM.matchFeatures(UAVKp,UAVDesc,ParticlesKp,ParticlesDesc,self.batch_mode)

            # Count matched features
            numMatchedFeaturePart = [np.sum(x) for x in inlierIdx]
            
            # Get most likelihood(the one has most match) part
            # with Timer('part cam'):
            if self.showFrame:
                idx_mostLikelihoodPart    = np.argmax(numMatchedFeaturePart) 
                mostLikelihoodPart        = partImgCenterWorldPos[idx_mostLikelihoodPart,:]
                mostlikelihoodPartLocalKp = LocalParticlesKp[idx_mostLikelihoodPart]
                FramemostLikelihoodPart   = self.snapPartImage(mostLikelihoodPart,partYaw[idx_mostLikelihoodPart],mostlikelihoodPartLocalKp)
                
            else:
                FramemostLikelihoodPart = None
            # Return as numpy array (or just list) for convenience
            
            self.partInfo = {'nMostKp': max([len(x) for x in LocalParticlesKp]) , 'nMostMatchedKp': max(numMatchedFeaturePart)} 
            
            # self.similarity = self.find_color_similarity(UAVframe,partImgCenterWorldPos)  # do not consider yaw[deal later]
            return FramemostLikelihoodPart, numMatchedFeaturePart
        
        else:
            similarity = self.find_color_similarity(UAVframe,partImgCenterWorldPos)  # do not consider yaw[deal later]
            if self.showFrame:
                idx_mostLikelihoodPart    = np.argmax(similarity) 
                mostLikelihoodPart        = partImgCenterWorldPos[idx_mostLikelihoodPart,:]
                mostlikelihoodPartSimilarity = similarity[idx_mostLikelihoodPart]
                FramemostLikelihoodPart   = self.snapPartImage(mostLikelihoodPart,None,None)
                
            else:
                FramemostLikelihoodPart = None
                
            self.partInfo = {'avgSim': np.mean(similarity) , 'MostSim': mostlikelihoodPartSimilarity} 
            
            return FramemostLikelihoodPart, similarity


    def findParticlesKeypointDescriptors(self,particlesWorldPos,particlesYaw):
        """
        Filters keypoints and descriptors that lie within a rotated rectangle.
        
        Parameters:
        - keypoints: List of cv2.KeyPoint objects.
        - descriptors: numpy array of shape (N, D), corresponding descriptors.
        - rect_center: tuple (x_c, y_c), center of the rectangle.
        - rect_size: tuple (w, h), dimensions of the rectangle (width, height).
        - angle: float, rotation angle of the rectangle in degrees (counterclockwise).

        Returns:
        - filtered_keypoints: List of cv2.KeyPoint objects inside the rectangle.
        - filtered_descriptors: numpy array of descriptors corresponding to those keypoints.
        """
        # Convert cv2.KeyPoint objects to NumPy array of coordinates
        
        # Convert from NED world frame to px(u,v)
        # particlesWorldPos NX2 array
        particlesPxPos = ned2px(particlesWorldPos,self.AIM.leftupperNED,self.AIM.mp,self.pxRned)

        w, h = self.snapDim
        N = particlesPxPos.shape[0]

        # Find min-max x,y in particles
        min_x = particlesPxPos[:,0].min() - (w // 2)
        max_x = particlesPxPos[:,0].max() + (w // 2)
        min_y = particlesPxPos[:,1].min() - (h // 2)
        max_y = particlesPxPos[:,1].max() + (h // 2)
        
        reduced_mask = (
                        (self.AIM.keypointBase_np[:, 0] <= max_x) & (self.AIM.keypointBase_np[:, 0] >= min_x) &
                        (self.AIM.keypointBase_np[:, 1] <= max_y) & (self.AIM.keypointBase_np[:, 1] >= min_y)
                        )
        
        # Mask features inside the big rectangle(UAV view) without yaw rotation
        reduced_keypoints = self.AIM.keypointBase_np[reduced_mask]
        reduced_descriptors = self.FeatureDM.MaskFeatures(self.AIM.featuresBase, self.snapDim , reduced_mask)        
            
        ParticlesLocalKeypoints = []            
        ParticlesKeypoints      = []
        ParticlesDescriptors    = []

        # with Timer('dd'):
        for i in range(N):
            yaw = particlesYaw[i]  # rad
            # Compute rotation matrix
            R = np.array([
                [ np.cos(yaw),  np.sin(yaw)],
                [-np.sin(yaw),  np.cos(yaw)]
            ])

            # Shift keypoints to rectangle's center
            shifted_keypoints = reduced_keypoints - particlesPxPos[i,:]

            # Rotate keypoints to rectangle's local frame
            local_keypoints = np.dot(shifted_keypoints, R) 
                    
            inside_mask = (
                (np.abs(local_keypoints[:, 0]) <= w // 2) &
                (np.abs(local_keypoints[:, 1]) <= h // 2)
            )
            
            # Mask inside features
            particle_keypoint        = reduced_keypoints[inside_mask]
            particle_local_keyppoint = local_keypoints[inside_mask]  + np.array([ w // 2, h // 2])    # Particles Local Keypoints (relative keypoints  to uppler left corner of particles px)
            particle_descriptor      = self.FeatureDM.MaskFeatures(reduced_descriptors,self.snapDim, inside_mask)        

            ParticlesKeypoints.append(particle_keypoint)
            ParticlesLocalKeypoints.append(particle_local_keyppoint)                
            ParticlesDescriptors.append(particle_descriptor)
                    
        return ParticlesLocalKeypoints,ParticlesKeypoints,ParticlesDescriptors

    def snapPartImage(self, partWorldPos, yaw, partLocalKp):
        """
        Snap images for each particle’s hypothetical position from the big map,
        and rotate them as needed. Returns a list of images (one per particle).
        In MATLAB, we used cellfun to avoid for loops, but in Python a list
        comprehension is both Pythonic and efficient.
        """
        
        #Convert part pos to pixel
        PartPxPos = ned2px(partWorldPos,self.AIM.leftupperNED,self.AIM.mp, self.pxRned).squeeze() # shape 2,

        w, h = self.snapDim

        # Find min-max x,y in UAV
        min_x = PartPxPos[0] - (w // 2)
        max_x = PartPxPos[0] + (w // 2)
        min_y = PartPxPos[1] - (h // 2)
        max_y = PartPxPos[1] + (h // 2)

        # Get frame 
        PartFrame = self.AIM.I[min_y:max_y, min_x:max_x]
        
        # Return a blank frame if particles are out of the map
        if (PartFrame.size) and (not self.useColorSimilarity):
            PartFrame = rotate_image(PartFrame,yaw)
            
            # Add local local keypoints to particle frame if requested
            if self.showFeatures and (not self.useColorSimilarity):
                PartFrame = drawKeypoints(PartFrame, partLocalKp)
        elif (PartFrame.size) and (self.useColorSimilarity):
            pass
            
        else:
            PartFrame = self.outmapFrame 
                        
        return PartFrame

    def find_color_similarity(self,UAVframe,partImgCenterWorldPos):
        
        ParticlesPxPos = ned2px(partImgCenterWorldPos, self.AIM.leftupperNED, self.AIM.mp, self.pxRned).squeeze() # shape 2,
        
        N = ParticlesPxPos.shape[0]
        similarity = np.zeros(N)
        
        #Compute color histogram of UAV
        bin = 8 
        bins = (bin,bin,bin) # tunable parameter
        histUAV =  compute_colorized_histogram(UAVframe, bins=bins, exclude_black=True)
                
        #Similarity find loop over each particle
        for i in range(N):
            
            w, h = self.snapDim

            # Find min-max x,y of Particle to be cropped
            min_x = ParticlesPxPos[i][0] - (w // 2)
            max_x = ParticlesPxPos[i][0] + (w // 2)
            min_y = ParticlesPxPos[i][1] - (h // 2)
            max_y = ParticlesPxPos[i][1] + (h // 2)

            # Get frame of particle
            PartFrame = self.AIM.I[min_y:max_y, min_x:max_x]

            # Find color similarity of each particles and UAV image
            histParticle =  compute_colorized_histogram(PartFrame, bins=bins, exclude_black=True)
            similarity[i] = color_similarity(histUAV, histParticle)

        return similarity
            
        