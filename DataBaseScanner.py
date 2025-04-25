import numpy as np
import torch
torch.set_grad_enabled(False)
import cv2
from utils import *


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

    def __init__(self, snap_dim=(400, 400), num_level=1, AIM=None, 
                 showFeatures = False, showFrame = True,
                 useColorSimilarity = False, batch_mode = True):
        """
        Constructor. In MATLAB, the class had optional arguments via varargin.
        Here we define explicit optional parameters or accept them as needed.
        """
        self.snapDim = snap_dim       # Snapped image dimension [W, H]
        self.num_level = num_level    # ORB num levels
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

        if self.AIM.detector == 'ORB':
            self.Matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        elif self.AIM.detector == 'SP':
            self.Matcher = LightGlue(features='superpoint', depth_confidence=0.9, width_confidence=0.95).eval().to(self.AIM.device)  
            # self.Matcher = LightGlue(features='sift', depth_confidence=0.9, width_confidence=0.95).eval().to(self.AIM.device)  

            # self.Matcher.compile(mode='reduce-overhead')


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

            # Get inlierIdx boolean arrays, one per particle
            # with Timer('mathcin'):
            inlierIdx = self.ImageMatching(UAVKp,UAVDesc,ParticlesKp,ParticlesDesc)

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
        
        # aug_keypoints_np = [kp for kp, mask in zip(keypoints_np, reduced_mask) if mask]
        reduced_keypoints = self.AIM.keypointBase_np[reduced_mask]
        if self.AIM.detector == 'ORB':
            reduced_descriptors = self.AIM.featuresBase[reduced_mask]
        elif self.AIM.detector == 'SP':
            keypoints, keypoint_scores, descriptors, image_size = self.AIM.featuresBase["keypoints"][:,reduced_mask,:] ,    \
                                                                  self.AIM.featuresBase["keypoint_scores"][:,reduced_mask], \
                                                                  self.AIM.featuresBase["descriptors"][:,reduced_mask,:] ,  \
                                                                  self.AIM.featuresBase["image_size"]
            image_size = torch.from_numpy(np.array([w,h])[np.newaxis, : ]).to(self.AIM.device)
            # reduced_descriptors = {"keypoints" : keypoints, "keypoint_scores" : keypoint_scores, "descriptors" : descriptors , "image_size" : image_size}
            
            # scales, oris =  self.AIM.featuresBase["scales"][:,reduced_mask] , self.AIM.featuresBase["oris"][:,reduced_mask]
            reduced_descriptors = {"keypoints"   : keypoints,    "keypoint_scores" : keypoint_scores, 
                                   "descriptors" : descriptors , "image_size"      : image_size}#, "scales" : scales, "oris" : oris}
            

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
            if self.AIM.detector == 'ORB':
                particle_descriptor = reduced_descriptors[inside_mask]
            elif self.AIM.detector == 'SP':
                keypoints, keypoint_scores, descriptors, image_size = reduced_descriptors["keypoints"][:,inside_mask,:] , reduced_descriptors["keypoint_scores"][:,inside_mask], reduced_descriptors["descriptors"][:,inside_mask,:] , reduced_descriptors["image_size"]
                image_size = torch.from_numpy(np.array([w,h])[np.newaxis, : ]).to(self.AIM.device)
                # particle_descriptor = {"keypoints" : keypoints, "keypoint_scores" : keypoint_scores, "descriptors" : descriptors , "image_size" : image_size}

                # scales, oris =  reduced_descriptors["scales"][:,inside_mask] , reduced_descriptors["oris"][:,inside_mask]
                particle_descriptor = {"keypoints" : keypoints, "keypoint_scores" : keypoint_scores, "descriptors" : descriptors , "image_size" : image_size}#, "scales" : scales, "oris" : oris}

            ParticlesLocalKeypoints.append(particle_local_keyppoint)                
            ParticlesKeypoints.append(particle_keypoint)
            ParticlesDescriptors.append(particle_descriptor)
                    
        return ParticlesLocalKeypoints,ParticlesKeypoints,ParticlesDescriptors

    def ImageMatching(self,UAVKp,UAVDesc,ParticlesKp,ParticlesDesc):
        """
        Return a list of boolean arrays (inlierIdx) indicating inlier matches
        for each particle’s image vs. the UAVimage.

        Steps in MATLAB:
          [featuresUAV, validpointsUAV] = extractFeatures(UAVimage, detectORBFeatures(UAVimage,...))
          ...
          indexPairs = cellfun(@(x) matchFeatures(featuresUAV,x),featuresPart,'UniformOutput',false)
          ...
          [~,inlierIdx,~] = cellfun(@(PartValid,idxPair) estgeotform2d(...), validpointsPart,indexPairs, ...)
        """

        #For each particle, match features
        indexPairsList = []

        # Batch mode for LightGlue Feature Matching
        if self.batch_mode == True and self.AIM.detector == 'SP':
            indexPairsList = self.matchFeatures(UAVDesc,ParticlesDesc,self.batch_mode)

        # Single mode for OpenCV and LightGlue Feature Matching
        else:
            for PartDesc in ParticlesDesc:
                try:
                    indexPairs = self.matchFeatures(UAVDesc,PartDesc)
                except Exception:
                #     # Emulate "ErrorHandler": return empty
                    indexPairs = np.empty((0, 2), dtype=int)
                indexPairsList.append(indexPairs)

        # Get inliers
        inlierIdxList = []
        for PartKp, idxPairs in zip(ParticlesKp, indexPairsList):
            # Convert the matched keypoints to (x,y) arrays
            if idxPairs.shape[0] == 0:
                inlierIdxList.append(np.array([], dtype=bool))
                continue

            src_pts = PartKp[idxPairs[:,1]]
            dst_pts = UAVKp[idxPairs[:,0]]

            try:
                _, inliers, _ = estgeotform2d(src_pts, dst_pts, transform_type="similarity")
            except Exception:
                # Emulate "ErrorHandler"
                inliers = np.array([], dtype=bool)
            inlierIdxList.append(inliers)
                
        return inlierIdxList


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
                # Convert to an Nx2 array: [ (i_idx1, i_idx2), ... ]
                index_pairs = np.array([[m.queryIdx, m.trainIdx] for m in matches], dtype=int)

            elif self.AIM.detector == 'SP':
                matches = self.Matcher({"image0": UAV_desc, "image1": part_desc})
                UAV_desc, part_desc, matches = [rbd(x) for x in [UAV_desc, part_desc, matches]]  # remove batch dimension
                #kpts0, kpts1, matches = feats0["keypoints"], feats1["keypoints"], matches01["matches"]
                #m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]
                index_pairs = matches["matches"].cpu().numpy()

        # Sort matches by distance
        # matches = sorted(matches, key=lambda x: x.distance)

        return index_pairs

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

            # if self.AIM.detector == 'ORB':
            #     keypoints, descriptors = self.AIM.ORB.detectAndCompute(rotated, None)
            #     PartFeaturesFrame = cv2.drawKeypoints(rotated, keypoints, None, color=(0,255,0), flags=0)
            #     # PartFeaturesFrame = drawKeypoints(rotated, partLocalKp)

            # elif self.AIM.detector == 'SP':
            #     feat = self.AIM.SP.extract(numpy_image_to_torch(rotated).to(self.AIM.device))
            #     keypoints, descriptors = feat["keypoints"] , feat
            #     keypoints_np = keypoints.cpu().numpy().squeeze()
            #     PartFeaturesFrame = drawKeypoints(rotated, keypoints_np)
            
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
            
        