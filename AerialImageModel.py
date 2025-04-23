import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from LightGlue.lightglue import SuperPoint,LightGlue, SIFT
from LightGlue.lightglue.utils import numpy_image_to_torch
import torch
import pickle
torch.set_grad_enabled(False)

class AerialImageModel:
    """
    Python equivalent of the MATLAB AerialImageModel class.

    Manages a Satellite/Aerial Image for terrain scan and matching tasks.
    """

    def __init__(self, area, num_level = 1, nfeatures= 2000000, detector = 'SP', preFeatureFlag = 1):
        """
        Constructor. Loads the corresponding satellite image file
        based on the 'area' parameter (e.g., "ITU").
        """
        self.mapDim          = None
        self.num_level       = num_level
        self.nfeatures       = nfeatures
        self.detector        = detector
        self.I               = None              # Will hold the base image in grayscale (float).
        self.mp              = None              # Meters/pixel ratio.
        self.leftupperNED    = None              # Most left upper pixel NED position
        self.featuresBase    = None
        self.keypointBase    = None
        self.keypointBase_np = None
        self.preFeatureFlag  = preFeatureFlag
        

        # Load data depending on the area
        if area.lower() == 'unity_fl1':
            # Construct path to the image (adjust as needed for your directory structure).
            # For example, if your data folder is next to this script:
            self.leftupperNED = np.array([10000,-10000])
            self.mp = 5
            script_dir = os.path.dirname(os.path.abspath(__file__))
            data_path = os.path.join(script_dir, 'data', 'OrtoMap.png')
            self.loadData(data_path)
        elif area.lower() == 'itu':
            self.nfeatures = 200000
            # self.mp = 725 / 4320   #itu_map_square.jpg
            self.mp  = 881 / 4320    #itusat.jpg 
            self.leftupperNED = np.array([self.mp*4320*0.5, -self.mp*4320*0.5, 0]) #this left upper is the [0,0] pixel position which is reference point for NED calculation,  center of the image is [0,0] N,E , LLA_left_upper,itusat.jpg =  [41.108116,  29.018083]
            # self.leftupperNED = np.array([0, 0 , 0]) 
            script_dir = os.path.dirname(os.path.abspath(__file__))
            data_path = os.path.join(script_dir, 'data', 'itu_sat.jpg')
            self.loadData(data_path)

        else:
            raise ValueError("Enter a valid area name (e.g. 'ITU').")
        
    def loadData(self, filename):
        """
        loadData: Load data from file, store in self.I as a float grayscale image,
        compute mapDim and mp, and detect ORB features as the base feature set.
        """
        # Read the image in grayscale with OpenCV
        self.I     = cv2.imread(filename)
        self.I     = cv2.cvtColor(self.I, cv2.COLOR_BGR2RGB)

        self.Igray = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)

        if self.I is None:
            raise IOError(f"Could not read image file: {filename}")

        self.mapDim = self.Igray.shape[:2]  # (height, width) in Python/NumPy
        
        
        # Feature Detector Create
        if self.detector == 'ORB':
            self.ORB    = cv2.ORB_create(nfeatures= self.nfeatures, nlevels = self.num_level)
            self.ORBcam = cv2.ORB_create(nfeatures= 430, nlevels = self.num_level, edgeThreshold = 5)  # Create a camera ORB detector

            # Detect ORB features for the base image
            keypoints, descriptors = self.ORB.detectAndCompute(self.Igray, None)
            keypoints_np = np.array([kp.pt for kp in keypoints])
            
        elif self.detector == 'SP':
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
            print(self.device)
            self.SP = SuperPoint(max_num_keypoints=2048).eval().to(self.device)  # load the extractor
            # self.SP    = SIFT(max_num_keypoints=2000).eval().to(self.device)  # load the extractor
            self.SPcam = SuperPoint(max_num_keypoints=2048).eval().to(self.device)  # load the extractor
            
            # Load pre-extracted features if preFeatureFlag is set
            if self.preFeatureFlag == 1:
                with open('data/feature_map/2048/descriptors.pkl', 'rb') as f:
                    descriptors = pickle.load(f)  
    
                with open('data/feature_map/2048/keypoints.pkl', 'rb') as f:
                    keypoints = pickle.load(f)  

                with open('data/feature_map/2048/keypoints_np.pkl', 'rb') as f:
                    keypoints_np = pickle.load(f)                  
                    
            # Extract features from the image and store them if preFeatureFlag is not set
            else:
                crop_size = 256
                h, w = self.mapDim[0:2]
                
                step_x = w // crop_size
                step_y = h // crop_size
                
                for i in range(step_y):
                    for j in range(step_x):
                        
                        cropped = self.I[i*crop_size:(i+1)*crop_size, j*crop_size:(j+1)*crop_size,:]
                        
                        feat = self.SP.extract(numpy_image_to_torch(cropped).to(self.device))
                        feat['keypoints'] = (feat['keypoints'].cpu().numpy().squeeze() + np.array([j*crop_size,i*crop_size])).astype(np.float32)   
                        feat['keypoint_scores'] =   feat['keypoint_scores'].cpu().numpy().squeeze().astype(np.float32)
                        feat['scales'] =   feat['scales'].cpu().numpy().squeeze().astype(np.float32)
                        feat['oris'] =   feat['oris'].cpu().numpy().squeeze().astype(np.float32)
                        feat['descriptors'] = feat['descriptors'].cpu().numpy().squeeze().astype(np.float32)
                        
                        if 'all_keypoints' not in locals():
                            all_keypoints       = feat['keypoints']
                            all_keypoint_scores = feat['keypoint_scores']
                            all_scales          = feat['scales']
                            all_oris            = feat['oris']
                            all_descriptors     = feat['descriptors']
                        else:
                            all_keypoints        = np.vstack((all_keypoints, feat['keypoints']))
                            all_keypoint_scores  = np.hstack((all_keypoint_scores, feat['keypoint_scores']))
                            all_scales           = np.hstack((all_scales, feat['scales']))
                            all_oris             = np.hstack((all_oris, feat['oris']))
                            all_descriptors      = np.vstack((all_descriptors, feat['descriptors']))

                all_keypoints         = torch.tensor(all_keypoints, device=self.device).unsqueeze(0)
                all_keypoint_scores   = torch.tensor(all_keypoint_scores, device=self.device).unsqueeze(0)
                all_scales            = torch.tensor(all_scales, device=self.device).unsqueeze(0)
                all_oris              = torch.tensor(all_oris, device=self.device).unsqueeze(0)
                all_descriptors       = torch.tensor(all_descriptors, device=self.device).unsqueeze(0)
                        
                feat = {'keypoints' : all_keypoints, 'keypoint_scores' : all_keypoint_scores, 'scales' : all_scales , 
                        'oris' : all_oris ,  'descriptors' : all_descriptors , 'image_size': torch.tensor(np.array([h,w],dtype=np.float32),device= self.device).unsqueeze(0)}
                keypoints, descriptors = feat["keypoints"] , feat
                keypoints_np = keypoints.cpu().numpy().squeeze()
        
        self.featuresBase = descriptors
        self.keypointBase = keypoints
        self.keypointBase_np = keypoints_np
        print(f'Base Image Loaded with ' + str(keypoints_np.shape[0]) + ' features')

    def slice(self, xmin, ymin, width, height, grayFlag = 0):
        """
        Return a slice of the loaded image data, analogous to imcrop in MATLAB.

        Note that in MATLAB, 'imcrop(obj.I, [xmin,ymin,width,height])' expects
        (xmin,ymin) in x=columns, y=rows space. In NumPy, indexing is [row, col].
        So we treat xmin, ymin carefully:

        - We interpret 'xmin' as column index,
          'ymin' as row index,
          'width' as number of columns,
          'height' as number of rows.
        """
        x_end = xmin + width
        y_end = ymin + height

        # Clip to image boundaries
        y_end = min(y_end, self.mapDim[0])
        x_end = min(x_end, self.mapDim[1])

        # Crop the image
        # self.I is shape (rows, cols)
        if grayFlag == 0:
            Icropped = self.I[ymin:y_end, xmin:x_end]
            
        else:
            Icropped = self.Igray[ymin:y_end, xmin:x_end]

        return Icropped

    def visualizeBaseAerialImage(self):
        """
        Show the entire base image in a matplotlib figure .
        """
        plt.figure()
        img = self.I[...,::-1] #matplotlib imshow read BGR order
        plt.imshow(img)
        plt.title('Base Aerial Image')
        plt.axis('off')
        plt.show()

    def visualizeSlicedAerialImage(self, xmin, ymin, width, height):
        """
        Visualize a slice of the loaded image data.
        """
        Icropped = self.slice(xmin, ymin, width, height)
        img = Icropped[...,::-1]
        plt.figure()
        plt.imshow(img)
        plt.title('Sliced Aerial Image')
        plt.axis('off')
        plt.show()

    def visualizeSlicedGrayAerialImage(self, xmin, ymin, width, height):
        """
        Visualize a slice of the loaded image data in gray color.
        (In your MATLAB code, the image is already gray, so this is effectively the same.)
        """
        Icropped = self.slice(xmin, ymin, width, height, grayFlag = 1)
        plt.figure()
        plt.imshow(Icropped, cmap='gray')
        plt.title('Sliced Gray Aerial Image')
        plt.axis('off')
        plt.show()
        
    def visualizeFeaturesAerialImage(self):  #DEAL LATER
        keypoints, descriptors = detectORBFeatures(self.ORB,self.Igray)
        
        featuresIMG = cv2.drawKeypoints(self.Igray, keypoints, None, color=(0,255,0), flags=0)
        plt.figure()
        plt.imshow(featuresIMG)
        plt.title('Detected Features of Aerial Image Database')
        plt.axis('off')
        plt.show()

