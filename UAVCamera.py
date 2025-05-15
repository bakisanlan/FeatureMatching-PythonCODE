import sys
import os

# Get the absolute path of the CUT(GAN) folder 
cut_path      = os.path.abspath(os.path.join(os.path.dirname(__file__), "CUT"))
cyclegan_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "CycleGAN_Turbo"))
# Add CUT to system path
sys.path.append(cut_path)
sys.path.append(cyclegan_path)

# from options.test_options import TestOptions
# from models import create_model
import torch
torch.set_grad_enabled(False)
from torchvision import transforms
import numpy as np
import cv2
from utils import *
from PIL import Image
from LightGlue.lightglue.utils import numpy_image_to_torch
from FeatureDetectorMatcher import FeatureDetectorMatcher 

# from src.cyclegan_turbo import CycleGAN_Turbo
# from src.my_utils.training_utils import build_transform

class UAVCamera:
    """
    This class represents a UAV camera that can process video frames and extract features.
    It can work with live video or pre-recorded video files. The class also supports the use of a GAN model for image generation.
    The class is designed to be used in conjunction with a database of aerial images and their corresponding features.
    The class provides methods to load video frames, process them, and extract keypoints and descriptors using different feature detectors.
    The class also provides methods to snap images from the UAV camera and show the features on the frames.
    """

    def __init__(self, FeatureDM = FeatureDetectorMatcher(), snap_dim=(400, 400), cropFlag = False, 
                 resizeFlag = False, fps = 30 , dt = 0.1, 
                 time_offset = 0, useGAN = False, PreProcessedVideoReal = None, PreProcessedVideoFake = None, 
                 videoName = 'itu_winter.mp4' , liveFlag = False):
        """
        Constructor. In MATLAB, the class had optional arguments via varargin.
        Here we define explicit optional parameters or accept them as needed.
        """
        self.PreProcessedVideoReal       = PreProcessedVideoReal       
        self.PreProcessedVideoFake       = PreProcessedVideoFake
        self.snapDim = snap_dim       # Snapped image dimension [W, H]
        self.dt = dt
        self.time = time_offset
        self.fps = fps
        self.liveFlag = liveFlag
        self.video_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data/videos', videoName) #itu_43_sat , itu_4_downsampled

        self.frames        = []
        self.fake_frames   = []
        self.tensor_frames = []
        self.useGAN = useGAN
        
        # Initialize the feature detector and matcher in default mode
        self.FeatureDM = FeatureDM
        
        #If liveFlag is true, use the live image from the UAV camera, if false, use the recorded video and load the frames
        if not self.liveFlag:
            self.LoadFrames(cropFlag,resizeFlag)
        
        # Initialize CUT(GAN) pre-trained model
        if self.useGAN:
            # opt = TestOptions().parse()  # get deafult test options then modify it
            # opt.name = 'ituUAV_v2_CUT'
            # opt.num_threads = 0   # test code only supports num_threads = 1
            # opt.batch_size = 1    # test code only supports batch_size = 1
            # opt.serial_batches = True  # disable data shuffling; comment this line if results on randomly chosen images are needed.
            # opt.no_flip = True    # no flip; comment this line if results on flipped images are needed.
            # opt.resize_or_crop = 'none'
            # opt.checkpoints_dir = 'CUT/checkpoints'
            # self.CUT = create_model(opt)      # create a model given opt.model and other options
            # self.CUT.setup(opt)               # regular setup: load and print networks; create schedulers
            # self.CUT.parallelize()
            # # if opt.eval:
            # self.CUT.eval()
            
            # Initialize model
            model_path = "output/cyclegan_turbo/winterUAV2summerSAT_v2/checkpoints/model_16001.pkl"
            self.CycleGAN_opt = {'prompt'    : "picture of summer SAT",
                                 'direction' : "a2b",
                                 'image_prep': "no_resize",
                                 'use_fp16'  : True}

            self.CycleGAN = CycleGAN_Turbo(pretrained_name=None, pretrained_path=model_path)
            self.CycleGAN.eval()
            self.CycleGAN.unet.enable_xformers_memory_efficient_attention()
            if self.CycleGAN_opt['use_fp16']:
                self.CycleGAN.half()
                
                
                    
    def LoadFrames(self,cropFlag, resizeFlag):
            """
            Load all frames from the video into a list.
            """
            
            if (self.PreProcessedVideoReal is None) and (self.PreProcessedVideoFake is None):
                frameCount = 0
                Video = cv2.VideoCapture(self.video_path)

                if not Video.isOpened():
                    raise ValueError("Could not open the video file.")

                while True:
                    ret, frame = Video.read()

                    if not ret:
                        break
                    # converting BGR to RGB
                    frame = frame[:, :, ::-1]  
                    
                    #Crop the image to a square of size crop_height x crop_height centered at the image center.
                    if cropFlag:
                        if (not frameCount): #only calculate crop dimensions once
                            
                            start_x, end_x, start_y, end_y = square_crop_from_center(frame, True)

                        frame = frame[start_y:end_y, start_x:end_x]

                    if resizeFlag:    
                        frame = resize_image(frame, self.snapDim)
                    
                    if self.useGAN:     # if use CUT, convert numpy array to torch tensor                   
                        tensor_frame = numpy_image_to_torch(frame, cutFlag = True)
                        self.tensor_frames.append(tensor_frame)
                        
                    self.frames.append(frame)
                    # cv2.imshow('d',frame)
                    # cv2.waitKey(0)
                    frameCount += 1
                
                Video.release()
                
            #Load preprocessed frames if usePreprocessedVideo is true
            else: 
                # self.frames =  np.load('data/cyclegan/turbo/frames_generated/data_winter.npy')  # shape: (num_frames, 256, 256, 3)
                # self.frames =  np.load('data/cyclegan/turbo/frames_generated/data_itu_fake_summer_5001.npy')  # shape: (num_frames, 256, 256, 3)
                # self.frames      =  np.load('data/cyclegan/turbo/frames_generated/itu_winter_org.npy')  # shape: (num_frames, 256, 256, 3)
                # self.frames = np.load('itu_video_25042025.npy')  # shape: (num_frames, 256, 256, 3)
                
                # self.frames = np.load('itu_video_05052025.npy')  # shape: (num_frames, 256, 256, 3)
                self.frames = np.load(self.PreProcessedVideoReal)  # shape: (num_frames, 256, 256, 3)
                frameCount = len(self.frames)
                
                if self.PreProcessedVideoFake is not None:
                    self.fake_frames = np.load(self.PreProcessedVideoFake)
                    # self.fake_frames =  np.load('data/cyclegan/turbo/frames_generated/itu_fake_video_05052025.npy')  # shape: (num_frames, 256, 256, 3)
                    
                    if frameCount != len(self.fake_frames):
                        raise ValueError("usePreprocessedVideo Error: The number of frames in the original and fake videos do not match.")
            
            print(f"Loaded {frameCount} frames from the video with {self.fps} FPS.")
            print(f"Video length: {frameCount / self.fps} seconds.")

    def snapUAVImage(self, DB = None, showFeatures = False, showFrame = True, 
                     UAVWorldPos = None, UAVYaw = None, 
                     frame = None):
        
        """"
        input arguments:
        -common arguments:
            - DB: Database as AerialImageModel object 
            - showFeatures: Boolean flag to show features on the frame
            - showFrame: Boolean flag to show the frame
        -snapUAVImageDataBase arguments:
            - UAVWorldPos: UAV world position (NED coordinates) for getting images from the database
            - UAVYaw: UAV yaw angle for getting images from the database
        -snapUAVImageVideo arguments:
            - frame: UAV camera frame (numpy array) for live image processing
        
        output arguments:
        - frame_org: Original frame (numpy array)
        - fake_frame: Fake frame (numpy array) if using GAN, otherwise None
        - keypoints_np: Keypoints (numpy array) in the frame
        - descriptors: Descriptors (numpy array) associated with the keypoints
        """
        
        
        if frame is not None:
            frame_org, fake_frame, keypoints_np, descriptors = self.snapUAVImageLive(DB, frame, showFeatures, showFrame)
            
        elif (UAVWorldPos is not None) and (UAVYaw is not None):
            fake_frame = None  # No fake frame for database images
            frame_org, keypoints_np, descriptors = self.snapUAVImageDataBase(DB, UAVWorldPos, UAVYaw, showFeatures, showFrame)
            
        else:
            frame_org, fake_frame, keypoints_np, descriptors = self.snapUAVImageVideo(DB, showFeatures, showFrame)
        
        return frame_org, fake_frame, keypoints_np, descriptors

    def snapUAVImageVideo(self,DB, showFeatures = False, showFrame = True):
        """
        Process and return the frame, keypoints, and descriptors at the specified time.

        Parameters:
        - time (float): The timestamp (in seconds) to extract the frame.

        Returns:
        - frame (numpy.ndarray): The extracted frame.
        - keypoints (list): The detected keypoints.
        - descriptors (numpy.ndarray): The descriptors associated with the keypoints.
        """

        #Placeholder for fake frame        
        fake_frame = None
        
        #Get index of the frame to be processed
        frame_index = int(self.time * self.fps)

        if frame_index < 0 or frame_index >= len(self.frames):
            raise ValueError("Requested time is out of video bounds.")
        
        #Generate fake frame using GAN if useGAN is true
        if self.useGAN:
            fake_frame = self.tensor_frames[frame_index]
            fake_frame = self.CUT.generateFake(fake_frame)
            fake_frame = (fake_frame.squeeze(0).cpu() + 1) / 2               # Convert from [-1, 1] to [0, 1]
            fake_frame = (fake_frame * 255).byte().permute(1, 2, 0).numpy()  # Convert to [0, 255] and HxWxC

            # Convert from RGB to BGR for OpenCV
            # fake_frame = cv2.cvtColor(fake_frame, cv2.COLOR_RGB2BGR)       # DEAL LATER
            
        #Get fake frame from preprocessed video if PreProcessedVideoFake is not None
        if self.PreProcessedVideoFake is not None:
            fake_frame = self.fake_frames[frame_index]

        #Get original frame from the video
        frame           = self.frames[frame_index]
        frame_org       = frame.copy()
        self.curr_frame = frame.copy()  #get raw UAV frame for Visual Odometry
        
        #Feature extraction
        if self.useGAN or (self.PreProcessedVideoFake is not None):  ## Use fake frame if using GAN or preprocessed video
            frame = fake_frame
        _, keypoints_np, descriptors = self.FeatureDM.detectFeatures(frame)
            
        # Get frames to be shown
        if showFrame: 
            if showFeatures: # Show the features if requested
                if self.useGAN or (self.PreProcessedVideoFake is not None):
                    fake_frame = drawKeypoints(fake_frame, keypoints_np)
                else:
                    frame_org = drawKeypoints(frame_org, keypoints_np)
        else:
            frame_org  = None
            fake_frame = None
        
        #increase time,  
        self.time += self.dt 
        # print(f"UAV Camera Number of features: {len(keypoints_np)}")
        
        return frame_org, fake_frame, keypoints_np, descriptors

    def snapUAVImageDataBase(self,DB,UAVWorldPos,UAVYaw, showFeatures = False, showFrame = True):
        """
        Filters ORB keypoints and descriptors that lie within a rotated rectangle.
        
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
        # UAVWorldPos 1X2 array
        UAVPxPos = ned2px(UAVWorldPos,DB.AIM.leftupperNED, DB.AIM.mp, DB.pxRned).squeeze() # shape 2,

        w, h = self.snapDim

        # Find min-max x,y in UAV
        min_x = UAVPxPos[0] - (w // 2)
        max_x = UAVPxPos[0] + (w // 2)
        min_y = UAVPxPos[1] - (h // 2)
        max_y = UAVPxPos[1] + (h // 2)
        
        reduced_mask = (
                        (DB.AIM.keypointBase_np[:, 0] <= max_x) & (DB.AIM.keypointBase_np[:, 0] >= min_x) &
                        (DB.AIM.keypointBase_np[:, 1] <= max_y) & (DB.AIM.keypointBase_np[:, 1] >= min_y)
                        )
        
        # Mask features inside the big rectangle(UAV view) without yaw rotation
        # reduced_keypoints = DB.AIM.keypointBase_np[reduced_mask]
        reduced_keypoints_np, reduced_descriptors = self.FeatureDM.MaskFeatures(DB.AIM.featuresBase, DB.AIM.keypointBase_np, self.snapDim ,reduced_mask)        

        # Compute rotation matrix
        yaw = UAVYaw  # rad
        R = np.array([
            [ np.cos(yaw),  np.sin(yaw)],
            [-np.sin(yaw),  np.cos(yaw)]
        ])

        # Shift keypoints to rectangle's center
        shifted_keypoints = reduced_keypoints_np - UAVPxPos

        # Rotate keypoints to rectangle's local frame
        local_keypoints = np.dot(shifted_keypoints, R.T) 
                
        inside_mask = (
            (np.abs(local_keypoints[:, 0]) <= w // 2) &
            (np.abs(local_keypoints[:, 1]) <= h // 2)
        )
        
        # UAV Local Keypoints (relative keypoints  to upper left corner of particles px)
        UAV_local_keypoint = local_keypoints[inside_mask]  + np.array([ w // 2, h // 2])    

        # Mask features inside the rectangle(view of UAV)
        # UAVKeypoints_np   = reduced_keypoints_np[inside_mask]
        UAVKeypoints_np, UAVDescriptors = self.FeatureDM.MaskFeatures(reduced_descriptors,reduced_keypoints_np, self.snapDim, inside_mask)
        
        # Get frame 
        UAVframe = None
        if showFrame:
            UAVframe = DB.AIM.I[min_y:max_y, min_x:max_x]
            UAVframe = rotate_image(UAVframe,yaw)
            self.curr_frame = UAVframe.copy()  #get pure UAV frame for Visual Odometry

            if showFeatures:
                UAVframe = drawKeypoints(UAVframe, UAV_local_keypoint)
                
        return UAVframe,UAVKeypoints_np,UAVDescriptors

    def snapUAVImageLive(self, DB, frame, showFeatures = False, showFrame = True):
        
        """
        Process and return the frame, keypoints, and descriptors at the specified time.
        """
        
        #Placeholder for fake frame        
        fake_frame = None
        
        ##Preprocess raw image of UAV
        #Crop and resize image
        frame = square_crop_from_center(frame)
        frame_org = frame.copy()

        # if CycleGAN is used, convert numpy array to torch tensor  
        if self.useGAN:               
            with torch.no_grad():
                T_val = build_transform(self.CycleGAN_opt["image_prep"])
                frame = T_val(frame)
                
                x_t = transforms.ToTensor()(frame)
                x_t = transforms.Normalize([0.5], [0.5])(x_t).unsqueeze(0).cuda()
                if self.CycleGAN_opt['use_fp16']:
                    x_t = x_t.half()
                fake_frame = self.CycleGAN(x_t, direction=self.CycleGAN_opt["direction"], caption=self.CycleGAN_opt["prompt"])
                fake_frame = transforms.ToPILImage()(fake_frame[0].cpu() * 0.5 + 0.5)
                # fake_frame = fake_frame.resize((self.snapDim[0], self.snapDim[1]), Image.LANCZOS)
                # Convert back to NumPy array and store
                fake_frame.append(np.array(fake_frame))


        #Feature extraction
        if self.useGAN or (self.PreProcessedVideoFake is not None):  ## Use fake frame if using GAN or preprocessed video
            frame = fake_frame
        _, keypoints_np, descriptors = self.FeatureDM.detectFeatures(frame)
            
        # Get frame 
        if showFrame: 
            if showFeatures: # Show the features if requested
                if self.useGAN or (self.PreProcessedVideoFake is not None):
                    fake_frame = drawKeypoints(fake_frame, keypoints_np)
                else:
                    frame_org = drawKeypoints(frame_org, keypoints_np)
        else:
            frame_org  = None
            fake_frame = None
        
        # print(f"UAV Camera Number of features: {len(keypoints_np)}")
        
        return frame_org, fake_frame, keypoints_np, descriptors
        