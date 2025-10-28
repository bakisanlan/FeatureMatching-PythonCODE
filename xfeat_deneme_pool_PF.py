import numpy as np
import imageio as imio
# import os
import torch
from time import time
from concurrent.futures import ThreadPoolExecutor
from utils import resize_image


from StateEstimatorVINS import StateEstimatorMPF
from FeatureDetectorMatcher import FeatureDetectorMatcher
from UAVCamera import UAVCamera
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from Timer import Timer
import pymap3d as pm




def _worker_initializer():
    """Initialize CUDA context in worker threads"""
    if torch.cuda.is_available():
        # Set the device for this worker thread
        torch.cuda.set_device(0)
        # Create a dummy tensor to initialize CUDA context
        dummy = torch.zeros(1, device='cuda:0')
        del dummy
        torch.cuda.synchronize()

def _measurement_update_worker():
    """Heavy measurement update in separate thread"""

    UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImageLive(im1, showFeatures = showFeatures, showFrame = showFrame)



    # n = 5
    # x2 = torch.randn(n,3,500,500)

    for i in range(50):
        torch.cuda.synchronize()
        start = time()
        # output2 = xfeat.detectAndCompute(x2, top_k = 1300)[0]
        # for j in range(50):
        hStateEstimatorMPF._find_likelihood_particles(
            np.array([0,0,0, 1,0,0,0]),  # pN, pE, pD, qW, qX, qY, qZ
            UAVKp, 
            UAVDesc
        )            
        end = time()
        torch.cuda.synchronize()

        print(f"Iteration {i}: {end - start:.4f} seconds")



#### Flight parameters
ReferenceFrame = 'NED'  #Reference frame for the drone's local coordinate system
MAP            = 'bacikoy'  #Satallite map name
detector       = 'XFEAT'   #Feature detector type (SP: SuperPoint, ORB: Oriented FAST and Rotated Brief)
snapFrame      = True
IMUtype        = 2  #deal later
LLA_leftupper  = [39.780238,  32.314440, 0]  # LLA reference point (latitude, longitude) left upper corner of the map
LLA_home       = [39.7785834, 32.3158889,0] # Initial position of the drone in LLA (latitude, longitude, altitude)
leftupperNED   = np.array(pm.geodetic2ned(LLA_leftupper[0], LLA_leftupper[1], LLA_leftupper[2], LLA_home[0], LLA_home[1], LLA_home[2]), dtype=float) + np.array([5,-8,0])


detector_opt = {'type' : 'XFEAT'}
hFeatureDM = FeatureDetectorMatcher(detector_opt= detector_opt)


#### Aerial Image DataBase
preFeatureFlag = True
hAIM = AerialImageModel(MAP, FeatureDM = hFeatureDM, preFeatureFlag= preFeatureFlag)
hAIM.leftupperNED = leftupperNED


#### UAV Camera
fx, fy, cx, cy = [635.4374739716663, 633.1552214084261, 486.7922140547102, 289.11649690690723]  # 4mm lens
snapDim = (200,200) #deal later
gimballedCamera       = False
useGAN                = False
showFeatures          = True
showFrame             = True
liveFlag              = True  # live video flag, if true, use the live image from the UAV camera, if false, use the recorded video
hUAVCamera = UAVCamera(FeatureDM = hFeatureDM, snapDim = snapDim, cropFlag = True, 
                       resizeFlag = True, useGAN = useGAN, liveFlag = liveFlag)
# frame_org, fake_frame, keypoints_np, descriptors = hUAVCamera.snapUAVImageLive(frame, showFeatures = False, showFrame = True):


#### Database Scanner
batch_mode = False
hDB = DatabaseScanner(FeatureDM = hFeatureDM, AIM=hAIM, snapDim=snapDim, 
                      showFeatures= showFeatures, showFrame= showFrame,
                      batch_mode = batch_mode)


### MPF State Esimator
useMPF          = True
KLDsamplingFlag = False
dt = 1/200  # NOTE: DEAL LATER!!! UPDATE IN WHILE LOOP
dt_mpf_meas_update = 10
N = 50
v = 0.05
mu_part  = np.array([0,0,0,0]) # pN, pE, yaw
std_part = np.array([1,1,0,np.deg2rad(2)])
mu_kalman  = None
cov_kalman = None
circular_var = [0,0,0,1]  # Circular variable for yaw
hStateEstimatorMPF                 = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman, circular_var,
                                                        dt,dt_mpf_meas_update,v,gimballedCamera, KLDsamplingFlag)
hStateEstimatorMPF.DataBaseScanner = hDB
is_first_IM = False

snap_dim = (300, 300)
hStateEstimatorMPF.DataBaseScanner.snapDim = snap_dim
hUAVCamera.snapDim                         = hStateEstimatorMPF.DataBaseScanner.snapDim



image0 = "/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0054_20251018_211221.jpg"
d = 300
im1_src = np.copy(imio.v2.imread(image0))
im1_src = resize_image(im1_src, (d,d))    
if im1_src.ndim == 2:  # grayscale -> add channel dimension
    im1 = np.copy(im1_src[..., np.newaxis])
else:  # color -> keep same handling as im2 (reverse channels)
    im1 = np.copy(im1_src[..., ::-1])




# --- GPU setup ---
torch.set_grad_enabled(False)
torch.backends.cudnn.benchmark = True
torch.set_num_threads(2)
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
print(device)

# Warm up CUDA in main thread
if torch.cuda.is_available():
    dummy = torch.zeros(1, device=device)
    del dummy
    torch.cuda.synchronize()

pool = ThreadPoolExecutor(
    max_workers=1,
    initializer=_worker_initializer
)

# xfeat = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained = True, top_k = 1300)



measurement_future = pool.submit(
    _measurement_update_worker
)
