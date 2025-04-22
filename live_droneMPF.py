import numpy as np
import matplotlib.pyplot as plt
from StateEstimatorMPF import StateEstimatorMPF
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from UAVCamera import UAVCamera
from INSBot import INSbot
from Timer import Timer
from plotter import plot_positions,PlotCamera,combineFrame,DynamicErrorPlot, TwoDynamicPlotter
from utils import *
from VisualOdometry import VisualOdometry
import pymap3d as pm


# This code is for a online drone localization using the MPF (Marginalized-Particle Filter) algorithm with Feature Matching.

#### Flight parameters
ReferenceFrame = 'NED'  #Reference frame for the drone's local coordinate system
MAP            = 'itu'  #Satallite map name
detector       = 'SP'   #Feature detector type (SP: SuperPoinnt, ORB: Oriented FAST and Rotated BRIEF)
snapFrame      = True
IMUtype        = 2  #deal later
lla0           = [41.108116,  29.018083]  # LLA reference point (latitude, longitude) left upper corner of the map

#### Get the initial states of drone
dt = 0.01 # NOTE: DEAL LATER!!! CALCULATE IN WHILE LOOP  time_usec, micro seconds

# Get the rough initial states of drone for particle ditribution
# NOTE: This is a rough initial state, not the real one. The real one will be estimated by the MPF algorithm.
lat , lon, alt = DEALLATER()  # get last lat, lon, alt from the mavlink, GLOBAL_POSITION_INT message
n, e, d = pm.geodetic2ned(lat, lon, alt, lla0[0], lla0[1], lla0[2])
pos0 = np.array([n, e, d])  # initial position relative to the left upper corner of the map

vn, ve, vd = DEALLATER()  # get last vn, ve, vd from the mavlink, LOCAL_POSITION_NED message
V0 = np.array([vn, ve, vd])  # initial velocity in NED frame

euler0 = np.array(DEALLATER()) # get last euler angles from the mavlink, ATTITUDE message
quat0  = eul2quat(euler0[0], euler0[1], euler0[2])  # convert euler angles to quaternion 

# Placeholder for bias of IMU, their value will be updated in INSbot class
acc_bias  = np.zeros(3)     #deal later
gyro_bias = np.zeros(3)     #deal later

#Concatanate initial state
init_state = np.concatenate((pos0,V0,quat0,acc_bias,gyro_bias),axis=0)

#### Initilize INSbot
hINS = INSbot(init_state, dt=dt, ReferenceFrame=ReferenceFrame, IMUtype=IMUtype)

#### Aerial Image DataBase
orb_num_level = 1 #ORB detector level
hAIM = AerialImageModel(MAP,num_level=orb_num_level, detector= detector)
hAIM.leftupperNED = np.array([0, 0 , 0]) 


#### UAV Camera
snap_dim = (300,300) #deal later
useGAN                = False
showFeatures          = False
showFrame             = True
liveFlag              = True  # live video flag, if true, use the live image from the UAV camera, if false, use the recorded video
usePreprocessedVideo  = False  #if true, use the preprocessed video, if false, use the original video(for liveFlag = False)
hUAVCamera            = UAVCamera(dt = dt, snap_dim = snap_dim, cropFlag = True, 
                                  resizeFlag = True, useGAN = useGAN, 
                                  usePreprocessedVideo = usePreprocessedVideo)

#### Database Scanner
useColorSimilarity = False
batch_mode = False
hDB = DatabaseScanner(AIM=hAIM, num_level=orb_num_level, snap_dim=snap_dim, showFeatures= showFeatures, showFrame= showFrame, useColorSimilarity = useColorSimilarity, batch_mode = batch_mode)


### MPF State Esimator
useMPF = True
dt_mpf_meas_update = 3
N = 200
mu_part  = np.array([0,0,0])
std_part = np.array([20,20,0])
mu_kalman  = np.zeros(12)
cov_kalman = np.zeros((12,12))
v = 0.05   #DEAL LATER
hStateEstimatorMPF                 = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman,dt,dt_mpf_meas_update,v)
hStateEstimatorMPF.DataBaseScanner = hDB
hStateEstimatorMPF.Accelerometer   = hINS.IMU.Accelerometer
hStateEstimatorMPF.Gyroscope       = hINS.IMU.Gyroscope


#### Visual Odometry
useVO = False
dt_vo = dt
cam2body = quat2rotm(eul2quat([np.pi/2, 0,0]))
body2inertia = quat2rotm(quat0)
cam2inertia = body2inertia @ cam2body
# Camera intrinsics (example values, replace with your calibrated values)
fx, fy = 2889.99*(256/2210), 2889.99*(256/2210)
cx, cy = 128, 128
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])
hVisualOdometry = VisualOdometry(K = K, nfeatures = 1000,nlevels= num_level, type = 'SIFT', t = pos0, R = cam2inertia)
n_VO = 1e-5