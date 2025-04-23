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
from MAVHandler import MAVHandler
from utilsFolder.redis_helper import RedisHelper



# This code is for a online drone localization using the MPF (Marginalized-Particle Filter) algorithm with Feature Matching.

#### Flight parameters
ReferenceFrame = 'NED'  #Reference frame for the drone's local coordinate system
MAP            = 'itu'  #Satallite map name
detector       = 'SP'   #Feature detector type (SP: SuperPoinnt, ORB: Oriented FAST and Rotated BRIEF)
snapFrame      = True
IMUtype        = 2  #deal later
LLA0           = [41.108116,  29.018083, 0]  # LLA reference point (latitude, longitude) left upper corner of the map

#### Get the initial states of drone
dt = 0.01 # NOTE: DEAL LATER!!! CALCULATE IN WHILE LOOP  time_usec, micro seconds
connection_str = "/dev/ttyACM0"
hMAVHandler = MAVHandler(connection_str)

# Get the rough initial states of drone for particle ditribution
# NOTE: This is a rough initial state, not the real one. The real one will be estimated by the MPF algorithm.

#### Initilize INSbot
states0 = hMAVHandler.get_states(LLA0) # get last states from the mavlink, message , pos0, V0, quat0
acc_bias  = np.zeros(3)                # Placeholder for bias of IMU, their value will be updated in INSbot class
gyro_bias = np.zeros(3)     #deal later
init_state = np.concatenate((states0,acc_bias,gyro_bias),axis=0)
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

### Redis Helper
hRedisHelper = RedisHelper()


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
#Camera to body rotation matrix. Camera is fixed and Z direction 
#coinside with the body Z direction. X direction of the camera is 
#the same as the body Y direction.
cam2body = quat2rotm(eul2quat([np.pi/2, 0,0]))   # Rotation matrix from camera frame to body frame
body2inertia = quat2rotm(states0[6:10])           # Rotation matrix from body frame to inertia frame
cam2inertia = body2inertia @ cam2body            # Rotation matrix from camera frame to inertia frame
# Camera intrinsics (example values, replace with your calibrated values)
fx, fy = 2889.99*(256/2210), 2889.99*(256/2210)  #DEAL LATER
cx, cy = 128, 128
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])
hVisualOdometry = VisualOdometry(K = K, nfeatures = 1000,nlevels= orb_num_level, type = 'SIFT', t = pos0, R = cam2inertia)
n_VO = 1e-5

# -------------------------------------------------------------------------
# 2) Prepare storage lists for:
#    - Dead-reckoning (INS) states
#    - XKF "ground truth" states
# -------------------------------------------------------------------------

INS_prd_position_list      = []
INS_prd_velocity_list      = []
INS_prd_euler_list         = []
INSpredState_list          = []

gt_position_list           = []
gt_velocity_list           = []
gt_euler_list              = []
gtState_list               = []

PF_position_list           = [] 
PF_velocity_list           = []
PF_euler_list              = []
PF_particles_position_list = []
estState_list              = []

### Figure create object holder for side by side view of UAV and most likelihood particle
useFramePlotter = True
sim_per_plot = 10
CamPlotter = PlotCamera(useFramePlotter= useFramePlotter)

### Conditions for the start of the Feature Matching Localization loop
# NOTE: DEAL LATER!
cond1 = states0[2] > 50

#### Main loop
plt.show(block=False)
prevIMU_timestanp = hMAVHandler.imu_data['timestamp']
flightTime = 0
if cond1:
    
    while True:
    
        # # ~~~ Grab the XKF as "ground truth" ~~~
        gt_states = hMAVHandler.get_states(LLA0) # get last states from the mavlink, message , pos0, V0, quat0
        # # GT Full State vector        
        gtState = np.concatenate((gt_states,acc_bias,gyro_bias),axis=0)
        
        # INS dt calculation
        IMU_timestanp = hMAVHandler.imu_data['timestamp']
        dt = (IMU_timestanp - prevIMU_timestanp) * 1e-6 # convert microseconds to seconds
        
        # get IMU measurement for input to MPF
        acc_body  = np.array([hMAVHandler['xacc'] , hMAVHandler['yacc'] , hMAVHandler['zacc']] , dtype=float)
        gyro_body = np.array([hMAVHandler['xgyro'], hMAVHandler['ygryo'], hMAVHandler['zgryo']], dtype=float)
        inputParticle = [acc_body, gyro_body]  

        # ~~~ Predict INS states (dead-reckoning) ~~~
        hINS.dt = dt
        hINS.predictIMU(acc_body, gyro_body, useVO)
        
        #UAV Snap Image(Get Measurement from Camera) 
        # rawFrame = vehicle.get_frame() # NOTE: DEAL LATER
        rawFrame = hRedisHelper.from_redis_2('frame_6')
        UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImage(DB = hStateEstimatorMPF.DataBaseScanner,frame= rawFrame ,showFeatures=showFeatures, showFrame=showFrame)
        if useGAN:
            UAVFrameMPF = UAVFakeFrame
        else:
            UAVFrameMPF = UAVFrame

        ### Visual Odometry Estimation
        if useVO and (flightTime % (dt_vo*n_VO) <= dt):
            frameVO = rawFrame
            if frameVO is not None:
                
                V_IMU_norm = np.linalg.norm(hINS.NomState[3:6])
                scale = V_IMU_norm * dt_vo
                
                R_cam, t_cam = hVisualOdometry.process_vo_frame(frameVO, scale)
                
                if t_cam is not None:
                    hINS.VO_update(t_cam)
            
                n_VO += 1
                
        #### MPF Estimation
        closedLoop = True
        predPerclosedLoop = 1
        
        # Create Combined Frame of GT,INS DEAD RECKON, PARTICLES
        particlesPos = hStateEstimatorMPF.particles + hINS.NomState[0:3].reshape(-1, 1) # shape 3,N
        pxGT  = ned2px(gtState[0:3].copy()          , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        pxINS = ned2px(hINS.NomState[0:3].copy()    , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        pxPF  = ned2px(particlesPos.T.copy()        , hAIM.leftupperNED, hAIM.mp, hDB.pxRned)   
        pxPF_with_weights = np.hstack((pxPF, hStateEstimatorMPF.weights.reshape(-1, 1)))

        ### Measurement Update Through Feature Matching Localization    
        hStateEstimatorMPF.dt = dt
        param = hStateEstimatorMPF.getEstimate(inputParticle, hINS.NomState, UAVKp, UAVDesc,
                                              closedLoop= closedLoop, predPerclosedLoop= predPerclosedLoop ,
                                              UAVframe= UAVFrameMPF)

        estState = hINS.correctINS(param["State"], closedLoop= closedLoop, predPerclosedLoop = predPerclosedLoop)
        particlesPos = hStateEstimatorMPF.particles + hINS.NomState[0:3].reshape(-1, 1) # shape 3,N
        FramemostLikelihoodPart = hStateEstimatorMPF.FramemostLikelihoodPart

        # Storing values
        gt_position_list.append(gtState[0:3].copy())
        # INS_prd_position_list.append(hINS.NomState[0:3].copy())
        PF_position_list.append(estState[0:3].copy())
        PF_particles_position_list.append(particlesPos.T.copy())

        gtState_list.append(gtState)
        estState_list.append(estState)
        # INSpredState_list.append(INS_pred_state)    
        
        flagErrorPlot = False
        flagFramePlot = True
        
        if ((flightTime % (dt * sim_per_plot)) < 0.1):
                
            # Error Plotter
            if flagErrorPlot:
                ErrorPlotter.update(gtState[0:16],hINS.NomState[0:16], estState[0:16], timeConstant = dt*sim_per_plot)

            # Update feature plot
            if flagFramePlot:
                combinedFrame = combineFrame(hAIM.I, pxGT, None, pxPF_with_weights)
                CamPlotter.snapNow(
                                (UAVFrame                 , 'UAV Camera'                         , f'Flight time is {flightTime:.2f} s '), # \n Detected Features: {UAVKp.shape[0]}'),
                                (UAVFakeFrame             , 'Generated Fake SAT Img'             , f'Detected Features: {UAVKp.shape[0]}'),
                                (FramemostLikelihoodPart  , 'Most likelihood Particle SAT View'  , f'Detected Features: {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[0]} \n Matched features:  {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[1]}'),
                                (combinedFrame            , 'Particles, Ground Truth in Map'     , f'Position XY RMSE: {np.sqrt(np.mean((gtState[0:3] - estState[0:2])**2)):.2f} m'),)
            
            
        flightTime += dt
plot_positions(gt_position_list,INS_prd_position_list,PF_position_list,PF_particles_position_list,plot_2d= True)
