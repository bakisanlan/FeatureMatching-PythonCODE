import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import pymap3d as pm
import csv
import sys
import os
import yaml
import logging
import threading
import signal
from pathlib import Path
from datetime import datetime

# This code is for a online drone localization using the MPF (Marginalized-Particle Filter) algorithm with Feature Matching.

# Custom libraries
# Add the path to the utils module if it's not in the same directory
from utils import *
from StateEstimatorVINS import StateEstimatorMPF
from FeatureDetectorMatcher import FeatureDetectorMatcher
from UAVCamera import UAVCamera
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from Timer import Timer
from plotter import plot_positions,PlotCamera,combineFrame,DynamicErrorPlot, TwoDynamicPlotter
from OV.odom_subscriber import OdomAndMavrosSubscriber
from OV.utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter, visualize2DgenTraj

# Setup logging and redirect stdout/stderr
setup_logging_with_redirect(log_dir_name="IM_logs_out", log_file_prefix="IM", level=logging.INFO)

# Initialize ROS 2 nodes
rclpy.init()
node_OdomVIO     = OdomAndMavrosSubscriber()
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
is_first_messages = True
is_first_IM       = True


#### Flight parameters
ReferenceFrame = 'NED'  #Reference frame for the drone's local coordinate system
MAP            = 'catalca'  #Satallite map name
detector       = 'XFEAT'   #Feature detector type (SP: SuperPoint, ORB: Oriented FAST and Rotated Brief)
snapFrame      = True
LLA_leftupper = [41.336322, 28.489583, 0]
LLA_home      = [41.332762, 28.494641, 0]
leftupperNED = np.array(
    pm.geodetic2ned(LLA_leftupper[0], LLA_leftupper[1], LLA_leftupper[2],
                    LLA_home[0], LLA_home[1], LLA_home[2]),
    dtype=float
) + np.array([0, 0, 0])

#Create Feature Detector-Matcher
# detector_opt = {'type' : 'SP', 'params' : {'max_num_keypoints': 512}}
# matcher_opt  = {'type' : 'LightGlue' ,  'params' : {'depth_confidence' : 0.9, 'width_confidence' : 0.95}}
# detector_opt = {'type' : 'ORB'}
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
dt_mpf_meas_update = 1
N = 2
v = 0.05   #DEAL LATER
# mu_part  = np.array([0,0,0])
# std_part = np.array([1,1,np.deg2rad(2)])
# mu_kalman  = None
# cov_kalman = None
# hStateEstimatorMPF                 = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman,
#                                                         dt,dt_mpf_meas_update,v,gimballedCamera, KLDsamplingFlag)
# hStateEstimatorMPF.DataBaseScanner = hDB
# hStateEstimatorMPF.Accelerometer   = hINS.IMU.Accelerometer
# hStateEstimatorMPF.Gyroscope       = hINS.IMU.Gyroscope



# -------------------------------------------------------------------------
# 2) Prepare storage lists for:
#    - Dead-reckoning (INS) states
#    - XKF "ground truth" states
# -------------------------------------------------------------------------

VIO_position_list      = []
VIO_velocity_list      = []
VIO_euler_list         = []
VIO_state_list          = []

gt_position_list           = []
gt_velocity_list           = []
gt_euler_list              = []
gt_state_list               = []

PF_position_list           = [] 
PF_velocity_list           = []
PF_euler_list              = []
PF_particles_position_list = []
PF_state_list              = []

### Figure create object holder for side by side view of UAV and most likelihood particle
useFramePlotter = True
plot_interval   = 1
last_plot_time  = time.time()
CamPlotter      = PlotCamera(useFramePlotter= useFramePlotter)

# Signal handler for Ctrl+C to plot trajectories before exiting
def signal_handler(sig, frame):
    """Handle Ctrl+C interrupt to plot trajectories before exiting."""
    print("\n\nCtrl+C detected! Plotting trajectories...")
    
    # Convert lists to numpy arrays
    if len(PF_position_list) > 0 and len(VIO_position_list) > 0:
        pf_pos_array = np.array(PF_position_list)[:, 0:2]   # Extract N, E components
        vio_pos_array = np.array(VIO_position_list)[:, 0:2]  # Extract N, E components
        
        # Plot using visualize2DgenTraj
        print(f"Plotting {len(PF_position_list)} PF positions and {len(VIO_position_list)} VIO positions...")
        visualize2DgenTraj(
            vio_pos_array, 
            second_points=pf_pos_array,
            # xlabel="North (m)",
            # ylabel="East (m)",
            title="VIO vs PF Trajectory Comparison"
        )
    else:
        print("No trajectory data to plot.")
    
    # Cleanup and exit
    print("Shutting down...")
    rclpy.shutdown()
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

#### Main loop
plt.show(block=False)

while True:
    #node_OdomVIO.first_vo_msg
    #node_OdomVIO.VIO_dict
    #is_velocity_body = True
    if node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_gps_fix_msg:
        
        # Check if the first messages of all publishers are received
        if is_first_messages:
            print("First messages received, processing data...")

            yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict.copy(), node_OdomVIO.gt_odom_dict.copy())

            # Initialize yaw vector from GPS/mag/inital guess 
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
            euler_gt = quat2eul(VIO_dict['orientation'])
            yaw_vector = np.array([np.cos(euler_gt[0]), np.sin(euler_gt[0])])
            print(np.rad2deg(yaw_diff))

            # Flag for first messages
            is_first_messages = False
            start_time = time.time()

        else:
        
            # Getting VIO data with converting to NED frame
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
            VIO_pos     = np.array(VIO_dict['position'])
            VIO_quat    = np.array(VIO_dict['orientation'])
            # VIO_vel     = np.array(VIO_dict['velocity'])  
            VIO_ang_vel = np.array(VIO_dict['angular_velocity'])
            VIO_Nom     = np.hstack((VIO_pos, VIO_quat))

            
            # Convert body angular velocity (p, q, r) to Euler angle rates (phi_dot, theta_dot, psi_dot)
            euler_vio = quat2eul(VIO_quat)
            euler_dot = bodyRates2eulerRates(VIO_ang_vel, euler_vio)
            
            
            # Get ground truth odometry data from GPS fix local frame
            GT_dict = ned_VIO_converter(node_OdomVIO.gt_odom_dict.copy(), 0, is_velocity_body = False)
            GT_pos  = np.array(GT_dict['position'])
            GT_quat = np.array(GT_dict['orientation'])
        
        
            # Condition for begin satellite image based localization
            startIM = -VIO_pos[2] > 40
            if startIM:

                # Initialize Image Matching State Estimation
                if is_first_IM:
                    print("Starting image-based localization...")
                    
                    
                    mu_part  = np.array([VIO_pos[0],VIO_pos[1],0,euler_vio[0]]) # pN, pE, yaw
                    std_part = np.array([1,1,0,np.deg2rad(2)])
                    mu_kalman  = None
                    cov_kalman = None
                    circular_var = [0,0,0,1]  # Circular variable for yaw
                    hStateEstimatorMPF                 = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman, circular_var,
                                                                            dt,dt_mpf_meas_update,v,gimballedCamera, KLDsamplingFlag)
                    hStateEstimatorMPF.DataBaseScanner = hDB
                    is_first_IM = False
                    
                    prev_time        = time.time()
                    prev_VIO_pos     = np.array(VIO_dict['position'])

                else:
                    # Update dt
                    dP = VIO_pos - prev_VIO_pos  
                    dt = time.time() - prev_time
                    VIO_vel = dP / (dt + 1e-16)
                    prev_time = time.time()
                    prev_VIO_pos = VIO_pos.copy()
                    print('hz:', 1/(dt+1e-16))
                    hStateEstimatorMPF.dt = dt


                    # Get Particle input from VIO
                    inputParticle = [VIO_vel[0], VIO_vel[1], euler_dot[2]]


                    # UAV Snap Image(Get Measurement from Camera)   
                    UAVKp    = None
                    UAVDesc  = None
                    UAVFrame = None
                    cond_meas_upt = time.time() - hStateEstimatorMPF.last_meas_update_time > hStateEstimatorMPF.dt_mpf_meas_update
                    
                    if cond_meas_upt:
                        
                        with Timer("UAV Image Feature Extraction Time"):
                            
                            # Adjust snap dimension based on altitude and camera parameters
                            hStateEstimatorMPF.DataBaseScanner.snapDim = int(((-VIO_pos[2]/fx) * 2 * cx) * (1/hAIM.mp)), int(((-VIO_pos[2]/fx) * 2 * cx) * (1/hAIM.mp))
                            hUAVCamera.snapDim                         = hStateEstimatorMPF.DataBaseScanner.snapDim
                            
                            
                            rawFrame = node_OdomVIO.camera_image
                            # rawFrame = rotate_image(rawFrame, np.pi)  # Rotate image if needed
                            UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImageLive(rawFrame, showFeatures = showFeatures, showFrame = showFrame)
                            
                            
                            hStateEstimatorMPF.cond_meas_upt = True

                            if useGAN:
                                UAVFrame = UAVFakeFrame

                    #### MPF Estimation
                    closedLoop = False
                    predPerclosedLoop = 1
                    
                    # Create Combined Frame of GT,INS DEAD RECKON, PARTICLES
                    # hStateEstimatorMPF.particles[0:3, :] = np.tile(GT_pos.reshape(3, 1), (1, hStateEstimatorMPF.particles.shape[1]))
                    particlesPos = hStateEstimatorMPF.particles[0:3, :] # shape 3,N
                
                    pxGT  = ned2px(GT_pos.copy()        , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
                    pxVIO = ned2px(VIO_pos.copy()       , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
                    pxPF  = ned2px(particlesPos.T.copy(), hAIM.leftupperNED, hAIM.mp, hDB.pxRned)   
                    pxPF_with_weights = np.hstack((pxPF , hStateEstimatorMPF.weights.reshape(-1, 1)))

                    ### Measurement Update Through Feature Matching Localization    
                    hStateEstimatorMPF.dt = dt
                    # print(f"Snap Dimension for MPF: {hStateEstimatorMPF.DataBaseScanner.snapDim}")

                    param = hStateEstimatorMPF.getEstimate(inputParticle, VIO_Nom, UAVKp, UAVDesc,
                                                        closedLoop= closedLoop, predPerclosedLoop= predPerclosedLoop)
                    
                    PF_pos = param["State"][0:3]
                    print(f"Estimated Position: {PF_pos[0:2]}, VIO Position: {VIO_pos[0:2]}")
                    # print(f"VIO velocity: {VIO_vel[0:2]}, inputParticle: {inputParticle}")
                    FramemostLikelihoodPart = hStateEstimatorMPF.FramemostLikelihoodPart
                    
                    # Store positions for later plotting
                    PF_position_list.append(PF_pos.copy())
                    VIO_position_list.append(VIO_pos.copy())


                    # Plotting 
                    flagErrorPlot = False
                    flagFramePlot = True

                    if ((time.time() - last_plot_time) > plot_interval) and UAVFrame is not None:
                        last_plot_time = time.time()

                        # Error Plotter
                        if flagErrorPlot:
                            ErrorPlotter.update(GT_pos,VIO_pos, PF_pos, timeConstant = plot_interval)

                        # Update feature plot
                        flightTime = time.time() - start_time
                        if flagFramePlot:
                            combinedFrame = combineFrame(hAIM.I, pxGT, None, pxPF_with_weights)
                            CamPlotter.snapNow(
                                            (UAVFrame                 , 'UAV Camera'                         , f'Flight time is {flightTime:.2f} s \n Detected Features: {UAVKp.shape[0]}'),
                                            # (UAVFakeFrame             , 'Generated Fake SAT Img'             , f'Detected Features: {UAVKp.shape[0]}'),
                                            (FramemostLikelihoodPart  , 'Most likelihood Particle SAT View'  , f'Detected Features: {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[0]} \n Matched features:  {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[1]}'),
                                            (combinedFrame            , 'Particles, Ground Truth in Map'     , f'Position XY RMSE: {np.sqrt(np.mean((GT_pos[0:2] - PF_pos[0:2])**2)):.2f} m'),
                                            )
                    



            else:
                print(f"Waiting to reach the altitude threshold for image-based localization, current altitude: {-VIO_pos[2]:.2f}\n")
                time.sleep(1)
                continue
        
        
    else:
        print("-----------------------------------------------------")

        if not node_OdomVIO.first_vo_msg:
            print("Waiting for first VIO message...")

        if not node_OdomVIO.first_gt_odom_msg:
            print("Waiting for first ground truth odometry message...")

        if not node_OdomVIO.first_gps_fix_msg:
            print("Waiting for first GPS fix message...")


        print("-----------------------------------------------------")
        time.sleep(1)
        continue





# if cond1:

#     while True:
#         cond2 =  hMAVHandler.imu_data['timestamp'] != prevIMU_timestanp

#         if cond2:
            
        
#             # # ~~~ Grab the XKF as "ground truth" ~~~
#             gt_states = hMAVHandler.get_states(LLA0) # get last states from the mavlink, message , pos0, V0, quat0
#             # # GT Full State vector        
#             gtState = np.concatenate((gt_states,acc_bias,gyro_bias),axis=0)
            
#             # INS dt calculation
#             IMU_timestanp = hMAVHandler.imu_data['timestamp']
#             dt = ((IMU_timestanp - prevIMU_timestanp) * 1e-6) # convert microseconds to seconds
#             prevIMU_timestanp = IMU_timestanp

            
#             # get VIO estimates for input to MPF
#             acc_body  = np.array([hMAVHandler.imu_data['xacc'] , hMAVHandler.imu_data['yacc'] , hMAVHandler.imu_data['zacc']] , dtype=float)
#             gyro_body = np.array([hMAVHandler.imu_data['xgyro'], hMAVHandler.imu_data['ygyro'], hMAVHandler.imu_data['zgyro']], dtype=float)
#             inputParticle = [acc_body, gyro_body]  

            
#             #UAV Snap Image(Get Measurement from Camera) 
#             # rawFrame = vehicle.get_frame() # NOTE: DEAL LATER
#             rawFrame = hRedisHelper.from_redis_2('frame_4')
#             UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImageLive(rawFrame, showFeatures = False, showFrame = True)

#             if useGAN:
#                 UAVFrameMPF = UAVFakeFrame
#             else:
#                 UAVFrameMPF = UAVFrame

                    
#             #### MPF Estimation
#             closedLoop = True
#             predPerclosedLoop = 1
            
#             # Create Combined Frame of GT,INS DEAD RECKON, PARTICLES
#             particlesPos = hStateEstimatorMPF.particles + hINS.NomState[0:3].reshape(-1, 1) # shape 3,N
#             pxGT  = ned2px(gtState[0:3].copy()          , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
#             pxINS = ned2px(hINS.NomState[0:3].copy()    , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
#             pxPF  = ned2px(particlesPos.T.copy()        , hAIM.leftupperNED, hAIM.mp, hDB.pxRned)   
#             pxPF_with_weights = np.hstack((pxPF, hStateEstimatorMPF.weights.reshape(-1, 1)))

#             ### Measurement Update Through Feature Matching Localization    
#             hStateEstimatorMPF.dt = dt
#             hStateEstimatorMPF.DataBaseScanner.snapDim = int(((-gt_POS[2]/fx) * 2 * cx) * (1/hAIM.mp)) , int(((-gt_POS[2]/fx) * 2 * cx) * (1/hAIM.mp))

#             param = hStateEstimatorMPF.getEstimate(inputParticle, hINS.NomState, UAVKp, UAVDesc,
#                                                 closedLoop= closedLoop, predPerclosedLoop= predPerclosedLoop ,
#                                                 UAVframe= UAVFrameMPF)

#             estState = hINS.correctINS(param["State"], closedLoop= closedLoop, predPerclosedLoop = predPerclosedLoop)
#             particlesPos = hStateEstimatorMPF.particles + hINS.NomState[0:3].reshape(-1, 1) # shape 3,N
#             FramemostLikelihoodPart = hStateEstimatorMPF.FramemostLikelihoodPart

#             # Storing values
#             gt_position_list.append(gtState[0:3].copy())
#             # INS_prd_position_list.append(hINS.NomState[0:3].copy())
#             PF_position_list.append(estState[0:3].copy())
#             PF_particles_position_list.append(particlesPos.T.copy())

#             gtState_list.append(gtState)
#             estState_list.append(estState)
#             # INSpredState_list.append(INS_pred_state)    
            
#             flagErrorPlot = False
#             flagFramePlot = True
            
#             if ((flightTime % (dt * sim_per_plot)) < 0.1):
                    
#                 # Error Plotter
#                 if flagErrorPlot:
#                     ErrorPlotter.update(gtState[0:16],hINS.NomState[0:16], estState[0:16], timeConstant = dt*sim_per_plot)

#                 # Update feature plot
#                 if flagFramePlot:
#                     combinedFrame = combineFrame(hAIM.I, pxGT, None, pxPF_with_weights)
#                     CamPlotter.snapNow(
#                                     (UAVFrame                 , 'UAV Camera'                         , f'Flight time is {flightTime:.2f} s '), # \n Detected Features: {UAVKp.shape[0]}'),
#                                     (UAVFakeFrame             , 'Generated Fake SAT Img'             , f'Detected Features: {UAVKp.shape[0]}'),
#                                     (FramemostLikelihoodPart  , 'Most likelihood Particle SAT View'  , f'Detected Features: {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[0]} \n Matched features:  {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[1]}'),
#                                     (combinedFrame            , 'Particles, Ground Truth in Map'     , f'Position XY RMSE: {np.sqrt(np.mean((gtState[0:2] - estState[0:2])**2)):.2f} m'),)
                
                
#             flightTime += dt

# plot_positions(gt_position_list,INS_prd_position_list,PF_position_list,PF_particles_position_list,plot_2d= True)
