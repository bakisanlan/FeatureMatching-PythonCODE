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
from FeatureDetectorMatcher import FeatureDetectorMatcher
import pymap3d as pm

#####Sim settings
simTime = 0
Tf      = 150
num_level = 8
ReferenceFrame = 'NED'
MAP            = 'itu'  #unity_fl1
IMUtype = 2  #deal later

#####Extract LOG data
csv_path = "data/log/log_05052025.csv"  # Replace with your actual path
# start_idx = 1306  #for itu video2
# end_idx   = 2622
start_idx = 1200  #for itu winter
end_idx   = 4222
data_dict = getLogData(csv_path, start_row=start_idx, end_row=end_idx)
                   
#Get dt
timestamps_ms = data_dict['timestamp']
dt_array = np.diff(timestamps_ms) / 1000.0  # convert to seconds
# Possibly handle if dt varies or take the mean
dt = np.mean(dt_array)

# -------------------------------------------------------------------------
#    Get initial state from XKF states: PN, PE, PD, VN, VE, VD, Roll, Pitch, Yaw
# -------------------------------------------------------------------------
# UAV_init_NED_pos_rel_img_ref  = np.array([-355.8719, 187.2249, 0],dtype= float) #video 30 seconds offset position, LLA = [41.1010349, 29.0254601, 0], we are going to use this as ofsett that account UAV position relative to center of the image
# UAV_init_NED_pos_rel_img_ref    = np.array([-405.8719, 171.2249, 0],dtype= float) #video initial for itu winter,   LLA = [41.1004505, 29.0252685], we are going to use this as ofsett that account UAV position relative to center of the image
# pos0 = UAV_init_NED_pos_rel_img_ref

LLA0  = [41.108116,  29.018083]  # LLA reference point (latitude, longitude) left upper corner of the map
LLA   = [ data_dict['GPS']['Lat'][0] , data_dict['GPS']['Lng'][0]]            # LLA of the UAV position at the first frame of the video
n, e, d = pm.geodetic2ned(LLA[0], LLA[1], 0, LLA0[0], LLA0[1], 0)
pos0 = np.array([n , e + 10, data_dict['XKF']['PD'][0]])  # initial position relative to the left upper corner of the map
UAV_init_NED_pos_rel_img_ref = pos0.copy() #offset UAV position relative to left upper of the image

pN_init,pE_init, pD_init       = data_dict['XKF']['PN'][0] , data_dict['XKF']['PE'][0] , data_dict['XKF']['PD'][0]
pos0_org = np.array([pN_init, pE_init, pD_init])

vN_init, vE_init, vD_init      = data_dict['XKF']['VN'][0] , data_dict['XKF']['VE'][0] , data_dict['XKF']['VD'][0]
V0 = np.array([vN_init, vE_init, vD_init])

roll_deg, pitch_deg, yaw_deg   = data_dict['XKF']['Roll'][0] , data_dict['XKF']['Pitch'][0] , data_dict['XKF']['Yaw'][0]
# Wrap and convert angles to radians
roll_rad  = np.deg2rad(wrap2_180(roll_deg))
pitch_rad = np.deg2rad(wrap2_180(pitch_deg))
yaw_rad   = np.deg2rad(wrap2_180(yaw_deg))
# Reorder for ZYX => [yaw, pitch, roll]
eul_angles_init = np.array([yaw_rad, pitch_rad, roll_rad])
quat0       = eul2quat(eul_angles_init)  # [w, x, y, z]

acc_bias  = np.zeros(3)     #deal later
gyro_bias = np.zeros(3)     #deal later

#Concatanate initial state
init_state = np.concatenate((pos0,V0,quat0,acc_bias,gyro_bias),axis=0)

#Initilize INSbot
hINS = INSbot(init_state, dt=dt, ReferenceFrame=ReferenceFrame, IMUtype=IMUtype)

#Create Feature Detector-Matcher
detector_opt = {'type' : 'SP', 'params' : {'max_num_keypoints': 2048}}
matcher_opt  = {'type' : 'LightGlue' ,  'params' : {'depth_confidence' : 0.9, 'width_confidence' : 0.95}}
# detector_opt = {'type' : 'ORB'}
hFeatureDM = FeatureDetectorMatcher(detector_opt)

# Aerial Image DataBase and Camera
preFeatureFlag = True
hAIM = AerialImageModel(MAP, FeatureDM = hFeatureDM, preFeatureFlag= preFeatureFlag)
hAIM.leftupperNED = np.array([0, 0, 0], dtype=float) #left upper corner of the map in NED coordinates

# hAIM.visualizeFeaturesAerialImage()
snap_dim = (600,600) #deal later
fps = 10             #deal later
# time_offset = 47     #initilize video cam at 30th second(which is altitude is constant for itu video 2)
time_offset = 110     #initilize video cam at 5th second(which is altitude is constant for itu winter video)

# UAV Camera
useGAN                  = False
showFeatures            = False
showFrame               = True
usePreprocessedVideo    = True
isPreprocessedVideoFake = True
videoName = 'itu_winter.mp4'
hUAVCamera = UAVCamera(FeatureDM = hFeatureDM, dt = dt, snap_dim = snap_dim, fps = fps, cropFlag = True, 
                       resizeFlag = True, time_offset= time_offset, useGAN = useGAN,
                       usePreprocessedVideo = usePreprocessedVideo, 
                       isPreprocessedVideoFake = isPreprocessedVideoFake,videoName= videoName)

# Database Scanner
batch_mode = False
hDB = DatabaseScanner(FeatureDM = hFeatureDM, AIM=hAIM,snap_dim=snap_dim, 
                      showFeatures= showFeatures, showFrame= showFrame,
                      batch_mode = batch_mode)

# MPF State Esimator
useMPF = True
dt_mpf_meas_update = 5
N = 100  # Number of particles
mu_part  = np.array([0,0,0])
std_part = np.array([20,20,0])
mu_kalman  = np.zeros(12)
cov_kalman = np.zeros((12,12))
v = 0.05   #DEAL LATER
gimballedCamera = True
hStateEstimatorMPF = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman,dt,dt_mpf_meas_update,v, gimballedCamera = gimballedCamera)
hStateEstimatorMPF.DataBaseScanner = hDB
hStateEstimatorMPF.Accelerometer   = hINS.IMU.Accelerometer
hStateEstimatorMPF.Gyroscope       = hINS.IMU.Gyroscope

# Visual Odometry 
useVO = False
dt_vo = dt
cam2body = quat2rotm(eul2quat([np.pi/2, 0,0]))
body2inertia = quat2rotm(quat0)
cam2inertia = body2inertia @ cam2body
# Camera intrinsics (example values, replace with your calibrated values)
fx, fy = 1546.99*(256/1080), 1546.99*(256/1080)
cx, cy = 128, 128
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])
hVisualOdometry = VisualOdometry(K = K, nfeatures = 1000,nlevels= num_level, type = 'SIFT', t = pos0, R = cam2inertia)
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

# Figure create object holder for side by side view of UAV and most likelihood particle
useFramePlotter = True
sim_per_plot = 10
CamPlotter = PlotCamera(useFramePlotter= useFramePlotter)
ErrorPlotter = DynamicErrorPlot()
# TwoPlotter = TwoDynamicPlotter()

# Number of samples
num_samples = len(data_dict['timestamp'])
idx = 0         #deal later

# Show the figure
plt.show(block = False)
while simTime < Tf:
   with Timer('Simulation Elapsed Time'):
        # ~~~ Grab the XKF as "ground truth" ~~~
        gt_pN, gt_pE, gt_pD = data_dict['XKF']['PN'][idx], data_dict['XKF']['PE'][idx], data_dict['XKF']['PD'][idx]
        gt_POS = np.array([gt_pN, gt_pE, gt_pD] ,dtype=float) - pos0_org + UAV_init_NED_pos_rel_img_ref  #offset UAV position relative to center of the image

        gt_vN, gt_vE, gt_vD = data_dict['XKF']['VN'][idx], data_dict['XKF']['VE'][idx], data_dict['XKF']['VD'][idx]
        gt_V = np.array([gt_vN, gt_vE, gt_vD], dtype=float)

        gt_roll, gt_pitch, gt_yaw = wrap2_180(data_dict['XKF']['Roll'][idx]), wrap2_180(data_dict['XKF']['Pitch'][idx]), wrap2_180(data_dict['XKF']['Yaw'][idx])
        gt_eul = np.array([np.deg2rad(gt_yaw), np.deg2rad(gt_pitch), np.deg2rad(gt_roll)], dtype=float)  # ZYX order
        gt_quat = eul2quat(gt_eul)  # [w, x, y, z]
        
        gt_GyroBiasX ,gt_GyroBiasY, gt_GyroBiasZ = data_dict['XKF']['GX'][idx], data_dict['XKF']['GY'][idx], data_dict['XKF']['GZ'][idx],
        gt_GyroBias = np.array([gt_GyroBiasX ,gt_GyroBiasY, gt_GyroBiasZ], dtype=float)

        # GT Full State vector        
        gtState = np.concatenate((gt_POS,gt_V,gt_quat,acc_bias,gt_GyroBias),axis=0)
        
        #DEAL LATER WHAT WOULD HAPPEN IF TRUE STATE IS OUT OF BORDER
        # if (hINS.NomState[0] > 8900) | (hINS.NomState[0] < -8900) | (hINS.NomState[1] > 8900) | (hINS.NomState[1] < -8900):
        #     print('UAV went outside of map border')
        #     break
        
        
        # ~~~ Get IMU data (body-frame) from the log IMU ~~~
        accX, accY, accZ = data_dict['IMU']['AccX'][idx], data_dict['IMU']['AccY'][idx], data_dict['IMU']['AccZ'][idx]
        gyrX, gyrY, gyrZ = data_dict['IMU']['GyrX'][idx], data_dict['IMU']['GyrY'][idx], data_dict['IMU']['GyrZ'][idx]

        acc_body  = np.array([accX, accY, accZ], dtype=float)
        gyro_body = np.array([gyrX, gyrY, gyrZ], dtype=float)
        inputParticle = [acc_body, gyro_body]  # store IMU measurement for input to MPF

        # ~~~ Predict INS states (dead-reckoning) ~~~
        hINS.predictIMU(acc_body, gyro_body, useVO)
        # Assuming rotation and altitude, vertical velocity is known
        hINS.NomState[2]    = gtState[2]
        hINS.NomState[5]    = gtState[5]
        # hINS.NomState[6:10] = gtState[6:10]

    #        ~~~ Store INS states ~~~
        #  - Position
        pN_INS, pE_INS, pD_INS = hINS.NomState[0:3]
        INS_pos = np.array([pN_INS, pE_INS, pD_INS], dtype=float) 
        INS_prd_position = ([pN_INS, pE_INS, -pD_INS])

        #  - Velocity
        vN_INS, vE_INS, vD_INS = hINS.NomState[3:6]
        INS_vel = np.array([vN_INS, vE_INS, vD_INS], dtype=float)
        INS_prd_velocity = ([vN_INS, vE_INS, -vD_INS])

        #  - Orientation: convert from quaternion to euler angles
        quat_INS = hINS.NomState[6:10]  # [w, x, y, z]
        eul_INS   = quat2eul(quat_INS) # [yaw, pitch, roll] rad
        INS_quat = np.array([quat_INS[1], quat_INS[2], quat_INS[3], quat_INS[0]], dtype=float)  # [x, y, z, w] 
        INS_prd_euler = (eul_INS) 
        
        INS_pred_state = np.concatenate((INS_pos,INS_vel,INS_quat,acc_bias,gt_GyroBias),axis=0)
        
        #UAV Snap Image(Get Measurement from Camera) 
        with Timer('UAV Cam'):  
            UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImage(DB = hStateEstimatorMPF.DataBaseScanner, showFeatures=showFeatures, showFrame=showFrame)
            # UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImage(DB = hStateEstimatorMPF.DataBaseScanner, showFeatures=showFeatures, UAVWorldPos= np.atleast_2d(gt_POS[0:3]),UAVYaw= quat2eul(gt_quat)[0] , showFrame=showFrame)
            if useGAN:
                UAVFrameMPF = UAVFakeFrame
            else:
                UAVFrameMPF = UAVFrame
                
            # UAVFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImageDataBase(hStateEstimatorMPF.DataBaseScanner,np.atleast_2d(gt_POS[0:3]),quat2eul(gt_quat)[0], showFeatures=showFeatures, showFrame=showFrame)

        ### Visual Odometry Estimation
        if useVO and (simTime % (dt_vo*n_VO) <= dt):
            frameVO = hUAVCamera.curr_frame
            if frameVO is not None:
                
                V_IMU_norm = np.linalg.norm(gtState[3:6])
                scale = V_IMU_norm * dt_vo
                
                R_cam, t_cam = hVisualOdometry.process_vo_frame(frameVO, scale)
                
                if t_cam is not None:
                    hINS.VO_update(gtState[0:3])
            
                n_VO += 1
        
        #### MPF Estimation
        closedLoop = True
        predPerclosedLoop = 1
        
        # Create Combined Frame of GT,INS DEAD RECKON, PARTICLES
        particlesPos = hStateEstimatorMPF.particles + hINS.NomState[0:3].reshape(-1, 1) # shape 3,N
        pxGT  = ned2px(gt_POS                , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        pxINS = ned2px(hINS.NomState[0:3]    , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        pxPF  = ned2px(particlesPos.T.copy() , hAIM.leftupperNED, hAIM.mp, hDB.pxRned)   
        pxPF_with_weights = np.hstack((pxPF, hStateEstimatorMPF.weights.reshape(-1, 1)))
        
        with Timer('MPF'):
            # for now, we use GT state as input to MPF 
            hStateEstimatorMPF.DataBaseScanner.snapDim = int(((-gt_POS[2]/fx) * 2 * cx) * (1/hAIM.mp)) , int(((-gt_POS[2]/fx) * 2 * cx) * (1/hAIM.mp))
            param = hStateEstimatorMPF.getEstimate(inputParticle, hINS.NomState, UAVKp, UAVDesc, closedLoop= closedLoop, predPerclosedLoop= predPerclosedLoop , UAVframe= UAVFrameMPF)
            # param = hStateEstimatorMPF.getEstimate(inputParticle, gtState       , UAVKp, UAVDesc, closedLoop= closedLoop, predPerclosedLoop= predPerclosedLoop , UAVframe= UAVFrameMPF)

        estState = hINS.correctINS(param["State"], closedLoop= closedLoop, predPerclosedLoop = predPerclosedLoop)
        # estState[0:3] = t_cam
        particlesPos = hStateEstimatorMPF.particles + hINS.NomState[0:3].reshape(-1, 1) # shape 3,N
        # particlesPos = hStateEstimatorMPF.particles + gt_POS.reshape(-1, 1) # shape 3,N

        FramemostLikelihoodPart = hStateEstimatorMPF.FramemostLikelihoodPart
        
        # FramemostLikelihoodPart = hDB.snapPartImage(gt_POS, np.deg2rad(gt_yaw), None)

        # Storing values
        gt_position_list.append(gt_POS)
        INS_prd_position_list.append(hINS.NomState[0:3].copy())
        PF_position_list.append(estState[0:3].copy())
        PF_particles_position_list.append(particlesPos.T.copy())

        # gt_position_list.append([gt_POS])
        # gt_velocity_list.append([gt_V])
        # gt_euler_list.append(gt_eul)

        # Time update
        simTime += dt
        idx += 1
        
        # # Create Combined Frame of GT,INS DEAD RECKON, PARTICLES
        # pxGT  = ned2px(gt_POS                , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        # pxINS = ned2px(hINS.NomState[0:3]    , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        # pxPF  = ned2px(particlesPos.T.copy() , hAIM.leftupperNED, hAIM.mp, hDB.pxRned)   
        # pxPF_with_weights = np.hstack((pxPF, hStateEstimatorMPF.weights.reshape(-1, 1)))
        
        flagErrorPlot = False
        flagFramePlot = True
        
        if ((simTime % (dt * sim_per_plot)) < 0.1):
                    
            # Error Plotter
            if flagErrorPlot:
                ErrorPlotter.update(gtState[0:16],hINS.NomState[0:16], estState[0:16], timeConstant = dt*sim_per_plot)
        
            # Update feature plot
            if flagFramePlot:
                combinedFrame = combineFrame(hAIM.I, pxGT, None, pxPF_with_weights)
                CamPlotter.snapNow(
                                (UAVFrame                 , 'UAV Camera'                         , f'Flight time is {simTime:.2f} s '), # \n Detected Features: {UAVKp.shape[0]}'),
                                (UAVFakeFrame             , 'Generated Fake SAT Img'             , f'Detected Features: {UAVKp.shape[0]}'),
                                (FramemostLikelihoodPart  , 'Most likelihood Particle SAT View'  , f'Detected Features: {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[0]} \n Matched features:  {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[1]}'),
                                (combinedFrame            , 'Particles, Ground Truth in Map'     , f'Position XY RMSE: {np.sqrt(np.mean((gt_POS[0:2] - estState[0:2])**2)):.2f} m'),)
        
        # if t_cam is not None:
        #     TwoPlotter.update_plots(t_cam,gt_POS)
        
        gtState_list.append(gtState)
        estState_list.append(estState)
        INSpredState_list.append(INS_pred_state)
 
plot_positions(gt_position_list,INS_prd_position_list,PF_position_list,PF_particles_position_list,plot_2d= True)

# frames_array = np.stack(gtState_list, axis=0)
# np.save('data_GT_try_like', frames_array)

# frames_array = np.stack(estState_list, axis=0)
# np.save('data_EST_like_th', frames_array)

# frames_array = np.stack(INSpredState_list, axis=0)
# np.save('data_INS', frames_array)