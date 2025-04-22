from UAVCamera import UAVCamera
from VisualOdometry import VisualOdometry
import numpy as np  
from utils import getLogData,quat2eul,quat2rotm,eul2quat,wrap2_180,rotm2quat
from plotter import PlotCamera 
import os
from PIL import Image
import matplotlib.pyplot as plt

#####Sim settings
simTime = 0
Tf      = 120
num_level = 1
ReferenceFrame = 'NED'
MAP            = 'itu'  #unity_fl1
detector       = 'SP'
snapFrame = 1
IMUtype = 2  #deal later

#####Extract LOG data
csv_path = "data/log/winter_flight.csv"  # Replace with your actual path
# start_idx = 1306  #for itu video2
# end_idx   = 2622
start_idx = 4465  #for itu winter
# end_idx   = 5389
end_idx   = 6000
data_dict = getLogData(csv_path, start_row=start_idx, end_row=end_idx)

#Get dt
timestamps_ms = data_dict['timestamp']
dt_array = np.diff(timestamps_ms) / 1000.0  # convert to seconds
# Possibly handle if dt varies or take the mean
dt = np.mean(dt_array)
sim_per_plot = 10


# UAV Camera
useCUT = False
showFeatures = False
showFrame = True
usePreVideo = True
snap_dim = (300,300) #deal later
time_offset = 10     #initilize video cam at 5th second(which is altitude is constant for itu winter video)
fps = 60
hUAVCamera = UAVCamera(dt = dt, snapFrame = snapFrame, snap_dim = snap_dim, fps = fps, cropFlag = True, resizeFlag = True, time_offset= time_offset, useCUT = useCUT, usePreVideo = usePreVideo)

# -------------------------------------------------------------------------
#    Get initial state from XKF states: PN, PE, PD, VN, VE, VD, Roll, Pitch, Yaw
# -------------------------------------------------------------------------
# UAV_init_NED_pos_rel_img_center  = np.array([-355.8719, 187.2249, 0],dtype= float) #video 30 seconds offset position, LLA = [41.1010349, 29.0254601, 0], we are going to use this as ofsett that account UAV position relative to center of the image
UAV_init_NED_pos_rel_img_center  = np.array([-405.8719, 171.2249, 0],dtype= float) #video initial for itu winter,   LLA = [41.1004505, 29.0252685], we are going to use this as ofsett that account UAV position relative to center of the image
pN_init,pE_init, pD_init       = data_dict['XKF']['PN'][0] , data_dict['XKF']['PE'][0] , data_dict['XKF']['PD'][0]
pos0_org = np.array([pN_init, pE_init, pD_init])
pos0 = UAV_init_NED_pos_rel_img_center

vN_init, vE_init, vD_init      = data_dict['XKF']['VN'][0] , data_dict['XKF']['VE'][0] , data_dict['XKF']['VD'][0]
V0 = np.array([vN_init, vE_init, vD_init])

roll_deg, pitch_deg, yaw_deg   = data_dict['XKF']['Roll'][0] , data_dict['XKF']['Pitch'][0] , data_dict['XKF']['Yaw'][0]
# Wrap and convert angles to radians
roll_rad  = np.deg2rad(wrap2_180(roll_deg))
pitch_rad = np.deg2rad(wrap2_180(pitch_deg))
yaw_rad   = np.deg2rad(wrap2_180(yaw_deg))
# Reorder for ZYX => [yaw, pitch, roll]
eul_angles_init = np.array([yaw_rad, pitch_rad, roll_rad])
init_quat       = eul2quat(eul_angles_init)  # [w, x, y, z]
quat0 = init_quat


# Visual Odometry 
n_VO = 1e-5
useVO = True
dt_vo = 20*dt
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

framesUAV_org =  np.load('data/cyclegan/turbo/frames_generated/itu_winter_org.npy')  # shape: (num_frames, 256, 256, 3)
idx = 0
prev_gt_POS = np.zeros(3)

VIO_list = []
GT_list = []

CamPlotter = PlotCamera(snapFrame= snapFrame)
t_cam = None
eul_cam = None

frameVO_list = []

while simTime < Tf:
    
    # ~~~ Grab the XKF as "ground truth" ~~~
    gt_pN, gt_pE, gt_pD = data_dict['XKF']['PN'][idx], data_dict['XKF']['PE'][idx], data_dict['XKF']['PD'][idx]
    gt_POS = np.array([gt_pN, gt_pE, gt_pD] ,dtype=float) - pos0_org + UAV_init_NED_pos_rel_img_center  #offset UAV position relative to center of the image

    gt_vN, gt_vE, gt_vD = data_dict['XKF']['VN'][idx], data_dict['XKF']['VE'][idx], data_dict['XKF']['VD'][idx]
    gt_V = np.array([gt_vN, gt_vE, gt_vD], dtype=float)

    gt_roll, gt_pitch, gt_yaw = wrap2_180(data_dict['XKF']['Roll'][idx]), wrap2_180(data_dict['XKF']['Pitch'][idx]), wrap2_180(data_dict['XKF']['Yaw'][idx])
    gt_eul = np.array([np.deg2rad(gt_yaw), np.deg2rad(gt_pitch), np.deg2rad(gt_roll)], dtype=float)  # ZYX order
    gt_quat = eul2quat(gt_eul)  # [w, x, y, z]
    
    gt_GyroBiasX ,gt_GyroBiasY, gt_GyroBiasZ = data_dict['XKF']['GX'][idx], data_dict['XKF']['GY'][idx], data_dict['XKF']['GZ'][idx],
    gt_GyroBias = np.array([gt_GyroBiasX ,gt_GyroBiasY, gt_GyroBiasZ], dtype=float)

    
    # UAVFrame, UAVFakeFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImage(hStateEstimatorMPF.DataBaseScanner, showFeatures=showFeatures, showFrame=showFrame)
    hUAVCamera.time += dt 

    frameVO     = framesUAV_org[int(hUAVCamera.time * hUAVCamera.fps)]
    # frameVO     = framesUAV_org[idx*100]

    # frameVO = hUAVCamera.curr_frame

    if useVO and (simTime % (dt_vo*n_VO) <= dt):

        if frameVO is not None:
            
            V_IMU_norm = np.linalg.norm(gt_V)
            scale = V_IMU_norm * dt_vo
            
            R_cam, t_cam = hVisualOdometry.process_vo_frame(frameVO, scale)
            R_cam = hVisualOdometry.R_rel
            t_cam = hVisualOdometry.t_rel
            
            # if t_cam is not None:
            #     hINS.VO_update(gtState[0:3])
            n_VO += 1
            
            frameVO_list.append(frameVO)
            # CamPlotter.snapNow((frameVO, 'a', f't_cam : {t_cam} \n eul_cam: {eul_cam}'), hVisualOdometry.VO_matchframe)
            
            
            
            

        
    if t_cam is not None:
        t_cam_gt = gt_POS - prev_gt_POS 
        
        quat_cam = rotm2quat(R_cam)
        eul_cam  = np.rad2deg(quat2eul(quat_cam))
        
        VIO_list.append(t_cam)
        GT_list.append(t_cam_gt)
    
    prev_gt_POS = gt_POS

    CamPlotter.snapNow((hVisualOdometry.VO_frame, 'a', f't_cam : {t_cam} \n eul_cam: {eul_cam}'), hVisualOdometry.VO_matchframe)
    
    
    simTime += dt
    idx += 1
    
# Create a folder to save the images
output_folder = "output_images"
os.makedirs(output_folder, exist_ok=True)

# Save all images in frameVO_list to the folder
for i, frame in enumerate(frameVO_list):
    image_path = os.path.join(output_folder, f"frame_{i}.png")
    image = Image.fromarray(frame.astype('uint8'))
    image.save(image_path)

plt.figure()
plt.plot(VIO_list, label='VIO')
plt.plot(GT_list, label='GT')
plt.legend()
plt.show() 