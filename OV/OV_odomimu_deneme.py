from odom_subscriber import OdomAndMavrosSubscriber
import rclpy
import threading
import time
import pymap3d as pm
import numpy as np
import csv
import sys
import os

# Custom libraries
# Add the path to the utils module if it's not in the same directory
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import *

rclpy.init()
node = OdomAndMavrosSubscriber()
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

first_flag = True

# --- open CSV and write header ---
csv_file = open('vio_gps_5hz_0107_10.csv', 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow([
    't', 
    # VIO in ENU
    'vio_e', 'vio_n', 'vio_u', 
    # VIO in LLA
    'vio_lat', 'vio_lon', 'vio_alt',
    # GPS in ENU
    'gps_e', 'gps_n', 'gps_u',
    # GPS in LLA
    'gps_lat', 'gps_lon', 'gps_alt',
])


while True:

    if node.first_vo_msg and node.first_gt_odom_msg and node.home_received and node.first_imu_mag_msg and node.first_gps_fix_msg:
    # if node.first_vo_msg:

        if first_flag:

            quat_vio = node.VIO_dict['orientation']
            qx, qy, qz, qw = quat_vio
            R_vio_old = quat2rotm([qw, qx, qy, qz])

            euler_vio = quat2eul([qw, qx, qy, qz], order='ZYX')   # Quaternion(s) should be in [w, x, y, z] format as input to that quat2eul function
            yaw_vio = euler_vio[0]

            quat_gt_odom = node.gt_odom_dict['orientation']
            qx_gt, qy_gt, qz_gt, qw_gt = quat_gt_odom
            R_gt_odom_first = quat2rotm([qw_gt, qx_gt, qy_gt, qz_gt])
            euler_gt_odom = quat2eul([qw_gt, qx_gt, qy_gt, qz_gt], order='ZYX')
            yaw_gt_odom = euler_gt_odom[0]

            yaw_diff = yaw_gt_odom - yaw_vio
            print(f"yaw_diff : {np.rad2deg(yaw_diff)}")

            first_flag = False
            start_time = time.time()

            # print(f"(yaw, pitch, roll)deg VIO : {np.rad2deg(euler_vio)} | GT Odom : {np.rad2deg(euler_gt_odom)}")
        else:
            time.sleep(1/5)
            # quat_vio = node.VIO_dict['orientation']
            # qx, qy, qz, qw = quat_vio
            # R_vio_new = quat2rotm([qw, qx, qy, qz])
            # R_yaw_correction = quat2rotm(eul2quat([yaw_diff, 0, 0], order='ZYX'))    # vio to inertia
            # R_vio_yaw_corrected = R_yaw_correction.dot(R_vio_new)
            # R_vio_diff = R_vio_new @ R_vio_old.T
            # R_vio_sol = R_vio_diff @ R_gt_odom_first
            # eul_vio_sol = np.rad2deg(quat2eul(rotm2quat(R_vio_sol)))
            # R_vio_old = R_vio_new
            # R_gt_odom_first = R_vio_sol
            
            # # euler_vio = quat2eul([qw, qx, qy, qz], order='ZYX')

            # quat_gt_odom = node.gt_odom_dict['orientation']
            # qx_gt, qy_gt, qz_gt, qw_gt = quat_gt_odom
            # euler_gt_odom = np.rad2deg(quat2eul([qw_gt, qx_gt, qy_gt, qz_gt], order='ZYX'))

            # # print(f"(yaw, pitch, roll)deg VIO : {np.rad2deg(euler_vio)} | GT Odom : {np.rad2deg(euler_gt_odom)}")
            # # print(f"(yaw, pitch, roll)deg VIO : {eul_vio_sol} | GT Odom : {euler_gt_odom}")

            # # print(f"GT Odom : {euler_gt_odom}")
            # # print(f"(yaw, pitch, roll)deg VIO : {np.rad2deg(quat2eul(rotm2quat(R_vio_yaw_corrected)))}, GT Odom : {euler_gt_odom}")

            # #GEt ENU position estimation from VIO
            # pos_vio = node.VIO_dict['position']
            # vio_enu_pos =  R_yaw_correction @ pos_vio
            # # print(f"ENU position estimation from VIO : {vio_enu}")
            # vio_lla_pos = pm.enu2geodetic(vio_enu_pos[0]  , vio_enu_pos[1]  , vio_enu_pos[2],
            #                           node.home_loc[0], node.home_loc[1], node.home_loc[2])

            # # print(f"LLA from VIO ENU position estimation : {vio_lla_pos[0]:.7f}, {vio_lla_pos[1]:.7f}, {vio_lla_pos[2]:.2f} m,  --- VIO ENU position estimation : {vio_enu_pos[0]:.2f}, {vio_enu_pos[1]:.2f}, {vio_enu_pos[2]:.2f} m")
            # gps_fix_loc_lla = node.gps_fix_loc
            # gps_fix_loc_enu = node.gt_odom_dict['position']

            # # print(f"GPS Fix location : {gps_fix_loc_lla[0]:.7f}, {gps_fix_loc_lla[1]:.7f}, {gps_fix_loc_lla[2]:.2f} m")

            # # print(quat2rotm(eul2quat([-yaw_diff, 0, 0], order='ZYX')))
            # print(gps_fix_loc_enu)

            # # write one row
            # t = time.time() - start_time
            # writer.writerow([
            #     f"{t:.3f}",
            #     f"{vio_enu_pos[0]:.3f}", f"{vio_enu_pos[1]:.3f}", f"{vio_enu_pos[2]:.3f}",
            #     f"{vio_lla_pos[0]:.7f}", f"{vio_lla_pos[1]:.7f}", f"{vio_lla_pos[2]:.2f}",
            #     f"{gps_fix_loc_enu[0]:.3f}", f"{gps_fix_loc_enu[1]:.3f}", f"{gps_fix_loc_enu[2]:.3f}",
            #     f"{gps_fix_loc_lla[0]:.7f}", f"{gps_fix_loc_lla[1]:.7f}", f"{gps_fix_loc_lla[2]:.2f}",
            #     ])
        csv_file.flush()  # ensure it's written
