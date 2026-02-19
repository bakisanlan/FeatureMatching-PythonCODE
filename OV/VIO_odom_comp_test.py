import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import pymap3d as pm
import numpy as np
import csv
import sys
import os
import yaml
import threading
import matplotlib.pyplot as plt



# Custom libraries
# Add the path to the utils module if it's not in the same directory
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import quat2eul, eul2quat
from plotter import plot_VIO_GT_comp_states 
from OV.odom_subscriber_tryYawCorrection import OdomAndMavrosSubscriber
from utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter
from plotter import visualizeTraj



# Initialize ROS 2 nodes
rclpy.init()
node_OdomVIO     = OdomAndMavrosSubscriber()
# Create a multithreaded executor and add both nodes
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
# node_OdomVIO.destroy
# executor.shutdown()_node()
# node_PixhawkCMD.destroy_node()
# rclpy.shutdown()

is_first_messages = True

# Waypoint Navigation Parameters
traj_id = 0
ref_pos = np.array([0.0, 0.0, -0.0])
ref_vel = np.array([0.0, 0.0, 0.0])

# Store list of posiitions for comparison
VIO_pos_list = []
VIO_vel_list = []
VIO_eul_list = []
VIO_ts_list = []

GT_pos_list  = []
GT_vel_list  = []
GT_eul_list  = []
GT_ts_list = []

yaw_corrected = False

is_first_messages = True
dt = 0.1  # Time step for the simulation

try:


    while True:
        #node_OdomVIO.first_vo_msg
        #node_OdomVIO.VIO_dict
        #is_velocity_body = True
        if  node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_gps_fix_msg:


            while node_OdomVIO.VIOned_dict['ts'] is None:

                time.sleep(0.1)
                print('waiting to yaw ref come') 



            VIO_dict = node_OdomVIO.VIOned_dict.copy()
            GT_dict  = node_OdomVIO.GTned_dict.copy()


            VIO_pos  = VIO_dict['position']
            VIO_vel  = VIO_dict['velocity']
            VIO_quat = VIO_dict['orientation']
            VIO_eul  = np.rad2deg(quat2eul(VIO_quat))
            VIO_ts   = VIO_dict['ts']

            GT_pos  = GT_dict['position']
            GT_vel  = GT_dict['velocity']
            GT_quat = GT_dict['orientation']
            GT_ts   = GT_dict['ts']


            # if abs(GT_pos[2]) > 50 and not yaw_corrected:
            #     node_OdomVIO._update_yaw_difference()
            #     yaw_corrected = True
            #     print("Yaw difference corrected.")


            try:
                GT_eul  = np.rad2deg(quat2eul(GT_quat))

            except:
                print(GT_quat)
                GT_eul = np.array([0.0, 0.0, 0.0])


            VIO_vel_norm = np.linalg.norm(VIO_vel)
            GT_vel_norm  = np.linalg.norm(GT_vel)
            
            # Store the states for comparison
            VIO_pos_list.append(VIO_pos)
            VIO_vel_list.append(VIO_vel)
            VIO_eul_list.append(VIO_eul)
            VIO_ts_list.append(VIO_ts)

            GT_pos_list.append(GT_pos)
            GT_vel_list.append(GT_vel)
            GT_eul_list.append(GT_eul)
            GT_ts_list.append(GT_ts)


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

except KeyboardInterrupt:
    print("KeyboardInterrupt detected. Stopping the script...")
    # plt.figure()
    # plt.plot(VIO_vel_list, label='VIO Velocity Norm')
    # plt.plot(GT_vel_list, label='GT Velocity Norm')
    # plt.xlabel('Sample')
    # plt.ylabel('Velocity Norm')
    # plt.title('VIO vs GT Velocity Norm')
    # plt.legend()
    # plt.show()

    plot_VIO_GT_comp_states(
        VIO_pos_list, VIO_vel_list, VIO_eul_list,
        GT_pos_list, GT_vel_list, GT_eul_list,
        dt = dt)
    
    # save VIO and GT list as npy
    np.save('VIO_pos_list_test.npy', np.array(VIO_pos_list))
    np.save('VIO_vel_list_test.npy', np.array(VIO_vel_list))
    np.save('VIO_eul_list_test.npy', np.array(VIO_eul_list))
    np.save('VIO_ts_list_test.npy', np.array(VIO_ts_list))
    np.save('GT_pos_list_test.npy', np.array(GT_pos_list))
    np.save('GT_vel_list_test.npy', np.array(GT_vel_list))
    np.save('GT_eul_list_test.npy', np.array(GT_eul_list))
    np.save('GT_ts_list_test.npy', np.array(GT_ts_list))
    print("Data saved successfully. Exiting...")





