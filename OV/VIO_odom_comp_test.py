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
from odom_subscriber import OdomAndMavrosSubscriber
from utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter, visualize2DgenTraj



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

GT_pos_list  = []
GT_vel_list  = []
GT_eul_list  = []

is_first_messages = True
dt = 0.1  # Time step for the simulation

while True:
    #node_OdomVIO.first_vo_msg
    #node_OdomVIO.VIO_dict
    #is_velocity_body = True
    if  node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_gps_fix_msg:


        if is_first_messages:

            yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict.copy(), node_OdomVIO.gt_odom_dict.copy())
            print(np.rad2deg(yaw_diff))

            # yaw_diff += np.deg2rad(10)

            # Initialize yaw vector from GPS/mag/inital guess 
            # VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)

            is_first_messages = False
        else:
            # yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict.copy(), node_OdomVIO.gt_odom_dict.copy())
            # print('yaw diff:', np.rad2deg(yaw_diff))

            GT_dict  = ned_VIO_converter(node_OdomVIO.gt_odom_dict.copy(), 0, is_velocity_body = False)
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)


            VIO_pos  = VIO_dict['position']
            VIO_vel  = VIO_dict['velocity']
            VIO_quat = VIO_dict['orientation']
            VIO_eul  = np.rad2deg(quat2eul(VIO_quat))

            GT_pos  = GT_dict['position']
            GT_vel  = GT_dict['velocity']
            GT_quat = GT_dict['orientation']
            GT_eul  = np.rad2deg(quat2eul(GT_quat))

            VIO_vel_norm = np.linalg.norm(VIO_vel)
            GT_vel_norm  = np.linalg.norm(GT_vel)

            print('VIO velocity norm: {:.6f}, GT velocity norm: {:.6f}'.format(VIO_vel_norm, GT_vel_norm))

            # print("Initialization status:", node_OdomVIO.initialization_status)
            # print("Ready status:", node_OdomVIO.ready_status)

            # Store the states for comparison
            VIO_pos_list.append(VIO_pos)
            VIO_vel_list.append(VIO_vel)
            VIO_eul_list.append(VIO_eul)

            GT_pos_list.append(GT_pos)
            GT_vel_list.append(GT_vel)
            GT_eul_list.append(GT_eul)

            try:
                time.sleep(dt)
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
                np.save('VIO_pos_list.npy', np.array(VIO_pos_list))
                np.save('VIO_vel_list.npy', np.array(VIO_vel_list))
                np.save('VIO_eul_list.npy', np.array(VIO_eul_list))
                np.save('GT_pos_list.npy', np.array(GT_pos_list))
                np.save('GT_vel_list.npy', np.array(GT_vel_list))
                np.save('GT_eul_list.npy', np.array(GT_eul_list))
                print("Data saved successfully. Exiting...")

                break



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

 