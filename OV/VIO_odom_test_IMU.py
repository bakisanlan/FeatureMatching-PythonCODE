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
from plotter import DynamicPlot



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

DynamicPlot = DynamicPlot()

# Store list of posiitions for comparison

is_first_messages = True
dt = 0.1  # Time step for the simulation

while True:
    #node_OdomVIO.first_vo_msg
    #node_OdomVIO.VIO_dict
    #is_velocity_body = True
    if  node_OdomVIO.first_imu_msg and node_OdomVIO.first_vo_msg:

         # Check if VIO is initialized correctly
        
        IMU_linear_acc = node_OdomVIO.IMU_RAW['body_linear_acceleration']
        VIO_linear_acc = node_OdomVIO.VIO_dict['body_linear_acceleration']

        if any(IMU_linear_acc) is not None and any(VIO_linear_acc) is not None:

            # Append the current acceleration magnetiude to the buffers
            # IMU_acc_buf.append(np.linalg.norm(np.array(IMU_linear_acc)))
            # VIO_acc_buf.append(np.linalg.norm(np.array(VIO_linear_acc)))

            # # Check if the average acceleration magnetiude is within a reasonable range
            # if len(IMU_acc_buf) == window_size and len(VIO_acc_buf) == window_size:
            #     acc_diff = np.abs(np.array(IMU_acc_buf) - np.array(VIO_acc_buf))

            # acc_diff_avg = np.mean(acc_diff)


            acc_diff_avg = np.array([np.abs(np.linalg.norm(np.array(IMU_linear_acc)) - np.linalg.norm(np.array(VIO_linear_acc)))])



                # if acc_diff_avg 
            print(VIO_linear_acc)
            print(IMU_linear_acc)
            print(acc_diff_avg)
            DynamicPlot.update(acc_diff_avg, dt = 1)

        print("-----------------------------------------------------")
        # print("Magnetometer Yaw: ", node_OdomVIO.magYawDeg)


  

    else:
        print("-----------------------------------------------------")
        if not node_OdomVIO.first_imu_mag_msg:
            print("Waiting for first IMU magnetometer message...")

        print("-----------------------------------------------------")
        time.sleep(1)
        continue

 