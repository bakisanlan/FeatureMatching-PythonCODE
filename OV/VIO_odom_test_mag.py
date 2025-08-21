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
    if  node_OdomVIO.first_imu_mag_msg:


        # DynamicPlot.update(node_OdomVIO.mag, dt = 1)

        print("-----------------------------------------------------")
        print("Magnetometer Yaw: ", node_OdomVIO.magYawDeg)

        continue


  

    else:
        print("-----------------------------------------------------")
        if not node_OdomVIO.first_imu_mag_msg:
            print("Waiting for first IMU magnetometer message...")

        print("-----------------------------------------------------")
        time.sleep(1)
        continue



 