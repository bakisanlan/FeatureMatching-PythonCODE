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

# Custom libraries
# Add the path to the utils module if it's not in the same directory
from odom_subscriber import OdomAndMavrosSubscriber
from PixhawkCommander import PixhawkCommander

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import quat2eul, eul2quat
from utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter, visualize2DgenTraj
from utils_OV.controller_utils import ControllerManager, from_pos_vel_to_angle_ref
from utils_OV.guidance_utils import TrajectoryGeneratorV2 

# Initialize ROS 2 nodes
rclpy.init()
node_OdomVIO     = OdomAndMavrosSubscriber()
node_PixhawkCMD  = PixhawkCommander()
# Create a multithreaded executor and add both nodes
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
executor.add_node(node_PixhawkCMD)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
# node_OdomVIO.destroy
# executor.shutdown()_node()
# node_PixhawkCMD.destroy_node()
# rclpy.shutdown()

is_first_messages = True

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

# # --- Position Controller LOG ---
# csv_file_pos_cont = open('pos_controller_log.csv', 'w', newline='')
# writer_pos = csv.writer(csv_file_pos_cont)
# writer_pos.writerow([
#     't',
# ])
LOG = True


# Guidance and control settings
wp_list = [[0, 0, 0],
           [100, 0, 0],
           [100, 500, 0],
           [500, 500, 0],
           [500, 200, 0],
           [200, 200, 0],
           [200, 0, 0],
           [0, 0, 0]]

alt_target_climb = 60.0  # Target altitude for climb

hControllerManager = ControllerManager(wp_list, alt_target_climb)
controller_dt     = hControllerManager.controller_dt

# Store list of posiitions for comparison
VIO_pos_list = []
GT_pos_list  = []

while True:

    # auto takeoff when VIO ready status(cam, imu ready) and first state message received
    if node_OdomVIO.ready_status and node_OdomVIO.first_state_msg:
        
        # Wait for mode to be GUIDED
        while True:
            print("Waiting for mode to be GUIDED/GUIDED_NOGPS")
            mode = node_OdomVIO.state_dict['mode']
            if (mode == "GUIDED" or mode == "GUIDED_NOGPS"):

                print("Start takoff process")
                break
            time.sleep(0.1)
            
            
        hControllerManager.control_UAV(node_OdomVIO, node_PixhawkCMD)

    else:
        print("-----------------------------------------------------")

        if not node_OdomVIO.first_gt_odom_msg:
            print("Waiting for first ground truth odometry message...")

        if not node_OdomVIO.first_gps_fix_msg:
            print("Waiting for first GPS fix message...")

        if not node_OdomVIO.first_state_msg:
            print("Waiting for first state message...")
            
        if not node_OdomVIO.first_camera_msg:
            print("Waiting for first camera message...")

        print("-----------------------------------------------------")
        time.sleep(1)
        continue

 


