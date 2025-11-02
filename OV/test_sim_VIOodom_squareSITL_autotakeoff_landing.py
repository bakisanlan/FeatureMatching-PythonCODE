import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
# import pymap3d as pm
# import numpy as np
# import csv
import sys
import os
# import yaml
import threading
# import logging
# from pathlib import Path
# from datetime import datetime
# Add the path to the utils module if it's not in the same directory
from odom_subscriber import OdomAndMavrosSubscriber
from PixhawkCommander import PixhawkCommander

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import setup_logging #quat2eul, eul2quat 
# from utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter, visualize2DgenTraj
from utils_OV.controller_utils import ControllerManager#, from_pos_vel_to_angle_ref
# from utils_OV.guidance_utils import TrajectoryGeneratorV2 


# #Start logging - MUST be before ROS 2 init to capture node prints
# setup_logging()

# Initialize ROS 2 nodes
rclpy.init()
setup_logging()
node_OdomVIO     = OdomAndMavrosSubscriber()
node_PixhawkCMD  = PixhawkCommander()
# Create a multithreaded executor and add both nodes
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
executor.add_node(node_PixhawkCMD)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


# Guidance and control settings
# wp_list = [[0, 0, 0],
#            [100, 0, 0],
#         #    [100, 500, 0],
#         #    [500, 500, 0],
#         #    [500, 200, 0],
#         #    [200, 200, 0],
#         #    [200, 0, 0],
#            [0, 0, 0]]

# wp_list = [[0, 0, 0],
#            [200, 0, 0],
#            [200, -200, 0],
#            [100, -200, 0],
#            [100, -100, 0],
#            [50,  -100, 0],
#            [50, -200, 0],
#            [0, -200, 0],
#            [0,  0, 0],
#            [200, 0, 0],
#            [200, -200, 0],
#            [100, -200, 0],
#            [100, -100, 0],
#            [50,  -100, 0],
#            [50, -200, 0],
#            [0, -200, 0],
#            [0,  0, 0]]

# wp_list = [[0, 0, 0],
#            [0, -100, 0],
#            [100, -100, 0],
#            [100, 0, 0],
#            [0, 0, 0],
#            [0, -100, 0],
#            [100, -100, 0],
#            [100, 0, 0],
#            [0, 0, 0],
#            [0, -100, 0],
#            [100, -100, 0],
#            [100, 0, 0],
#            [0, 0, 0]]

wp_list = [[0, 0, 0],
            [50, 0, 0],
            [50, -250, 0],
            [250, -250, 0],
            [250, -100, 0],
            [100, -100, 0],
            [100, 0, 0],
            [0, 0, 0],
            [50, 0, 0],
            [50, -250, 0],
            [250, -250, 0],
            [250, -100, 0],
            [100, -100, 0],
            [100, 0, 0],
            [0, 0, 0]]


alt_target_climb = 60.0  # Target altitude for climb

hControllerManager = ControllerManager(wp_list, alt_target_climb)
controller_dt      = hControllerManager.controller_dt

# Store list of posiitions for comparison
VIO_pos_list = []
GT_pos_list  = []

while True:

    # auto takeoff when VIO ready status(cam, imu ready) and first state message received
    if node_OdomVIO.ready_status and node_OdomVIO.first_state_msg and node_OdomVIO.first_pressure_msg:
        
        # Wait for mode to be GUIDED
        while not (node_OdomVIO.state_dict['mode'] == "GUIDED" or node_OdomVIO.state_dict['mode'] == "GUIDED_NOGPS"):
            print("Waiting for mode to be GUIDED/GUIDED_NOGPS, current mode:", node_OdomVIO.state_dict['mode'])
            time.sleep(1)
        
        # Call the controller manager if mode is GUIDED/GUIDED_NOGPS        
        hControllerManager.control_UAV(node_OdomVIO, node_PixhawkCMD)

        print("Exiting main loop.")
        break

    else:
        print("-----------------------------------------------------")

        if not node_OdomVIO.first_pressure_msg:
            print("Waiting for first pressure(barometric altimeter) message...")

        if not node_OdomVIO.first_state_msg:
            print("Waiting for first state message...")
            
        if not node_OdomVIO.first_camera_msg:
            print("Waiting for first camera message...")

        print("-----------------------------------------------------")
        time.sleep(1)
        continue




