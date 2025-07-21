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

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import quat2eul, eul2quat
from utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter, visualize2DgenTraj
from utils_OV.controller_utils import PositionControllerBumpless, from_pos_vel_to_angle_ref
from utils_OV.guidance_utils import TrajectoryGeneratorV2 

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


# Store list of posiitions for comparison
VIO_pos_list = []
GT_pos_list  = []

while True:
    #node_OdomVIO.first_vo_msg
    #node_OdomVIO.VIO_dict
    #is_velocity_body = True
    if  node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_imu_mag_msg and node_OdomVIO.first_gps_fix_msg and node_OdomVIO.first_state_msg:
        print("All node sensors are initialized.")

        # Check if the first messages of all publishers are received
        if is_first_messages:
            print("First messages received, processing data...")

            yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict.copy(), node_OdomVIO.gt_odom_dict.copy())

            # Initialize yaw vector from GPS/mag/inital guess 
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
            euler_gt = quat2eul(VIO_dict['orientation'])
            yaw_vector = np.array([np.cos(euler_gt[0]), np.sin(euler_gt[0])])
            print("Initial yaw vector:", yaw_vector)

            # Flag for first messages
            is_first_messages = False
            start_time = time.time()

        # Main loop
        else:

            # Wait for mode to be GUIDED
            while True:
                print("Waiting for mode to be GUIDED")
                mode = node_OdomVIO.state_dict['mode']
                if (mode == "GUIDED" or mode == "GUIDED_NOGPS"):

                    print("Start trajectory")
                    break
                time.sleep(0.1)

            # Getting first VIO data with converting to NED frame
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
            VIO_pos_first = np.array(VIO_dict['position'].copy())

            t_prev = time.time()
            last_print_time = time.time() - 4

            # Guidance and control loop
            while True:

                # GEt odometry data from VIO
                VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
                VIO_pos = np.array(VIO_dict['position'])
                VIO_vel = np.array(VIO_dict['velocity'])
                # ref_vel = VIO_vel

                # Get ground truth odometry data from GPS fix local frame
                GT_dict = ned_VIO_converter(node_OdomVIO.gt_odom_dict.copy(), 0, is_velocity_body = False)
                GT_pos = np.array(GT_dict['position'])
                GT_vel = np.array(GT_dict['velocity'])

                # Set attitude and thrust to Pixhawk from attitude reference and thrust reference

                # Store UAV position and GT position for visualization
                VIO_pos_list.append(VIO_pos[0:2].copy())
                GT_pos_list.append(GT_pos[0:2].copy())


                mode = node_OdomVIO.state_dict['mode']
                if not (mode == "GUIDED" or mode == "GUIDED_NOGPS") :
                    print("Mode is not GUIDED or GUIDED_NOGPS, stopping trajectory.")
                    # visualize2DgenTraj(generated_traj['pos'][:,0:2], np.array(UAV_pos_list))
                    date_var = time.strftime("%Y%m%d-%H%M%S")
                    np.save('logs/VIO_pos_list_{}.npy'.format(date_var),   np.array(VIO_pos_list))
                    np.save('logs/GT_pos_list_{}.npy'.format(date_var),    np.array(GT_pos_list))

                    # Reset lists
                    VIO_pos_list = []
                    GT_pos_list  = []

                    break
            

    else:
        print("-----------------------------------------------------")
        if not node_OdomVIO.first_vo_msg:
            print("Waiting for first VIO message...")

        if not node_OdomVIO.first_gt_odom_msg:
            print("Waiting for first ground truth odometry message...")

        if not node_OdomVIO.home_received:
            print("Waiting for home position...")

        if not node_OdomVIO.first_imu_mag_msg:
            print("Waiting for first IMU magnetometer message...")

        if not node_OdomVIO.first_gps_fix_msg:
            print("Waiting for first GPS fix message...")

        print("-----------------------------------------------------")
        time.sleep(1)
        continue
