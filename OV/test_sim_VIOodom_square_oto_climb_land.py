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
from utils_OV.controller_utils import PositionControllerBumpless, from_pos_vel_to_angle_ref
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
with open('./config/guidance_and_control_parameters.yaml') as f:
    gc_params = yaml.safe_load(f)

pos_controller_x = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], gc_params['max_acc_xy'], log_file_name ="logs/x_ref.txt")
pos_controller_y = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], gc_params['max_acc_xy'], log_file_name ="logs/y_ref.txt")
controller_dt = gc_params['gc_dt']
max_acc = gc_params['max_acc_xy']
WPNAV_SPEED = gc_params['wpnav_speed'] # cm/s
WPNAV_SPEED_UP = gc_params['wpnav_speed_up'] # cm/s
WPNAV_SPEED_DN = gc_params['wpnav_speed_dn'] # cm/S

# Trajectory settings
## wp navigation
traj = TrajectoryGeneratorV2(sampling_freq=1/controller_dt, max_vel=[5.0, 5.0, 8.0], max_acc=[5.0, 5.0, 5.0])
traj_climb = TrajectoryGeneratorV2(sampling_freq=1/controller_dt, max_vel=[1.0, 1.0, 1.0], max_acc=[5.0, 5.0, 5.0])

wpts_alt = 0.0
CLIMB   = True
DESCEND = False
alt_target_climb       = 60.0
alt_target_descend     = 15.0
alt_thresh_climb_low   = 10.0
alt_thresh_descend_low = 15.0


DEFAULT_TAKEOFF_THRUST = 0.7
DEFAULT_LANDING_THRUST = 0.3

Vel_zero = np.array([0.0, 0.0, 0.0])
edge_length = 50.0
edge_inter_cp = 1
# Generate waypoints
edge_length_climb = 5
wp_list_climb = traj_climb.generate_wp_list_for_square_trajectory(edge_length_climb, wpts_alt, edge_inter_cp = edge_inter_cp, order = 'WN')
wp_list_climb = np.vstack((wp_list_climb, wp_list_climb, wp_list_climb, wp_list_climb, wp_list_climb, wp_list_climb, wp_list_climb, wp_list_climb))
wp_list = traj.generate_wp_list_for_square_trajectory(edge_length, wpts_alt, edge_inter_cp = edge_inter_cp, order = 'WN') # order = 'NW' or 'WN'
# wp_list = np.vstack((wp_list, wp_list, wp_list))

yaw_cutoff_freq = 5.0
alpha_yaw = lambda _dt: _dt / (_dt + 1/(2.0 * np.pi * yaw_cutoff_freq)) #low pass filter for yaw smoothing
acc_limit = 10.0
jerk_limit = 5.0
prev_acc_cmd_xy = np.zeros(3)

# Waypoint Navigation Parameters
traj_id = 0
traj_id_climb = 0
ref_pos = np.array([0.0, 0.0, -0.0])
ref_vel = np.array([0.0, 0.0, 0.0])

# Store list of posiitions for comparison
VIO_pos_list = []
GT_pos_list  = []

while True:
    #node_OdomVIO.first_vo_msg
    #node_OdomVIO.VIO_dict
    #is_velocity_body = True
    if node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_imu_mag_msg and node_OdomVIO.first_gps_fix_msg and node_OdomVIO.first_state_msg:
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

            # RESET position controllers
            pos_controller_x.reset()
            pos_controller_y.reset()


        # Main loop
        else:

            # Wait for mode to be GUIDED
            while True:
                print("Waiting for mode to be GUIDED")
                mode = node_OdomVIO.state_dict['mode']
                if (mode == "GUIDED" or mode == "GUIDED_NOGPS"):

                    print("Start trajectory")
                    yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict.copy(), node_OdomVIO.gt_odom_dict.copy())

                    break
                time.sleep(0.1)

            # Getting first VIO data with converting to NED frame
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
            VIO_pos_first = np.array(VIO_dict['position'].copy())

            # Generating trajectory from waypoints
            wp_list = np.array(wp_list)
            shaped_wp_list = wp_list + VIO_pos_first
            traj.generate_traj_from_wplist_interp(shaped_wp_list, coordinate_type="ned")
            generated_traj = traj.get_pos_vel_acc_in_ned()

            wp_list_climb = np.array(wp_list_climb)
            shaped_wp_list_climb = wp_list_climb + VIO_pos_first
            traj_climb.generate_traj_from_wplist_interp(shaped_wp_list_climb, coordinate_type="ned")
            generated_traj_climb = traj_climb.get_pos_vel_acc_in_ned()


            # visualize2DgenTraj(generated_traj['pos'][:,0:2])

            if LOG:
                date_var = time.strftime("%Y%m%d-%H%M%S")
                log_file_name = "logs/pos_controller_test_with_odom_{}.txt".format(date_var)

                ref_pos = np.array([0.0,0.0, -0.0])  # Initial reference position
                ref_vel = np.array([0.0, 0.0, 0.0])  # Initial reference velocity
                VIO_vel = np.array([0.0, 0.0, 0.0])  # Initial VIO velocity
                # Set first VIO position and velocity as

                ref_angles = np.array([0.0, 0.0, 0.0])
                acc_cmd_xy = np.array([0.0, 0.0, 0.0])  # Initial acceleration command
                ref_posvel = np.array([VIO_pos_first, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
                ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
                with open(log_file_name, "w") as f:
                    np.savetxt(f, ref_posvel,  delimiter=',')

            t_prev = time.time()
            last_print_time = time.time() - 4

            # Guidance and control loop
            while True:

                if time.time() - t_prev > controller_dt:  # apply control at the specified controller_dt
                    t_prev = time.time()


                    # Climb to target altitude
                    if CLIMB:
                        # GEt odometry data from VIO
                        VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
                        VIO_pos = np.array(VIO_dict['position'])
                        VIO_vel = np.array(VIO_dict['velocity'])

                        # # Get ground truth odometry data from GPS fix local frame
                        # GT_dict = ned_VIO_converter(node_OdomVIO.gt_odom_dict.copy(), 0, is_velocity_body = False)
                        # GT_pos = np.array(GT_dict['position'])
                        # GT_vel = np.array(GT_dict['velocity'])

                        # Lateral position control
                        ref_pos, ref_vel = VIO_pos_first, Vel_zero

                        # Get reference position and velocity from trajectory generation
                        #traj_id_climb +=1
                        #if traj_id_climb >= len(generated_traj_climb["pos"])-1:
                        #    traj_id_climb = len(generated_traj_climb["pos"])-1
                        #ref_pos = generated_traj_climb["pos"][traj_id_climb,:].copy()
                        #ref_vel = generated_traj_climb["vel"][traj_id_climb,:].copy()
                        #ref_acc = generated_traj_climb["acc"][traj_id_climb,:].copy()

                        acc_cmd_x = pos_controller_x.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
                        acc_cmd_y = pos_controller_y.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
                        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0]) 

                        a_n = acc_cmd_xy[0]
                        a_e = acc_cmd_xy[1]

                        yaw_target = 0.0
                        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=max_acc)

                        # Vertical position control
                        alt_diff = alt_target_climb - (-VIO_pos[2])

                        if alt_diff > 5:

                            print("Climbing to target altitude: ", alt_target_climb, "Current altitude: ", -VIO_pos[2], "diff: ", alt_diff)
                            if alt_diff > alt_thresh_climb_low:
                                thrust_target = DEFAULT_TAKEOFF_THRUST

                            else:
                                thrust_target = max(min(0.5 + (0.2/alt_thresh_climb_low) * alt_diff, DEFAULT_TAKEOFF_THRUST), 0.5)
                        else:
                            thrust_target = 0.5
                            CLIMB = False
                            print('Climb altitude reached...')


                            # Generating trajectory from waypoints
                            wp_list = np.array(wp_list)
                            shaped_wp_list = wp_list + VIO_pos
                            traj.generate_traj_from_wplist_interp(shaped_wp_list, coordinate_type="ned")
                            generated_traj = traj.get_pos_vel_acc_in_ned()
                            

                        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)

                    if DESCEND:

                        # GEt odometry data from VIO
                        VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
                        VIO_pos = np.array(VIO_dict['position'])
                        VIO_vel = np.array(VIO_dict['velocity'])

                        # # Get ground truth odometry data from GPS fix local frame
                        # GT_dict = ned_VIO_converter(node_OdomVIO.gt_odom_dict.copy(), 0, is_velocity_body = False)
                        # GT_pos = np.array(GT_dict['position'])
                        # GT_vel = np.array(GT_dict['velocity'])

                        # Lateral position control
                        ref_pos, ref_vel = ref_pos, Vel_zero

                        # Get reference position and velocity from trajectory generation
                        #traj_id_climb +=1
                        #if traj_id_climb >= len(generated_traj_climb["pos"])-1:
                        #    traj_id_climb = len(generated_traj_climb["pos"])-1
                        #ref_pos = generated_traj_climb["pos"][traj_id_climb,:].copy()
                        #ref_vel = generated_traj_climb["vel"][traj_id_climb,:].copy()
                        #ref_acc = generated_traj_climb["acc"][traj_id_climb,:].copy()

                        acc_cmd_x = pos_controller_x.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
                        acc_cmd_y = pos_controller_y.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
                        acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0]) 

                        a_n = acc_cmd_xy[0]
                        a_e = acc_cmd_xy[1]

                        yaw_target = 0.0
                        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=max_acc)

                        # Vertical position control
                        alt_diff = (-VIO_pos[2]) - alt_target_descend

                        if alt_diff > 5:

                            print("Descending to target altitude: ", alt_target_descend, "Current altitude: ", -VIO_pos[2], "diff: ", alt_diff)
                            if alt_diff > alt_thresh_descend_low:
                                thrust_target = DEFAULT_LANDING_THRUST

                            else:
                                thrust_target = min(max(0.5 - (0.2/alt_thresh_descend_low) * alt_diff, DEFAULT_LANDING_THRUST), 0.5)
                        else:
                            thrust_target = 0.5
                            print('Descend altitude reached...')
                            # DESCEND = False
                            
                        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_target)


                    # Square trajectory tracking
                    elif (not CLIMB) and (not DESCEND):

                        # GEt odometry data from VIO
                        VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict.copy(), yaw_diff, is_velocity_body = True)
                        VIO_pos = np.array(VIO_dict['position'])
                        VIO_vel = np.array(VIO_dict['velocity'])

                        # Get ground truth odometry data from GPS fix local frame
                        GT_dict = ned_VIO_converter(node_OdomVIO.gt_odom_dict.copy(), 0, is_velocity_body = False)
                        GT_pos = np.array(GT_dict['position'])
                        GT_vel = np.array(GT_dict['velocity'])

                        # Get reference position and velocity from trajectory generation
                        traj_id +=1
                        if traj_id >= len(generated_traj["pos"])-1:
                            traj_id = len(generated_traj["pos"])-1
                            DESCEND = True

                        ref_pos = generated_traj["pos"][traj_id,:].copy()
                        ref_vel = generated_traj["vel"][traj_id,:].copy()
                        ref_acc = generated_traj["acc"][traj_id,:].copy()

                        # Get acc commands from position controllers
                        acc_cmd_x = pos_controller_x.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
                        acc_cmd_y = pos_controller_y.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
                        acc_cmd_xy = np.array([acc_cmd_x + 0*ref_acc[0], acc_cmd_y + 0*ref_acc[1], 0]) 

                        # Limit acceleration command
                        if np.linalg.norm(acc_cmd_xy) > acc_limit:
                            acc_cmd_xy = acc_cmd_xy / np.linalg.norm(acc_cmd_xy) * acc_limit 
                        # Clip acceleration command to limit jerk with previous acceleration command
                        ref_acc = np.clip(acc_cmd_xy, prev_acc_cmd_xy - jerk_limit*controller_dt, prev_acc_cmd_xy + jerk_limit*controller_dt)
                        prev_acc_cmd_xy = np.copy(acc_cmd_xy)
                        a_n = acc_cmd_xy[0]
                        a_e = acc_cmd_xy[1]

                        # Get yaw target from velocity reference
                        if np.linalg.norm(ref_vel[0:2]) > 2:
                            a = alpha_yaw(controller_dt) # low pass filter for yaw
                            yaw_vector_ref = (ref_vel[0:2] / np.linalg.norm(ref_vel[0:2]))
                            yaw_vector = yaw_vector + a * (yaw_vector_ref - yaw_vector)  
                            yaw_vector = yaw_vector / np.linalg.norm(yaw_vector)
                        yaw_target = np.rad2deg(np.arctan2(yaw_vector[1], yaw_vector[0]))
                        yaw_target = 0.0

                        ## Get thrust reference from altitude reference 
                        # climb_rate_ref =  -(ref_pos[2] - VIO_pos[2])
                        # climb_rate_ref = np.sign(climb_rate_ref) * np.sqrt(np.abs(climb_rate_ref))
                        # climb_rate_ref = min(WPNAV_SPEED_UP/100, max(-WPNAV_SPEED_DN/100, climb_rate_ref))
                        # if climb_rate_ref > 0:
                        #     thrust_ref = 0.5 + 0.5 * climb_rate_ref / (WPNAV_SPEED_UP/100)
                        # else:
                        #     thrust_ref = 0.5 + 0.5 * (climb_rate_ref / (WPNAV_SPEED_DN/100))
                        thrust_ref = 0.5 

                        # Get attitude reference from acceleration commands and yaw target
                        pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=acc_limit)
                        
                        # Set attitude and thrust to Pixhawk from attitude reference and thrust reference
                        node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=thrust_ref)

                        # Store UAV position and GT position for visualization
                        VIO_pos_list.append(VIO_pos[0:2].copy())
                        GT_pos_list.append(GT_pos[0:2].copy())

                        # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
                        if last_print_time + 1 < time.time():
                            print("diff_pos: " , ref_pos - VIO_pos)
                            print('diff_vel: ' , ref_vel - VIO_vel)
                            print("VIO pos:", VIO_pos)
                            print("VIO vel:", VIO_vel)
                            print("ref_pos:", ref_pos)
                            print("ref_vel:", ref_vel)
                            print("acc:", a_n, a_e)
                            print("RPY:",yaw_target, pitch_target, roll_target)
                            last_print_time = time.time()

                        if LOG:
                            ref_angles = np.array([yaw_target, pitch_target, roll_target])
                            ref_posvel = np.array([VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
                            ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
                            with open(log_file_name, "ab") as f:
                                np.savetxt(f, ref_posvel,  delimiter=',')

                mode = node_OdomVIO.state_dict['mode']
                if not (mode == "GUIDED" or mode == "GUIDED_NOGPS") :
                    print("Mode is not GUIDED or GUIDED_NOGPS, stopping trajectory.")
                    # visualize2DgenTraj(generated_traj['pos'][:,0:2], np.array(UAV_pos_list))
                    np.save('logs/VIO_pos_list_{}.npy'.format(date_var),   np.array(VIO_pos_list))
                    np.save('logs/generated_traj_{}.npy'.format(date_var), generated_traj['pos'][:,0:2])
                    np.save('logs/GT_pos_list_{}.npy'.format(date_var),    np.array(GT_pos_list))

                    # RESET position controllers
                    pos_controller_x.reset()
                    pos_controller_y.reset()

                    # Reset traj id
                    traj_id = 0

                    # Reset lists
                    VIO_pos_list = []
                    GT_pos_list  = []

                    # Reset yaw
                    yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict.copy(), node_OdomVIO.gt_odom_dict.copy())
                    break
            

    else:
        print("-----------------------------------------------------")

        if not node_OdomVIO.first_vo_msg:
            print("Waiting for first VIO message...")

        if not node_OdomVIO.first_gt_odom_msg:
            print("Waiting for first ground truth odometry message...")

        if not node_OdomVIO.first_imu_mag_msg:
            print("Waiting for first IMU magnetometer message...")

        if not node_OdomVIO.first_gps_fix_msg:
            print("Waiting for first GPS fix message...")

        if not node_OdomVIO.first_state_msg:
            print("Waiting for first state message...")

        print("-----------------------------------------------------")
        time.sleep(1)
        continue

 


