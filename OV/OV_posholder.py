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
import logging


logger = logging.getLogger(__name__)


# Custom libraries
# Add the path to the utils module if it's not in the same directory
from odom_subscriber import OdomAndMavrosSubscriber
from PixhawkCommander import PixhawkCommander

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import quat2eul, eul2quat
from utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter
from utils_OV.controller_utils import PositionControllerBumpless, from_pos_vel_to_angle_ref

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
log_file_name = "logs/pos_controller_test_with_odom_{}.txt".format(time.strftime("%Y%m%d-%H%M%S"))


# Guidance and control settings
with open('./config/guidance_and_control_parameters.yaml') as f:
    gc_params = yaml.safe_load(f)

pos_controller_x = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], gc_params['max_acc_xy'], log_file_name ="logs/x_ref.txt")
pos_controller_y = PositionControllerBumpless(gc_params['kp_pos'], gc_params['kp_vel'], gc_params['kd_vel'], gc_params['ki_vel'], gc_params['vel_filter_tc'], gc_params['gc_dt'], gc_params['max_acc_xy'], log_file_name ="logs/y_ref.txt")
sampling_time = gc_params['gc_dt']
max_acc = gc_params['max_acc_xy']
logger.debug("OV_posholder starting")
while True:

    if node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_imu_mag_msg and node_OdomVIO.first_gps_fix_msg:
        logger.info("All node sensors are initialized.")

        if is_first_messages:
            logger.info("First messages received, processing data...")

            yaw_diff = yaw_diff_finder(node_OdomVIO.VIO_dict, node_OdomVIO.gt_odom_dict)

            is_first_messages = False
            start_time = time.time()

            # print(f"(yaw, pitch, roll)deg VIO : {np.rad2deg(euler_vio)} | GT Odom : {np.rad2deg(euler_gt_odom)}")
        else:

            # find ENU frame VIO datas except angular velocity that are in body frame

            while True:
                logger.info("Waiting for mode to be GUIDED")
                mode = node_OdomVIO.state_dict['mode']
                if (mode == "GUIDED" or mode == "GUIDED_NOGPS"):

                    logger.info("Start trajectory")
                    break
                time.sleep(0.1)

            # Creating a PositionController reference position and velocity
            # Lets try to hower
            t_prev = time.time()
            VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict, yaw_diff)
            VIO_pos = np.array(VIO_dict['position']).copy()
            # VIO_vel = np.array(VIO_dict['velocity']).copy()
            VIO_vel = np.array([0,0,0])
            # set first VIO position and velocity as reference
            ref_pos, ref_vel = VIO_pos, VIO_vel
            yaw_target = np.rad2deg(quat2eul(VIO_dict['orientation'])[0])  

            if LOG:
                ref_angles = np.array([0.0, 0.0, 0.0])
                acc_cmd_xy = np.array([0.0, 0.0, 0.0])  # Initial acceleration command
                ref_posvel = np.array([VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
                ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
                with open(log_file_name, "w") as f:
                    np.savetxt(f, ref_posvel,  delimiter=',')

            last_print_time = time.time() - 4

            while True:
                if time.time() - t_prev > sampling_time:
                    t_prev = time.time()

                    # ref_pos = ref_pos + (ref_vel * sampling_time)

                    VIO_dict = ned_VIO_converter(node_OdomVIO.VIO_dict, yaw_diff)
                    VIO_pos = np.array(VIO_dict['position'])
                    VIO_vel = np.array(VIO_dict['velocity'])

                    acc_cmd_x = pos_controller_x.update(ref_pos[0], ref_vel[0], VIO_pos[0], VIO_vel[0])
                    acc_cmd_y = pos_controller_y.update(ref_pos[1], ref_vel[1], VIO_pos[1], VIO_vel[1])
                    acc_cmd_xy = np.array([acc_cmd_x, acc_cmd_y, 0]) 

                    a_n = acc_cmd_xy[0]
                    a_e = acc_cmd_xy[1]

                    pitch_target, roll_target = from_pos_vel_to_angle_ref(a_n, a_e, 0, yaw_target, yaw_in_degrees=True, max_accel=max_acc)
                    node_PixhawkCMD.set_attitude(np.deg2rad([yaw_target, pitch_target, roll_target]), thrust=0.5)

                    # print("ref_pos:", ref_pos, "acc:", a_n, a_e)
                    if last_print_time + 1 < time.time():
                        logger.info("diff_pos=%s", (ref_pos - VIO_pos))
                        logger.info("diff_vel=%s", (ref_vel - VIO_vel))
                        logger.info("VIO pos=%s", VIO_pos)
                        logger.info("VIO vel=%s", VIO_vel)
                        logger.info("ref_pos=%s", ref_pos)
                        logger.info("ref_vel=%s", ref_vel)
                        logger.info("acc=%s %s", a_n, a_e)
                        logger.info("rpy=%s %s %s", yaw_target, pitch_target, roll_target)
                        last_print_time = time.time()


                    if LOG:
                        ref_angles = np.array([yaw_target, pitch_target, roll_target])
                        ref_posvel = np.array([VIO_pos, VIO_vel, ref_pos, ref_vel, acc_cmd_xy, ref_angles]).reshape(1,-1)  
                        ref_posvel = np.insert(ref_posvel, 0, time.time(), axis=1).reshape(1,-1) 
                        with open(log_file_name, "ab") as f:
                            np.savetxt(f, ref_posvel,  delimiter=',')

                mode = node_OdomVIO.state_dict['mode']
                if not (mode == "GUIDED" or mode == "GUIDED_NOGPS") :
                    logger.warning("Mode is not GUIDED or GUIDED_NOGPS, stopping trajectory.")
                    break
    else:
        logger.info("-----------------------------------------------------")
        if not node_OdomVIO.first_vo_msg:
            logger.info("Waiting for first VIO message...")

        if not node_OdomVIO.first_gt_odom_msg:
            logger.info("Waiting for first ground truth odometry message...")

        if not node_OdomVIO.home_received:
            logger.info("Waiting for home position...")

        if not node_OdomVIO.first_imu_mag_msg:
            logger.info("Waiting for first IMU magnetometer message...")

        if not node_OdomVIO.first_gps_fix_msg:
            logger.info("Waiting for first GPS fix message...")

        logger.info("-----------------------------------------------------")
        time.sleep(1)
        continue
