import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import pymap3d as pm
import csv
import sys
import os
import yaml
import logging
import threading
import signal
from pathlib import Path
from datetime import datetime

# This code is for a online drone localization using the MPF (Marginalized-Particle Filter) algorithm with Feature Matching.

# Custom libraries
# Add the path to the utils module if it's not in the same directory
from utils import *
from StateEstimatorVINS import StateEstimatorMPF
from FeatureDetectorMatcher import FeatureDetectorMatcher
from UAVCamera import UAVCamera
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from Timer import Timer
from plotter import plot_positions,PlotCamera,combineFrame,DynamicErrorPlot, TwoDynamicPlotter
from OV.odom_subscriber import OdomAndMavrosSubscriber
from OV.utils_OV.common_utils import yaw_diff_finder, ned_VIO_converter
from plotter import visualizeTraj

# Setup logging and redirect stdout/stderr
setup_logging_with_redirect(log_dir_name="IM_logs_out", log_file_prefix="IM", level=logging.INFO)

# Initialize ROS 2 nodes
rclpy.init()
node_OdomVIO     = OdomAndMavrosSubscriber()
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
is_first_messages = True
is_first_IM       = True



# -------------------------------------------------------------------------
# 2) Prepare storage lists for:
#    - Dead-reckoning (INS) states
#    - XKF "ground truth" states
# -------------------------------------------------------------------------

VIO_position_list      = []
VIO_velocity_list      = []
VIO_euler_list         = []
VIO_state_list          = []

gt_position_list           = []
gt_velocity_list           = []
gt_euler_list              = []
gt_state_list               = []

PF_position_list           = [] 
PF_velocity_list           = []
PF_euler_list              = []
PF_particles_position_list = []
PF_state_list              = []

### Figure create object holder for side by side view of UAV and most likelihood particle
useFramePlotter = True
plot_interval   = 0.1
last_plot_time  = time.time()
CamPlotter      = PlotCamera(useFramePlotter= useFramePlotter)

# Signal handler for Ctrl+C to plot trajectories before exiting
def signal_handler(sig, frame):
    """Handle Ctrl+C interrupt to plot trajectories before exiting."""
    print("\n\nCtrl+C detected! Plotting trajectories...")
    
    # Convert lists to numpy arrays
    if len(PF_position_list) > 0 and len(VIO_position_list) > 0:
        gt_pos_array = np.array(gt_position_list)[:, 0:2]
        pf_pos_array = np.array(PF_position_list)[:, 0:2]   # Extract N, E components
        vio_pos_array = np.array(VIO_position_list)[:, 0:2]  # Extract N, E components
        # pf_part_array = np.array(PF_particles_position_list)[:, :,0:2] # Shape: (num_samples, num_particles, 3)


        
        # Plot using visualize2DgenTraj
        print(f"Plotting {len(PF_position_list)} PF positions and {len(VIO_position_list)} VIO positions...")
        visualizeTraj(
            GPS_POS= gt_pos_array,
            VIO_POS=vio_pos_array, 
            PF_POS=pf_pos_array,
            # particles=pf_part_array,
            # xlabel="North (m)",
            # ylabel="East (m)",
            title="VIO vs PF Trajectory Comparison"
        )
    else:
        print("No trajectory data to plot.")
    
    # Cleanup and exit
    print("Shutting down...")
    rclpy.shutdown()
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

#### Main loop
plt.show(block=False)

while True:
    #node_OdomVIO.first_vo_msg
    #node_OdomVIO.VIO_dict
    #is_velocity_body = True
    if node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg and node_OdomVIO.first_gps_fix_msg and node_OdomVIO.first_pf_pose_msg:
            

        # pxGT  = ned2px(GT_pos.copy()        , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        # pxVIO = ned2px(VIO_pos.copy()       , hAIM.leftupperNED, hAIM.mp, hDB.pxRned).squeeze()
        # pxPF  = ned2px(particlesPos.T.copy(), hAIM.leftupperNED, hAIM.mp, hDB.pxRned)   
        # pxPF_with_weights = np.hstack((pxPF , hStateEstimatorMPF.weights.reshape(-1, 1)))

        try:
            
            VIO_pos = node_OdomVIO.VIOned_dict['position'].copy()
            PF_pos = node_OdomVIO.pf_pos_dict['position'].copy()
            GT_pos = node_OdomVIO.GTned_dict['position'].copy()

            if len(VIO_position_list) > 1:

                if np.array_equal(VIO_pos, VIO_position_list[-1]):
                    print("No new VIO position data.")

                if np.array_equal(PF_pos, PF_position_list[-1]):
                    print("No new PF position data.")

            # print(f"Estimated Position: {PF_pos[0:2]}, VIO Position: {VIO_pos[0:2]}")
            # # print(f"VIO velocity: {VIO_vel[0:2]}, inputParticle: {inputParticle}")
            # FramemostLikelihoodPart = hStateEstimatorMPF.FramemostLikelihoodPart
            
            # Store positions for later plotting
            gt_position_list.append(GT_pos.copy())
            PF_position_list.append(PF_pos.copy())
            VIO_position_list.append(VIO_pos.copy())


            # PF_particles_position_list.append(particlesPos.T.copy())


            # Plotting 
            # flagErrorPlot = False
            # flagFramePlot = True

            # if ((time.time() - last_plot_time) > plot_interval) and UAVFrame is not None:
            #     last_plot_time = time.time()

            #     # Error Plotter
            #     if flagErrorPlot:
            #         ErrorPlotter.update(GT_pos,VIO_pos, PF_pos, timeConstant = plot_interval)

            #     # Update feature plot
            #     flightTime = time.time() - start_time
            #     if flagFramePlot:
            #         combinedFrame = combineFrame(hAIM.I, pxGT, None, pxPF_with_weights)
            #         CamPlotter.snapNow(
            #                         # (UAVFrame                 , 'UAV Camera'                         , f'Flight time is {flightTime:.2f} s \n Detected Features: {UAVKp.shape[0]}'),
            #                         # (UAVFakeFrame             , 'Generated Fake SAT Img'             , f'Detected Features: {UAVKp.shape[0]}'),
            #                         (FramemostLikelihoodPart  , 'Most likelihood Particle SAT View'  , f'Detected Features: {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[0]} \n Matched features:  {list(hStateEstimatorMPF.DataBaseScanner.partInfo.values())[1]}'),
            #                         (combinedFrame            , 'Particles, Ground Truth in Map'     , f'Position XY RMSE: {np.sqrt(np.mean((GT_pos[0:2] - PF_pos[0:2])**2)):.2f} m'),
            #                         )
            
            time.sleep(1/20)

        except Exception as e:
            print(f"Error during processing: {e}")
        
    else:
        print("-----------------------------------------------------")

        if not node_OdomVIO.first_vo_msg:
            print("Waiting for first VIO message...")

        if not node_OdomVIO.first_gt_odom_msg:
            print("Waiting for first ground truth odometry message...")

        if not node_OdomVIO.first_gps_fix_msg:
            print("Waiting for first GPS fix message...")

        if not node_OdomVIO.first_pf_pose_msg:
            print("Waiting for first Particle Filter position message...")


        print("-----------------------------------------------------")
        time.sleep(1)
        continue

