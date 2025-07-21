# from plotter import plot_VIO_GT_comp

import os
import sys
from matplotlib import pyplot as plt
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import numpy as np
from utils_OV.common_utils import visualize2DgenTraj, load_data, plot_position_comparison, plot_velocity_comparison, plot_acceleration_commands, plot_reference_angles




# plot_VIO_GT_comp(csv_file='vio_gps_5hz_0107_2.csv')
date = 130419
ref_traj = np.load('logs/generated_traj_20250720-{}.npy'.format(date), allow_pickle=True)
GPS_pos  = np.load('logs/GT_pos_list_20250720-{}.npy'.format(date), allow_pickle=True)
VIO_pos  = np.load('logs/VIO_pos_list_20250720-{}.npy'.format(date), allow_pickle=True)

visualize2DgenTraj(ref_traj, GPS_pos, VIO_pos)


filepath = 'logs/pos_controller_test_with_odom_20250711-183342.txt'.format(date)
df = load_data(filepath)

plot_position_comparison(df)
plot_velocity_comparison(df)
plot_acceleration_commands(df)
plot_reference_angles(df)

plt.show()

