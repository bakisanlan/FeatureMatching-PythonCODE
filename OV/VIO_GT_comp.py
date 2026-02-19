# from plotter import plot_VIO_GT_comp

import os
import sys
from matplotlib import pyplot as plt
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import numpy as np
from plotter import visualizeTraj


# plot_VIO_GT_comp(csv_file='vio_gps_5hz_0107_2.csv')
date = '20251224-153403'
ref_traj = np.load('/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/logs/generated_traj_{}.npy'.format(date), allow_pickle=True)
GPS_pos = np.load('/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/logs/GT_pos_list_{}.npy'.format(date), allow_pickle=True)
VIO_pos = np.load('/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/logs/VIO_pos_list_{}.npy'.format(date), allow_pickle=True)
PF_pos  = np.load('/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/logs/PF_pos_list_{}.npy'.format(date), allow_pickle=True)
print(VIO_pos.shape)
print(GPS_pos.shape)
print(PF_pos.shape)
print(ref_traj.shape)

# visualizeTraj(VIO_pos[:,0:2], generated_traj=ref_traj[:,0:2])
visualizeTraj(VIO_pos[:,0:2], ref_traj[:,0:2],  GPS_pos[:,0:2], PF_pos[:,0:2], plot_3d=False)

# VIO_pos = np.load('VIO_pos_list.npy')
# GPS_pos = np.load('GT_pos_list.npy')
# PF_pos = np.load('PF_particles.npy')
# visualize2DgenTraj(GPS_pos[:,0:2], VIO_pos[:,0:2], particles=particles)


# filepath = 'logs/pos_controller_test_with_odom_20250722-084756.txt'.format(date)
# df = load_data(filepath)

# plot_position_comparison(df)
# plot_velocity_comparison(df)
# plot_acceleration_commands(df)
# plot_reference_angles(df)

# plt.show()

