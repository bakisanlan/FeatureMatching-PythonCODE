import numpy as np

import sys
import os

from collections import deque


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from plotter import plot_VIO_GT_comp_states 


alpha = 0.2

#load VIO and GT lists from npy files
VIO_pos_list = np.load('VIO_pos_list.npy', allow_pickle=True).tolist()
VIO_vel_list = np.load('VIO_vel_list.npy', allow_pickle=True).tolist()
VIO_eul_list = np.load('VIO_eul_list.npy', allow_pickle=True).tolist()

print(type(VIO_pos_list))
print(VIO_pos_list[0])

SMT_pos_list = [VIO_pos_list[0]]
SMT_vel_list = [VIO_vel_list[0]]

# Initialize buffers for smoothing
window_size = 5
buf_px , buf_py , buf_pz =  deque(maxlen=window_size) , deque(maxlen=window_size), deque(maxlen=window_size)
buf_vx , buf_vy , buf_vz =  deque(maxlen=window_size) , deque(maxlen=window_size), deque(maxlen=window_size)    

for i in range(len(VIO_pos_list)-1):

    # print(i)

    # px, py, pz = VIO_pos_list[i+1]
    # vx, vy, vz = VIO_vel_list[i+1]

    # # Smooth the velocity and position if smoothing is enabled

    # raw_position = np.array([px, py, pz])
    # raw_velocity = np.array([vx, vy, vz])

    # prev_position = np.array(VIO_pos_list[i])
    # prev_velocity = np.array(VIO_vel_list[i])
    # # spike rejection
    # # if np.linalg.norm(raw_position - prev_position) > spike_thresh:
    # #     raw = smoothed_vio_pos
    # # # exponential moving average
    # px, py, pz = alpha * raw_position + (1 - alpha) * prev_position
    # vx, vy, vz = alpha * raw_velocity + (1 - alpha) * prev_velocity

    px, py, pz = VIO_pos_list[i]
    vx, vy, vz = VIO_vel_list[i]

    ### Median smoothing
    buf_px.append(px), buf_py.append(py), buf_pz.append(pz)
    buf_vx.append(vx), buf_vy.append(vy), buf_vz.append(vz)

    px, py, pz = np.mean(np.array(buf_px)), np.mean(np.array(buf_py)), np.mean(np.array(buf_pz))
    vx, vy, vz = np.mean(np.array(buf_vx)), np.mean(np.array(buf_vy)), np.mean(np.array(buf_vz))
            

    SMT_pos_list.append((px, py, pz))
    SMT_vel_list.append((vx, vy, vz))

plot_VIO_GT_comp_states(
    VIO_pos_list, VIO_vel_list, VIO_eul_list,
    SMT_pos_list, SMT_vel_list, VIO_eul_list,
    dt = 1/20)
    