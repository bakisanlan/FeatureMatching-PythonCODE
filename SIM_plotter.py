import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

def plot_errors(GT, PFwinter, PFsummer, dt=0.1):
    """
    Plot the position and velocity errors for PFwinter and PFsummer with respect to GT
    in two separate figures. The first figure shows position errors (North and East)
    and the second figure shows velocity errors (North and East).

    Parameters:
        GT (np.ndarray): Ground truth array of shape (N, 15)
        PFwinter (np.ndarray): PFwinter array of shape (N, 15)
        PFsummer (np.ndarray): PFsummer array of shape (N, 15)
        dt (float): Time step (default 0.1)

    Notes:
        - For positions, columns [0, 1] (i.e., 1st and 2nd columns) represent North and East.
        - For velocities, columns [3, 4] (i.e., 4th and 5th columns) represent North and East.
        - Errors are computed as PF - GT.
    """
    # Extract position columns (North, East) and velocity columns (North, East)
    GT_pos = GT[:, [0, 1]]
    PFwinter_pos = PFwinter[:, [0, 1]]
    PFsummer_pos = PFsummer[:, [0, 1]]
    
    GT_vel = GT[:, [3, 4]]
    PFwinter_vel = PFwinter[:, [3, 4]]
    PFsummer_vel = PFsummer[:, [3, 4]]
    
    # Compute errors (PF - GT)
    error_winter_pos = PFwinter_pos - GT_pos
    error_summer_pos = PFsummer_pos - GT_pos
    
    error_winter_vel = PFwinter_vel - GT_vel
    error_summer_vel = PFsummer_vel - GT_vel
    
    # Create a time vector (assuming each row corresponds to a time step)
    time = np.arange(GT.shape[0]) * dt
    
    # Common plotting parameters
    linewidth       = 4
    fontsize        = 15
    fontize_xlabel  = 12
    fontize_ylabel  = 12
    tick_size       = 12
    legend_props    = {'weight': 'bold', 'size': tick_size}  # Legend font properties


    # ------------------------------
    # Figure 1: Position Errors
    # ------------------------------
    fig1, axs1 = plt.subplots(2, 1, figsize=(7, 8))
    
    # Position Error - North (right subplot)
    axs1[0].plot(time, error_winter_pos[:, 0], label='Winter Season Camera View ', linewidth=linewidth)
    axs1[0].plot(time, error_summer_pos[:, 0], label='Generated SAT View', linewidth=linewidth)
    axs1[0].tick_params(axis='both', which='major', labelsize=tick_size)
    axs1[0].set_title('Position Error - North', fontweight='bold', fontsize=fontsize)
    axs1[0].set_xlabel('Time', fontsize=fontize_xlabel, fontweight='bold')
    axs1[0].set_ylabel('Error [m]', fontsize=fontize_ylabel)
    axs1[0].legend(loc='best', prop=legend_props, framealpha = 1)
    axs1[0].grid(True)
    
    # Position Error - East (Right subplot)
    axs1[1].plot(time, error_winter_pos[:, 1], label='Winter Season Camera View ', linewidth=linewidth)
    axs1[1].plot(time, error_summer_pos[:, 1], label='Generated SAT View', linewidth=linewidth)
    axs1[1].tick_params(axis='both', which='major', labelsize=tick_size)
    axs1[1].set_title('Position Error - East', fontweight='bold', fontsize=fontsize)
    axs1[1].set_xlabel('Time', fontsize=fontize_xlabel)
    axs1[1].set_ylabel('Error [m]', fontsize=fontize_ylabel)
    axs1[1].legend(loc='best', prop=legend_props, framealpha = 1)
    axs1[1].grid(True)
    
    # Bold tick labels and axis labels for fig1
    for ax in axs1:
        plt.setp(ax.get_xticklabels(), fontweight='bold')
        plt.setp(ax.get_yticklabels(), fontweight='bold')
        plt.setp(ax.xaxis.label, fontweight='bold')
        plt.setp(ax.yaxis.label, fontweight='bold')


    
    fig1.tight_layout()
    fig1.align_ylabels(axs1)
    
    # ------------------------------
    # Figure 2: Velocity Errors
    # ------------------------------
    fig2, axs2 = plt.subplots(2, 1, figsize=(7, 8))
    
    # Velocity Error - North (Left subplot)
    axs2[0].plot(time, error_winter_vel[:, 0], label='Winter Season Camera View ', linewidth=linewidth)
    axs2[0].plot(time, error_summer_vel[:, 0], label='Generated SAT View', linewidth=linewidth)
    axs2[0].tick_params(axis='both', which='major', labelsize=tick_size)
    axs2[0].set_title('Velocity Error - North', fontweight='bold', fontsize=fontsize)
    axs2[0].set_xlabel('Time', fontsize=fontize_xlabel, fontweight='bold')
    axs2[0].set_ylabel('Error [m/s]', fontsize=fontize_ylabel)
    axs2[0].legend(loc='best', prop=legend_props, framealpha = 1)
    axs2[0].grid(True)
    
    # Velocity Error - East (Right subplot)
    axs2[1].plot(time, error_winter_vel[:, 1], label='Winter Season Camera View ', linewidth=linewidth)
    axs2[1].plot(time, error_summer_vel[:, 1], label='Generated SAT View', linewidth=linewidth)
    axs2[1].tick_params(axis='both', which='major', labelsize=tick_size)
    axs2[1].set_title('Velocity Error - East', fontweight='bold', fontsize=fontsize)
    axs2[1].set_xlabel('Time', fontsize=fontize_xlabel)
    axs2[1].set_ylabel('Error [m/s]', fontsize=fontize_ylabel)
    axs2[1].legend(loc='best', prop=legend_props, framealpha = 1)
    axs2[1].grid(True)
    
    # Bold tick labels and axis labels for fig2
    for ax in axs2:
        plt.setp(ax.get_xticklabels(), fontweight='bold')
        plt.setp(ax.get_yticklabels(), fontweight='bold')
        plt.setp(ax.xaxis.label, fontweight='bold')
        plt.setp(ax.yaxis.label, fontweight='bold')
    
    fig2.tight_layout()
    fig2.align_ylabels(axs2)

    
    # Display both figures
    plt.show()

GT       = np.load('data_EST_like_th.npy')  # shape: (num_frames, 256, 256, 3)
PFwinter = np.load('data_EST_like_th.npy')  # shape: (num_frames, 256, 256, 3)
PFsummer = np.load('data_INS.npy')  # shape: (num_frames, 256, 256, 3)

cut = -1
print(PFsummer)
print(PFwinter)
plot_errors(GT[0:cut,:], PFwinter[0:cut,:], PFsummer[0:cut,:])