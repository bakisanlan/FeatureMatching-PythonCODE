import os
import sys
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd

#Custom libraries
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import quat2rotm, eul2quat, quat2eul, rotm2quat, wrap2_pi


def yaw_diff_finder(VIO_dict, gt_odom_dict, magYawDeg = None, manualYaw = False):
    """
    Computes the yaw difference between VIO arbitrary global frame and ENU frame.
    
    Parameters:
    - quat_vio: Quaternion representing the VIO orientation in [x, y, z, w] format .
    - quat_gt_odom: Quaternion representing the ground truth odometry orientation in [x, y, z, w] format.
    
    Returns:
    - yaw_diff: Yaw difference in radians.
    """
    
    quat_vio_ref2body = VIO_dict['orientation']
    qx, qy, qz, qw = quat_vio_ref2body
    euler_vio_ref2body = quat2eul([qw, qx, qy, qz], order='ZYX')   # Quaternion(s) should be in [w, x, y, z] format as input to that quat2eul function
    yaw_vio_ref2body = euler_vio_ref2body[0]

    # print('yaw vio ref2body:', np.rad2deg(yaw_vio_ref2body))  # Print yaw in degrees

    if magYawDeg is None:
        quat_gt_enu2body = gt_odom_dict['orientation']
        qx_gt, qy_gt, qz_gt, qw_gt = quat_gt_enu2body
        euler_gt_enu2body = quat2eul([qw_gt, qx_gt, qy_gt, qz_gt], order='ZYX')
        yaw_gt_enu2body = euler_gt_enu2body[0]
    else:

        if manualYaw:
            yaw_frd = 0
        else:
            yaw_frd = np.deg2rad(magYawDeg)
        yaw_gt_enu2body = np.pi/2 - yaw_frd


    # print('yaw gt enu2body:', np.rad2deg(yaw_gt_enu2body))  # Print yaw in degrees

    yaw_vioref2enu = (yaw_vio_ref2body - yaw_gt_enu2body) # vioref to ENU frame

    print(np.rad2deg(yaw_vioref2enu))

    return yaw_vioref2enu


def enu_VIO_converter(VIO_dict, yaw_vioref2enu, is_velocity_body = True, convert_CG = False):
    """
    Converts VIO coordinates to ENU coordinates based on a yaw reference.
    
    Parameters:
    - VIO_dict: Dictionary containing VIO data in xyz
    - yaw_diff: Yaw difference from the VIO odom reference frame to ENU frame in rad.
    
    Returns:
    - A tuple (x_enu, y_enu, z_enu) representing the ENU coordinates.
    """

    if convert_CG:
        R_toCG = np.array([[ 1.05326835e-02 , 1.00062379e+00 , 1.40446370e-02 , -1.12056291e-02],
                           [-9.99942567e-01 , 1.07818714e-02 ,-2.12012926e-03 , -1.52438771e-04],
                           [-4.52722161e-03 ,-4.81550852e-03 , 1.01038118e+00 , -1.20250907e-01],
                           [ 1.87387663e-02 ,-6.84392075e-02 ,-8.70585075e-02 ,  1.00000000e+00]])
        
    else:
        R_toCG = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    R_enu2vioref = R_toCG[0:3,0:3] @ quat2rotm(eul2quat([yaw_vioref2enu, 0, 0], order='ZYX'))   # ENU to VIO reference frame rotation matrix
    # print('yaw vioref2enu : ', np.rad2deg(quat2eul(rotm2quat(R_vioref2enu)))[0])  # Print yaw difference in degrees

    position         = VIO_dict['position']
    quat_vio         = VIO_dict['orientation']  
    velocity         = VIO_dict['velocity']
    angular_velocity = VIO_dict['angular_velocity']

    # Convert position to ENU coordinates
    position_enu = R_enu2vioref.T @ np.array(position)

    # COnvert orientation to body to ENU frame
    qx, qy, qz, qw = quat_vio
    R_body2vioref = quat2rotm([qw, qx, qy, qz])
    # print('VIO ref to body:', np.rad2deg(quat2eul(rotm2quat(R_vioref2body))[0]))  # Convert quaternion to Euler angles
    R_enu2body    = R_body2vioref.T @ R_enu2vioref  # NOTE: I guess for inertia to body order should be from left to right, i.e. R_body2enu = R_vioref2body.T @ R_vioref2enu
    quat_enu2body = rotm2quat(R_enu2body)
    # print('body to ENU:', np.rad2deg(quat2eul(quat_body2enu, order='ZYX')))  # Convert quaternion to Euler angles

    # Convert body frame velocity to ENU frame
    if is_velocity_body:
        velocity_enu = R_enu2body.T @ np.array(velocity)  # NOTE : check if this is correct, i.e. R_body2enu.T @ velocity
    else:
        velocity_enu = velocity

    quat_body2enu = rotm2quat(quat2rotm(quat_enu2body).T)

    VIO_dict['position']         = position_enu
    VIO_dict['orientation']      = quat_body2enu
    VIO_dict['velocity']         = velocity_enu
    VIO_dict['angular_velocity'] = angular_velocity  # Angular velocity remains in body frame

    return VIO_dict

def ned_VIO_converter(VIO_dict, yaw_vioref2enu, is_velocity_body = True, convert_CG = False):

    VIO_dict_enu = enu_VIO_converter(VIO_dict, yaw_vioref2enu, is_velocity_body, convert_CG)

    position_enu          = VIO_dict_enu['position']
    orientation_flu       = VIO_dict_enu['orientation']
    velocity_enu          = VIO_dict_enu['velocity']
    angular_velocity_body = VIO_dict['angular_velocity']   # Angular velocity remains in body frame

    position_ned = np.array([position_enu[1], position_enu[0], -position_enu[2]])
    velocity_ned = np.array([velocity_enu[1], velocity_enu[0], -velocity_enu[2]])
    
    euler_flu = quat2eul(orientation_flu)   # euler angles is inertia to body 
    euler_frd = np.array([np.pi/2 - euler_flu[0], -euler_flu[1], euler_flu[2]])
    orientation_frd = eul2quat(euler_frd, order = 'ZYX')

    VIO_dict_ned = VIO_dict_enu.copy()  # Copy the ENU VIO dictionary to NED
    VIO_dict_ned['position']          = position_ned         
    VIO_dict_ned['orientation']       = orientation_frd
    VIO_dict_ned['velocity']          = velocity_ned      
    VIO_dict_ned['angular_velocity']  = angular_velocity_body # Angular velocity remains in body frameangular_velocity_body

    return VIO_dict_ned


def visualize2DgenTraj(points: np.ndarray,
                       second_points: np.ndarray = None,
                       third_points: np.ndarray = None,
                       xlabel: str = "X",
                       ylabel: str = "Y",
                       title: str = None,
                       equal_aspect: bool = True,
                       **scatter_kwargs):
    """
    Plot up to three sets of 2D positions as a scatter plot.

    Parameters
    ----------
    points : np.ndarray, shape (N, 2)
        The reference trajectory points to plot (labelled "Traj Ref").
    second_points : np.ndarray, shape (M, 2), optional
        A second set of points to plot (labelled "UAV pos").
    third_points : np.ndarray, shape (K, 2), optional
        A third set of points to plot (labelled "GPS pos").
    xlabel : str, optional
        Label for the X-axis.
    ylabel : str, optional
        Label for the Y-axis.
    title : str or None, optional
        Plot title.
    equal_aspect : bool, optional
        If True, use `axis('equal')` so X and Y have the same scale.
    **scatter_kwargs
        Passed to all scatter calls (first uses `color` from kwargs or "C0",
        second uses "C1", third uses "C2"). Other kwargs like `s`, `marker`, etc.
        apply to all series.
    """
    # Basic validation
    pts = np.asarray(points)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError(f"points must be (N,2), got {pts.shape}")

    # extract base scatter kwargs
    base_kwargs = scatter_kwargs.copy()
    color0 = base_kwargs.pop("color", "C0")

    fig = plt.figure()

    # first set: Traj Ref
    plt.scatter(pts[:, 0], pts[:, 1],
                label="VIO Pos",
                color=color0,
                **base_kwargs)

    # second set: GPS pos
    if second_points is not None:
        up = np.asarray(second_points)
        if up.ndim != 2 or up.shape[1] != 2:
            raise ValueError(f"second_points must be (M,2), got {up.shape}")
        plt.scatter(up[:, 0], up[:, 1],
                    label="GPS pos",
                    color="C1",
                    **base_kwargs)

    # third set: VIO pos
    if third_points is not None:
        gp = np.asarray(third_points)
        if gp.ndim != 2 or gp.shape[1] != 2:
            raise ValueError(f"third_points must be (K,2), got {gp.shape}")
        plt.scatter(gp[:, 0], gp[:, 1],
                    label="VIO pos",
                    color="C2",
                    **base_kwargs)

    # axes and styling
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if title:
        plt.title(title)
    if equal_aspect:
        plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show(block=True)

def load_data(filepath: str) -> pd.DataFrame:
    """
    Load the data from a comma-separated txt file into a pandas DataFrame.
    Assumes columns are:
    time, vio_n, vio_e, vio_d,
    vio_v_n, vio_v_e, vio_v_d,
    ref_n, ref_e, ref_d,
    ref_v_n, ref_v_e, ref_v_d,
    acc_x, acc_y,
    yaw, pitch, roll
    """
    col_names = [
        'time',
        'vio_n', 'vio_e', 'vio_d',
        'vio_v_n', 'vio_v_e', 'vio_v_d',
        'ref_n', 'ref_e', 'ref_d',
        'ref_v_n', 'ref_v_e', 'ref_v_d',
        'acc_x', 'acc_y',
        'yaw', 'pitch', 'roll'
    ]
    return pd.read_csv(filepath, header=None, names=col_names)

def plot_position_comparison(df: pd.DataFrame):
    """Scatter N, E, D position comparison between VIO and reference in three subplots."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharex=True)
    coords = [('n', 'North'), ('e', 'East'), ('d', 'Down')]
    for ax, (suffix, label) in zip(axes, coords):
        ax.scatter(df['time'], df[f'vio_{suffix}'], s=5, label='VIO', alpha=0.7)
        ax.scatter(df['time'], df[f'ref_{suffix}'], s=5, label='Reference', alpha=0.7)
        ax.set_title(f'{label} Position')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [m]')
        ax.legend()
    fig.tight_layout()

def plot_velocity_comparison(df: pd.DataFrame):
    """Scatter N, E, D velocity comparison between VIO and reference in three subplots."""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharex=True)
    coords = [('n', 'North'), ('e', 'East'), ('d', 'Down')]
    for ax, (suffix, label) in zip(axes, coords):
        ax.scatter(df['time'], df[f'vio_v_{suffix}'], s=5, label='VIO', alpha=0.7)
        ax.scatter(df['time'], df[f'ref_v_{suffix}'], s=5, label='Reference', alpha=0.7)
        ax.set_title(f'{label} Velocity')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [m/s]')
        ax.legend()
    fig.tight_layout()

def plot_acceleration_commands(df: pd.DataFrame):
    """Scatter acceleration commands X and Y in a single figure."""
    plt.figure(figsize=(6, 4))
    plt.scatter(df['time'], df['acc_x'], s=5, label='Acc Cmd X', alpha=0.7)
    plt.scatter(df['time'], df['acc_y'], s=5, label='Acc Cmd Y', alpha=0.7)
    plt.title('Acceleration Command (X, Y)')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s²]')
    plt.legend()
    plt.tight_layout()

def plot_reference_angles(df: pd.DataFrame):
    """Scatter reference yaw, pitch, and roll angles in a single figure."""
    plt.figure(figsize=(6, 4))
    plt.scatter(df['time'], df['yaw'], s=5, label='Yaw', alpha=0.7)
    plt.scatter(df['time'], df['pitch'], s=5, label='Pitch', alpha=0.7)
    plt.scatter(df['time'], df['roll'], s=5, label='Roll', alpha=0.7)
    plt.title('Reference Angles')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [rad]')
    plt.legend()
    plt.tight_layout()

def main():
    # Adjust the path to your data file as needed
    filepath = 'data.txt'
    df = load_data(filepath)
    
    plot_position_comparison(df)
    plot_velocity_comparison(df)
    plot_acceleration_commands(df)
    plot_reference_angles(df)
    
    plt.show()

