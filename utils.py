import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import pandas as pd
from typing import Tuple
import math
import logging
import sys
from pathlib import Path
from datetime import datetime
import os
import atexit
import glob

# Logging is centralized in OV/utils_OV/logging_utils.py.
# Keep backward compatible re-exports here because many scripts import from `utils`.
from OV.utils_OV.logging_utils import (
    setup_unified_logging,
    attach_ros_logger_to_python_logging,
)

def setup_logging_with_redirect(log_dir_name="logs_out", log_file_prefix="log", level=logging.INFO):
    """
    Set up logging to both file and console, and redirect stdout/stderr to logger.
    
    Parameters
    ----------
    log_dir_name : str
        Name of the directory to store log files (relative to caller's location).
    log_file_prefix : str
        Prefix for the log file name.
    level : int
        Logging level (e.g., logging.INFO, logging.DEBUG).
    
    Returns
    -------
    log_file : Path
        Path to the created log file.
    """
    # Create log directory
    log_out_dir = Path(log_dir_name)
    log_out_dir.mkdir(exist_ok=True)
    
    # Create log file with timestamp
    LOG_FORMAT = "%(asctime)s [%(threadName)s] %(levelname)-8s %(name)s: %(message)s"
    log_out_file = log_out_dir / f"{log_file_prefix}_{datetime.now():%Y%m%d_%H%M%S}.log"
    
    # Configure logging
    logging.basicConfig(
        level=level,
        format=LOG_FORMAT,
        handlers=[
            logging.FileHandler(log_out_file),
            logging.StreamHandler(sys.stdout)
        ],
    )
    
    # Stream to logger class for redirecting stdout/stderr
    class StreamToLogger:
        def __init__(self, logger, level):
            self.logger = logger
            self.level = level
        
        def write(self, buf):
            for line in buf.rstrip().splitlines():
                self.logger.log(self.level, line.rstrip())
        
        def flush(self):
            pass
    
    # Replace stdout and stderr
    sys.stdout = StreamToLogger(logging.getLogger("STDOUT"), logging.INFO)
    sys.stderr = StreamToLogger(logging.getLogger("STDERR"), logging.ERROR)
    
    return log_out_file


# Backward-compatible alias used throughout the repo.
def setup_logging(*args, **kwargs):
    """Backward-compatible wrapper.

    Many files import `setup_logging` already; we map it to the unified logger so
    prints + logging across scripts/nodes land in a single `.log`.

    You can force all processes to share the same file by exporting:
        UNIFIED_LOG_FILE=/abs/path/to/run.log
    """
    return setup_unified_logging(*args, **kwargs)

def wrap2_180(angle_deg: float) -> float:
    """
    Wrap an angle in degrees to the range [-180, 180).
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0

def wrap2_pi(angle_rad: float) -> float:
    """
    Wrap an angle in radians to the range [-pi, pi).
    """
    pi = np.pi
    return (angle_rad + pi) % (2*pi) - pi


def lla2ned(lat_deg: float, lon_deg: float, alt_m: float,
            lat_ref_deg: float, lon_ref_deg: float, alt_ref_m: float) -> Tuple[float, float, float]:
    """
    Convert latitude/longitude/altitude to local tangent-plane NED coordinates,
    relative to lat_ref_deg/lon_ref_deg/alt_ref_m as the origin.
    
    NOTE: This uses a simplistic spherical Earth approximation. For higher
          accuracy, use a professional geodesy library like pyproj.
    """
    R_earth = 6378137.0  # Earth radius in meters (approx for WGS84)

    # Convert lat/lon to radians
    lat_rad = np.deg2rad(lat_deg)
    lon_rad = np.deg2rad(lon_deg)
    lat_ref_rad = np.deg2rad(lat_ref_deg)
    lon_ref_rad = np.deg2rad(lon_ref_deg)

    # Differences
    dLat = lat_rad - lat_ref_rad
    dLon = lon_rad - lon_ref_rad
    dAlt = alt_m - alt_ref_m

    # Approx for small distances
    dNorth = dLat * R_earth
    dEast  = dLon * R_earth * np.cos(lat_ref_rad)
    dDown  = -dAlt
    return (dNorth, dEast, dDown)

def lla2enu(lat_deg: float, lon_deg: float, alt_m: float,
            lat_ref_deg: float, lon_ref_deg: float, alt_ref_m: float) -> Tuple[float, float, float]:
    
    ned = lla2ned(lat_deg, lon_deg, alt_m, lat_ref_deg, lon_ref_deg, alt_ref_m)

    return ned[1], ned[0], -ned[2]  # Convert NED to ENU by negating the vertical component
    

def quat2rotm(q):
    """
    Convert quaternion(s) in MATLAB convention [w, x, y, z]
    to rotation matrix/matrices using SciPy.

    Parameters
    ----------
    q : array_like, shape (4,) or (N, 4)
        Input quaternion(s) in [w, x, y, z] order.

    Returns
    -------
    rotm : ndarray, shape (3,3) or (N,3,3)
        Rotation matrix (for a single quaternion) or stack of
        rotation matrices (for N quaternions). 
        Rotation is body to inertia frame.
    """
    q = np.asarray(q)
    # Check dimensions
    if q.ndim == 1:
        # reorder to [x, y, z, w]
        q_scipy = q[[1, 2, 3, 0]]
    elif q.ndim == 2:
        # reorder each row
        q_scipy = q[:, [1, 2, 3, 0]]
    else:
        raise ValueError(f"Input must be 1D or 2D array, got array with ndim={q.ndim}")

    # build rotation(s) and return matrix or stack of matrices
    rot_obj = R.from_quat(q_scipy)
    return rot_obj.as_matrix()

def quat2eul(q, order: str = "ZYX") -> np.ndarray:
    """
    Quaternion(s) ➜ Euler angles.

    Parameters
    ----------
    q : array-like, shape (4,) or (N, 4)
        Quaternion(s) in [w, x, y, z] format.
    order : str, default "ZYX"
        Axis order for the Euler angles.

    Returns
    -------
    np.ndarray, shape (3,) or (N, 3)
        Euler angles (yaw, pitch, roll) in radians, same dimensionality as `q`.
    """
    q = np.asarray(q)

    # Re-order to [x, y, z, w] for SciPy
    if q.ndim == 1:
        q_scipy = q[[1, 2, 3, 0]]
    elif q.ndim == 2:
        q_scipy = q[:, [1, 2, 3, 0]]
    else:
        raise ValueError("`q` must be 1-D or 2-D (N, 4) array.")

    euler = R.from_quat(q_scipy).as_euler(order, degrees=False)
    return euler   # shape matches the input dimensionality

def bodyRates2eulerRates(body_rates, euler_angles):
    """
    Convert body angular rates (p, q, r) to Euler angle rates (phi_dot, theta_dot, psi_dot).
    
    Uses the transformation matrix that relates body rates to Euler rates:
    [phi_dot]   [1  sin(phi)*tan(theta)  cos(phi)*tan(theta)] [p]
    [theta_dot] = [0  cos(phi)            -sin(phi)          ] [q]
    [psi_dot]   [0  sin(phi)/cos(theta)  cos(phi)/cos(theta)] [r]
    
    Parameters
    ----------
    body_rates : array-like, shape (3,)
        Body angular rates [p, q, r] in rad/s (roll rate, pitch rate, yaw rate in body frame).
    euler_angles : array-like, shape (3,)
        Current Euler angles [yaw, pitch, roll] or [phi, theta, psi] in radians.
        Order depends on convention but typically [yaw, pitch, roll].
    
    Returns
    -------
    euler_rates : np.ndarray, shape (3,)
        Euler angle rates [phi_dot, theta_dot, psi_dot] in rad/s.
    
    Notes
    -----
    This transformation is singular when theta (pitch) approaches ±90 degrees.
    """
    body_rates = np.asarray(body_rates)
    euler_angles = np.asarray(euler_angles)
    
    # Extract Euler angles (assuming order is [yaw, pitch, roll])
    # Adjust indices if your convention is different
    psi, theta, phi = euler_angles  # yaw, pitch, roll

    # Extract body rates
    p, q, r = body_rates
    
    # Transformation matrix from body rates to Euler rates
    T = np.array([
        [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
    ])
    
    # Compute Euler rates
    euler_rates = T @ body_rates
    
    return euler_rates


def eul2quat(euler_angles, order: str = "ZYX") -> np.ndarray:
    """
    Euler angles ➜ quaternion(s).

    Parameters
    ----------
    euler_angles : array-like, shape (3,) or (N, 3)
        Angles (yaw, pitch, roll) in radians.
    order : str, default "ZYX"
        Axis order corresponding to the angles.

    Returns
    -------
    np.ndarray, shape (4,) or (N, 4)
        Quaternion(s) in [w, x, y, z] format, same dimensionality as `euler_angles`.
    """
    euler_angles = np.asarray(euler_angles)
    quat_scipy = R.from_euler(order, euler_angles, degrees=False).as_quat()  # [x, y, z, w]

    if quat_scipy.ndim == 1:
        quat = quat_scipy[[3, 0, 1, 2]]
    elif quat_scipy.ndim == 2:
        quat = quat_scipy[:, [3, 0, 1, 2]]
    else:
        raise ValueError("`euler_angles` must be 1-D or 2-D (N, 3) array.")

    return quat   # shape matches the input dimensionality

def rotm2quat(rotm):
    """
    Convert a 3x3 rotation matrix to a quaternion [w, x, y, z] using SciPy.
    Rotation is body to inertia frame.
    """
    # Create a Rotation object from the rotation matrix
    r = R.from_matrix(rotm)
    
    # Convert to quaternion (SciPy outputs [w, x, y, z])
    quat_scipy = r.as_quat()

    quat = quat_scipy[[3, 0, 1, 2]]

    
    # Reorder quaternion to [w, x, y, z] to match MATLAB convention
    return quat

def quatmultiply(q1,q2):
    """
    inputs : q1(M X 4 ) q2(M X 4 )
    outputs : r(M X 4)
    """
    
    # Ensure both q1 and q2 are at least 2D
    q1 = np.atleast_2d(q1)
    q2 = np.atleast_2d(q2)
    
    # Extract components of q1
    w1, x1, y1, z1 = q1[:, 0], q1[: ,1], q1[:, 2], q1[:, 3]

    # Extract components of q2
    w2, x2, y2, z2 = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]

    # Compute quaternion multiplication
    w3 = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x3 = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y3 = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z3 = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    # Stack results to form r (Nx4) 
    r = np.column_stack((w3, x3, y3, z3))
    
    return r.squeeze()

def exp_quat(d_rot):
    """
    Exponential map from small angle rotation (3 x N) to quaternion (4 x N).

    If d_rot is 1D (3,), output will be 1D (4,).
    Otherwise (3, N) --> (4, N).
    """

    # Reshape d_rot to (3, N), whether it starts as (3,) or (3, N).
    d_rot = np.reshape(d_rot, (3, -1))  # no if-statement needed

    # norm of each column
    norms = np.linalg.norm(d_rot, axis=0)
    half = 0.5 * norms
    qw = np.cos(half)

    # Avoid divide-by-zero
    qr = np.zeros_like(d_rot,dtype= float)
    mask = norms > 1e-12
    qr[:, mask] = (d_rot[:, mask] / norms[mask]) * np.sin(half[mask])

    # stack into [w; x; y; z] (shape: (4, N))
    quat = np.vstack([qw, qr])
    
    # Squeeze so that (4,1) becomes (4,), while (4,N) stays (4,N) for N>1
    return quat.squeeze()

def exp_rot(d_rot):
    """
    Exponential map from small angle vector (3,) to rotation matrix (3 x 3).

    Implements eq. 78 from "Quaternion kinematics for the error-state Kalman Filter"
    or the standard Rodrigues' rotation formula.

    d_rot : ndarray of shape (3,)
    Returns
    -------
    rotM : ndarray of shape (3,3)
    """
    if d_rot.ndim != 1 or d_rot.size != 3:
        raise ValueError("d_rot must be a 3-element vector.")

    angle = np.linalg.norm(d_rot)
    if abs(angle) < 1e-12:
        # No rotation
        return np.eye(3)

    axis = d_rot / angle
    s = np.sin(angle)
    c = np.cos(angle)
    # Skew-symmetric of axis
    axis_skew = np.array([
        [0,       -axis[2],  axis[1]],
        [axis[2],  0,       -axis[0]],
        [-axis[1], axis[0],  0]
    ])
    rotM = c * np.eye(3) + s * axis_skew + (1 - c) * np.outer(axis, axis)
    return rotM

def rotate_image(image, angle_rad):
    """
    Rotate image by angle_degrees about the center, 'crop' style.
    In MATLAB, 'imrotate(img, angle, 'crop')' rotates about the center and keeps
    the image size the same. Below is an example using OpenCV that approximates it.
    """
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)
    # Get rotation matrix for the desired angle
    R = cv2.getRotationMatrix2D(center, np.degrees(-angle_rad), 1.0)  # getRotationMatrix2D accept (+) angle in counter clock wise, we use NED which is yaw positive in clock wise    
    # # Warp (rotate) keeping the original image size
    rotated = cv2.warpAffine(image, R, (w, h))
    
    
    # # Compute the sine and cosine of the rotation angle
    # abs_cos = abs(R[0, 0])
    # abs_sin = abs(R[0, 1])

    # # Compute new bounding dimensions
    # new_w = int(h * abs_sin + w * abs_cos)
    # new_h = int(h * abs_cos + w * abs_sin)

    # # Adjust the rotation matrix to account for translation
    # R[0, 2] += (new_w / 2) - center[0]
    # R[1, 2] += (new_h / 2) - center[1]
    # Perform the actual rotation and resize
    # rotated = cv2.warpAffine(image, R, (new_w, new_h))
    
    
    return rotated

def square_crop_from_center(image, flagCropIndex=False):
    """
    Crop a square region from the center of the image.
    
    Parameters:
    - image: Input image (numpy array).
    
    Returns:
    - Cropped square image (numpy array).
    """
    h, w               = image.shape[:2]
    center_x, center_y = w // 2, h // 2
    crop_height        = min(h, w)
    half_crop          = crop_height // 2

    start_x = max(center_x - half_crop, 0)
    end_x   = min(center_x + half_crop, w)
    start_y = max(center_y - half_crop, 0)
    end_y   = min(center_y + half_crop, h)
    
    if flagCropIndex:
        return start_x, end_x, start_y, end_y

    else:
        return image[start_y:end_y, start_x:end_x]
    
    
def resize_image(frame, snapDim = (256, 256)):
    
    return cv2.resize(frame, snapDim, interpolation=cv2.INTER_AREA)  

  

def drawKeypoints(frame, keypoints, color=(0, 255, 0), radius=3, thickness=2):
    """
    Draw keypoints (circles) on an image.

    :param frame:      The input image (NumPy array).
    :param keypoints:  A NumPy array of shape (N, 2) containing keypoint coordinates (x, y).
    :param color:      BGR color tuple for the circles (default=(0, 255, 0)).
    :param radius:     Radius of the drawn circles (default=3).
    :param thickness:  Thickness of the circle boundary.
                      If set to -1, the circle is drawn filled (default=-1).
    :return:           The image with keypoints drawn.
    """
    # Ensure keypoints array is of type int or round the coordinates properly
    # before passing them to cv2.circle.
    
    frame     = np.ascontiguousarray(frame)
    # w,h       = frame.shape[:2]
    # radius    = 3*(w//100) # ensure radius is reasonable
    # thickness = 2*(w//100)
    
    radius = 1
    thickness = -1
    
    if frame.ndim == 2:
        
        frame = cv2.merge([frame, frame, frame])
        
    for (x, y) in keypoints:
        # Use int casting in case keypoints are floats
        cv2.circle(frame, (int(x), int(y)), radius, color, thickness)
        
        # frame = cv2.imread('data/UAV_img.jpg')
        # cv2.circle(frame, (int(x), int(y)), radius, color, thickness)
        # cv2.imshow('deneme',frame)
    return frame


def extract_rotated_patch_optimized(
    image: np.ndarray,
    center: Tuple[float, float],
    patch_size: Tuple[int, int],
    yaw_angle: float
) -> np.ndarray:
    """
    Optimized version: Extract rotated patch with single transformation.
    
    Parameters:
    -----------
    image : np.ndarray
        Input satellite image (H, W, C) or (H, W)
    center : Tuple[float, float]
        Center coordinates (x, y) in the original image
    patch_size : int
        Size of the square patch to extract
    yaw_angle : float
        Rotation angle in degrees (clockwise positive)
        
    Returns:
    --------
    np.ndarray
        Rotated square patch of size (patch_size, patch_size, C)
    """
    cx, cy = float(center[0]), float(center[1])
    w, h = patch_size
    
    # Get rotation matrix around the center point
    M = cv2.getRotationMatrix2D((cx, cy), -yaw_angle, 1.0)
    
    # Adjust translation to center the patch
    M[0, 2] += w / 2 - cx
    M[1, 2] += h / 2 - cy
    
    # Extract rotated patch in one step
    patch = cv2.warpAffine(
        image,
        M,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )
    
    return patch

def getLogData(csvDataPath, start_row = 0, end_row = None):
    # Load CSV file
    df = pd.read_csv(csvDataPath)
    
    # Extract ORGN data before filtering other data
    orgn_data = {
        'Lat': np.array(df['ORGN[0].Lat']),
        'Lng': np.array(df['ORGN[0].Lng']),
        'Alt': np.array(df['ORGN[0].Alt'])
    }
    
    # Filter data from the specified row
    if end_row is not None:
        df = df.iloc[start_row:end_row]
    else:
        df = df.iloc[start_row:]

    # Convert to numpy arrays and normalize timestamps
    timestamps = np.array(df['timestamp(ms)'])
    timestamps = timestamps - timestamps[0]  # Normalize timestamps to start at 0

    # --- FIX: convert raw GPS ints into floats with decimal after 2 digits ---
    def insert_decimal(x):
        s = str(int(x))
        # if value shorter than 3 digits, just convert to float
        if len(s) <= 2:
            return float(s)
        return float(s[:2] + '.' + s[2:])

    df['GPS[0].Lat'] = df['GPS[0].Lat'].apply(insert_decimal)
    df['GPS[0].Lng'] = df['GPS[0].Lng'].apply(insert_decimal)

    # GPS Data
    gps_data = {
        'Lat': np.array(df['GPS[0].Lat']),
        'Lng': np.array(df['GPS[0].Lng']),
        'Alt': np.array(df['GPS[0].Alt']),
        'Spd': np.array(df['GPS[0].Spd']),
        'VZ' : np.array(df['GPS[0].VZ']),
        'Yaw': np.array(df['GPS[0].Yaw'])
    }

    # IMU Data
    imu_data = {
        'GyrX': np.array(df['IMU[0].GyrX']),
        'GyrY': np.array(df['IMU[0].GyrY']),
        'GyrZ': np.array(df['IMU[0].GyrZ']),
        'AccX': np.array(df['IMU[0].AccX']),
        'AccY': np.array(df['IMU[0].AccY']),
        'AccZ': np.array(df['IMU[0].AccZ'])
        # 'GHz' : np.array(df['IMU[0].GHz'])
    }

    # XKF Data
    xkf_data = {
        'PN'        : np.array(df['XKF1[0].PN']),
        'PE'        : np.array(df['XKF1[0].PE']),
        'PD'        : np.array(df['XKF1[0].PD']),
        'VN'        : np.array(df['XKF1[0].VN']),
        'VE'        : np.array(df['XKF1[0].VE']),
        'VD'        : np.array(df['XKF1[0].VD']),
        'Roll'      : np.array(df['XKF1[0].Roll']),
        'Pitch'     : np.array(df['XKF1[0].Pitch']),
        'Yaw'       : np.array(df['XKF1[0].Yaw']),
        'GX' : np.array(df['XKF1[0].GX']),
        'GY' : np.array(df['XKF1[0].GY']),
        'GZ' : np.array(df['XKF1[0].GZ'])

    }

    # Output Data
    data_dict = {
        'timestamp': timestamps,
        'GPS': gps_data,
        'IMU': imu_data,
        'ORGN': orgn_data,
        'XKF': xkf_data
    }

    # Print parsed data keys to check structure
    print(f'Log files processed as : {data_dict.keys()}')
    return data_dict


def resize_save_video(input_video_path, output_video_path, width):
    """
    Resize each frame of an MP4 video and save the resized version.

    Parameters:
    - input_video_path (str): Path to the input video file.
    - output_video_path (str): Path to save the resized video.
    - width (int): Desired width of the resized frames.
    - height (int): Desired height of the resized frames.
    """

    # Open the input video
    cap = cv2.VideoCapture(input_video_path)

    if not cap.isOpened():
        raise ValueError("Error: Could not open video file.")

    # Get original video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # keep ratio of the original video
    width_original = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height_original = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    height = int((width / width_original) * height_original)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Get the codec used

    # Define the codec and create a VideoWriter object
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # End of video

        # Resize the frame
        resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        # Write the resized frame to the output video
        out.write(resized_frame)

    # Release resources
    cap.release()
    out.release()
    print(f"Resized video saved as: {output_video_path}")
    
    
def px2ned(px, leftupperNED, mp, pxRned):
    """
    Convert pixel coordinates to NED coordinates.
    """

    return np.append(np.dot(px*mp, pxRned)) + leftupperNED.astype(float)

def ned2px(ned,leftupperNED, mp, pxRned):
    """
    Convert NED coordinates to pixel coordinates.
    """
    result = np.round(np.dot((ned - leftupperNED).reshape(-1, 3), pxRned.T)[:, :2] / mp).astype(int)

    if result.shape == (2,1) or result.shape == (1,2):
        result = result.squeeze()
        
    return np.atleast_2d(result)

def resize_video(input_video_path, output_video_path, width, height):
    """
    Resize each frame of an MP4 video and save the resized version.

    Parameters:
    - input_video_path (str): Path to the input video file.
    - output_video_path (str): Path to save the resized video.
    - width (int): Desired width of the resized frames.
    - height (int): Desired height of the resized frames.
    """

    # Open the input video
    cap = cv2.VideoCapture(input_video_path)

    if not cap.isOpened():
        raise ValueError("Error: Could not open video file.")

    # Get original video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))  # Get the codec used

    # Define the codec and create a VideoWriter object
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height))
    frameCount = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # End of video

        if (not frameCount): #only calculate crop dimensions once
            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2
            crop_height = min(h, w)
            half_crop = crop_height // 2

            start_x = max(center_x - half_crop, 0)
            end_x = min(center_x + half_crop, w)
            start_y = max(center_y - half_crop, 0)
            end_y = min(center_y + half_crop, h)

        frame = frame[start_y:end_y, start_x:end_x]
        
        frameCount += 1

        # Resize the frame
        resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        # Write the resized frame to the output video
        out.write(resized_frame)

    # Release resources
    cap.release()
    out.release()
    print(f"Resized video saved as: {output_video_path}")


def compute_colorized_histogram(image, bins=(8, 8, 8), exclude_black=True):
    """
    Compute a normalized color histogram for an image in the HSV color space.
    Optionally excludes pixels that are fully black (0,0,0 in BGR).
    
    Args:
        image (numpy.ndarray): Input image in BGR format.
        bins (tuple): Number of bins for each channel (H, S, V).
        exclude_black (bool): Whether to exclude full black pixels from the histogram.
    
    Returns:
        numpy.ndarray: Flattened and normalized histogram.
    """
    
    mask=None
    # Create a mask to exclude full black pixels if needed
    if exclude_black:
        # This mask will be 255 (white) for pixels that are NOT full black
        # and 0 for pixels that are exactly (0,0,0) in BGR.
        mask = cv2.inRange(image, np.array([1, 1, 1], dtype=np.uint8),
                           np.array([255, 255, 255], dtype=np.uint8))
    
    # Convert the image to the HSV color space.
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Compute the histogram for the HSV image using the mask (if provided)
    hist = cv2.calcHist([image], channels=[0, 1, 2], mask=mask,
                        histSize=bins, ranges=[0, 180, 0, 256, 0, 256])
    
    # Normalize the histogram so that the sum is 1.
    hist = cv2.normalize(hist, hist).flatten()
    
    return hist

# def color_similarity(hist1, hist2):
#     """
#     Compute a similarity metric between two images based solely on their color distribution,
#     while excluding full black pixels. This metric is rotation invariant since it relies on
#     the global color histogram.
    
#     Args:
#         image1, image2 (numpy.ndarray): Input images in BGR format.
#         bins (tuple): Number of bins for each HSV channel.
#         exclude_black (bool): Whether to exclude full black pixels from the histogram.
    
#     Returns:
#         float: Similarity score between 0 and 1.
#     """

#     # 0 is Corrolation based calculation
#     return cv2.compareHist(hist1, hist2, 0)


def draw_custom_matches(imgA, kpA,
                        imgB, kpB,
                        matchesIdx,
                        circle_radius=2,
                        circle_color=(0, 255, 0),
                        line_color=(0, 255, 0),
                        line_thickness=1):
    """
    Draw matched keypoints from imgA to imgB side by side, 
    connecting only those specified by matchesIdx.

    Parameters
    ----------
    imgA : np.ndarray
        Left image (BGR or grayscale) with shape (H1, W1) or (H1, W1, 3).
    kpA : np.ndarray
        (N_A, 2) array of (x, y) coordinates for *all* keypoints in imgA.
    imgB : np.ndarray
        Right image (BGR or grayscale) with shape (H2, W2) or (H2, W2, 3).
    kpB : np.ndarray
        (N_B, 2) array of (x, y) coordinates for *all* keypoints in imgB.
    matchesIdx : np.ndarray or list
        (M, 2) array-like. Each row is [iA, iB], meaning:
          - iA is an index into kpA
          - iB is an index into kpB
        indicating a match between kpA[iA] and kpB[iB].
    circle_radius : int, optional
        Radius of the circle to draw around each keypoint. Default is 5.
    circle_color : (B, G, R) tuple
        BGR color for circles. Default is green.
    line_color : (B, G, R) tuple
        BGR color for lines connecting matches. Default is green.
    line_thickness : int, optional
        Thickness of the connecting lines. Default is 2.

    Returns
    -------
    output : np.ndarray
        A new image showing imgA (left) and imgB (right) side by side,
        with lines drawn connecting matched keypoints.
    """

    # Ensure both images are in BGR if they are grayscale
    if len(imgA.shape) == 2:
        imgA = cv2.cvtColor(imgA, cv2.COLOR_GRAY2BGR)
    if len(imgB.shape) == 2:
        imgB = cv2.cvtColor(imgB, cv2.COLOR_GRAY2BGR)

    hA, wA = imgA.shape[:2]
    hB, wB = imgB.shape[:2]

    # Create an output canvas to hold both images side-by-side
    out_height = max(hA, hB)
    out_width  = wA + wB
    output = np.zeros((out_height, out_width, 3), dtype=np.uint8)

    # Place imgA (left) and imgB (right) on the canvas
    output[:hA, :wA] = imgA
    output[:hB, wA:wA + wB] = imgB

    # Draw matches
    for (iA, iB) in matchesIdx:
        # Extract keypoint coordinates from kpA and kpB
        xA, yA = kpA[iA]
        xB, yB = kpB[iB]

        # Offset xB by wA since imgB is to the right of imgA
        xB_offset = xB + wA

        # Draw circles at each keypoint
        cv2.circle(output, (int(xA), int(yA)), circle_radius, circle_color, -1)
        cv2.circle(output, (int(xB_offset), int(yB)), circle_radius, circle_color, -1)

        # Draw a line connecting them
        cv2.line(output, (int(xA), int(yA)), (int(xB_offset), int(yB)), line_color, thickness=line_thickness)

    return output


def generate_orthoprojection(
    img,
    K_cam,
    dist_coeffs,
    R_ned_c,
    t_ned,
    landmarks_ned,
    x_range=(-120, 120.0),  # meters in NED (East direction)
    y_range=(-120, 120.0),  # meters in NED (North direction)
    resolution=550 / 1800,           # meters per pixel in Ω
    flagENU = False
):
    """
    Generate orthoprojection Ω from a single UAV camera image, following the
    plane-fitting + homography method described in the paper.

    All inputs are expected in NED (North-East-Down) coordinate frame.

    INPUTS
    ------
    img : (H, W, 3) or (H, W) uint8
        Distorted camera image at the LAST frame of the batch. Pixels in
        the camera image frame {C_img} (OpenCV convention, origin = top-left).

    K_cam : (3, 3) float64
        Camera intrinsic matrix for the *distorted* image, mapping camera-frame
        rays {C} to pixels {C_img}.

    dist_coeffs : (4 or 5,) float64
        Radial–tangential distortion coefficients [k1, k2, p1, p2, (k3)].

    R_ned_c : (3, 3) float64
        Rotation from CAMERA frame {C} to NED frame.
        X_ned = R_ned_c @ X_c  (no translation here).
        
        NOTE: Camera frame convention typically is:
          - X_c: right
          - Y_c: down  
          - Z_c: forward (optical axis)

    t_ned : (3,) float64
        Camera center in NED frame (translation component).
        Position = [North, East, Down] in meters.

    landmarks_ned : (N, 3) float64
        3D landmark positions l_i in NED frame, assumed to lie on
        static, locally flat terrain in the area of interest.
        Each row = [North, East, Down] in meters.

    x_range : (2,) float
        (x_min, x_max) of Ω in NED frame, in meters (East direction).
        The ortho image width spans from x_min (East) to x_max (East).

    y_range : (2,) float
        (y_min, y_max) of Ω in NED frame, in meters (North direction).
        The ortho image height spans from y_min (North) to y_max (North).

    resolution : float
        Ground resolution of Ω: meters per pixel.

    OUTPUTS
    -------
    ortho_img : (H_ortho, W_ortho, 3) uint8
        Orthoprojection Ω at the requested resolution and ranges.
        - Image Y-axis (rows) corresponds to North (increasing row = decreasing North)
        - Image X-axis (cols) corresponds to East (increasing col = increasing East)

    ortho_mask : (H_ortho, W_ortho) uint8
        Mask Ω_m, 255 = valid pixel (came from original image), 0 = invalid.

    P_center_ned : (3,) float64
        Intersection point of image center ray with ground plane, in NED frame.
        This can be used as the position correction for particles.

    plane_offset : float
        Plane offset d such that n^T X + d = 0 in NED frame.

    H_img_to_ortho : (3, 3) float64
        Homography that maps image pixels (u,v,1) in undistorted image
        directly to orthoprojection pixel coordinates (j,i,1) in Ω.
    """

    # ------------------------------------------------------------
    # 0. Undistort the last camera image
    # ------------------------------------------------------------
    h, w = img.shape[:2]

    # alpha=0 → crop to valid region, best for geometry
    newK, _ = cv2.getOptimalNewCameraMatrix(K_cam, dist_coeffs, (w, h), alpha=0)
    img_undist = cv2.undistort(img, K_cam, dist_coeffs, None, newK)

    K_u = newK.astype(np.float64)
    h_u, w_u = img_undist.shape[:2]

    # ------------------------------------------------------------
    # 1. Fit a plane q: n^T X + d = 0 to landmarks in NED frame
    # ------------------------------------------------------------
    # Shift landmarks N,E relative to camera center for numerical stability
    pts = np.asarray(landmarks_ned, dtype=np.float64).copy()
    C_ned = np.asarray(t_ned, dtype=np.float64).reshape(3)
    pts[:, 0:2] -= C_ned.reshape(1, 3)[:, :2]  # Shift N,E only
    C_ned_shifted = np.array([0.0, 0.0, C_ned[2]], dtype=np.float64)  # Keep Down component
    
    assert pts.shape[1] == 3 and pts.shape[0] >= 3, "Need at least 3 landmarks"

    centroid = pts.mean(axis=0)
    pts_centered = pts - centroid
    # SVD: last singular vector = plane normal
    _, _, vh = np.linalg.svd(pts_centered, full_matrices=False)
    n = vh[-1, :]  # normal
    n /= np.linalg.norm(n)

    # For NED frame (Z = Down), the ground plane normal should point UP,
    # which means the D-component of normal should be NEGATIVE (opposite to Down)
    if n[2] > 0:
        n = -n

    d_plane = -np.dot(n, centroid)

    # ------------------------------------------------------------
    # 2. Ray-plane intersection for the 4 image corners (undistorted image)
    # ------------------------------------------------------------

    # Corners in image pixels (u,v) for undistorted image
    corners_img = np.array([
        [0.0,      0.0     ],  # upper-left  (ul)
        [w_u - 1., 0.0     ],  # upper-right (ur)
        [0.0,      h_u - 1.],  # lower-left  (ll)
        [w_u - 1., h_u - 1.]   # lower-right (lr)
    ], dtype=np.float64)

    # Center coordinate of image in pixel (u,v) for finding ground intersection
    center_img = np.array([w_u // 2, h_u // 2], dtype=np.float64)
    center_img = np.concatenate([center_img, [1.0]], axis=0).reshape(3, 1)  # (3,1) homogeneous

    # Back-project pixel corners to camera rays (camera frame {C})
    # Form homogeneous pixels
    pixels_h = np.concatenate(
        [corners_img, np.ones((4, 1), dtype=np.float64)],
        axis=1
    ).T  # shape (3, 4)

    K_u_inv = np.linalg.inv(K_u)
    rays_c = K_u_inv @ pixels_h   # (3,4), unnormalized
    rays_c /= np.linalg.norm(rays_c, axis=0, keepdims=True)

    ray_center_c = K_u_inv @ center_img  # (3,1)
    ray_center_c /= np.linalg.norm(ray_center_c, axis=0, keepdims=True)

    # Convert rays to NED frame: d_ned = R_ned_c @ d_c
    R_ned_c = np.asarray(R_ned_c, dtype=np.float64)
    rays_ned = R_ned_c @ rays_c  # shape (3,4)
    ray_center_ned = R_ned_c @ ray_center_c  # (3,1)

    # Ray-plane intersection: X = C_ned_shifted + lambda * d_ned
    # n^T X + d = 0  -> lambda = -(n^T C_ned_shifted + d) / (n^T d_ned)
    n_dot_C = np.dot(n, C_ned_shifted)
    n_dot_rays = n @ rays_ned  # shape (4,)
    
    # Avoid division by zero
    eps = 1e-9
    n_dot_rays[np.abs(n_dot_rays) < eps] = eps

    n_dot_ray_center = n @ ray_center_ned  # (1,)
    if np.abs(n_dot_ray_center) < eps:
        n_dot_ray_center = eps

    lambdas = -(n_dot_C + d_plane) / n_dot_rays  # shape (4,)
    lambda_center = -(n_dot_C + d_plane) / n_dot_ray_center  # scalar

    # Intersection points in NED frame (shifted coordinates)
    P_ned = C_ned_shifted.reshape(3, 1) + rays_ned * lambdas  # (3,4)
    P_ned = P_ned.T  # (4,3): [pul, pur, pll, plr] each row is [N, E, D]
    
    P_center_shifted = C_ned_shifted.reshape(3, 1) + ray_center_ned * lambda_center  # (3,1)
    
    # Convert P_center back to original NED coordinates (unshift)
    P_center_ned = P_center_shifted.flatten()
    # P_center_ned[0] += C_ned[0]  # Add back North offset
    # P_center_ned[1] += C_ned[1]  # Add back East offset

    # For homography, we use (East, North) as (x, y) coordinates
    # NED format: [N, E, D] -> we want to map to ortho image where:
    #   - Image column (j) corresponds to East (E = index 1)
    #   - Image row (i) corresponds to North (top = max North)
    # So dst_xy should be [E, N] = [column 1, column 0] of P_ned
    dst_xy = np.column_stack([P_ned[:, 1], P_ned[:, 0]]).astype(np.float32)  # (4,2): [E, N]
    if flagENU:
        dst_xy = np.column_stack([P_ned[:, 0], P_ned[:, 1]]).astype(np.float32)  # (4,2): [E, N]

    # ------------------------------------------------------------
    # 3. Homography: image pixels -> (E, N) in NED frame
    # ------------------------------------------------------------
    src_uv = corners_img.astype(np.float32)  # (4,2)
    H_img_to_xy = cv2.getPerspectiveTransform(src_uv, dst_xy)  # maps (u,v) -> (E, N)

    # ------------------------------------------------------------
    # 4. Build scaling+translation from NED (E, N) -> Ω pixel indices (j, i)
    #    Ω spans x_range (East), y_range (North) with 'resolution' m/pixel.
    # ------------------------------------------------------------
    x_min, x_max = x_range  # East range
    y_min, y_max = y_range  # North range
    res = float(resolution)

    W_ortho = int(np.round((x_max - x_min) / res))  # Width in pixels (East)
    H_ortho = int(np.round((y_max - y_min) / res))  # Height in pixels (North)

    # Mapping from (E, N) to pixel (j, i):
    #   j = (E - x_min) / res          (column index, increases with East)
    #   i = (y_max - N) / res          (row index, increases southward = decreasing North)
    S_world_to_ortho = np.array([
        [1.0 / res,       0.0,      -x_min / res],   # j = (E - E_min) / res
        [      0.0, -1.0 / res,       y_max / res],  # i = (N_max - N) / res  
        [      0.0,       0.0,                1.0]
    ], dtype=np.float64)

    # Compose: image pixels -> (E, N) -> ortho pixel indices
    H_img_to_ortho = S_world_to_ortho @ H_img_to_xy  # (3,3)

    # No 180° rotation hack needed when frame conventions are correct

    # ------------------------------------------------------------
    # 5. Warp the undistorted camera image to orthoprojection Ω
    # ------------------------------------------------------------
    ortho_img = cv2.warpPerspective(
        img_undist,
        H_img_to_ortho,
        dsize=(W_ortho, H_ortho),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )

    # ------------------------------------------------------------
    # 6. Generate mask Ω_m of valid pixels (non-black)
    # ------------------------------------------------------------
    mask_src = np.ones((h_u, w_u), dtype=np.uint8) * 255
    ortho_mask = cv2.warpPerspective(
        mask_src,
        H_img_to_ortho,
        dsize=(W_ortho, H_ortho),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )

    # Mask the black borders by cropping to non-black region
    ortho_img, crop_bbox = crop_non_black(ortho_img)
    if crop_bbox is not None:
        y_crop_min, y_crop_max, x_crop_min, x_crop_max = crop_bbox
        ortho_mask = ortho_mask[y_crop_min:y_crop_max, x_crop_min:x_crop_max]

    return ortho_img, ortho_mask, P_center_ned, d_plane, H_img_to_ortho

def crop_non_black(image: np.ndarray, threshold: int = 0):
    """
    Crop an image (H×W or H×W×C) to the minimal axis-aligned rectangle
    that contains all pixels whose value > threshold (for any channel).

    Parameters
    ----------
    image : np.ndarray
        Input image. Can be grayscale (H,W) or color (H,W,C).
    threshold : int or float, optional
        Pixels with all channels <= threshold are treated as "black".
        Default is 0.

    Returns
    -------
    cropped : np.ndarray
        Cropped image containing only the non-black region.
    bbox : tuple or None
        (y_min, y_max, x_min, x_max) of the crop in original image
        (y_max and x_max are exclusive, usable directly in slicing).
        If the image is completely black, returns (image, None).
    """
    if image.ndim == 3:
        # any channel > threshold → non-black
        mask = np.any(image > threshold, axis=2)
    elif image.ndim == 2:
        mask = image > threshold
    else:
        raise ValueError("image must be 2D or 3D numpy array")

    # Find rows/cols that contain any non-black pixel
    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]

    if rows.size == 0 or cols.size == 0:
        # Entire image is black
        return image, None

    y_min, y_max = rows[0], rows[-1] + 1  # +1 for slicing
    x_min, x_max = cols[0], cols[-1] + 1

    cropped = image[y_min:y_max, x_min:x_max]
    return cropped, (y_min, y_max, x_min, x_max)


def calculate_heading_mag(mag, quat_mavros, declination=np.deg2rad(6.03)):
    """
    Calculate heading (true north referenced) from raw magnetometer readings and attitude.

    Args:
        mag: tuple or list (Mx, My, Mz) raw magnetometer readings.
        quat: tuple or list (qx, qy, qz, qw) quaternion representing the device's orientation in mavros format.
        declination: magnetic declination δ in radians (default 0).

    Returns:
        heading: heading angle in radians, from true north, normalized to [-π, π].
    """
    Mx, My, Mz = mag

    qx, qy, qz, qw = quat_mavros

    euler = quat2eul([qw, qx, qy, qz], order='ZYX')   # Quaternion(s) should be in [w, x, y, z] format as input to that quat2eul function

    _, pitch, roll = euler  # Extract yaw, pitch, roll angles in radians

    # Tilt compensation
    # 1) Pitch rotation about the body-y axis(inertia to body frame):
    R_pitch = np.array([
        [ np.cos(pitch), 0,  -np.sin(pitch)],
        [             0, 1,              0],
        [ np.sin(pitch), 0,   np.cos(pitch)]
    ])

    # 2)  Roll rotation about the body-x axis (inertia to body frame):
    R_roll = np.array([
        [1,              0,               0],
        [0,  np.cos(roll),  np.sin(roll)],
        [0, -np.sin(roll),  np.cos(roll)]
    ])

    # 3) Yaw rotation about the body-z axis (inertia to body frame):
    # R_yaw = np.array([
    #     [ np.cos(0) , np.sin(0), 0],
    #     [-np.sin(0) , np.cos(0), 0],
    #     [0,              0, 1]
    # ])  # yaw is not used here, but included for completeness

    # 3) Combined: first pitch, then inverse roll
    R = R_roll @ R_pitch #Note: Inertia to body frame)
    R_level = R.T  # Inverse rotation (body to inertia frame level(no tilt))

    # 4) Apply to mag vector
    Xh, Yh, _ = R_level @ np.array([Mx, My, Mz])

    # 2. Magnetic heading
    heading_mag = math.atan2(Yh, Xh)

    # 3. True heading
    heading_true = heading_mag + declination

    # 4. Normalize to [-π, π]
    heading_true = wrap2_pi(heading_true)

    return heading_true


class StreamToLogger:
    """Redirects print/stdout/stderr to logging."""
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level

    def write(self, buf):
        for line in buf.rstrip().splitlines():
            self.logger.log(self.level, line.rstrip())

    def flush(self):
        pass


def setup_logging(
        name="VIO",
        level=logging.INFO,
        log_dir_name="logs_out",
        to_stdout=True,
):
    """Backward-compatible logging entrypoint.

    Historically, this function created a new timestamped log file and also
    configured RCUTILS_LOG_FILE_PATH for ROS 2.

    In this repo we now prefer a single unified multi-process log sink via
    `setup_unified_logging()`, controlled by `UNIFIED_LOG_FILE`.

    - If `UNIFIED_LOG_FILE` is set, everything (python logging + redirected
      stdout/stderr) will append to that shared file.
    - If it's not set, we fall back to creating a timestamped file under
      `log_dir_name` using `name` as the prefix.
    """
    # Delegate to unified logging.
    log_path = setup_unified_logging(
        log_file=None,
        log_dir_name=log_dir_name,
        log_file_prefix=name,
        level=level,
    )

    # Respect the old behavior of toggling console output.
    # `setup_unified_logging` always attaches a console handler; we can disable it
    # by removing StreamHandlers from the root logger.
    if not to_stdout:
        root = logging.getLogger()
        root.handlers = [h for h in root.handlers if not isinstance(h, logging.StreamHandler)]

    logging.getLogger(__name__).info("Logging initialized (unified): %s", str(log_path))


def rotate_waypoints(wp_list, yaw_deg):
    """
    Rotates all waypoints around origin (0,0,0) by yaw_deg degrees.
    
    Args:
        wp_list (list of [x,y,z]): Input waypoint list.
        yaw_deg (float): Rotation angle in degrees.
        
    Returns:
        list of [x_rot, y_rot, z]: Rotated waypoint list.
    """

    yaw = math.radians(yaw_deg)  # convert to radians
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    rotated = []
    for (x, y, z) in wp_list:
        # Standard 2D rotation about Z axis
        x_r = x * cos_y - y * sin_y
        y_r = x * sin_y + y * cos_y
        rotated.append([x_r, y_r, z])

    return rotated


def zncc_particles_from_satellite_map(sat_img, template, xs, ys, template_mask=None, eps=1e-12):
    """
    Compute ZNCC scores between a template and multiple patches from a satellite image.
    Vectorized implementation without for loops, with optional mask support.
    
    Parameters
    ----------
    sat_img : np.ndarray, shape (H, W)
        Grayscale satellite image.
    template : np.ndarray, shape (h, w)
        Grayscale UAV template image.
    xs : np.ndarray, shape (N,)
        X-coordinates (left) of particle patch top-left corners in sat_img.
    ys : np.ndarray, shape (N,)
        Y-coordinates (top) of particle patch top-left corners in sat_img.
    template_mask : np.ndarray, shape (h, w), optional
        Binary mask for the template (255=valid, 0=invalid/black region).
        If None, all pixels are considered valid.
    eps : float, optional
        Small epsilon to prevent division by zero. Default is 1e-12.
    
    Returns
    -------
    zncc : np.ndarray, shape (N,)
        ZNCC scores for each particle location.
        
    Notes
    -----
    - xs and ys must satisfy: 0 <= xs <= W-w and 0 <= ys <= H-h
    - When using a mask, only valid (non-zero) pixels contribute to ZNCC
    - Invalid patches (too few valid pixels) will have ZNCC score of 0.0
    """
    sat = sat_img.astype(np.float32)
    T = template.astype(np.float32)
    h, w = T.shape
    
    # Handle mask
    if template_mask is None:
        # No mask: all pixels valid
        mask = np.ones((h, w), dtype=np.uint8) * 255
    else:
        mask = template_mask.astype(np.uint8)
    
    # Binary mask (1=valid, 0=invalid)
    mask_binary = (mask > 0).astype(np.float32)
    
    # Count valid pixels
    Npix = np.sum(mask_binary)
    
    # Require at least 50% valid pixels for meaningful ZNCC
    if Npix < 0.5 * h * w:
        xs = np.asarray(xs, dtype=np.int32)
        ys = np.asarray(ys, dtype=np.int32)
        return np.zeros(len(xs), dtype=np.float32)
    
    # Masked template: zero out invalid pixels
    T_masked = T * mask_binary
    
    # Compute mean over valid pixels only
    T_mean = np.sum(T_masked) / Npix
    
    # Zero-mean template (only for valid pixels)
    T0 = (T - T_mean) * mask_binary
    
    # Template denominator: sqrt(sum(T0^2))
    denom_T = np.sqrt(np.sum(T0 * T0)) + eps
    
    # === Numerator: sum(patch * T0) ===
    # Use matchTemplate with masked template
    num_map = cv2.matchTemplate(sat, T0, cv2.TM_CCORR)
    
    # === Denominator: compute patch statistics with mask ===
    # We need: sum(patch * mask) and sum((patch * mask)^2)
    
    # Create masked versions for integral images
    # For each patch, we need:
    #   S = sum(I * mask)
    #   S2 = sum((I * mask)^2)
    #   N_valid = sum(mask)
    
    # Integral images
    sum_map, sqsum_map = cv2.integral2(sat)  # (H+1, W+1)
    
    # For mask-aware computation, we need patch-specific mask sums
    # Create integral image of the mask
    mask_expanded = np.zeros_like(sat, dtype=np.float32)
    
    xs = np.asarray(xs, dtype=np.int32)
    ys = np.asarray(ys, dtype=np.int32)
    N = len(xs)
    
    # Pre-compute mask sum (constant for all particles)
    mask_sum = np.sum(mask_binary)
    
    # For each particle, extract patch and compute masked statistics
    x0 = xs
    y0 = ys
    x1 = xs + w
    y1 = ys + h
    
    # Vectorized patch extraction using advanced indexing
    # Extract all patches at once
    zncc = np.zeros(N, dtype=np.float32)
    
    for i in range(N):
        # Extract patch from satellite
        patch = sat[y0[i]:y1[i], x0[i]:x1[i]]
        
        # Apply mask
        patch_masked = patch * mask_binary
        
        # Compute statistics over valid pixels
        patch_sum = np.sum(patch_masked)
        patch_mean = patch_sum / Npix
        
        # Zero-mean patch (only valid pixels)
        patch_zero_mean = (patch - patch_mean) * mask_binary
        
        # Denominator for patch
        denom_patch = np.sqrt(np.sum(patch_zero_mean * patch_zero_mean)) + eps
        
        # Numerator (correlation)
        numerator = np.sum(T0 * patch_zero_mean)
        
        # ZNCC score
        zncc[i] = numerator / (denom_T * denom_patch)
    
    return zncc


def zncc_particles_from_satellite_map_fast(sat_img, template, xs, ys, template_mask=None, eps=1e-12):
    """
    Fast vectorized ZNCC computation without mask (for backward compatibility).
    Use this when template_mask is None or all pixels are valid.
    
    Parameters
    ----------
    sat_img : np.ndarray, shape (H, W)
        Grayscale satellite image.
    template : np.ndarray, shape (h, w)
        Grayscale UAV template image.
    xs : np.ndarray, shape (N,)
        X-coordinates of particle patch top-left corners.
    ys : np.ndarray, shape (N,)
        Y-coordinates of particle patch top-left corners.
    template_mask : np.ndarray, optional
        If provided, falls back to masked version.
    eps : float, optional
        Small epsilon to prevent division by zero.
    
    Returns
    -------
    zncc : np.ndarray, shape (N,)
        ZNCC scores for each particle location.
    """
    # If mask is provided, use masked version
    if template_mask is not None:
        return zncc_particles_from_satellite_map(sat_img, template, xs, ys, template_mask, eps)
    
    sat = sat_img.astype(np.float32)
    T = template.astype(np.float32)
    h, w = T.shape
    Npix = float(h * w)

    # Zero-mean template
    T0 = T - T.mean()
    denom_T = np.sqrt(np.sum(T0 * T0)) + eps

    # Numerator map
    num_map = cv2.matchTemplate(sat, T0, cv2.TM_CCORR)

    # Integral images for fast patch statistics
    sum_map, sqsum_map = cv2.integral2(sat)

    xs = np.asarray(xs, dtype=np.int32)
    ys = np.asarray(ys, dtype=np.int32)

    x0 = xs
    y0 = ys
    x1 = xs + w
    y1 = ys + h

    # Vectorized patch sums
    S  = sum_map[y1, x1] - sum_map[y0, x1] - sum_map[y1, x0] + sum_map[y0, x0]
    S2 = sqsum_map[y1, x1] - sqsum_map[y0, x1] - sqsum_map[y1, x0] + sqsum_map[y0, x0]

    # Variance computation
    ss = S2 - (S * S) / Npix
    ss = np.maximum(ss, eps)

    denom = np.sqrt(ss) * denom_T

    # Get numerator at particle locations
    num = num_map[y0, x0]

    zncc = num / denom

    return zncc


