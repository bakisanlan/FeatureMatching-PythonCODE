import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import pandas as pd
from typing import Tuple


def wrap2_180(angle_deg: float) -> float:
    """
    Wrap an angle in degrees to the range [-180, 180).
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0


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

def quat2rotm(q):
    """
    Convert quaternion [w, x, y, z] to a 3x3 rotation matrix using SciPy.
    """
    # MATLAB convention is [w, x, y, z]; SciPy is [x, y, z, w]
    q_scipy = np.array([q[1], q[2], q[3], q[0]])
    return R.from_quat(q_scipy).as_matrix()

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
    """
    # Create a Rotation object from the rotation matrix
    r = R.from_matrix(rotm)
    
    # Convert to quaternion (SciPy outputs [w, x, y, z])
    quat = r.as_quat(scalar_first = True)
    
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
    
    frame = np.ascontiguousarray(frame)
    
    
    if frame.ndim == 2:
        
        frame = cv2.merge([frame, frame, frame])
        
    for (x, y) in keypoints:
        # Use int casting in case keypoints are floats
        cv2.circle(frame, (int(x), int(y)), radius, color, thickness)
        
        # frame = cv2.imread('data/UAV_img.jpg')
        # cv2.circle(frame, (int(x), int(y)), radius, color, thickness)
        # cv2.imshow('deneme',frame)
    return frame

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

def color_similarity(hist1, hist2):
    """
    Compute a similarity metric between two images based solely on their color distribution,
    while excluding full black pixels. This metric is rotation invariant since it relies on
    the global color histogram.
    
    Args:
        image1, image2 (numpy.ndarray): Input images in BGR format.
        bins (tuple): Number of bins for each HSV channel.
        exclude_black (bool): Whether to exclude full black pixels from the histogram.
    
    Returns:
        float: Similarity score between 0 and 1.
    """

    # 0 is Corrolation based calculation
    return cv2.compareHist(hist1, hist2, 0)


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