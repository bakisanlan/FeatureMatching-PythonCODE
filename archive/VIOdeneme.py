import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# ----------------------------
# Helper function: skew-symmetric matrix
# ----------------------------
def skew(vec):
    """
    Returns the skew-symmetric matrix of a 3-vector.
    """
    return np.array([
        [0, -vec[2], vec[1]],
        [vec[2], 0, -vec[0]],
        [-vec[1], vec[0], 0]
    ])

# ----------------------------
# Extended Kalman Filter class for IMU-VO fusion
# ----------------------------
class IMUVOEKF:
    def __init__(self, dt, init_state=None):
        """
        Initialize the filter.
        
        Parameters:
          dt         : nominal IMU time-step (seconds)
          init_state : optional dictionary with keys 'p', 'v', 'q', 'b_a', 'b_g'
                       p, v, b_a, b_g are 3D numpy arrays and q is a quaternion stored
                       as a SciPy Rotation (default is identity rotation).
        """
        # Initialize state
        if init_state is None:
            self.state = {
                'p': np.zeros(3),                   # position
                'v': np.zeros(3),                   # velocity
                'q': R.from_quat([0, 0, 0, 1]),       # orientation (x,y,z,w)
                'b_a': np.zeros(3),                 # accelerometer bias
                'b_g': np.zeros(3)                  # gyroscope bias
            }
        else:
            self.state = init_state

        # Error state covariance (15x15)
        self.P = np.eye(15) * 1e-3  # small initial uncertainty
        
        # Process noise standard deviations (tune these!)
        self.sigma_acc = 0.1        # [m/s^2]
        self.sigma_gyro = 0.01      # [rad/s]
        self.sigma_acc_bias = 0.001   # [m/s^2]
        self.sigma_gyro_bias = 0.0001 # [rad/s]
        
        self.dt = dt
        
        # Measurement noise covariance for VO (position-only measurement)
        self.R_meas = np.eye(3) * 0.05  # e.g., 5cm std dev
        
        # Gravity vector (assumed constant in world frame; here along -z)
        self.g = np.array([0, 0, -9.81])
    
    def propagate(self, acc_meas, gyro_meas, dt=None):
        """
        Propagate the state using IMU measurements.
        
        Parameters:
          acc_meas  : measured acceleration (3D numpy array)
          gyro_meas : measured angular velocity (3D numpy array)
          dt        : time step (if None, uses self.dt)
        """
        if dt is None:
            dt = self.dt

        # Extract state components
        p = self.state['p']
        v = self.state['v']
        q = self.state['q']  # as a SciPy Rotation object
        b_a = self.state['b_a']
        b_g = self.state['b_g']
        
        # Remove biases
        acc_corr = acc_meas - b_a
        gyro_corr = gyro_meas - b_g
        
        # Get rotation matrix from body frame to world frame
        Rwb = q.as_matrix()
        
        # Propagate position and velocity (discrete-time kinematics)
        p_new = p + v * dt + 0.5 * (Rwb @ acc_corr + self.g) * dt**2
        v_new = v + (Rwb @ acc_corr + self.g) * dt
        
        # Propagate orientation:
        # Use the rotation vector (gyro_corr * dt) to form a small rotation update.
        delta_q = R.from_rotvec(gyro_corr * dt)
        q_new = delta_q * q  # left multiplication
        q_new = q_new.normalized()
        
        # Update the state
        self.state['p'] = p_new
        self.state['v'] = v_new
        self.state['q'] = q_new
        # Biases (b_a and b_g) are assumed constant here (could add random walk)
        
        # --- EKF Covariance Propagation ---
        # Define the continuous-time error-state Jacobian (F)
        F = np.eye(15)
        # dp/dv
        F[0:3, 3:6] = np.eye(3) * dt
        # dp/dθ: -0.5 * Rwb * skew(acc_corr) * dt^2
        F[0:3, 6:9] = -0.5 * Rwb @ skew(acc_corr) * dt**2
        # dp/d(b_a)
        F[0:3, 9:12] = -0.5 * Rwb * dt**2
        
        # dv/dθ: - Rwb * skew(acc_corr) * dt
        F[3:6, 6:9] = - Rwb @ skew(acc_corr) * dt
        # dv/d(b_a)
        F[3:6, 9:12] = - Rwb * dt
        
        # dθ/dθ: approximated as I - skew(gyro_corr)*dt
        F[6:9, 6:9] = np.eye(3) - skew(gyro_corr) * dt
        # dθ/d(b_g)
        F[6:9, 12:15] = - np.eye(3) * dt
        
        # The bias blocks remain as identity (bias modeled as random walk)
        
        # Define the process noise covariance Q (for the error state)
        Q = np.zeros((15, 15))
        Q[0:3, 0:3] = (0.5 * dt**2)**2 * (self.sigma_acc**2) * np.eye(3)
        Q[3:6, 3:6] = (dt)**2 * (self.sigma_acc**2) * np.eye(3)
        Q[6:9, 6:9] = (dt)**2 * (self.sigma_gyro**2) * np.eye(3)
        Q[9:12, 9:12] = dt * (self.sigma_acc_bias**2) * np.eye(3)
        Q[12:15, 12:15] = dt * (self.sigma_gyro_bias**2) * np.eye(3)
        
        # Propagate covariance
        self.P = F @ self.P @ F.T + Q

    def update(self, vo_position):
        """
        Update the filter with a VO position measurement.
        
        Parameters:
          vo_position : measured position (3D numpy array)
        """
        # Measurement model: z = p + noise.
        # Measurement Jacobian H is:
        #    H = [I_3, 0_3x12]
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)
        
        # Innovation (measurement residual)
        p_est = self.state['p']
        y = vo_position - p_est
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_meas
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Compute the error state update (15x1 vector)
        delta_x = K @ y
        
        # ----- Correct the nominal state -----
        # Position and velocity are updated additively.
        self.state['p'] = self.state['p'] + delta_x[0:3]
        self.state['v'] = self.state['v'] + delta_x[3:6]
        
        # Orientation update: use the small-angle error (delta_theta)
        delta_theta = delta_x[6:9]
        delta_q = R.from_rotvec(delta_theta)
        # Left-multiplicative update: q_new = delta_q * q_estimated
        self.state['q'] = (delta_q * self.state['q']).normalized()
        
        # Update the biases
        self.state['b_a'] = self.state['b_a'] + delta_x[9:12]
        self.state['b_g'] = self.state['b_g'] + delta_x[12:15]
        
        # Covariance update (Joseph form for numerical consistency)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_meas @ K.T

    def get_state(self):
        """
        Returns a copy of the current state. The quaternion is returned as [x, y, z, w].
        """
        state_out = self.state.copy()
        state_out['q'] = self.state['q'].as_quat()  # convert Rotation to quaternion array
        return state_out

# ----------------------------
# Visual Odometry function using ORB and Essential matrix
# ----------------------------
def process_vo_frame(prev_frame, curr_frame, K):
    """
    Process two consecutive frames to compute the relative camera motion.
    
    Parameters:
      prev_frame : previous image frame (BGR or grayscale)
      curr_frame : current image frame (BGR or grayscale)
      K          : camera intrinsic matrix (3x3 numpy array)
    
    Returns:
      R_rel : relative rotation matrix (3x3 numpy array)
      t_rel : relative translation vector (3x1 numpy array; up to scale)
              If not enough matches are found, returns (None, None).
    """
    # Convert to grayscale if needed.
    if len(prev_frame.shape) == 3:
        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    else:
        prev_gray = prev_frame.copy()
    if len(curr_frame.shape) == 3:
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
    else:
        curr_gray = curr_frame.copy()
        
    # Initialize ORB detector.
    orb = cv2.ORB_create(nfeatures=1000)
    
    # Detect keypoints and compute descriptors.
    kp1, des1 = orb.detectAndCompute(prev_gray, None)
    kp2, des2 = orb.detectAndCompute(curr_gray, None)
    
    if des1 is None or des2 is None:
        return None, None
    
    # Use BFMatcher with Hamming distance.
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    
    if len(matches) < 8:
        return None, None
    
    # Sort matches by descriptor distance.
    matches = sorted(matches, key=lambda x: x.distance)
    
    # Extract matched keypoints.
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
    
    # Compute the essential matrix using RANSAC.
    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None:
        return None, None

    # Recover pose from the essential matrix.
    retval, R_rel, t_rel, mask_pose = cv2.recoverPose(E, pts1, pts2, K)
    return R_rel, t_rel

# ----------------------------
# Main function: Simulate IMU propagation and fuse with VO from video frames
# ----------------------------
if __name__ == "__main__":
    # -------- Simulation / Data Parameters --------
    total_time = 10.0       # seconds
    imu_rate = 100.0        # Hz (IMU update frequency)
    imu_dt = 1.0 / imu_rate

    # Video parameters
    video_path = "your_flight_video.mp4"  # replace with your video file path
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError("Cannot open video file.")
    # Use video frame rate as VO update rate.
    vo_rate = cap.get(cv2.CAP_PROP_FPS)
    vo_dt = 1.0 / vo_rate

    # Camera intrinsics (example values, replace with your calibrated values)
    fx, fy = 718.8560, 718.8560
    cx, cy = 607.1928, 185.2157
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]])

    # Create an instance of the EKF with the IMU time step.
    ekf = IMUVOEKF(dt=imu_dt)
    
    num_steps = int(total_time * imu_rate)
    
    # For plotting and analysis.
    est_positions = []
    vo_positions  = []  # cumulative VO positions used for EKF updates.
    time_axis = []

    # ----------------------------
    # Initialize VO pose (cumulative transformation).
    # We represent the pose as a rotation matrix and translation vector.
    T_vo_R = np.eye(3)      # cumulative rotation (world frame)
    T_vo_t = np.zeros(3)    # cumulative translation (world frame)

    # Read the first frame for VO.
    ret, prev_frame = cap.read()
    if not ret:
        raise IOError("Could not read the first video frame.")

    next_vo_time = vo_dt  # time at which next VO update should occur

    # For simulation, we assume a simple ground truth: constant velocity in x.
    true_velocity = np.array([1.0, 0.0, 0.0])  # [m/s]
    true_position = np.zeros(3)
    true_acc = np.zeros(3)
    true_gyro = np.zeros(3)
    
    print("Starting fusion loop...")
    # Main loop: simulate IMU propagation and (when available) process a new video frame for VO.
    for i in range(num_steps):
        t = i * imu_dt
        time_axis.append(t)
        
        # -------- Ground Truth Propagation (for simulation) --------
        true_position = true_position + true_velocity * imu_dt
        
        # -------- Simulate IMU measurements (add Gaussian noise) --------
        acc_meas = true_acc + ekf.sigma_acc * np.random.randn(3)
        gyro_meas = true_gyro + ekf.sigma_gyro * np.random.randn(3)
        
        # Propagate the EKF state using the (noisy) IMU measurements.
        ekf.propagate(acc_meas, gyro_meas, imu_dt)
        
        # -------- Process VO update when it's time --------
        if t >= next_vo_time:
            ret, curr_frame = cap.read()
            if not ret:
                print("End of video reached.")
                break
            
            R_rel, t_rel = process_vo_frame(prev_frame, curr_frame, K)
            if R_rel is not None and t_rel is not None:
                # t_rel is unit-norm; scale it using an approximate scale.
                # Here, we use the EKF estimated velocity norm times the VO frame interval.
                v_est_norm = np.linalg.norm(ekf.state['v'])
                scale = v_est_norm * vo_dt
                # If the estimated velocity is too small (e.g. near zero), choose a default scale.
                if scale < 0.1:
                    scale = 1.0
                t_rel_scaled = t_rel.flatten() * scale  # ensure it's a 1D array
                
                # -------- Integrate the relative motion to update the cumulative VO pose --------
                # New translation is the old translation plus the old rotation applied to the scaled relative translation.
                T_vo_t = T_vo_t + T_vo_R @ t_rel_scaled
                # New rotation is the composition of the old rotation and the relative rotation.
                T_vo_R = T_vo_R @ R_rel
                
                vo_position = T_vo_t.copy()
                vo_positions.append(vo_position)
                
                # Use the cumulative VO position as the measurement in the EKF update.
                ekf.update(vo_position)
                
                # For the next VO update, set the current frame as the new previous frame.
                prev_frame = curr_frame.copy()
            else:
                print(f"VO update at time {t:.2f}s: Not enough matches or pose not recovered.")
            
            next_vo_time += vo_dt
        
        # Save the estimated position for plotting.
        est_positions.append(ekf.state['p'].copy())
    
    cap.release()
    
    # Convert lists to numpy arrays for plotting.
    est_positions = np.array(est_positions)
    vo_positions = np.array(vo_positions) if vo_positions else np.empty((0,3))
    # For comparison, ground truth positions from simulation.
    true_positions = np.array([true_velocity * t for t in time_axis])
    
    # -------- Plotting the Trajectories (XY Projection) --------
    plt.figure(figsize=(10, 6))
    plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label='Ground Truth')
    plt.plot(est_positions[:, 0], est_positions[:, 1], 'b-', label='EKF Estimate')
    if vo_positions.shape[0] > 0:
        plt.scatter(vo_positions[:, 0], vo_positions[:, 1], c='r', marker='x', label='VO Updates')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('IMU-VO EKF Fusion Trajectory (XY Projection)')
    plt.legend()
    plt.grid(True)
    plt.show()
