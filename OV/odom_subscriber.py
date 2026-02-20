#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
import time
import logging

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField,NavSatFix, Imu, PointCloud2, Image, FluidPressure
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError  # Import CvBridge

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from mavros_msgs.msg import HomePosition,State
from rclpy.qos import qos_profile_sensor_data
import os
import sys
from rclpy.callback_groups import ReentrantCallbackGroup


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import calculate_heading_mag, quat2rotm
from OV.utils_OV.common_utils import ned_VIO_converter, yaw_diff_finder, ned_SLAM_PC_converter


logger = logging.getLogger(__name__)


class OdomDataLogger:
    """
    Efficient data logger for odometry data at 20 Hz.
    
    Uses pre-allocated NumPy arrays for memory efficiency.
    Logs position, velocity, and orientation for GT, VIO, and PF sources.
    """
    
    def __init__(self, log_rate_hz: float = 20.0, max_duration_sec: float = 600.0):
        """
        Initialize the data logger.
        
        Args:
            log_rate_hz: Target logging rate in Hz (default 20 Hz = 50ms interval)
            max_duration_sec: Maximum duration to log in seconds (default 600s = 10 min)
        """
        self.log_interval = 1.0 / log_rate_hz  # 50ms for 20 Hz
        self.max_samples = int(log_rate_hz * max_duration_sec)
        
        # Pre-allocate arrays for each data source
        # GT (Ground Truth): position(3), velocity(3), orientation(4)
        self._gt_ts = np.full(self.max_samples, np.nan, dtype=np.float64)
        self._gt_pos = np.full((self.max_samples, 3), np.nan, dtype=np.float32)
        self._gt_vel = np.full((self.max_samples, 3), np.nan, dtype=np.float32)
        self._gt_ori = np.full((self.max_samples, 4), np.nan, dtype=np.float32)
        
        # VIO: position(3), velocity(3), orientation(4)
        self._vio_ts = np.full(self.max_samples, np.nan, dtype=np.float64)
        self._vio_pos = np.full((self.max_samples, 3), np.nan, dtype=np.float32)
        self._vio_vel = np.full((self.max_samples, 3), np.nan, dtype=np.float32)
        self._vio_ori = np.full((self.max_samples, 4), np.nan, dtype=np.float32)
        
        # PF (Particle Filter): position(3), orientation(4) - no velocity
        self._pf_ts = np.full(self.max_samples, np.nan, dtype=np.float64)
        self._pf_pos = np.full((self.max_samples, 3), np.nan, dtype=np.float32)
        self._pf_ori = np.full((self.max_samples, 4), np.nan, dtype=np.float32)
        
        # Indices for each source (they may update at different rates)
        self._gt_idx = 0
        self._vio_idx = 0
        self._pf_idx = 0
        
        # Last log time for rate limiting
        self._last_log_time = 0.0
        
        # Logging active flag
        self._active = False
        
        logger.info('OdomDataLogger initialized: %.1f Hz, max %.1f sec (%d samples)', 
                    log_rate_hz, max_duration_sec, self.max_samples)
    
    def start(self):
        """Start logging and reset buffers."""
        self._gt_idx = 0
        self._vio_idx = 0
        self._pf_idx = 0
        self._last_log_time = 0.0
        self._active = True
        logger.info('OdomDataLogger started')
    
    def stop(self):
        """Stop logging."""
        self._active = False
        logger.info('OdomDataLogger stopped (GT: %d, VIO: %d, PF: %d samples)', 
                    self._gt_idx, self._vio_idx, self._pf_idx)
    
    def log_gt(self, gt_dict: dict):
        """Log ground truth data if rate limit allows."""
        if not self._active or self._gt_idx >= self.max_samples:
            return
        
        ts = gt_dict.get('ts')
        if ts is None:
            return
            
        # Rate limiting
        if ts - self._last_log_time < self.log_interval:
            return
        self._last_log_time = ts
        
        pos = gt_dict.get('position')
        vel = gt_dict.get('velocity')
        ori = gt_dict.get('orientation')
        
        if pos is not None and None not in pos:
            self._gt_ts[self._gt_idx] = ts
            self._gt_pos[self._gt_idx] = pos
            if vel is not None and None not in vel:
                self._gt_vel[self._gt_idx] = vel
            if ori is not None and None not in ori:
                self._gt_ori[self._gt_idx] = ori
            self._gt_idx += 1
    
    def log_vio(self, vio_dict: dict):
        """Log VIO data."""
        if not self._active or self._vio_idx >= self.max_samples:
            return
        
        ts = vio_dict.get('ts')
        if ts is None:
            return
        
        pos = vio_dict.get('position')
        vel = vio_dict.get('velocity')
        ori = vio_dict.get('orientation')
        
        if pos is not None and None not in pos:
            self._vio_ts[self._vio_idx] = ts
            self._vio_pos[self._vio_idx] = pos
            if vel is not None and None not in vel:
                self._vio_vel[self._vio_idx] = vel
            if ori is not None and None not in ori:
                self._vio_ori[self._vio_idx] = ori
            self._vio_idx += 1
    
    def log_pf(self, pf_dict: dict):
        """Log particle filter data."""
        if not self._active or self._pf_idx >= self.max_samples:
            return
        
        ts = pf_dict.get('ts')
        if ts is None:
            return
        
        pos = pf_dict.get('position')
        ori = pf_dict.get('orientation')
        
        if pos is not None:
            # Handle both tuple and numpy array
            try:
                if hasattr(pos, '__iter__') and None not in pos:
                    self._pf_ts[self._pf_idx] = ts
                    self._pf_pos[self._pf_idx] = pos
                    if ori is not None and None not in ori:
                        self._pf_ori[self._pf_idx] = ori
                    self._pf_idx += 1
            except (TypeError, ValueError):
                pass
    
    def save(self, log_dir: str):
        """
        Save logged data to .npy files.
        
        Args:
            log_dir: Directory to save files (will create 'traj' subdirectory)
        """
        import os
        traj_dir = os.path.join(log_dir, 'traj')
        os.makedirs(traj_dir, exist_ok=True)

        def _valid_rows(*cols: np.ndarray) -> np.ndarray:
            """Return boolean mask of rows that contain no NaNs across all provided arrays.

            Each `col` must have the same first dimension (N). It can be 1D (N,) or
            2D (N, D). For 2D, a row is valid only if all elements are finite.
            """
            if not cols:
                return np.zeros((0,), dtype=bool)
            mask = np.ones((cols[0].shape[0],), dtype=bool)
            for c in cols:
                if c.ndim == 1:
                    mask &= np.isfinite(c)
                else:
                    mask &= np.all(np.isfinite(c), axis=1)
            return mask
        
        # Save GT data (trimmed to actual size)
        if self._gt_idx > 0:
            gt_ts = self._gt_ts[:self._gt_idx]
            gt_pos = self._gt_pos[:self._gt_idx]
            gt_vel = self._gt_vel[:self._gt_idx]
            gt_ori = self._gt_ori[:self._gt_idx]
            gt_mask = _valid_rows(gt_ts, gt_pos, gt_vel, gt_ori)
            np.save(os.path.join(traj_dir, 'GT_ts_list.npy'), gt_ts[gt_mask])
            np.save(os.path.join(traj_dir, 'GT_pos_list.npy'), gt_pos[gt_mask])
            np.save(os.path.join(traj_dir, 'GT_vel_list.npy'), gt_vel[gt_mask])
            np.save(os.path.join(traj_dir, 'GT_ori_list.npy'), gt_ori[gt_mask])
        
        # Save VIO data
        if self._vio_idx > 0:
            vio_ts = self._vio_ts[:self._vio_idx]
            vio_pos = self._vio_pos[:self._vio_idx]
            vio_vel = self._vio_vel[:self._vio_idx]
            vio_ori = self._vio_ori[:self._vio_idx]
            vio_mask = _valid_rows(vio_ts, vio_pos, vio_vel, vio_ori)
            np.save(os.path.join(traj_dir, 'VIO_ts_list.npy'), vio_ts[vio_mask])
            np.save(os.path.join(traj_dir, 'VIO_pos_list.npy'), vio_pos[vio_mask])
            np.save(os.path.join(traj_dir, 'VIO_vel_list.npy'), vio_vel[vio_mask])
            np.save(os.path.join(traj_dir, 'VIO_ori_list.npy'), vio_ori[vio_mask])
        
        # Save PF data
        if self._pf_idx > 0:
            pf_ts = self._pf_ts[:self._pf_idx]
            pf_pos = self._pf_pos[:self._pf_idx]
            pf_ori = self._pf_ori[:self._pf_idx]
            pf_mask = _valid_rows(pf_ts, pf_pos, pf_ori)
            np.save(os.path.join(traj_dir, 'PF_ts_list.npy'), pf_ts[pf_mask])
            np.save(os.path.join(traj_dir, 'PF_pos_list.npy'), pf_pos[pf_mask])
            np.save(os.path.join(traj_dir, 'PF_ori_list.npy'), pf_ori[pf_mask])
        
        logger.info('OdomDataLogger saved to %s (GT: %d, VIO: %d, PF: %d samples)',
                    traj_dir, self._gt_idx, self._vio_idx, self._pf_idx)
    
    @property
    def sample_counts(self) -> dict:
        """Return current sample counts for each source."""
        return {
            'gt': self._gt_idx,
            'vio': self._vio_idx,
            'pf': self._pf_idx
        }


class OdomAndMavrosSubscriber(Node):
    def __init__(self):
        super().__init__('odom_and_mavros_subscriber')

        # Smoothing parameters
        self.smoothing = True
        self.manualYaw = True

        # Initialize the CvBridge once
        self.bridge = CvBridge()

        # Initialize VIO dictionary with None values
        self.VIO_dict = {
            'ts': None,
            'dt': None,
            'position': (None, None, None),
            'orientation': (None, None, None, None),
            'velocity': (None, None, None),
            'angular_velocity': (None, None, None)
        }
        
        # Yaw difference for NED conversion
        self.yaw_vioref2enu = None
        self.ned_conversion_initialized = False
        self.last_yaw_update_time = None
        self.yaw_update_interval = 30000000.0  # Update yaw difference every 30 seconds
        
        # VIO divergence detection
        self.vio_divergence_detected = False
        self.try_recover_maneuver    = False
        self.low_slam_pc_start_time = None
        self.slam_pc_threshold = 2
        self.divergence_time_threshold = 5.0  # seconds
        self.divergence_recovery_threshold = 3.0  # seconds

        # --- OpenVINS odometry ---        
        self.first_vo_msg = False
        self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self.VIO_odom_callback,
            qos_profile_sensor_data)

        # Create publishers for status topics
        # Initialize status variables
        self.initialization_status = False
        self.initialization_status_pub = self.create_publisher(
            Bool,
            '/ov_msckf/initialization_status',
            10
        )
        
        self.ready_status = False
        self.ready_status_pub = self.create_publisher(
            Bool,
            '/ov_msckf/ready_status',
            10
        )

        # --- OpenVINS Slam Features --- 
        self.SLAM_PC     = None       
        self.SLAM_PC_ned     = None       

        self.SLAM_PC_num = 0
        self.create_subscription(
            PointCloud2,
            '/ov_msckf/points_slam',
            self.VIO_SLAM_PC_callback,
            10)

        self.first_imu_msg = False
        self.create_subscription(
            Imu,
            '/mavros/imu/data_raw',
            self.imu_callback,
            qos_profile_sensor_data
        )

        # # subscribe to the magnetometer with the sensor_data QoS (best_effort, low latency)
        self.first_imu_mag_msg = False
        self.magYawDeg = None
        self.create_subscription(
            MagneticField,
            '/mavros/imu/mag',
            self.mag_callback,
            qos_profile_sensor_data
        )

        # --- Home‐position (geodetic) ---
        self.home_received = False
        self.home_loc = None    
        self.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self.home_position_callback,
            qos_profile_sensor_data
        )
        
        # ---subscribe to the /mavros/global_position/local topic 
        self.first_gt_odom_msg = False
        self.gt_odom_dict = {
            'ts': None,
            'position': (None, None, None),
            'orientation': (None, None, None, None),
            'velocity': (None, None, None),
            'angular_velocity': (None, None, None)
        }
        
        self.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.gt_odom_callback,
            qos_profile_sensor_data
        )

        self.first_gps_fix_msg = False
        self.gps_fix_loc = None
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_fix_callback,
            qos_profile_sensor_data
        )

        # --- MAVROS state (connected, armed, mode, etc.) ---
        self.first_state_msg = False
        self.state_dict = {
            'connected': None,
            'armed': None,
            'guided': None,
            'mode': None,
            'system_status': None
        }
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sensor_data  # or use 10 for default reliability
        )
        
        # ---Subscribe to Camera image
        self.first_camera_msg = False
        self.camera_image = None
        
        # Track camera callback rate
        self.camera_callback_count = 0
        self.camera_callback_last_log_time = time.time()
        
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10  
        )

        # --Subscribe to IMU static pressure to get altitude
        self.first_pressure_msg = False
        self.pressure = None
        self.p0       = None
        self.baroAlt  = None
        self.create_subscription(
            FluidPressure,
            '/mavros/imu/static_pressure',
            self.pressure_callback,
            qos_profile_sensor_data)
        
        # --- Publishers for NED frame data ---
        self.vio_ned_pub = self.create_publisher(
            Odometry,
            '/vio/odom_ned',
            10
        )
        
        self.gt_ned_pub = self.create_publisher(
            Odometry,
            '/gt/odom_ned',
            10
        )
        
        # Initialize NED dictionaries to store converted odometry
        self.VIOned_dict = {
            'ts': None,
            'dt': None,
            'position': (None, None, None),
            'orientation': (None, None, None, None),
            'velocity': (None, None, None),
            'angular_velocity': (None, None, None)
        }

        self.GTned_dict = {
            'ts': None,
            'dt': None,
            'position': (None, None, None),
            'orientation': (None, None, None, None),
            'velocity': (None, None, None),
            'angular_velocity': (None, None, None)
        }
        
        # --- Particle Filter pose estimate (position + orientation) ---
        self.pf_pos_dict = {
            'ts': None,
            'position': (None, None, None),
            'orientation': (None, None, None, None),
        }

        self.first_pf_pose_msg = False

        # Track subscription rate for PF pose
        self.pf_pose_sub_count = 0
        self.pf_pose_sub_last_log_time = time.time()

        # PF pose: combined estimate (position + orientation)
        self.create_subscription(
            PoseStamped,
            '/pf/pose_estimate',
            self.pf_pose_callback,
            10,
            callback_group = ReentrantCallbackGroup()    ## NOTE : DO REENTRALCALLLBACK WHEN USE PF
        )
        
        # --- Odometry Data Logger for GUI plotting ---
        self.odom_data_logger = OdomDataLogger(log_rate_hz=20.0, max_duration_sec=600.0)
        self.odom_data_logger.start()  # Start logging immediately
        
        # --- Particle Filter particles ---
        # self.first_pf_particles_msg = False
        # self.pf_particles = None  # Will store (N, 2) array of particle positions
        # self.create_subscription(
        #     PoseArray,
        #     '/pf/particles',
        #     self.pf_particles_callback,
        #     10
        # )


    # --------- Callbacks for messages --------------
    def VIO_odom_callback(self, msg: Odometry):
        # --- timestamp ---
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # --- pose ---
        px, py, pz = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        # --- twist ---
        vx, vy, vz = (
            msg.twist.twist.linear.x,  # body frame linear velocity
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        wx, wy, wz = (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        )

        # Smooth the velocity and position if smoothing is enabled
        if self.first_vo_msg:
            if self.smoothing: 

                ### Median smoothing
                self.buf_px.append(px), self.buf_py.append(py), self.buf_pz.append(pz)
                self.buf_vx.append(vx), self.buf_vy.append(vy), self.buf_vz.append(vz)

                px, py, pz = np.median(np.array(self.buf_px)), np.median(np.array(self.buf_py)), np.median(np.array(self.buf_pz))
                vx, vy, vz = np.median(np.array(self.buf_vx)), np.median(np.array(self.buf_vy)), np.median(np.array(self.buf_vz))

        if (not self.first_vo_msg):
            logger.info('VO odom subscriber is initialized')
            self.first_vo_msg = True
            
            # Initialize buffers for smoothing
            window_size = 5
            self.buf_px , self.buf_py , self.buf_pz = deque(maxlen=window_size) , deque(maxlen=window_size), deque(maxlen=window_size)
            self.buf_vx , self.buf_vy , self.buf_vz = deque(maxlen=window_size) , deque(maxlen=window_size), deque(maxlen=window_size)
            
            # Publish initialization status
            self._publish_initialization_status(True)

        # Update dictionary values instead of recreating
        self.VIO_dict_prev = self.VIO_dict.copy()  # Store previous VIO dict for delta calculations for _publish_ned_vio
        prev_ts = self.VIO_dict['ts']
        self.VIO_dict['ts'] = ts
        self.VIO_dict['dt'] = ts - prev_ts if prev_ts is not None else 0
        self.VIO_dict['position'] = (px, py, pz)
        self.VIO_dict['orientation'] = (qx, qy, qz, qw)
        self.VIO_dict['velocity'] = (vx, vy, vz)
        self.VIO_dict['angular_velocity'] = (wx, wy, wz)
        

        # Initialize NED conversion if both VIO and GT are available or VIO and mag are available but GT is not
        if not self.ned_conversion_initialized:
            if self.first_vo_msg:
                self._initialize_ned_conversion()
                
        # Update yaw difference periodically
        if self.ned_conversion_initialized:
            if self.first_vo_msg:
                current_time = time.time()
                if self.last_yaw_update_time is None or (current_time - self.last_yaw_update_time) >= self.yaw_update_interval:
                    self._update_yaw_difference()
        
        # Publish NED frame VIO data if conversion is initialized
        if self.VIO_dict_prev['ts'] is not None:
            self._publish_ned_vio()

    def _publish_initialization_status(self, status: bool):
        """Publish initialization status"""
        if self.initialization_status != status:
            self.initialization_status = status
            msg = Bool()
            msg.data = status
            self.initialization_status_pub.publish(msg)
            
            if status:
                logger.info("🟢 OpenVINS INITIALIZED successfully!")
            else:
                logger.info("🟡 OpenVINS is trying to initialize...")
                
    def _check_and_publish_ready_status(self):
        """Check and publish ready status when both IMU and camera are ready"""
        if self.first_imu_msg and self.first_camera_msg and not self.ready_status:
            self.ready_status = True
            msg = Bool()
            msg.data = True
            self.ready_status_pub.publish(msg)
            logger.info("🔵 OpenVINS READY - receiving IMU and camera data")
    
    def _check_vio_divergence(self):
        """Check for VIO divergence based on SLAM point cloud count"""
        current_time = time.time()
        
        if self.SLAM_PC_num < self.slam_pc_threshold:
            # Low SLAM points detected
            if self.low_slam_pc_start_time is None:
                # Start tracking low SLAM points
                self.low_slam_pc_start_time = current_time
                logger.warning('Low SLAM points detected: %s points', self.SLAM_PC_num)
            else:
                # Check if it's been low for 5 seconds
                time_elapsed = current_time - self.low_slam_pc_start_time
                if time_elapsed >= self.divergence_time_threshold and not self.vio_divergence_detected:
                    # Divergence detected!
                    self.vio_divergence_detected = True
                    self.try_recover_maneuver    = False   # do not try recover maneuver anymore

                    logger.error(
                        '🔴 VIO DIVERGENCE DETECTED! SLAM points < %s for %.1f seconds',
                        self.slam_pc_threshold,
                        time_elapsed,
                    )
                    
                elif time_elapsed >= self.divergence_recovery_threshold and not self.vio_divergence_detected and not self.try_recover_maneuver:
                    logger.warning('⚠️ VIO instability ongoing. Try maneuver for recovering on %.1fs', time_elapsed)
                    self.try_recover_maneuver = True
        else:
            # SLAM points are healthy
            if self.low_slam_pc_start_time is not None:
                # Reset if points recovered before divergence was declared
                time_elapsed = current_time - self.low_slam_pc_start_time
                if not self.vio_divergence_detected:
                    logger.info(
                        '✅ SLAM points recovered: %s points (was low for %.1fs)',
                        self.SLAM_PC_num,
                        time_elapsed,
                    )                
                self.low_slam_pc_start_time = None
                self.try_recover_maneuver   = False

    def VIO_SLAM_PC_callback(self, msg):
        """Callback for OpenVINS SLAM PointCloud2 messages"""

        # Process the PointCloud2 message

        points = point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True,
            reshape_organized_cloud = True
        )

        # Store the point cloud and count
        self.SLAM_PC     = points
        self.SLAM_PC_num = points.shape[0]
        
        # Check for VIO divergence
        self._check_vio_divergence()

        if not self.ned_conversion_initialized:
            return
        else:
            self.SLAM_PC_ned = ned_SLAM_PC_converter(self.SLAM_PC.copy(), self.yaw_vioref2enu)

    def mag_callback(self, msg: MagneticField):
        # get raw magnetometer readings (in Tesla)

        offset = [-150.0439, -107.71817, -127.41827]
        # mx = msg.magnetic_field.x + offset[0]
        # my = msg.magnetic_field.y + offset[1]
        # mz = msg.magnetic_field.z + offset[2]

        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        self.mag       = (mx, my, mz)

        if self.VIO_dict['ts'] is not None: 
            heading_true = calculate_heading_mag((mx, my, mz), self.VIO_dict['orientation'])  # radians

        elif self.gt_odom_dict['ts'] is not None:
            heading_true = calculate_heading_mag((mx, my, mz), self.gt_odom_dict['orientation'])

            # print(f"Magnetometer readings: mx={mx:.3f}, my={my:.3f}, mz={mz:.3f}, yaw_deg={yaw_deg:.2f}, heading_true={np.rad2deg(heading_true):.2f}")
        else:
            heading_true = calculate_heading_mag((mx, my, mz), [0, 0, 0, 1])  # default orientation if no VIO or GT odom data

        if not self.first_imu_mag_msg:
            logger.info('MAVROS magnetometer subscriber is initialized')
            self.first_imu_mag_msg = True

        self.magYawDeg = np.rad2deg(heading_true)


    def home_position_callback(self, msg: HomePosition):
        if not self.home_received:
            # geographic home‐position
            lat = msg.geo.latitude
            lon = msg.geo.longitude
            alt = msg.geo.altitude

            self.home_loc = [lat, lon, alt]

            logger.info(
                'MAVROS home_position subscriber is initialized. Home location --> lat %.7f, lon %.7f, alt %.2f m',
                lat,
                lon,
                alt,
            )

            self.home_received = True

    def gt_odom_callback(self, msg: Odometry):
        # --- timestamp ---
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # --- pose ---
        px, py, pz = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        qx, qy, qz, qw = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        # --- twist ---
        vx, vy, vz = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        wx, wy, wz = (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        )

        # Update dictionary values instead of recreating
        self.gt_odom_dict['ts'] = ts
        self.gt_odom_dict['position'] = (px, py, pz)
        self.gt_odom_dict['orientation'] = (qx, qy, qz, qw)
        self.gt_odom_dict['velocity'] = (vx, vy, -vz)
        self.gt_odom_dict['angular_velocity'] = (wx, wy, wz)

        if not self.first_gt_odom_msg:
            logger.info('MAVROS global_position/local subscriber is initialized')
            self.first_gt_odom_msg = True
        
            
        # Publish NED frame GT data if conversion is initialized
        self._publish_ned_gt()

    def gps_fix_callback(self, msg: NavSatFix):
        # get the GPS fix location
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        self.gps_fix_loc = [lat, lon, alt]

        if not self.first_gps_fix_msg:
            logger.info('MAVROS global_position/global subscriber is initialized')
            self.first_gps_fix_msg = True

    def pressure_callback(self, msg):
        # The 'fluid_pressure' field is in Pascals
        current_pressure = msg.fluid_pressure

        # On the first message, set the ground-level pressure
        if not self.first_pressure_msg:
            self.p0 = current_pressure
            logger.info('Ground pressure P0 set to: %.2f Pa', float(self.p0))
            self.first_pressure_msg = True
            return

        # --- Barometric Formula ---
        # Altitude = 44330.0 * (1.0 - (P / P0)^(1/5.255))
        # (1/5.255) is approx 0.190284

        pressure_ratio = current_pressure / self.p0
        self.baroAlt = 44330.0 * (1.0 - pressure_ratio ** 0.19029495718363465)

        # self.get_logger().info(f'Current Pressure: {current_pressure:.2f} Pa | Calculated Altitude: {self.baroAlt:.2f} m')

    def state_callback(self, msg: State):
        # Update dictionary values instead of recreating
        self.state_dict['connected'] = msg.connected
        self.state_dict['armed'] = msg.armed
        self.state_dict['guided'] = msg.guided
        self.state_dict['mode'] = msg.mode
        self.state_dict['system_status'] = msg.system_status

        if not self.first_state_msg:
            logger.info('MAVROS /state subscriber initialized')
            self.first_state_msg = True

    def camera_callback(self, msg: Image):
        """
        Callback using cv_bridge to process camera image messages
        """
        try:
            # Convert the ROS Image message to an OpenCV format (NumPy array)
            # "bgr8" is the most common target for OpenCV processing.
            # Use "passthrough" if you want the raw encoding without conversion.
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        
        except CvBridgeError as e:
            # Log any errors during conversion
            logger.error('CvBridge Error: %s', e)
            self.camera_image = None
            return
        
        # Your remaining logic can stay the same
        if not self.first_camera_msg:
            # Getting dimensions is simpler from the cv_image
            if self.camera_image is not None:
                h, w = self.camera_image.shape[:2]
                logger.info('Camera (cv_bridge) initialized - size: %sx%s, encoding: mono8', w, h)
                self.first_camera_msg = True
                
                # Check if ready status should be published
                self._check_and_publish_ready_status()
                
        # Check if ready status should be published
        self._check_and_publish_ready_status()
    
    def _initialize_ned_conversion(self):
        """Initialize the yaw difference for NED conversion"""
        try:
            # Calculate yaw difference between VIO arbitrary frame and ENU
            self.yaw_vioref2enu = yaw_diff_finder(
                self.VIO_dict.copy(), 
                self.gt_odom_dict.copy(),
                magYawDeg=self.magYawDeg,
                manualYaw=self.manualYaw
            )
            self.ned_conversion_initialized = True
            self.last_yaw_update_time = time.time()
            logger.info(
                'NED conversion initialized with yaw difference: %.2f degrees',
                float(np.rad2deg(self.yaw_vioref2enu)),
            )
        except Exception as e:
            logger.exception('Failed to initialize NED conversion: %s', e)
    
    def _update_yaw_difference(self):
        """Update the yaw difference periodically"""
        try:
            old_yaw = self.yaw_vioref2enu
            # Recalculate yaw difference
            self.yaw_vioref2enu = yaw_diff_finder(
                self.VIO_dict.copy(), 
                self.gt_odom_dict.copy(),
                magYawDeg=self.magYawDeg,
                manualYaw=self.manualYaw
            )
            self.last_yaw_update_time = time.time()
            
            # Log if there's a significant change (more than 0.5 degrees)
            yaw_change = np.rad2deg(abs(self.yaw_vioref2enu - old_yaw))
            if yaw_change > 2:
                logger.info(
                    'Yaw difference updated: %.2f deg (changed by %.2f deg)',
                    float(np.rad2deg(self.yaw_vioref2enu)),
                    float(yaw_change),
                )
        except Exception as e:
            logger.exception('Failed to update yaw difference: %s', e)
    
    def _publish_ned_vio(self):
        """Publish VIO data in NED frame"""
        if not self.ned_conversion_initialized:
            return
            
        try:
            # Convert to NED frame
            # For converting to NED frame, we use heading difference between VIO frame and ENU frame using GPS/magnetometer.
            # Directly converting position using new heading info can cause large jumps if there's a change in yaw difference.
            # Thus, we compute the difference in position since last VIO message and convert that delta, then we correct heading on delta position, then integrate.
            # However, orientation and velocity can be directly converted since they are relative to body frame.

            # Compute difference since last VIO message to calculate delta in position
            VIO_diff = self.VIO_dict.copy()
            VIO_diff['position'] = tuple(np.array(self.VIO_dict['position']) - np.array(self.VIO_dict_prev['position']))
            # VIO_diff['velocity'] = tuple(np.array(self.VIO_dict['velocity']) - np.array(self.VIO_dict_prev['velocity']))

            vio_ned_dict_diff = ned_VIO_converter(
                VIO_diff, 
                self.yaw_vioref2enu, 
                is_velocity_body=True
            )

            # Update internal NED dict
            prev_ts = self.VIOned_dict['ts']
            self.VIOned_dict['ts']               = self.VIO_dict['ts']
            self.VIOned_dict['dt']               = self.VIO_dict['ts'] - prev_ts if prev_ts is not None else 0
            self.VIOned_dict['position']         = self.VIOned_dict['position'] + vio_ned_dict_diff['position'] if prev_ts is not None else vio_ned_dict_diff['position']
            self.VIOned_dict['orientation']      = vio_ned_dict_diff['orientation']
            self.VIOned_dict['velocity']         = vio_ned_dict_diff['velocity'] #self.VIOned_dict['velocity'] + vio_ned_dict_diff['velocity'] #
            self.VIOned_dict['angular_velocity'] = vio_ned_dict_diff['angular_velocity']

            # Create Odometry message
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom_ned'
            msg.child_frame_id = 'base_link'
            
            # Position
            msg.pose.pose.position.x = float(self.VIOned_dict['position'][0])
            msg.pose.pose.position.y = float(self.VIOned_dict['position'][1])
            msg.pose.pose.position.z = float(self.VIOned_dict['position'][2])
            
            # Orientation (quaternion w,x,y,z -> x,y,z,w for ROS)
            qw, qx, qy, qz = self.VIOned_dict['orientation']
            msg.pose.pose.orientation.x = float(qx)
            msg.pose.pose.orientation.y = float(qy)
            msg.pose.pose.orientation.z = float(qz)
            msg.pose.pose.orientation.w = float(qw)
            
            # Velocity
            msg.twist.twist.linear.x = float(self.VIOned_dict['velocity'][0])
            msg.twist.twist.linear.y = float(self.VIOned_dict['velocity'][1])
            msg.twist.twist.linear.z = float(self.VIOned_dict['velocity'][2])
            
            # Angular velocity
            msg.twist.twist.angular.x = float(self.VIOned_dict['angular_velocity'][0])
            msg.twist.twist.angular.y = float(self.VIOned_dict['angular_velocity'][1])
            msg.twist.twist.angular.z = float(self.VIOned_dict['angular_velocity'][2])
            
            # Publish
            self.vio_ned_pub.publish(msg)
            
            # Log VIO data at 20 Hz for GUI plotting
            # self.odom_data_logger.log_vio(self.VIOned_dict)
            
        except Exception as e:
            logger.exception('Error publishing NED VIO: %s', e)
    
    def _publish_ned_gt(self):
        """Publish ground truth data in NED frame"""
        if not self.ned_conversion_initialized:
            return
            
        try:
            # Convert to NED frame (yaw_diff=0 since GT is already in ENU)
            gt_ned_dict = ned_VIO_converter(
                self.gt_odom_dict.copy(), 
                yaw_vioref2enu=0, 
                is_velocity_body=False
            )
            
            # Update internal NED dict
            prev_ts = self.GTned_dict['ts']
            self.GTned_dict['ts']               = self.gt_odom_dict['ts']
            self.GTned_dict['dt']               = self.gt_odom_dict['ts'] - prev_ts if prev_ts is not None else 0
            self.GTned_dict['position']         = gt_ned_dict['position']
            self.GTned_dict['orientation']      = gt_ned_dict['orientation']
            self.GTned_dict['velocity']         = gt_ned_dict['velocity']
            self.GTned_dict['angular_velocity'] = gt_ned_dict['angular_velocity']

            # Create Odometry message
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom_ned'
            msg.child_frame_id = 'base_link'
            
            # Position
            msg.pose.pose.position.x = float(gt_ned_dict['position'][0])
            msg.pose.pose.position.y = float(gt_ned_dict['position'][1])
            msg.pose.pose.position.z = float(gt_ned_dict['position'][2])
            
            # Orientation (quaternion w,x,y,z -> x,y,z,w for ROS)
            qw, qx, qy, qz = gt_ned_dict['orientation']
            msg.pose.pose.orientation.x = float(qx)
            msg.pose.pose.orientation.y = float(qy)
            msg.pose.pose.orientation.z = float(qz)
            msg.pose.pose.orientation.w = float(qw)
            
            # Velocity
            msg.twist.twist.linear.x = float(gt_ned_dict['velocity'][0])
            msg.twist.twist.linear.y = float(gt_ned_dict['velocity'][1])
            msg.twist.twist.linear.z = float(gt_ned_dict['velocity'][2])
            
            # Angular velocity
            msg.twist.twist.angular.x = float(gt_ned_dict['angular_velocity'][0])
            msg.twist.twist.angular.y = float(gt_ned_dict['angular_velocity'][1])
            msg.twist.twist.angular.z = float(gt_ned_dict['angular_velocity'][2])
            
            # Publish
            self.gt_ned_pub.publish(msg)
            
            # Log GT data at 20 Hz for GUI plotting
            # self.odom_data_logger.log_gt(self.GTned_dict)

            # Update heading with magnetometer/GPS periodically
            self._update_yaw_difference()
            
        except Exception as e:
            logger.exception('Error publishing NED GT: %s', e)
    

    def pf_pose_callback(self, msg: PoseStamped):
        """Callback for combined PF pose estimate.

        Stores:
          - pf_pos_dict['position'] (np.array([x,y,z]))
          - pf_pos_dict['yaw'] (radians)
        """
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self.pf_pos_dict['ts'] = ts
        self.pf_pos_dict['position'] = np.array([px, py, pz])
        self.pf_pos_dict['orientation'] = np.array([qw, qx, qy, qz])   # this will be used by quat2eul which takes w,x,y,z
        
        # Log PF data at 20 Hz for GUI plotting
        self.odom_data_logger.log_pf(self.pf_pos_dict)
        self.odom_data_logger.log_vio(self.VIOned_dict)
        self.odom_data_logger.log_gt(self.GTned_dict)

        if not self.first_pf_pose_msg:
            logger.info('Particle Filter pose estimate subscriber initialized')
            self.first_pf_pose_msg = True

        # Track subscription rate
        self.pf_pose_sub_count += 1
        current_time = time.time()
        time_elapsed = current_time - self.pf_pose_sub_last_log_time
        
        if time_elapsed >= 1.0:
            sub_rate = self.pf_pose_sub_count / time_elapsed
            logger.debug("PF pose subscriber rate: %.2f Hz" % sub_rate)
            self.pf_pose_sub_count = 0
            self.pf_pose_sub_last_log_time = current_time

    def pf_particles_callback(self, msg: PoseArray):
        """Callback for particle filter particles"""
        # Extract all particle positions (N x 2)
        N = len(msg.poses)
        particles = np.zeros((N, 2))
        
        for i, pose in enumerate(msg.poses):
            particles[i, 0] = pose.position.x
            particles[i, 1] = pose.position.y
        
        self.pf_particles = particles
        
        if not self.first_pf_particles_msg:
            logger.info('Particle Filter particles subscriber initialized - receiving %s particles', N)
            self.first_pf_particles_msg = True

def main(args=None):    
    rclpy.init(args=args)
    node = OdomAndMavrosSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
