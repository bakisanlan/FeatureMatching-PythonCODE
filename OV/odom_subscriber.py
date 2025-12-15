#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField,NavSatFix, Imu, PointCloud2, Image, FluidPressure
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge, CvBridgeError  # Import CvBridge

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, PoseArray
from mavros_msgs.msg import HomePosition,State
from rclpy.qos import qos_profile_sensor_data
import os
import sys
from rclpy.callback_groups import ReentrantCallbackGroup


sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import calculate_heading_mag, quat2rotm, setup_logging
from OV.utils_OV.common_utils import ned_VIO_converter, yaw_diff_finder, ned_SLAM_PC_converter

class OdomAndMavrosSubscriber(Node):
    def __init__(self):
        super().__init__('odom_and_mavros_subscriber')

        # Smoothing parameters
        # self.alpha = 0.8
        # self.spike_thresh = 2  # threshold for spike rejection in meters
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
            'angular_velocity': (None, None, None),
            'body_linear_acceleration': (None, None, None)
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
        self.SLAM_PC_num = 0
        self.create_subscription(
            PointCloud2,
            '/ov_msckf/points_slam',
            self.VIO_SLAM_PC_callback,
            10)

        # Subscribe to the IMU data
        self.IMU_RAW = {
            'body_linear_acceleration': (None, None, None),
            'angular_velocity': (None, None, None)
        }
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
        
        # --- Particle Filter position estimate ---
        self.first_pf_pos_msg = False
        self.pf_pos_dict = {
            'ts': None,
            'position': (None, None, None)
        }
        
        # Track subscription rate for pf_pos_estimate
        self.pf_pos_sub_count = 0
        self.pf_pos_sub_last_log_time = time.time()
        
        self.create_subscription(
            PointStamped,
            '/pf/pos_estimate',
            self.pf_pos_callback,
            10,
            callback_group = ReentrantCallbackGroup()    ## NOTE : DO REENTRALCALLLBACK WHEN USE PF
        )
        
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

                ### Low pass filter smoothing
                # raw_position = np.array([px, py, pz])
                # raw_velocity = np.array([vx, vy, vz])

                # prev_position = np.array(self.VIO_dict['position'])
                # prev_velocity = np.array(self.VIO_dict['velocity'])
                # # spike rejection
                # # if np.linalg.norm(raw_position - prev_position) > spike_thresh:
                # #     raw = smoothed_vio_pos
                # # # exponential moving average
                # px, py, pz = self.alpha * raw_position + (1 - self.alpha) * prev_position
                # vx, vy, vz = self.alpha * raw_velocity + (1 - self.alpha) * prev_velocity

                ### Median smoothing
                self.buf_px.append(px), self.buf_py.append(py), self.buf_pz.append(pz)
                self.buf_vx.append(vx), self.buf_vy.append(vy), self.buf_vz.append(vz)

                px, py, pz = np.median(np.array(self.buf_px)), np.median(np.array(self.buf_py)), np.median(np.array(self.buf_pz))
                vx, vy, vz = np.median(np.array(self.buf_vx)), np.median(np.array(self.buf_vy)), np.median(np.array(self.buf_vz))

            # Calculate linear acceleration
            prev_ts = self.VIO_dict['ts']
            dt = ts - prev_ts if prev_ts is not None else 0
            V_prev = np.array(self.VIO_dict['velocity'])
            V_curr = np.array([vx, vy, vz])
            if dt > 0:
                ax = (V_curr[0] - V_prev[0]) / dt
                ay = (V_curr[1] - V_prev[1]) / dt
                az = (V_curr[2] - V_prev[2]) / dt

                g_inertia = np.array([0, 0, -9.80665])  # local gravity vector in m/s^2 inertial frame
                R_body2inertia = quat2rotm([qw, qx, qy, qz])  # rotation from body frame to inertia frame
                g_body = R_body2inertia.T @ g_inertia  # transform gravity to body frame

                # Subtract gravity from the body acceleration to get the linear acceleration
                ax -= g_body[0]
                ay -= g_body[1]
                az -= g_body[2]

            else:
                ax, ay, az = None, None, None

        if (not self.first_vo_msg):
            self.get_logger().info('VO odom subscriber is initialized')
            self.first_vo_msg = True
            
            # Initialize buffers for smoothing
            window_size = 5
            self.buf_px , self.buf_py , self.buf_pz = deque(maxlen=window_size) , deque(maxlen=window_size), deque(maxlen=window_size)
            self.buf_vx , self.buf_vy , self.buf_vz = deque(maxlen=window_size) , deque(maxlen=window_size), deque(maxlen=window_size)

            # Set the first acceleration values to None
            ax, ay, az = None, None, None
            
            # Publish initialization status
            self._publish_initialization_status(True)

        # Update dictionary values instead of recreating
        prev_ts = self.VIO_dict['ts']
        self.VIO_dict['ts'] = ts
        self.VIO_dict['dt'] = ts - prev_ts if prev_ts is not None else 0
        self.VIO_dict['position'] = (px, py, pz)
        self.VIO_dict['orientation'] = (qx, qy, qz, qw)
        self.VIO_dict['velocity'] = (vx, vy, vz)
        self.VIO_dict['angular_velocity'] = (wx, wy, wz)
        self.VIO_dict['body_linear_acceleration'] = (ax, ay, az)
        

        # Initialize NED conversion if both VIO and GT are available or VIO and mag are available but GT is not
        if not self.ned_conversion_initialized:
            # if self.first_vo_msg and (self.first_gt_odom_msg or (self.first_imu_mag_msg and not self.first_gt_odom_msg)):
            # if self.first_vo_msg and self.first_gt_odom_msg:
            if self.first_vo_msg:
                self._initialize_ned_conversion()
                
        # Update yaw difference periodically
        if self.ned_conversion_initialized:
            # if self.first_vo_msg and (self.first_gt_odom_msg or (self.first_imu_mag_msg and not self.first_gt_odom_msg)): 
            # if self.first_vo_msg and self.first_gt_odom_msg:
            if self.first_vo_msg:
                current_time = time.time()
                if self.last_yaw_update_time is None or (current_time - self.last_yaw_update_time) >= self.yaw_update_interval:
                    self._update_yaw_difference()
        
        
        # Publish NED frame VIO data if conversion is initialized
        self._publish_ned_vio()

    def _publish_initialization_status(self, status: bool):
        """Publish initialization status"""
        if self.initialization_status != status:
            self.initialization_status = status
            msg = Bool()
            msg.data = status
            self.initialization_status_pub.publish(msg)
            
            if status:
                self.get_logger().info("🟢 OpenVINS INITIALIZED successfully!")
            else:
                self.get_logger().info("🟡 OpenVINS is trying to initialize...")
                
    def _check_and_publish_ready_status(self):
        """Check and publish ready status when both IMU and camera are ready"""
        if self.first_imu_msg and self.first_camera_msg and not self.ready_status:
            self.ready_status = True
            msg = Bool()
            msg.data = True
            self.ready_status_pub.publish(msg)
            self.get_logger().info("🔵 OpenVINS READY - receiving IMU and camera data")
    
    def _check_vio_divergence(self):
        """Check for VIO divergence based on SLAM point cloud count"""
        current_time = time.time()
        
        if self.SLAM_PC_num < self.slam_pc_threshold:
            # Low SLAM points detected
            if self.low_slam_pc_start_time is None:
                # Start tracking low SLAM points
                self.low_slam_pc_start_time = current_time
                self.get_logger().warn(f'Low SLAM points detected: {self.SLAM_PC_num} points')
                print(f'Low SLAM points detected: {self.SLAM_PC_num} points')
            else:
                # Check if it's been low for 5 seconds
                time_elapsed = current_time - self.low_slam_pc_start_time
                if time_elapsed >= self.divergence_time_threshold and not self.vio_divergence_detected:
                    # Divergence detected!
                    self.vio_divergence_detected = True
                    self.try_recover_maneuver    = False   # do not try recover maneuver anymore

                    self.get_logger().error(f'🔴 VIO DIVERGENCE DETECTED! SLAM points < {self.slam_pc_threshold} for {time_elapsed:.1f} seconds')
                    
                    # # Optionally: Publish initialization status as False
                    # self._publish_initialization_status(False)

                elif time_elapsed >= self.divergence_recovery_threshold and not self.vio_divergence_detected and not self.try_recover_maneuver:
                    self.get_logger().warn(f'⚠️ VIO instability ongoing. Try maneuver for recovering on {time_elapsed:.1f}s')
                    self.try_recover_maneuver = True
        else:
            # SLAM points are healthy
            if self.low_slam_pc_start_time is not None:
                # Reset if points recovered before divergence was declared
                time_elapsed = current_time - self.low_slam_pc_start_time
                if not self.vio_divergence_detected:
                    self.get_logger().info(f'✅ SLAM points recovered: {self.SLAM_PC_num} points (was low for {time_elapsed:.1f}s)')
                # else:
                #     # Recovery from divergence
                #     self.get_logger().info(f'✅ VIO RECOVERED! SLAM points: {self.SLAM_PC_num}')
                #     self.vio_divergence_detected = False
                #     # self._publish_initialization_status(True)
                
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

        # if not self.ned_conversion_initialized:
        #     return
        # else:
        #     self.SLAM_PC_ned = ned_SLAM_PC_converter(self.SLAM_PC.copy(), self.yaw_vioref2enu)

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x  # substract local gravity
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Update dictionary values instead of recreating
        self.IMU_RAW['body_linear_acceleration'] = (ax, ay, az)
        self.IMU_RAW['angular_velocity'] = (wx, wy, wz)

        if not self.first_imu_msg:
            self.get_logger().info('MAVROS IMU subscriber is initialized')
            self.first_imu_msg = True
            
            # Check if ready status should be published
            self._check_and_publish_ready_status()
        # self.get_logger().info(f'Linear Acceleration: x={ax:.3f}, y={ay:.3f}, z={az:.3f}')


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

        # compute heading: atan2(Y, X) in degrees [–180, +180]
        # yaw_rad = math.atan2(my, mx)
        # yaw_deg = math.degrees(yaw_rad)

        if self.VIO_dict['ts'] is not None: 
            heading_true = calculate_heading_mag((mx, my, mz), self.VIO_dict['orientation'])  # radians

        elif self.gt_odom_dict['ts'] is not None:
            heading_true = calculate_heading_mag((mx, my, mz), self.gt_odom_dict['orientation'])

            # print(f"Magnetometer readings: mx={mx:.3f}, my={my:.3f}, mz={mz:.3f}, yaw_deg={yaw_deg:.2f}, heading_true={np.rad2deg(heading_true):.2f}")
        else:
            heading_true = calculate_heading_mag((mx, my, mz), [0, 0, 0, 1])  # default orientation if no VIO or GT odom data

        if not self.first_imu_mag_msg:
            self.get_logger().info('MAVROS magnetometer subscriber is initialized')
            self.first_imu_mag_msg = True

        self.magYawDeg = np.rad2deg(heading_true)


    def home_position_callback(self, msg: HomePosition):
        if not self.home_received:
            # geographic home‐position
            lat = msg.geo.latitude
            lon = msg.geo.longitude
            alt = msg.geo.altitude

            self.home_loc = [lat, lon, alt]

            self.get_logger().info(f'MAVROS home_position subscriber is initialized. Home location --> lat {lat:.7f}, lon {lon:.7f}, alt {alt:.2f} m')

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
            self.get_logger().info('MAVROS global_position/local subscriber is initialized')
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
            self.get_logger().info('MAVROS global_position/global subscriber is initialized')
            self.first_gps_fix_msg = True

    def pressure_callback(self, msg):
        # The 'fluid_pressure' field is in Pascals
        current_pressure = msg.fluid_pressure

        # On the first message, set the ground-level pressure
        if not self.first_pressure_msg:
            self.p0 = current_pressure
            self.get_logger().info(f'Ground pressure P0 set to: {self.p0:.2f} Pa')
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
            self.get_logger().info('MAVROS /state subscriber initialized')
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
            self.get_logger().error(f'CvBridge Error: {e}')
            self.camera_image = None
            return
        
        # Your remaining logic can stay the same
        if not self.first_camera_msg:
            # Getting dimensions is simpler from the cv_image
            if self.camera_image is not None:
                h, w = self.camera_image.shape[:2]
                self.get_logger().info(f'Camera (cv_bridge) initialized - size: {w}x{h}, encoding: mono8')
                self.first_camera_msg = True
                
                # Check if ready status should be published
                self._check_and_publish_ready_status()
        
        # # Track camera callback rate
        # self.camera_callback_count += 1
        # current_time = time.time()
        # time_elapsed = current_time - self.camera_callback_last_log_time
        
        # if time_elapsed >= 1.0:
        #     callback_rate = self.camera_callback_count / time_elapsed
        #     self.get_logger().info(f"Camera callback rate: {callback_rate:.2f} Hz")
        #     self.camera_callback_count = 0
        #     self.camera_callback_last_log_time = current_time
        
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
            self.get_logger().info(f'NED conversion initialized with yaw difference: {np.rad2deg(self.yaw_vioref2enu):.2f} degrees')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize NED conversion: {str(e)}')
    
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
                self.get_logger().info(f'Yaw difference updated: {np.rad2deg(self.yaw_vioref2enu):.2f} deg (changed by {yaw_change:.2f} deg)')
        except Exception as e:
            self.get_logger().error(f'Failed to update yaw difference: {str(e)}')
    
    def _publish_ned_vio(self):
        """Publish VIO data in NED frame"""
        if not self.ned_conversion_initialized:
            return
            
        try:
            # Convert to NED frame
            vio_ned_dict = ned_VIO_converter(
                self.VIO_dict.copy(), 
                self.yaw_vioref2enu, 
                is_velocity_body=True
            )

            # Update internal NED dict
            prev_ts = self.VIOned_dict['ts']
            self.VIOned_dict['ts']               = self.VIO_dict['ts']
            self.VIOned_dict['dt']               = self.VIOned_dict['ts'] - prev_ts if prev_ts is not None else 0
            self.VIOned_dict['position']         = vio_ned_dict['position']
            self.VIOned_dict['orientation']      = vio_ned_dict['orientation']
            self.VIOned_dict['velocity']         = vio_ned_dict['velocity']
            self.VIOned_dict['angular_velocity'] = vio_ned_dict['angular_velocity']

            # Create Odometry message
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom_ned'
            msg.child_frame_id = 'base_link'
            
            # Position
            msg.pose.pose.position.x = float(vio_ned_dict['position'][0])
            msg.pose.pose.position.y = float(vio_ned_dict['position'][1])
            msg.pose.pose.position.z = float(vio_ned_dict['position'][2])
            
            # Orientation (quaternion w,x,y,z -> x,y,z,w for ROS)
            qw, qx, qy, qz = vio_ned_dict['orientation']
            msg.pose.pose.orientation.x = float(qx)
            msg.pose.pose.orientation.y = float(qy)
            msg.pose.pose.orientation.z = float(qz)
            msg.pose.pose.orientation.w = float(qw)
            
            # Velocity
            msg.twist.twist.linear.x = float(vio_ned_dict['velocity'][0])
            msg.twist.twist.linear.y = float(vio_ned_dict['velocity'][1])
            msg.twist.twist.linear.z = float(vio_ned_dict['velocity'][2])
            
            # Angular velocity
            msg.twist.twist.angular.x = float(vio_ned_dict['angular_velocity'][0])
            msg.twist.twist.angular.y = float(vio_ned_dict['angular_velocity'][1])
            msg.twist.twist.angular.z = float(vio_ned_dict['angular_velocity'][2])
            
            # Publish
            self.vio_ned_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing NED VIO: {str(e)}')
    
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
            
        except Exception as e:
            self.get_logger().error(f'Error publishing NED GT: {str(e)}')
    
    def pf_pos_callback(self, msg: PointStamped):
        """Callback for particle filter position estimates"""
        # Extract timestamp
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract position
        px = msg.point.x
        py = msg.point.y
        pz = msg.point.z
        
        # Update dictionary
        self.pf_pos_dict['ts'] = ts
        self.pf_pos_dict['position'] = np.array([px, py, pz])
        
        if not self.first_pf_pos_msg:
            self.get_logger().info('Particle Filter position estimate subscriber initialized')
            self.first_pf_pos_msg = True
        
        # Track subscription rate
        # self.pf_pos_sub_count += 1
        # current_time = time.time()
        # time_elapsed = current_time - self.pf_pos_sub_last_log_time
        
        # if time_elapsed >= 1.0:
        #     sub_rate = self.pf_pos_sub_count / time_elapsed
        #     self.get_logger().info(f"PF position subscriber rate: {sub_rate:.2f} Hz")
        #     self.pf_pos_sub_count = 0
        #     self.pf_pos_sub_last_log_time = current_time

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
            self.get_logger().info(f'Particle Filter particles subscriber initialized - receiving {N} particles')
            self.first_pf_particles_msg = True

def main(args=None):
    rclpy.init(args=args)
    setup_logging()
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
