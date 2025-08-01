#!/usr/bin/env python3
import math
import numpy as np
from collections import deque


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField,NavSatFix, Imu, PointCloud2
from sensor_msgs_py import point_cloud2

from std_msgs.msg import Bool
from mavros_msgs.msg import HomePosition,State
from rclpy.qos import qos_profile_sensor_data
import os
import sys

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import calculate_heading_mag, quat2rotm

class OdomAndMavrosSubscriber(Node):
    def __init__(self):
        super().__init__('odom_and_mavros_subscriber')

        # Smoothing parameters
        # self.alpha = 0.8
        # self.spike_thresh = 2  # threshold for spike rejection in meters
        self.smoothing = True


        # --- OpenVINS odometry ---        
        self.first_vo_msg = False
        self.VIO_dict = None
        self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self.VIO_odom_callback,
            qos_profile_sensor_data)

        # Create subscribers for both status topics
        # Initialize status variables
        self.initialization_status = False
        self.create_subscription(
            Bool,
            '/ov_msckf/initialization_status',
            self.initialization_status_callback,
            qos_profile_sensor_data
        )

        self.ready_status = False
        self.create_subscription(
            Bool,
            '/ov_msckf/ready_status',
            self.ready_status_callback,
            qos_profile_sensor_data
        )

        # --- OpenVINS Slam Features ---        
        self.SLAM_PC_num = 0
        self.create_subscription(
            PointCloud2,
            '/ov_msckf/points_slam',
            self.VIO_SLAM_PC_callback,
            qos_profile_sensor_data)

        # Subscribe to the IMU data
        self.IMU_RAW = None
        self.first_imu_msg = False
        self.create_subscription(
            Imu,
            '/mavros/imu/data_raw',
            self.imu_callback,
            qos_profile_sensor_data
        )

        # # subscribe to the magnetometer with the sensor_data QoS (best_effort, low latency)
        self.first_imu_mag_msg = True
        # self.magYawDeg = None
        # self.create_subscription(
        #     MagneticField,
        #     '/mavros/imu/mag',
        #     self.mag_callback,
        #     qos_profile_sensor_data
        # )

        # --- Home‐position (geodetic) ---
        self.home_received = False
        self.home_loc = None    
        self.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self.home_position_callback,
            qos_profile_sensor_data)
        
        # ---subscribe to the /mavros/global_position/local topic 
        self.first_gt_odom_msg = False
        self.gt_odom_dict = None
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
        self.state_dict = None
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sensor_data  # or use 10 for default reliability
        )


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
            dt = ts - self.VIO_dict['ts']
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

        # … further processing of ts, px/py/pz, qx/…/wz …


        self.VIO_dict = {
            'ts': ts,
            'position':            (px, py, pz),         #arbitrary yaw global frame position
            'orientation':         (qx, qy, qz, qw),     #arbitrary yaw global frame to body frame
            'velocity':            (vx, vy, vz),         #body frame linear velocity
            'angular_velocity':    (wx, wy, wz),         #body frame angular velocity
            'body_linear_acceleration':   (ax, ay, az)          # placeholder for body acceleration
        }

    def initialization_status_callback(self, msg):
        """Callback for initialization status messages"""
        old_status = self.initialization_status
        self.initialization_status = msg.data
        
        if old_status != self.initialization_status:
            if self.initialization_status:
                self.get_logger().info("🟢 OpenVINS INITIALIZED successfully!")
            else:
                self.get_logger().info("🟡 OpenVINS is trying to initialize...")
                
    def ready_status_callback(self, msg):
        """Callback for ready status messages"""
        old_status = self.ready_status
        self.ready_status = msg.data
        
        if old_status != self.ready_status:
            if self.ready_status:
                self.get_logger().info("🔵 OpenVINS READY - receiving IMU and camera data")
            else:
                self.get_logger().info("🟠 OpenVINS waiting for sensor data...")

    def VIO_SLAM_PC_callback(self, msg):
        """Callback for OpenVINS SLAM PointCloud2 messages"""

        # Process the PointCloud2 message
        # For now, we just store the message in a variable
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"))

        # Convert to list if needed
        self.SLAM_PC_num = len(list(points))
        

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x  # substract local gravity
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        self.IMU_RAW = {
            'body_linear_acceleration': (ax, ay, az),
            'angular_velocity':         (wx, wy, wz)
        }

        if not self.first_imu_msg:
            self.get_logger().info('MAVROS IMU subscriber is initialized')
            self.first_imu_msg = True
        # self.get_logger().info(f'Linear Acceleration: x={ax:.3f}, y={ay:.3f}, z={az:.3f}')


    def mag_callback(self, msg: MagneticField):
        # get raw magnetometer readings (in Tesla)
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        # compute heading: atan2(Y, X) in degrees [–180, +180]
        yaw_rad = math.atan2(my, mx)
        yaw_deg = math.degrees(yaw_rad)

        calculate_heading_mag((mx, my, mz), self.VIO_dict['orientation'])

        self.magYawDeg = yaw_deg
        self.mag       = (mx, my, mz)

        if not self.first_imu_mag_msg:
            self.get_logger().info('MAVROS magnetometer subscriber is initialized')
            self.first_imu_mag_msg = True

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

        self.gt_odom_dict = {
            'ts': ts,
            'position':         (px, py, pz),       # Position on ENU frame
            'orientation':      (qx, qy, qz, qw),   # rotation inertia frame to body frame
            'velocity':         (vx, vy, -vz),      # linear velocity on ENU frame, NOTE: z is inverted interestingly from MAVROS, search it
            'angular_velocity': (wx, wy, wz),
        }

        if not self.first_gt_odom_msg:
            self.get_logger().info('MAVROS global_position/local subscriber is initialized')
            self.first_gt_odom_msg = True

    def gps_fix_callback(self, msg: NavSatFix):
        # get the GPS fix location
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        self.gps_fix_loc = [lat, lon, alt]

        if not self.first_gps_fix_msg:
            self.get_logger().info('MAVROS global_position/global subscriber is initialized')
            self.first_gps_fix_msg = True

    def state_callback(self, msg: State):
        # build a dict of the MAVROS state
        self.state_dict = {
            'connected':     msg.connected,
            'armed':         msg.armed,
            'guided':        msg.guided,
            'mode':          msg.mode,
            'system_status': msg.system_status
        }

        if not self.first_state_msg:
            self.get_logger().info('MAVROS /state subscriber initialized')
            self.first_state_msg = True

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomAndMavrosSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
