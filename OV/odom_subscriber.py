#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField,NavSatFix
from mavros_msgs.msg import HomePosition,State
from rclpy.qos import qos_profile_sensor_data

class OdomAndMavrosSubscriber(Node):
    def __init__(self):
        super().__init__('odom_and_mavros_subscriber')

        # --- OpenVINS odometry ---        
        self.first_vo_msg = False
        self.VIO_dict = None
        self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self.VIO_odom_callback,
            qos_profile_sensor_data)


        # subscribe to the magnetometer with the sensor_data QoS (best_effort, low latency)
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
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        wx, wy, wz = (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        )

        self.VIO_dict = {
            'ts': ts,
            'position':         (px, py, pz),         #arbitrary yaw global frame position
            'orientation':      (qx, qy, qz, qw),     #arbitrary yaw global frame to body frame
            'velocity':         (vx, vy, vz),         #body frame linear velocity
            'angular_velocity': (wx, wy, wz),         #body frame angular velocity
        }

        if (not self.first_vo_msg):
            self.get_logger().info('VO odom subscriber is initialized')
            self.first_vo_msg = True

        # … further processing of ts, px/py/pz, qx/…/wz …

    def mag_callback(self, msg: MagneticField):
        # get raw magnetometer readings (in Tesla)
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y

        # compute heading: atan2(Y, X) in degrees [–180, +180]
        yaw_rad = math.atan2(my, mx)
        yaw_deg = math.degrees(yaw_rad)

        self.magYawDeg = yaw_deg

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
            'velocity':         (vx, vy, vz),       # linear velocity on ENU frame
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
