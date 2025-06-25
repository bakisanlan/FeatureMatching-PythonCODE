#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from mavros_msgs.msg import HomePosition
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
            self.odom_callback,
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

    def odom_callback(self, msg: Odometry):
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
            'position':         (px, py, pz),
            'orientation':      (qx, qy, qz, qw),
            'velocity':         (vx, vy, vz),
            'angular_velocity': (wx, wy, wz),
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
            lat = msg.geo.pose.position.latitude
            lon = msg.geo.pose.position.longitude
            alt = msg.geo.pose.position.altitude

            self.home_loc = [lat, lon, alt]

            self.get_logger().info('MAVROS home_position subscriber is initialized')

            self.get_logger().info(
                f'Initial home location → '
                f'lat: {lat:.7f}, lon: {lon:.7f}, alt: {alt:.2f} m'
            )
            self.home_received = True

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
