#!/usr/bin/env python3
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data
import time

from std_msgs.msg import Float64
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion, Vector3

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utils import eul2quat, rotm2quat, quatmultiply


class PixhawkCommander(Node):
    def __init__(self):
        super().__init__('pixhawk_commander')

        # --- Service clients ---
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_cli    = self.create_client(SetMode,    '/mavros/set_mode')
        self.arm_cli     = self.create_client(CommandBool, '/mavros/cmd/arming')

        # --- Attitude publisher ---
        self.att_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

        # --- Subscribe to relative altitude ---
        self.current_alt = None
        self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self._altitude_callback,
            qos_profile_sensor_data
    )

    def _altitude_callback(self, msg: Float64):
        self.current_alt = msg.data


    def wait_for(self, client, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'{client.srv_name} not available')
            return False
        return True

    def call(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def arm(self, do_arm: bool = True):
        req = CommandBool.Request()
        req.value = do_arm
        if self.wait_for(self.arm_cli):
            res = self.call(self.arm_cli, req)
            self.get_logger().info(f'Arming: {res.success}')

    def set_mode(self, mode: str = 'OFFBOARD'):
        req = SetMode.Request()
        req.custom_mode = mode
        if self.wait_for(self.mode_cli):
            res = self.call(self.mode_cli, req)
            self.get_logger().info(f'Set mode to {mode}: {res.mode_sent}')

    def takeoff(self,
                altitude: float,
                latitude: float  = 0.0,
                longitude: float = 0.0,
                yaw: float       = 0.0,
                min_pitch: float = 0.0):
        req = CommandTOL.Request()
        req.altitude  = float(altitude)
        req.latitude  = float(latitude)
        req.longitude = float(longitude)
        req.yaw       = float(yaw)
        req.min_pitch = float(min_pitch)
        if self.wait_for(self.takeoff_cli):
            res = self.call(self.takeoff_cli, req)
            self.get_logger().info(f'Takeoff result: success={res.success}, result={res.result}')

    def takeoff_until_altitude(self, target_alt: float, threshold: float = 1, retry_interval: float = 1.0):
        """
        Repeatedly call takeoff() until current_alt >= target_alt - threshold.
        """
        while rclpy.ok():
            if self.current_alt is not None:
                err = target_alt - self.current_alt
                self.get_logger().info(f'Current alt: {self.current_alt:.2f} m; remaining {err:.2f} m')
                if err <= threshold:
                    self.get_logger().info('🎯 Target altitude reached.')
                    break
            else:
                self.get_logger().info('Waiting for altitude data...')

            # call takeoff again in case it wasn’t accepted or we drifted
            self.takeoff(altitude=target_alt)
            # allow callbacks to update current_alt
            rclpy.spin_once(self, timeout_sec=retry_interval)
            time.sleep(retry_interval)

    def land(self, latitude=0.0, longitude=0.0, yaw=0.0):
        req = CommandTOL.Request()
        req.altitude  = 0.0
        req.latitude  = float(latitude)
        req.longitude = float(longitude)
        req.yaw       = float(yaw)
        if self.wait_for(self.takeoff_cli):
            res = self.call(self.takeoff_cli, req)
            self.get_logger().info(f'Landing: success={res.success}, result={res.result}')

    def set_attitude(self,
                     eul: np.ndarray = np.array([0.0, 0.0, 0.0]),  # Euler in [yaw,pitch,rol] in rad
                     yaw_rate: float = 0.0,
                     thrust: float = 0.5):
        
        eul = np.array([np.pi/2 - eul[0], -eul[1], eul[2]])
        q = eul2quat(eul, order='ZYX')  # Example Euler angles in degrees
        q = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])  # Convert to ROS Quaternion format [x, y, z, w]

        msg = AttitudeTarget()
        msg.header.stamp = Clock().now().to_msg()
        msg.orientation = q
        msg.thrust      = float(thrust)
    
        msg.body_rate = Vector3(x=0.0, y=0.0, z=float(yaw_rate))
        # mask out p,q fields so only attitude+thrust are applied
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE  |
            AttitudeTarget.IGNORE_PITCH_RATE 
                            )
        
        self.att_pub.publish(msg)
        self.get_logger().debug(f'Published attitude setpoint: q={q}, thrust={thrust}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = PixhawkCommander()

#     # Example usage:
#     # node.set_mode('STABILIZE')
#     # node.arm(True)
#     # time.sleep(2)  # Wait for mode change to take effect

#     # time.sleep(2)  # Wait for mode change to take effect
#     # node.set_mode('GUIDED')

#     # node.takeoff_until_altitude(target_alt=10.0)
#     # # send a level attitude at half-thrust
#     # # q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     # R = np.array([[0 , 1, 0], [1, 0, 0], [0, 0, -1]])

#     eul = np.deg2rad([-30, 0, 0])
#     eul = np.array([np.pi/2 - eul[0], -eul[1], eul[2]])

#     q = eul2quat(eul, order='ZYX')  # Example Euler angles in degrees

#     for _ in range(1000):
#         node.set_attitude(q, thrust=0.5)
#         print(q)
#         time.sleep(0.05)
#     node.set_mode('GUIDED')

#     node.set_attitude(q, thrust=0.5)

#     # node.land(latitude=-35.35977147, longitude=149.16315051 , yaw=30.0)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
