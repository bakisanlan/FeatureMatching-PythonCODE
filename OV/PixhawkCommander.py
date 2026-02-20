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
from utils import eul2quat
import logging


logger = logging.getLogger(__name__)

class PixhawkCommander(Node):
    def __init__(self):
        super().__init__('pixhawk_commander')
        
        self.last_ctrl_time     = time.time()
        self.ctr_print_interval = 1.0

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
            logger.error('%s not available', client.srv_name)
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
            logger.info('Arming: %s', res.success)

    def set_mode(self, mode: str = 'OFFBOARD'):
        req = SetMode.Request()
        req.custom_mode = mode
        if self.wait_for(self.mode_cli):
            res = self.call(self.mode_cli, req)
            logger.info('Set mode to %s: %s', mode, res.mode_sent)

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
            logger.info('Takeoff result: success=%s, result=%s', res.success, res.result)

    def takeoff_until_altitude(self, target_alt: float, threshold: float = 1, retry_interval: float = 1.0):
        """
        Repeatedly call takeoff() until current_alt >= target_alt - threshold.
        """
        while rclpy.ok():
            if self.current_alt is not None:
                err = target_alt - self.current_alt
                logger.info('Current alt: %.2f m; remaining %.2f m', self.current_alt, err)
                if err <= threshold:
                    logger.info('🎯 Target altitude reached.')
                    break
            else:
                logger.info('Waiting for altitude data...')

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
            logger.info('Landing: success=%s, result=%s', res.success, res.result)

    def set_attitude(self,
                     eul: np.ndarray = np.array([0.0, 0.0, 0.0]),  # Euler in [yaw,pitch,rol] in rad
                     yaw_rate: float = 0.0,
                     thrust: float = 0.5):
        
        # convert from frd to flu 
        eul = np.array([np.pi/2 - eul[0], -eul[1], eul[2]])
        q = eul2quat(eul, order='ZYX')  # Example Euler angles in rad
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
        if time.time() - self.last_ctrl_time > self.ctr_print_interval:
            self.last_ctrl_time = time.time()
            logger.debug(
                'Published attitude setpoint: eul_deg=[%.3f %.3f %.3f] thrust=%.3f',
                *np.rad2deg(eul),
                float(thrust),
            )

def main(args=None):    
    rclpy.init(args=args)
    node = PixhawkCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
