#!/usr/bin/env python3
"""
Test script to verify NED frame odometry conversion and publishing
"""
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import threading

# Import the OdomAndMavrosSubscriber
from OV.odom_subscriber import OdomAndMavrosSubscriber
from utils import quat2eul

# Initialize ROS 2
rclpy.init()
node_OdomVIO = OdomAndMavrosSubscriber()
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

print("=" * 80)
print("NED Frame Odometry Test Script")
print("=" * 80)
print("\nWaiting for odometry messages...")
print("-" * 80)

# Wait for first messages
while not (node_OdomVIO.first_vo_msg and node_OdomVIO.first_gt_odom_msg):
    if not node_OdomVIO.first_vo_msg:
        print("Waiting for VIO message...", end='\r')
    if not node_OdomVIO.first_gt_odom_msg:
        print("Waiting for GT odometry message...", end='\r')
    time.sleep(0.5)

print("\n✓ First messages received!")
print("-" * 80)

# Wait for NED conversion to initialize
while not node_OdomVIO.ned_conversion_initialized:
    print("Waiting for NED conversion initialization...", end='\r')
    time.sleep(0.5)

print("\n✓ NED conversion initialized!")
print(f"  Yaw difference: {np.rad2deg(node_OdomVIO.yaw_vioref2enu):.2f} degrees")
print("=" * 80)

# Main loop to print odometry data
try:
    loop_count = 0
    while rclpy.ok():
        time.sleep(1)  # Print every second
        loop_count += 1
        
        print(f"\n{'='*80}")
        print(f"Loop {loop_count} - Time: {time.strftime('%H:%M:%S')}")
        print(f"{'='*80}")
        
        # ========== VIO NED Frame ==========
        if node_OdomVIO.VIOned_dict['ts'] is not None:
            print("\n[VIO - NED Frame]")
            print(f"  Position (N,E,D):    {node_OdomVIO.VIOned_dict['position'][0]:7.2f}, {node_OdomVIO.VIOned_dict['position'][1]:7.2f}, {node_OdomVIO.VIOned_dict['position'][2]:7.2f} m")
            quat_vio_ned = node_OdomVIO.VIOned_dict['orientation']
            euler_vio_ned = quat2eul([quat_vio_ned[0], quat_vio_ned[1], quat_vio_ned[2], quat_vio_ned[3]])
            print(f"  Orientation (r,p,y): {np.rad2deg(euler_vio_ned[2]):7.2f}, {np.rad2deg(euler_vio_ned[1]):7.2f}, {np.rad2deg(euler_vio_ned[0]):7.2f} deg")
            print(f"  Velocity (N,E,D):    {node_OdomVIO.VIOned_dict['velocity'][0]:7.2f}, {node_OdomVIO.VIOned_dict['velocity'][1]:7.2f}, {node_OdomVIO.VIOned_dict['velocity'][2]:7.2f} m/s")
        
        # ========== GT NED Frame ==========
        if node_OdomVIO.GTned_dict['ts'] is not None:
            print("\n[Ground Truth - NED Frame]")
            print(f"  Position (N,E,D):    {node_OdomVIO.GTned_dict['position'][0]:7.2f}, {node_OdomVIO.GTned_dict['position'][1]:7.2f}, {node_OdomVIO.GTned_dict['position'][2]:7.2f} m")
            quat_gt_ned = node_OdomVIO.GTned_dict['orientation']
            euler_gt_ned = quat2eul([quat_gt_ned[0], quat_gt_ned[1], quat_gt_ned[2], quat_gt_ned[3]])
            print(f"  Orientation (r,p,y): {np.rad2deg(euler_gt_ned[2]):7.2f}, {np.rad2deg(euler_gt_ned[1]):7.2f}, {np.rad2deg(euler_gt_ned[0]):7.2f} deg")
            print(f"  Velocity (N,E,D):    {node_OdomVIO.GTned_dict['velocity'][0]:7.2f}, {node_OdomVIO.GTned_dict['velocity'][1]:7.2f}, {node_OdomVIO.GTned_dict['velocity'][2]:7.2f} m/s")
        
        # ========== Position Error ==========
        if node_OdomVIO.VIOned_dict['ts'] is not None and node_OdomVIO.GTned_dict['ts'] is not None:
            pos_error = np.array(node_OdomVIO.VIOned_dict['position']) - np.array(node_OdomVIO.GTned_dict['position'])
            pos_error_2d = np.linalg.norm(pos_error[0:2])
            print(f"\n[Position Error (VIO NED - GT NED)]")
            print(f"  Error (N,E,D):       {pos_error[0]:7.2f}, {pos_error[1]:7.2f}, {pos_error[2]:7.2f} m")
            print(f"  2D Error (NE):       {pos_error_2d:7.2f} m")
        
        print(f"\n{'='*80}")
        print("Press Ctrl+C to exit...")

except KeyboardInterrupt:
    print("\n\n" + "="*80)
    print("Test script terminated by user")
    print("="*80)
    rclpy.shutdown()
