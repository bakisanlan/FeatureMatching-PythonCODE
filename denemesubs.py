from odom_subscriber import OdomAndMavrosSubscriber
import rclpy
import threading
import time

def detect_VIO_divergence(node):
    """
    Detects divergence in VIO data based on the provided VIO dictionary and raw IMU data.
    
    Parameters:
    - VIO_dict: Dictionary containing VIO data including orientation, velocity, and angular velocity.
    - rawIMU: Dictionary containing raw IMU data including acceleration and angular velocity.
    
    Returns:
    - bool: True if divergence is detected, False otherwise.
    """
#   loop each new frame k+1:
#   update VIO with (IMU, image)
#   compute:
#     err_rot, err_trans        ← IMU‐vs‐VIO consistency
#     NIS_visual, NIS_imu       ← innovation tests
#     reproj_error, inlier_pct  ← camera integrity
#     num_tracks                ← feature health
#     grav_err                  ← gravity alignment
#     speed, Δbias              ← velocity & bias sanity
#     is_static                 ← IMU‐variance < ε

#   if any of:
#      err_rot/trans bursts,
#      repeated NIS > χ²,
#      reproj_error too large,
#      inlier_pct too low,
#      num_tracks too few,
#      grav_err too big,
#      speed unrealistic,
#      Δbias too big,
#      (is_static and speed > small_thresh)
#   then
#      trigger “re-initialize” or “reset”  
    VIO_dict = node.VIO_dict


    # Simple velocity magnitude check for divergence detection for 2 seconds
    velocity_mag_max = 15
    time_now = time.time()
    err_sum = 0
    count = 0
    while time_now - time.time() < 2:
        
        velocity_mag = (VIO_dict['velocity'][0]**2 + VIO_dict['velocity'][1]**2 + VIO_dict['velocity'][2]**2)**0.5
        velocity_mag_thresh = 5
        err_sum += abs(velocity_mag - velocity_mag_max)
        
        count += 1
    
    if err_sum / count > velocity_mag_thresh:
        print("VIO divergence detected based on mean velocity magnitude.")
        return True
   

rclpy.init()
node = OdomAndMavrosSubscriber()
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

# Wait for the node to initialize
time_now = time.time()
while not node.home_received or not node.first_vo_msg or not node.first_imu_mag_msg:

    if time.time() - time_now > 3:
        print("Waiting for node initialization...")
        time_now = time.time()

# Now the node is initialized and ready to use
print("All node sensors are initialized and ready to use.")

#Vehicle taking off detection
# NOTE: When UAV begin to take off, following things happen:
# 1. Collect vehicle heading (yaw) at take-off
# 2. Start VIO algoritm(IMU data connection, camera, OPENVINS etc.)
# 2.optional: Save rosbag for debugging purposes
# ...
# 
while True: 

    if vehicle.mode.name =='STABILIZE' or vehicle.state.throttle > 0.3:
        print("Vehicle is in STABILIZE mode or throttle is above 0.3, which means UAV is taking of, get init yaw.")
        take_off_yaw = node.magYawDeg  # This is the yaw angle at take-off, used for initial orientation correction
        break
    else:
        print("Vehicle is not in STABILIZE mode or throttle is below 0.3, waiting for take-off...")
        time.sleep(0.5)

# Main loop of VIO navigation
## 1. Monitor VIO data and detect divergence
#  1.0 when divergence is detected
#    1.1 re-initialize VIO algorithm
#    1.2 hower drone, altitude hold get yaw measurement from mag sensor
#    1.3 try to re-initialize VIO with giving aggressive pitch and roll angles 
## 2. If VIO is not diverged, use VIO data for navigation
#    2.0 get VIO angular velocity, linear velocity and roll/pitch angles
#    2.1 For NED position prediction use rotated linear velocity
#    2.2 For orientation prediction use roll and pitch angles directly but integrate yaw rate to find yaw

while True:

    # Check for VIO divergence
    if detect_VIO_divergence(node):
        print("VIO divergence detected, re-initializing VIO algorithm...")
        
        while True:
            vehicle.mode = VehicleMode("ALT_HOLD")  # Hover the drone
            time.sleep(3)  # Allow some time for the drone to stabilize
            initialize_VIO(pitch_rate)

            if VIO_initialized:
                print("VIO re-initialized successfully.")
                take_off_yaw = node.magYawDeg  # Update the take-off yaw after re-initialization-
                break


    else:
        print("VIO is stable, using VIO data for navigation.")
        
        # Use VIO data for navigation
        
        # VIO_pos = node.VIO_dict['position']
        VIO_ori     = node.VIO_dict['orientation']  #Will be used for roll and pitch measurement, [x,y,z,w]

        VIO_vel     = node.VIO_dict['velocity']              #Will be used for position estimation
        VIO_ang_vel = node.VIO_dict['angular_velocity']  #Will be used for yaw measurement
    
    
    
    

    

    
