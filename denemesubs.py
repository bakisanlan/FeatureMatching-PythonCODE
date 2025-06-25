from odom_subscriber import OdomAndMavrosSubscriber
import rclpy
import threading
import time

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

while True:

    VIO_pos = node.VIO_dict['position']
    VIO_vel = node.VIO_dict['velocity']
    VIO_ori = node.VIO_dict['orientation']

    
