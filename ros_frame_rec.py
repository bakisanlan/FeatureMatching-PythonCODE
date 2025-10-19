import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from OV.odom_subscriber import OdomAndMavrosSubscriber
from utils import rotate_image, resize_image, square_crop_from_center
import time
import cv2
from pathlib import Path
from datetime import datetime

# Initialize ROS 2 nodes
rclpy.init()
node_OdomVIO     = OdomAndMavrosSubscriber()
executor = MultiThreadedExecutor()
executor.add_node(node_OdomVIO)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

snapDim=(540, 540)

last_snap = time.time()
snap_freq = 1   # seconds

# Save images to a folder  
folder_dir = Path("./captured_frames/540/")
folder_dir.mkdir(exist_ok=True)

# Frame counter for sequential naming
frame_counter = 0

while True:
    if node_OdomVIO.first_vo_msg:
        if time.time() - last_snap >= snap_freq:
            
            last_snap = time.time()
            frame = node_OdomVIO.camera_image
            
            if frame is not None:
                frame = square_crop_from_center(frame)
                frame = rotate_image(frame, np.pi)  # Rotate image if needed
                frame = resize_image(frame, snapDim=snapDim)
                
                # Generate filename with timestamp and frame counter
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = folder_dir / f"frame_{frame_counter:04d}_{timestamp}.jpg"
                
                # Save the frame as JPG
                cv2.imwrite(str(filename), frame)
                print(f"Saved: {filename}")
                
                frame_counter += 1
            else:
                print("Warning: No camera image available")



