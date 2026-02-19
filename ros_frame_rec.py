import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
from OV.odom_subscriber import OdomAndMavrosSubscriber
from utils import quat2eul, quat2rotm, rotate_image, resize_image, square_crop_from_center
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
snap_freq = 3   # seconds

# Save images to a folder
path = "/captured_frames/raw_data_both_new/"
folder_dir = Path(path)
try:
    folder_dir.mkdir(parents=True, exist_ok=True)
except PermissionError:
    # fallback to local directory if creating at root is not permitted
    folder_dir = Path.cwd() / "captured_frames" / "raw_data_both_new"
    folder_dir.mkdir(parents=True, exist_ok=True)

# Frame counter for sequential naming
frame_counter = 0

R_c_imu =    np.array([[-0.0278616592128909, -0.9995298103661786, -0.01280180203159867],
                    [-0.9996117282427698, 0.027854921448106484, 0.0007043512074339127],
                    [-0.0003474268388132173, 0.012816455846735854, -0.9999178054990925]])

while True:
    if node_OdomVIO.first_camera_msg and node_OdomVIO.SLAM_PC_ned is not None:
        if time.time() - last_snap >= snap_freq:
            
            last_snap = time.time()
            frame = node_OdomVIO.camera_image
            landmarks_pts_ned = node_OdomVIO.SLAM_PC_ned
            landmarks_pts_vio = node_OdomVIO.SLAM_PC


            qx, qy, qz, qw = node_OdomVIO.VIOned_dict['orientation']

            try:
                qx_gt, qy_gt, qz_gt, qw_gt = node_OdomVIO.GTned_dict['orientation']
                print(quat2eul([qw_gt, qx_gt, qy_gt, qz_gt])[0] * 180/np.pi)  # yaw

            except:
                pass

            R_ned_imu = quat2rotm([qw, qx, qy, qz])
            qx, qy, qz, qw = node_OdomVIO.VIO_dict['orientation']
            R_vio_imu = quat2rotm([qw, qx, qy, qz])
            R_ned_c = R_ned_imu @ R_c_imu.T
            R_vio_c = R_vio_imu @ R_c_imu.T
            t_ned          = np.array(node_OdomVIO.VIOned_dict['position']).reshape(3, 1)
            t_vio          = np.array(node_OdomVIO.VIO_dict['position']).reshape(3, 1)

            # print(-t_ned[2])

            
            if frame is not None:
                # frame = square_crop_from_center(frame)
                # frame = rotate_image(frame, np.pi)  # Rotate image if needed
                # frame = resize_image(frame, snapDim=snapDim)
                
                # Generate filename with timestamp and frame counter
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = folder_dir / f"frame_{frame_counter:04d}_{timestamp}.jpg"

                # save landmarks_pts, R_wc and t_wc to dict and save
                data_to_save = {
                    'landmarks_pts_ned': landmarks_pts_ned,
                    'landmarks_pts_vio': landmarks_pts_vio,
                    'R_ned_c': R_ned_c,
                    'R_vio_c': R_vio_c,
                    'R_ned_imu': R_ned_imu,
                    'R_vio_imu': R_vio_imu,
                    't_ned': t_ned,
                    't_vio': t_vio,
                    'R_c_imu' : R_c_imu
                }
                np.savez(folder_dir / f"frame_{frame_counter:04d}_{timestamp}_data.npz", **data_to_save)
                
                # Save the frame as JPG
                cv2.imwrite(str(filename), frame)
                print(f"Saved: {filename}")
                
                frame_counter += 1
    else:
        print("Wa:rning: No camera image available")



