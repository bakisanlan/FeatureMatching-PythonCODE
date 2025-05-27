#!/usr/bin/env python3
import cv2
import time
from datetime import datetime
import os
import json

from MAVHandler import MAVHandler
from Timer import Timer
from pathlib import Path

def main():
    drone = MAVHandler("/dev/ttyACM0", baud_rate=115200)


    if not os.path.exists("imu_raw"):
        os.makedirs("imu_raw")
    if not os.path.exists("imu_scaled"):
        os.makedirs("imu_scaled")
    if not os.path.exists("imu_highres"):
        os.makedirs("imu_highres")
    if not os.path.exists("cam0"):
        os.makedirs("cam0")
    if not os.path.exists("gps0"):
        os.makedirs("gps0")

    output_dir = f"cam0/output_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    imu_raw_file = open(f"imu_raw/imu_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    imu_raw_file.write("timestamp,xgyro,ygxro,zgyro,xacc,yacc,zacc,timestamp_px\n")
    imu_scaled_file = open(f"imu_scaled/imu_scaled_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    imu_scaled_file.write("timestamp,xgyro,ygyro,zgyro,xacc,yacc,zacc,timestamp_px\n")
    imu_highres_file = open(f"imu_highres/imu_highres_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    imu_highres_file.write("timestamp,xgyro,ygyro,zgyro,xacc,yacc,zacc,timestamp_px\n")
    gt_file = open(f"gps0/gps_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    gt_file.write("timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],gps_lat, gps_lon, gps_alt\n")

    data_dir = Path("cam0")

    home = drone.get_locationLLA()

    config_txt = open("config.txt", "w")
    config_txt.write(f"home: {home}\n")

    MAIN_RESOLUTION = (1920, 1080)

    # GStreamer pipeline for camera 0
    gst_pipeline_0 = (
        "nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM),width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]},"
        "framerate=59/1,format=NV12 ! " ################################################################
        "queue max-size-buffers=1 leaky=downstream ! "
        "nvvidconv flip-method=2 ! "
        f"video/x-raw,width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]} ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "nvvidconv ! videoconvert ! video/x-raw,format=BGR ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "appsink drop=true sync=false"
    )

    # # GStreamer pipeline for camera 1
    # gst_pipeline_1 = (
    #     "nvarguscamerasrc sensor-id=1 ! "
    #     f"video/x-raw(memory:NVMM),width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]},"
    #     "framerate=21/1,format=NV12 ! "
    #     "queue max-size-buffers=1 leaky=downstream ! "
    #     "nvvidconv flip-method=2 ! "
    #     f"video/x-raw,width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]} ! "
    #     "queue max-size-buffers=1 leaky=downstream ! "
    #     "nvvidconv ! videoconvert ! video/x-raw,format=BGR ! "
    #     "queue max-size-buffers=1 leaky=downstream ! "
    #     "appsink drop=true sync=false"
    # )

    # Open both cameras
    cap0 = cv2.VideoCapture(gst_pipeline_0, cv2.CAP_GSTREAMER)
    # cap1 = cv2.VideoCapture(gst_pipeline_1, cv2.CAP_GSTREAMER)

    if not cap0.isOpened():
        print("Error: Could not open camera 0 with the given pipeline.")
        return
    # if not cap1.isOpened():
    #     print("Error: Could not open camera 1 with the given pipeline.")
    #     return

    try:
        while True:
            timestamp = time.time()
            ret0, frame0 = cap0.read()
            # Save camera frame to file
            if ret0:
                cv2.imwrite(os.path.join(output_dir, f"frame_{timestamp}.jpg"), frame0)
                # fname = f"{timestamp}.png"
                # (data_dir / fname).write_bytes(frame0)
            else:
                print("Error: Failed to capture frame from camera 0.")

            # Write IMU data to CSV files
            imu_raw_data = drone.imu_raw_data
            imu_raw_file.write(f"{timestamp},{imu_raw_data['xgyro']},{imu_raw_data['ygyro']},{imu_raw_data['zgyro']},{imu_raw_data['xacc']},{imu_raw_data['yacc']},{imu_raw_data['zacc']},{imu_raw_data['timestamp']}\n")
            imu_scaled_data = drone.imu_scaled_data
            imu_scaled_file.write(f"{timestamp},{imu_scaled_data['xgyro']},{imu_scaled_data['ygyro']},{imu_scaled_data['zgyro']},{imu_scaled_data['xacc']},{imu_scaled_data['yacc']},{imu_scaled_data['zacc']},{imu_scaled_data['timestamp']}\n")
            imu_highres_data = drone.imu_highres_data
            imu_highres_file.write(f"{timestamp},{imu_highres_data['xgyro']},{imu_highres_data['ygyro']},{imu_highres_data['zgyro']},{imu_highres_data['xacc']},{imu_highres_data['yacc']},{imu_highres_data['zacc']},{imu_highres_data['timestamp']}\n")
            # Write GPS data to CSV file
            ground_truth = drone.get_states(LLA0=home)
            gps_data = drone.get_locationLLA()
            gt_file.write(f"{timestamp},{ground_truth[0]},{ground_truth[1]},{ground_truth[2]},{ground_truth[6]},{ground_truth[7]},{ground_truth[8]},{ground_truth[9]},{ground_truth[3]},{ground_truth[4]},{ground_truth[5]},{gps_data[0]},{gps_data[1]},{gps_data[2]}\n")

            print(f"IMU Raw Data: {imu_raw_data}")

            if not ret0:
                print("Error: Failed to capture frame from one of the cameras.")
                break
            

            fps = 1 / (time.time() - timestamp)
            #print(f"FPS: {fps:.2f}")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        cap0.release()
        # cap1.release()

if __name__ == "__main__":
    main()