#!/usr/bin/env python3
import cv2
import time
from datetime import datetime
from redis_helper import RedisHelper
import os
import json

from MAVHandler import MAVHandler

def main():
    redis = RedisHelper()
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

    imu_raw_file = open(f"imu_raw/imu_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    imu_raw_file.write("timestamp,xacc,yacc,zacc,xgyro,ygyro,zgyro,timestamp_px\n")
    imu_scaled_file = open(f"imu_scaled/imu_scaled_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    imu_scaled_file.write("timestamp,xacc,yacc,zacc,xgyro,ygyro,zgyro,timestamp_px\n")
    imu_highres_file = open(f"imu_highres/imu_highres_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    imu_highres_file.write("timestamp,xacc,yacc,zacc,xgyro,ygyro,zgyro,timestamp_px\n")
    gps_file = open(f"gps0/gps_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
    gps_file.write("timestamp,lat,lon,alt\n")


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
                cv2.imwrite(f"cam0/{timestamp}.png", frame0)
            else:
                print("Error: Failed to capture frame from camera 0.")

            # Write IMU data to CSV files
            imu_raw_data = drone.imu_raw_data
            imu_raw_file.write(f"{timestamp},{imu_raw_data['gyro_x']},{imu_raw_data['gyro_y']},{imu_raw_data['gyro_z']},"
                               f"{imu_raw_data['acc_x']},{imu_raw_data['acc_y']},{imu_raw_data['acc_z']}\n")
            imu_scaled_data = drone.imu_scaled_data 
            imu_scaled_file.write(f"{timestamp},{imu_scaled_data['gyro_x']},{imu_scaled_data['gyro_y']},{imu_scaled_data['gyro_z']},"
                                  f"{imu_scaled_data['acc_x']},{imu_scaled_data['acc_y']},{imu_scaled_data['acc_z']}\n")
            imu_highres_data = drone.imu_highres_data
            imu_highres_file.write(f"{timestamp},{imu_highres_data['gyro_x']},{imu_highres_data['gyro_y']},{imu_highres_data['gyro_z']},"
                                    f"{imu_highres_data['acc_x']},{imu_highres_data['acc_y']},{imu_highres_data['acc_z']}\n")
            # Write GPS data to CSV file
            gps_data = drone.get_locationLLA()
            gps_file.write(f"{timestamp},{gps_data[0]},{gps_data[1]},{gps_data[2]}\n")


            if not ret0:
                print("Error: Failed to capture frame from one of the cameras.")
                break


            fps = 1 / (time.time() - timestamp)
            print(f"FPS: {fps:.2f}")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        cap0.release()
        # cap1.release()

if __name__ == "__main__":
    main()