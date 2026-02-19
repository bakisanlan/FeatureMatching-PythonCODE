#!/usr/bin/env python3
import os
import csv
import time
import threading
from pathlib import Path

from MAVHandler import MAVHandler    # your class from above
from utilsFolder.redis_helper import RedisHelper
# 1. CONFIGURATION
OUTPUT_ROOT = "MH_01_easy"    # sequence name
DURATION_SEC = 60.0           # how long to record
GPS_RATE_HZ = 10.0
CAM_RATE_HZ = 60.0
# IMU comes in at whatever rate your vehicle pushes RAW_IMU → we’ll log every msg.

# 2. MAKE DIRECTORY STRUCTURE
def make_dirs(root):
    (Path(root) / "mav0"/"imu0").mkdir(parents=True, exist_ok=True)
    (Path(root) / "mav0"/"cam0"/"data").mkdir(parents=True, exist_ok=True)
    (Path(root) / "mav1"/"mocap0").mkdir(parents=True, exist_ok=True)

# 3. THREAD FUNCTIONS
def imu_logger(vehicle, name, msg):
    """
    Callback for every RAW_IMU message.
    Writes one line to imu0/data.csv in:
      timestamp_ns, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
    """
    ts = msg.time_usec * 1_000  # msg.time_usec is μs; convert to ns
    writer = imu_logger.writer
    
    writer.writerow([
        ts,
        msg.xgyro, msg.ygyro, msg.zgyro,
        msg.xacc,  msg.yacc,  msg.zacc
    ])

def gps_thread_fn(handler, csv_writer, stop_event):
    """
    Poll vehicle.location at GPS_RATE_HZ and write to mav1/mocap0/data.csv as:
      timestamp_ns, lat, lon, alt
    """
    interval = 1.0 / GPS_RATE_HZ
    while not stop_event.is_set():
        now_ns = time.time_ns()
        pN, pE, pD, vN, vE, vD, q0, q1, q2, q3 = handler.getstates()
        csv_writer.writerow([now_ns, pN, pE, pD, q0, q1, q2, q3, vN, vE, vD])
        time.sleep(interval)

def cam_thread_fn(hRedisHelper, csv_writer, stop_event):
    """
    At ~CAM_RATE_HZ, grab a frame via getFrame(), save it and log:
      timestamp_ns, filename
    """
    interval = 1.0 / CAM_RATE_HZ
    idx = 0
    data_dir = Path(OUTPUT_ROOT) / "mav0" / "cam0" / "data"
    while not stop_event.is_set():
        now_ns = time.time_ns()
        frame = hRedisHelper.from_redis_2('frame_4')        
        fname = f"{idx:06d}.png"
        (data_dir / fname).write_bytes(frame)  # if `frame` is PNG bytes; otherwise use cv2.imwrite
        csv_writer.writerow([now_ns, fname])
        idx += 1
        time.sleep(interval)

# 4. MAIN
def main():
    make_dirs(OUTPUT_ROOT)

    # --- open files ---
    imu_f   = open(Path(OUTPUT_ROOT)/"mav0"/"imu0"/"data.csv",   "w", newline='')
    cam_f   = open(Path(OUTPUT_ROOT)/"mav0"/"cam0"/"data.csv",   "w", newline='')
    gps_f   = open(Path(OUTPUT_ROOT)/"mav1"/"mocap0"/"data.csv", "w", newline='')

    imu_writer = csv.writer(imu_f)
    cam_writer = csv.writer(cam_f)
    gps_writer = csv.writer(gps_f)

    # optional: write EuRoC style comments at top
    imu_f.write("#timestamp [ns], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1], a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]\n")
    cam_f.write("# timestamp [ns], filename\n")
    gps_f.write("#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1]\n")

    # --- start vehicle ---
    handler = MAVHandler("/dev/ttyACM0", baud_rate=57600)
    
    # --- start Redis ---
    hRedisHelper = RedisHelper()
    

    # 4a) hook IMU callback
    imu_logger.writer = imu_writer
    handler.vehicle.add_message_listener('RAW_IMU', imu_logger)

    # 4b) start GPS logger thread
    stop_event = threading.Event()
    gps_thread = threading.Thread(target=gps_thread_fn,
                                  args=(handler, gps_writer, stop_event),
                                  daemon=True)
    gps_thread.start()

    # 4c) start CAMERA logger thread
    cam_thread = threading.Thread(target=cam_thread_fn,
                                  args=(hRedisHelper, cam_writer, stop_event),
                                  daemon=True)
    cam_thread.start()

    # 5. RUN FOR DURATION, THEN CLEAN UP
    try:
        time.sleep(DURATION_SEC)
    except KeyboardInterrupt:
        print("Interrupted by user")

    print("Stopping logging…")
    stop_event.set()
    gps_thread.join()
    cam_thread.join()

    handler.vehicle.remove_message_listener('RAW_IMU', imu_logger)
    handler.close_connection()

    imu_f.close()
    cam_f.close()
    gps_f.close()
    print("All done. Your EuRoC‐style dataset is in:", OUTPUT_ROOT)

if __name__ == "__main__":
    main()
