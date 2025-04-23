#!/usr/bin/env python3
import cv2
import time
from datetime import datetime
from redis_helper import RedisHelper
import os
import json

def main():
    redis = RedisHelper()

    MAIN_RESOLUTION = (960, 540)

    # GStreamer pipeline for camera 0
    gst_pipeline_0 = (
        "nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM),width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]},"
        "framerate=21/1,format=NV12 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "nvvidconv flip-method=2 ! "
        f"video/x-raw,width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]} ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "nvvidconv ! videoconvert ! video/x-raw,format=BGR ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "appsink drop=true sync=false"
    )

    # GStreamer pipeline for camera 1
    gst_pipeline_1 = (
        "nvarguscamerasrc sensor-id=1 ! "
        f"video/x-raw(memory:NVMM),width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]},"
        "framerate=21/1,format=NV12 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "nvvidconv flip-method=2 ! "
        f"video/x-raw,width={MAIN_RESOLUTION[0]},height={MAIN_RESOLUTION[1]} ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "nvvidconv ! videoconvert ! video/x-raw,format=BGR ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "appsink drop=true sync=false"
    )

    # Open both cameras
    cap0 = cv2.VideoCapture(gst_pipeline_0, cv2.CAP_GSTREAMER)
    cap1 = cv2.VideoCapture(gst_pipeline_1, cv2.CAP_GSTREAMER)

    if not cap0.isOpened():
        print("Error: Could not open camera 0 with the given pipeline.")
        return
    if not cap1.isOpened():
        print("Error: Could not open camera 1 with the given pipeline.")
        return

    try:
        while True:
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            # If one of the cameras fails, you can decide what to do:
            if not ret0 or not ret1:
                print("Error: Failed to capture frame from one of the cameras.")
                break

            # Publish each frame separately to Redis
            redis.toRedis('frame_4', frame1)
            redis.toRedis('frame_6', frame0)

            # If you want to process or visualize the frames further:
            # obj_center = json.loads(redis.r.get('obj_center'))
            # drone_mode = redis.r.get('drone_mode').decode('utf-8')
            # frame_center = (MAIN_RESOLUTION[0] // 2, MAIN_RESOLUTION[1] // 2)

            # if drone_mode == "GUIDED":
            #     if len(obj_center) != 0:
            #         cv2.circle(frame0, (int(obj_center[0]), int(obj_center[1])), 3, (0, 0, 255), -1)
            #         cv2.circle(frame0, (int(frame_center[0]), int(frame_center[1])), 5, (0, 0, 255), -1)
            #         cv2.line(frame0, (int(frame_center[0]), int(frame_center[1])),
            #                  (int(obj_center[0]), int(obj_center[1])), (0, 255, 0), 2)

            #     cv2.imwrite(f"{output_folder}/frame0_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg", frame0)
            #     cv2.imwrite(f"{output_folder}/frame1_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg", frame1)

            # If you want some delay:
            # time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        cap0.release()
        cap1.release()

if __name__ == "__main__":
    main()
