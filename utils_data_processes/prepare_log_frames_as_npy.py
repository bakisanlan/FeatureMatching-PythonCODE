import sys
import os
import cv2
import numpy as np
from utils import square_crop_from_center, resize_image

# add parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

def process_images(folder_path, crop_size=256, step=6, output_file='itu_video_25052025.npy'):
    # Get sorted image filenames based on timestamp in name
    image_files = sorted(
        [f for f in os.listdir(folder_path) if f.endswith('.jpg')],
        key=lambda x: float(x.split('_')[1][:-4])
    )

    selected_images = image_files[::step]  # take every 10th image
    cropped_images = []

    for i, filename in enumerate(selected_images):
        path = os.path.join(folder_path, filename)
        img = cv2.imread(path)
    
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        

        if img is None:
            print(f"Warning: Could not read {filename}, skipping.")
            continue

        if img.shape[0] < crop_size or img.shape[1] < crop_size:
            print(f"Warning: Image {filename} too small to crop, skipping.")
            continue

        cropped = square_crop_from_center(img)
        cropped = resize_image(cropped, (crop_size,crop_size))
        cropped_images.append(cropped)

    cropped_images = np.array(cropped_images)
    np.save(output_file, cropped_images)
    print(f"Saved {len(cropped_images)} cropped images to {output_file}")

# Example usage:
process_images('data/camera-001/output_20250505_112658', crop_size=256, step=3, output_file='itu_video_20250505.npy')
