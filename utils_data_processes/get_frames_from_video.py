import numpy as np
import cv2
import os
import sys

# add parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Load the .npy file (assuming it contains a list or array of images)
file_path = 'itu_video_20250505.npy'  # change this to your file name
output_folder = '20250505'
num_frames_to_save = 30

# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Load frames
frames = np.load(file_path, allow_pickle=True)

total_frames = len(frames)
if total_frames < num_frames_to_save:
    raise ValueError(f"The array contains only {total_frames} frames, fewer than requested {num_frames_to_save}.")

# Calculate equally spaced indices
indices = np.linspace(0, total_frames - 1, num_frames_to_save, dtype=int)

for i, idx in enumerate(indices):
    frame = frames[idx]

    # Ensure the frame is in uint8 format
    if frame.dtype != np.uint8:
        frame = frame.astype(np.uint8)

    # Convert to BGR if frame is grayscale or RGB (optional, depends on your data)
    if frame.ndim == 2:  # grayscale
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif frame.shape[2] == 3:  # RGB
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        frame_bgr = frame  # assume already BGR

    # Save image
    output_path = os.path.join(output_folder, f'frame_{i:02d}.png')
    cv2.imwrite(output_path, frame_bgr)

    print(f"Saved {output_path}")

print("Finished saving frames.")
