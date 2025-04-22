import sys
import os

cut_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "CUT"))

# Add CUT to system path
sys.path.append(cut_path)

from options.test_options import TestOptions
from models import create_model
import torch
torch.set_grad_enabled(False)
import cv2
from Timer import Timer

opt = TestOptions().parse()  # get test options
opt.name = 'ituUAV_v2_CUT'

opt.num_threads = 0   # test code only supports num_threads = 1
opt.batch_size = 1    # test code only supports batch_size = 1
opt.serial_batches = True  # disable data shuffling; comment this line if results on randomly chosen images are needed.
opt.no_flip = True    # no flip; comment this line if results on flipped images are needed.
opt.resize_or_crop = 'none'
opt.checkpoints_dir = 'CUT/checkpoints'
model = create_model(opt)      # create a model given opt.model and other options
model.setup(opt)               # regular setup: load and print networks; create schedulers
model.parallelize()
# if opt.eval:
model.eval()

# Open the input video
cap = cv2.VideoCapture('data/videos/itu_4_downsampled.MP4')  # Replace with your video path

if not cap.isOpened():
    raise ValueError("Error: Could not open video file.")

# Get original video properties
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Correct codec for MP4

# Define the codec and create a VideoWriter object
out = cv2.VideoWriter('data/videos/itu_sat.mp4', fourcc, fps, (256, 256))
frameCount = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break  # End of video

    if (not frameCount): #only calculate crop dimensions once
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        crop_height = min(h, w)
        half_crop = crop_height // 2

        start_x = max(center_x - half_crop, 0)
        end_x = min(center_x + half_crop, w)
        start_y = max(center_y - half_crop, 0)
        end_y = min(center_y + half_crop, h)

    frame = frame[start_y:end_y, start_x:end_x]
    
    # Resize the frame
    frame = cv2.resize(frame, (256, 256), interpolation=cv2.INTER_AREA)
    
    
    # Convert the image to a PyTorch tensor (normalize to [0, 1] by dividing by 255)
    tensor = torch.tensor(frame, dtype=torch.float32).permute(2, 0, 1).to("cuda") / 255.0  # Normalize to [0, 1]
    tensor = (tensor * 2) - 1  # Normalize to [-1, 1]

    # Add a batch dimension to make it 1x3x512x512
    tensor = tensor.unsqueeze(0)

    fake = model.generateFake(tensor)
    
    frame = (fake.squeeze(0).cpu() + 1) / 2  # Convert from [-1, 1] to [0, 1]
    frame = (frame * 255).byte().permute(1, 2, 0).numpy()  # Convert to [0, 255] and HxWxC    
    
    frameCount += 1

    # Write the resized frame to the output video
    out.write(frame)

# Release resources
cap.release()
out.release()
print(f"Resized video saved as: ")
    