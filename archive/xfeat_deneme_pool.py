import numpy as np
import imageio as imio
# import os
import torch
from time import time
from concurrent.futures import ThreadPoolExecutor

# import tqdm
# import matplotlib.pyplot as plt
from utils import *
from utils import resize_image


def _worker_initializer():
    """Initialize CUDA context in worker threads"""
    if torch.cuda.is_available():
        # Set the device for this worker thread
        torch.cuda.set_device(0)
        # Create a dummy tensor to initialize CUDA context
        dummy = torch.zeros(1, device='cuda:0')
        del dummy
        torch.cuda.synchronize()

def _measurement_update_worker():
    """Heavy measurement update in separate thread"""
    #Load some example images
    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0038_20251018_211205.jpg"
    image0 = "/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0054_20251018_211221.jpg"
    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0025_20251018_211152.jpg"
    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0038_20251018_211205.jpg"
    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0041_20251018_211208.jpg"
    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0052_20251018_211219.jpg
    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/256/frame_0058_20251018_205641.jpg"

    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/256/frame_0054_20251018_205637.jpg"
    # image1 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/540/frame_0042_20251018_211209.jpg"

    # image0 = "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/satellite/540/bacikoy_sat (1).jpg"


    # image0 =  "/home/ituarc/Documents/Github/FeatureMatching-PythonCODE/captured_frames_bacikoy/satellite/1400/bacikoy_sat (1).jpg"

    image1 = "/home/ituarc/Documents/GitHub/FeatureMatching-PythonCODE/captured_frames_bacikoy/satellite/1400/bacikoy_sat (1).jpg"
    d = 300
    im1_src = np.copy(imio.v2.imread(image0))
    im1_src = resize_image(im1_src, (d,d))

    im2_src = imio.v2.imread(image1)
    im2_src = resize_image(im2_src, (d,d))

    # im2_src = resize_image(im2_src, [540,540]) 

    if im2_src.ndim == 2:  # grayscale -> add channel dimension
        im2 = np.copy(im2_src[..., np.newaxis])
    else:  # color -> keep same handling as im1 (reverse channels)
        im2 = np.copy(im2_src[..., ::-1])
        
    if im1_src.ndim == 2:  # grayscale -> add channel dimension
        im1 = np.copy(im1_src[..., np.newaxis])
    else:  # color -> keep same handling as im2 (reverse channels)
        im1 = np.copy(im1_src[..., ::-1])


    output0 = xfeat.detectAndCompute(im1)[0]
    output1 = xfeat.detectAndCompute(im2)[0]


    # # im1 = im1[None, ...]
    # # im2 = im2[None, ...]
    # # output0 = xfeat.detectAndComputeDense(torch.tensor(im1).permute(0,3,1,2)/255, top_k = 1300)
    # # output1 = xfeat.detectAndComputeDense(torch.tensor(im2).permute(0,3,1,2)/255, top_k = 1300)


    # # #Update with image resolution (required)
    output0.update({'image_size': (im1.shape[1], im1.shape[0])})
    output1.update({'image_size': (im2.shape[1], im2.shape[0])})


    n = 5
    x2 = torch.randn(n,3,500,500)

    for i in range(50):
        torch.cuda.synchronize()
        start = time()
        # output2 = xfeat.detectAndCompute(x2, top_k = 1300)[0]
        for j in range(50):
            matches = xfeat.match_lighterglue(output0, output1)
            
        end = time()
        torch.cuda.synchronize()

        print(f"Iteration {i}: {end - start:.4f} seconds")

# --- GPU setup ---
torch.set_grad_enabled(False)
torch.backends.cudnn.benchmark = True
torch.set_num_threads(2)
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
print(device)

# Warm up CUDA in main thread
if torch.cuda.is_available():
    dummy = torch.zeros(1, device=device)
    del dummy
    torch.cuda.synchronize()

pool = ThreadPoolExecutor(
    max_workers=1,
    initializer=_worker_initializer
)

xfeat = torch.hub.load('verlab/accelerated_features', 'XFeat', pretrained = True, top_k = 1300)



measurement_future = pool.submit(
    _measurement_update_worker
)
