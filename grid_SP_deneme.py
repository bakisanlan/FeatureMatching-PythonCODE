#!/usr/bin/env python3
import cv2
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import torch
import pickle
# LightGlue imports (only used if method != 'orb' and != 'sift')
from LightGlue.lightglue import LightGlue, SuperPoint, SIFT
from LightGlue.lightglue.utils import rbd, numpy_image_to_torch

from utils import draw_custom_matches, drawKeypoints

def keypoints_to_list(kps):
    return [(
        kp.pt, kp.size, kp.angle, kp.response, kp.octave, kp.class_id
    ) for kp in kps]

script_dir = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(script_dir, 'data', 'itu_sat.jpg')
crop_size = 256


detector = 'ORB'  # 'SP', 'ORB'

if detector == 'SP':
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
    print(device)
    SP = SuperPoint(max_num_keypoints=2048,remove_borders = 1 ).eval().to(device)  # load the extractor
    I     = cv2.imread(data_path)

elif detector == 'ORB':
    
    ORB = cv2.ORB_create(nfeatures=20000, edgeThreshold = 3 , patchSize = 3, nlevels = 8)
    I    = cv2.imread(data_path)
    I    =  cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    # Convert the 2D grayscale image to a 3D grayscale image by adding an extra dimension
    I = np.expand_dims(I, axis=-1)

mapDim = I.shape
h, w = mapDim[0:2]
step_x = w // crop_size
step_y = h // crop_size
print(step_x)

rem_x = w % crop_size
rem_y = h % crop_size


print(step_y, step_x)

for i in range(step_y+1):
    print(i)
    for j in range(step_x+1):
        print(j)
        
        if j == step_x:
            cropped = I[i*crop_size: (i+1)*crop_size, j*crop_size: j*crop_size + rem_x,:]
            
        elif i == step_y:
            cropped = I[i*crop_size: i*crop_size + rem_y, j*crop_size: (j+1)*crop_size,:]
            
        elif j == step_x and i == step_y:
            cropped = I[i*crop_size: i*crop_size + rem_y, j*crop_size: j*crop_size + rem_x,:]
            
        else:   
            cropped = I[i*crop_size: (i+1)*crop_size, j*crop_size:(j+1)*crop_size,:]
            
        if detector == 'SP':

            feat = SP.extract(numpy_image_to_torch(cropped).to(device))
            feat['keypoints'] = (feat['keypoints'].cpu().numpy().squeeze() + np.array([j*crop_size,i*crop_size])).astype(np.float32)   
            feat['keypoint_scores'] =   feat['keypoint_scores'].cpu().numpy().squeeze().astype(np.float32)
            # feat['scales'] =   feat['scales'].cpu().numpy().squeeze().astype(np.float32)
            # feat['oris'] =   feat['oris'].cpu().numpy().squeeze().astype(np.float32)
            feat['descriptors'] = feat['descriptors'].cpu().numpy().squeeze().astype(np.float32)
            
            if 'all_keypoints' not in locals():
                all_keypoints       = feat['keypoints']
                all_keypoint_scores = feat['keypoint_scores']
                # all_scales          = feat['scales']
                # all_oris            = feat['oris']
                all_descriptors     = feat['descriptors']
            else:
                all_keypoints        = np.vstack((all_keypoints, feat['keypoints']))
                all_keypoint_scores  = np.hstack((all_keypoint_scores, feat['keypoint_scores']))
                # all_scales           = np.hstack((all_scales, feat['scales']))
                # all_oris             = np.hstack((all_oris, feat['oris']))
                all_descriptors      = np.vstack((all_descriptors, feat['descriptors']))
                
        elif detector == 'ORB':

            keypoints, descriptors = ORB.detectAndCompute(cropped.squeeze(), None)
            
            for keypoint in keypoints:
                keypoint.pt = (keypoint.pt[0] + j*crop_size, keypoint.pt[1] + i*crop_size)

            if 'all_keypoints' not in locals():
                all_keypoints       = keypoints
                all_descriptors     = descriptors
            
            else:
                # all_keypoints.extend(keypoints)
                all_keypoints   = all_keypoints + keypoints
                all_descriptors = np.vstack((all_descriptors, descriptors))
                
                
if detector == 'SP':
    all_keypoints         = torch.tensor(all_keypoints, device=device).unsqueeze(0)
    all_keypoint_scores   = torch.tensor(all_keypoint_scores, device=device).unsqueeze(0)
    # all_scales            = torch.tensor(all_scales, device=device).unsqueeze(0)
    # all_oris              = torch.tensor(all_oris, device=device).unsqueeze(0)
    all_descriptors       = torch.tensor(all_descriptors, device=device).unsqueeze(0)
        
    # feat = {'keypoints' : all_keypoints, 'keypoint_scores' : all_keypoint_scores, 'scales' : all_scales , 
    #         'oris' : all_oris ,  'descriptors' : all_descriptors , 'image_size': torch.tensor(np.array([h,w],dtype=np.float32),device= device).unsqueeze(0)}

    feat = {'keypoints' : all_keypoints, 'keypoint_scores' : all_keypoint_scores, 
            'descriptors' : all_descriptors , 'image_size': torch.tensor(np.array([h,w],dtype=np.float32),device= device).unsqueeze(0)}
    keypoints, descriptors = feat["keypoints"] , feat
    keypoints_np = keypoints.cpu().numpy().squeeze()
    
elif detector == 'ORB': 
    descriptors = all_descriptors
    keypoints = all_keypoints
    keypoints_np = np.array([keypoint.pt for keypoint in keypoints], dtype=np.float32)
    keypoints = keypoints_to_list(all_keypoints)
    
    

# Save feat, keypoints, descriptors, and keypoints_np separately
with open('keypoints_2048_0thresh.pkl', 'wb') as f:
    pickle.dump(keypoints, f)

with open('descriptors_2048_0thresh.pkl', 'wb') as f:
    pickle.dump(descriptors, f)

with open('keypoints_np_2048_0thresh.pkl', 'wb') as f:
    pickle.dump(keypoints_np, f)

# # Load feat, keypoints, descriptors, and keypoints_np separately
# with open('keypoints.pkl', 'rb') as f:
#     keypoints = pickle.load(f)
    
# with open('descriptors.pkl', 'rb') as f:
#     descriptors = pickle.load(f)
    
# with open('keypoints_np.pkl', 'rb') as f:
#     keypoints_np = pickle.load(f)
    

print(np.shape(keypoints_np))

if I.shape[2] != 3:
    I = I.squeeze()
kp_frame = drawKeypoints(I, keypoints_np, color=(0, 255, 0), radius=3, thickness=2)

plt.imshow(cv2.cvtColor(kp_frame, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()
