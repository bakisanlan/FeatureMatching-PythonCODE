
import cv2
from utils import *
import matplotlib.pyplot as plt 
from matplotlib.gridspec import GridSpec
import os

ORB    = cv2.ORB_create(nfeatures = 100000000,nlevels = 4)

data = '000094'
# data_folder = 'cycleganCUTv1'
data_folder = 'turbo'

real_uav = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_real_uav.png')
fake_sat = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_fake_sat.png')
real_sat = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_real_sat.jpg')

real_uav = cv2.cvtColor(real_uav ,cv2.COLOR_BGR2RGB )
fake_sat = cv2.cvtColor(fake_sat ,cv2.COLOR_BGR2RGB )
real_sat = cv2.cvtColor(real_sat ,cv2.COLOR_BGR2RGB )



# extract features
real_uav_keypoints, real_uav_descriptors = ORB.detectAndCompute(real_uav, None)
fake_sat_keypoints, fake_sat_descriptors = ORB.detectAndCompute(fake_sat, None)
real_sat_keypoints, real_sat_descriptors = ORB.detectAndCompute(real_sat, None)

real_uav_FeaturesFrame = cv2.drawKeypoints(real_uav, real_uav_keypoints,None , color= (0,255,0))
fake_sat_FeaturesFrame = cv2.drawKeypoints(fake_sat, fake_sat_keypoints,None , color= (0,255,0))
real_sat_FeaturesFrame = cv2.drawKeypoints(real_sat, real_sat_keypoints,None , color= (0,255,0))

#match features
Matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

real_uav_matches  = Matcher.match(real_uav_descriptors,real_sat_descriptors)
fake_sat_matches = Matcher.match(fake_sat_descriptors,real_sat_descriptors)

real_uav_src_pts = np.float32([ real_uav_keypoints[m.queryIdx].pt for m in real_uav_matches ]).reshape(-1,1,2)
real_uav_dst_pts = np.float32([ real_sat_keypoints[m.trainIdx].pt for m in real_uav_matches ]).reshape(-1,1,2)

M, mask = cv2.findHomography(real_uav_src_pts, real_uav_dst_pts, cv2.RANSAC,5.0)
real_uav_matchesMask = mask.ravel().tolist()

fake_sat_src_pts = np.float32([ fake_sat_keypoints[m.queryIdx].pt for m in fake_sat_matches ]).reshape(-1,1,2)
fake_sat_dst_pts = np.float32([ real_sat_keypoints[m.trainIdx].pt for m in fake_sat_matches ]).reshape(-1,1,2)

M, mask = cv2.findHomography(fake_sat_src_pts, fake_sat_dst_pts, cv2.RANSAC,5.0)
fake_sat_matchesMask = mask.ravel().tolist()

real_uav_matches = cv2.drawMatches(real_uav,real_uav_keypoints,real_sat,real_sat_keypoints,real_uav_matches,None,flags=2,matchColor= (0,0,0),  singlePointColor = None ,matchesMask =real_uav_matchesMask )
fake_sat_matches = cv2.drawMatches(fake_sat,fake_sat_keypoints,real_sat,real_sat_keypoints,fake_sat_matches,None,flags=2,matchColor= (0,255,0),singlePointColor = None, matchesMask= fake_sat_matchesMask)

print(sum(real_uav_matchesMask))
print(sum(fake_sat_matchesMask))

# # Create subplots
fig1, axs1 = plt.subplots(1, 3, figsize=(15, 5))  # 1 row, 3 columns

axs1[0].imshow(real_uav_FeaturesFrame)
axs1[0].axis('off')
axs1[0].set_title('real_uav')

axs1[1].imshow(fake_sat_FeaturesFrame)
axs1[1].axis('off')
axs1[1].set_title('fake_sat')

axs1[2].imshow(real_sat_FeaturesFrame)
axs1[2].axis('off')
axs1[2].set_title('real_sat')

plt.tight_layout()
plt.show(block = False)

# Figure 2: Two big images
fig2, axs2 = plt.subplots(1, 2, figsize=(15, 8))  # 1 row, 2 columns

axs2[0].imshow(real_uav_matches)
axs2[0].axis('off')
axs2[0].set_title('real uav matches')

axs2[1].imshow(fake_sat_matches)
axs2[1].axis('off')
axs2[1].set_title('fake sat matches')

plt.tight_layout()
plt.show()
