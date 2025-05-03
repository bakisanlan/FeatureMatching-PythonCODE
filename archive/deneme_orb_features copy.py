import cv2
from utils import *
import matplotlib.pyplot as plt 
from matplotlib.gridspec import GridSpec
import torch
from LightGlue.lightglue import SIFT,LightGlue,SuperPoint
from LightGlue.lightglue.utils import rbd,numpy_image_to_torch
from LightGlue.lightglue import viz2d
from utils import drawKeypoints

data = '000094'
# data_folder = 'cycleganCUTv1'
data_folder = 'turbo'


real_uav = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_real_uav.png')
fake_sat = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_fake_sat.png')
real_sat = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_real_sat.jpg')

real_uav = cv2.cvtColor(real_uav ,cv2.COLOR_BGR2RGB )
fake_sat = cv2.cvtColor(fake_sat ,cv2.COLOR_BGR2RGB )
real_sat = cv2.cvtColor(real_sat ,cv2.COLOR_BGR2RGB )


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
print(device)
SP = SuperPoint(max_num_keypoints=None).eval().to(device)  # load the extractor

# real uav feature extraction
real_uav_feat = SP.extract(numpy_image_to_torch(real_uav).to(device))
real_uav_keypoints, real_uav_descriptors = real_uav_feat["keypoints"] , real_uav_feat
real_uav_keypoints_np = real_uav_keypoints.cpu().numpy().squeeze()

# fake sat feature extraction
fake_sat_feat = SP.extract(numpy_image_to_torch(fake_sat).to(device))
fake_sat_keypoints, fake_sat_descriptors = fake_sat_feat["keypoints"] , fake_sat_feat
fake_sat_keypoints_np = fake_sat_keypoints.cpu().numpy().squeeze()


# real sat feature extraction
real_sat_feat_org = SP.extract(numpy_image_to_torch(real_sat).to(device))
real_sat_keypoints, real_sat_descriptors = real_sat_feat_org["keypoints"] , real_sat_feat_org
real_sat_keypoints_np = real_sat_keypoints.cpu().numpy().squeeze()

# # draw features
# real_uav_FeaturesFrame = drawKeypoints(real_uav, real_uav_keypoints_np,None , color= (0,255,0))
# fake_sat_FeaturesFrame = drawKeypoints(fake_sat, real_uav_keypoints_np,None , color= (0,255,0))
# real_sat_FeaturesFrame = drawKeypoints(real_sat, real_uav_keypoints_np,None , color= (0,255,0))

#match features
Matcher = LightGlue(features='superpoint', depth_confidence=0.9, width_confidence=0.95).eval().to(device)  

real_uav_matches = Matcher({"image0": real_uav_feat, "image1": real_sat_feat_org})
real_uav_feat, real_sat_feat, real_uav_matches = [
    rbd(x) for x in [real_uav_feat, real_sat_feat_org, real_uav_matches]
]  # remove batch dimension

kpts0, kpts1, matches = real_uav_feat["keypoints"], real_sat_feat["keypoints"], real_uav_matches["matches"]
m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]
m_kpts0_np = m_kpts0.cpu().numpy().squeeze()
m_kpts1_np = m_kpts1.cpu().numpy().squeeze()
matches_np = matches.cpu().numpy()
print(m_kpts0.shape)

fig, ax = viz2d.plot_images([real_uav, real_sat])
viz2d.plot_matches(m_kpts0, m_kpts1, color="lime", lw=0.2)
# viz2d.add_text(0, f'Stop after {real_uav_matches["stop"]} layers', fs=20)
# print(m_kpts0.shape)
plt.show(block = False)

#
fake_sat_matches = Matcher({"image0": fake_sat_feat, "image1": real_sat_feat_org})
fake_sat_feat, real_sat_feat, fake_sat_matches = [
    rbd(x) for x in [fake_sat_feat, real_sat_feat_org, fake_sat_matches]
]  # remove batch dimension

kpts0, kpts1, matches = fake_sat_feat["keypoints"], real_sat_feat["keypoints"], fake_sat_matches["matches"]
m_kpts0, m_kpts1 = kpts0[matches[..., 0]], kpts1[matches[..., 1]]
print(m_kpts0.shape)

fig, ax = viz2d.plot_images([fake_sat, real_sat])
viz2d.plot_matches(m_kpts0, m_kpts1, color="lime", lw=0.2)
# viz2d.add_text(0, f'Stop after {real_uav_matches["stop"]} layers', fs=20)
# print(m_kpts0.shape)
plt.show()

