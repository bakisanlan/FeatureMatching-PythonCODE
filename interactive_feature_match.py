#!/usr/bin/env python3
import cv2
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import torch
torch.set_grad_enabled(False)


# LightGlue imports (only used if method != 'orb' and != 'sift')
from LightGlue.lightglue import LightGlue, SuperPoint, SIFT
from LightGlue.lightglue.utils import rbd, numpy_image_to_torch

from utils import draw_custom_matches

def main(method='orb'):
    """Choose 'orb', 'sift', or anything else for SuperPoint+LightGlue."""
    # Paths to your folders (update as needed)
    real_UAV = 'data/cyclegan/turbo/winterUAV2summerSAT_test/v1/fid_reference_b2a'
    # real_SAT = 'data/cyclegan/turbo/winterUAV2summerSAT_test/real_sat_bait'
    real_SAT = 'data/cyclegan/turbo/winterUAV2summerSAT_test/real_sat_asvideo'
    fake_SAT = 'data/cyclegan/turbo/winterUAV2summerSAT_test/v2/fid-16251/samples_a2b'
    # fake_SAT = 'data/cyclegan/turbo/winterUAV2summerSAT_test/v2/bait_test'

    
    # Collect images
    real_UAV_paths = sorted(glob.glob(os.path.join(real_UAV, '*')))
    real_SAT_paths = sorted(glob.glob(os.path.join(real_SAT, '*')))
    fake_SAT_paths = sorted(glob.glob(os.path.join(fake_SAT, '*')))

    # Number of images to process
    num_images = min(len(real_UAV_paths), len(real_SAT_paths), len(fake_SAT_paths))
    if num_images == 0:
        print("No images found. Check your folder paths.")
        return

    # Containers for final display
    matched_images = []
    match_counts   = []  # [ [#inliersUAV, #inliersFake], ... ]
    feature_counts = []  # [ [ (UAV_kpts, SAT_kpts), (Fake_kpts, SAT_kpts) ], ... ]

    # ---------------------------------
    # SUPERPOINT + LIGHTGLUE BRANCH
    # ---------------------------------
    if (method != 'orb') and (method != 'sift'):
        print("Using SuperPoint + LightGlue method.")
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("Running on device:", device)

        # Instantiate SuperPoint and LightGlue
        SP = SuperPoint(max_num_keypoints=None).eval().to(device)
        match_conf = {
            'width_confidence': 0.95,  # for point pruning
            'depth_confidence': 0.9,  # for early stopping,
            'flash': True,
}
#         match_conf = {
#             'width_confidence': -1,  # for point pruning
#             'depth_confidence': -1,  # for early stopping,
#             'flash': True,
# }
        Matcher = LightGlue(features='superpoint', **match_conf).eval().to(device)
        # SP = SIFT(max_num_keypoints=None).eval().to(device)
        # Matcher = LightGlue(features='sift', depth_confidence=0.9, width_confidence=0.95).eval().to(device)

        for i in range(num_images):
            real_UAV_img = cv2.imread(real_UAV_paths[i], cv2.IMREAD_COLOR)
            real_SAT_img = cv2.imread(real_SAT_paths[i], cv2.IMREAD_COLOR)
            fake_SAT_img = cv2.imread(fake_SAT_paths[i], cv2.IMREAD_COLOR)

            if real_UAV_img is None or real_SAT_img is None or fake_SAT_img is None:
                print(f"Could not read:\n  {real_UAV_paths[i]}\n  {real_SAT_paths[i]}\n  {fake_SAT_paths[i]}")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (0,0), (0,0) ])
                continue

            # Convert BGR -> RGB for SuperPoint
            real_UAV_img_rgb = cv2.cvtColor(real_UAV_img, cv2.COLOR_BGR2RGB)
            real_SAT_img_rgb = cv2.cvtColor(real_SAT_img, cv2.COLOR_BGR2RGB)
            fake_SAT_img_rgb = cv2.cvtColor(fake_SAT_img, cv2.COLOR_BGR2RGB)

            # Extract keypoints + descriptors with SuperPoint
            real_UAV_feat = SP.extract(numpy_image_to_torch(real_UAV_img_rgb).to(device))
            fake_SAT_feat = SP.extract(numpy_image_to_torch(fake_SAT_img_rgb).to(device))
            real_SAT_feat_org = SP.extract(numpy_image_to_torch(real_SAT_img_rgb).to(device))

            # Keypoint arrays
            real_UAV_keypoints_np = real_UAV_feat["keypoints"].cpu().numpy().squeeze()
            fake_SAT_keypoints_np = fake_SAT_feat["keypoints"].cpu().numpy().squeeze()
            real_SAT_keypoints_np = real_SAT_feat_org["keypoints"].cpu().numpy().squeeze()

            # Count how many keypoints were detected
            num_uav_kpts  = real_UAV_keypoints_np.shape[0]
            num_fake_kpts = fake_SAT_keypoints_np.shape[0]
            num_sat_kpts  = real_SAT_keypoints_np.shape[0]

            # ---------- Real UAV vs Real SAT ----------
            real_UAV_matches = Matcher({"image0": real_UAV_feat, "image1": real_SAT_feat_org})
            real_UAV_feat, real_SAT_feat, real_UAV_matches = [rbd(x) for x in [real_UAV_feat, real_SAT_feat_org, real_UAV_matches]]
            kpts0, kpts1, matches = real_UAV_feat["keypoints"], real_SAT_feat["keypoints"], real_UAV_matches["matches"]

            m_kpts0 = kpts0[matches[..., 0]]  # matched keypoints in UAV
            m_kpts1 = kpts1[matches[..., 1]]  # matched keypoints in real SAT
            m_kpts0_np = m_kpts0.cpu().numpy().squeeze()
            m_kpts1_np = m_kpts1.cpu().numpy().squeeze()

            # Homography + inlier mask
            H_real_UAV, mask_real_UAV = cv2.findHomography(m_kpts0_np, m_kpts1_np, cv2.RANSAC, 5.0)
            if H_real_UAV is None or mask_real_UAV is None:
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                print("SuperPoint: homography not found (real_UAV->real_SAT).")
                continue

            mask_real_UAV = mask_real_UAV.ravel().tolist()
            inliers_real_UAV = [(matches[i]) for i, inl in enumerate(mask_real_UAV) if inl == 1]

            # Convert matched indices into the format for our custom draw function
            inliers_real_UAV_idx = [[m[0], m[1]] for m in inliers_real_UAV]

            # Draw inliers
            matched_real_UAV_draw = draw_custom_matches(
                real_UAV_img_rgb, real_UAV_keypoints_np,
                real_SAT_img_rgb, real_SAT_keypoints_np,
                inliers_real_UAV_idx
            )

            # ---------- Fake SAT vs Real SAT ----------
            fake_SAT_matches = Matcher({"image0": fake_SAT_feat, "image1": real_SAT_feat_org})
            fake_SAT_feat, real_SAT_feat, fake_SAT_matches = [rbd(x) for x in [fake_SAT_feat, real_SAT_feat_org, fake_SAT_matches]]
            kpts0, kpts1, matches = fake_SAT_feat["keypoints"], real_SAT_feat["keypoints"], fake_SAT_matches["matches"]

            m_kpts0 = kpts0[matches[..., 0]]  # matched keypoints in Fake SAT
            m_kpts1 = kpts1[matches[..., 1]]  # matched keypoints in Real SAT
            print(m_kpts0.shape)

            m_kpts0_np = m_kpts0.cpu().numpy().squeeze()
            m_kpts1_np = m_kpts1.cpu().numpy().squeeze()

            if len(m_kpts0_np) < 4 or len(m_kpts1_np) < 4:
                print("ORB: not enough matches for homography.")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue


            H_fake_SAT, mask_fake_SAT = cv2.findHomography(m_kpts0_np, m_kpts1_np, cv2.RANSAC, 5.0)
            if H_fake_SAT is None or mask_fake_SAT is None:
                matched_images.append(None)
                match_counts.append([len(inliers_real_UAV_idx), 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                print("SuperPoint: homography not found (fake_SAT->real_SAT).")
                continue

            mask_fake_SAT = mask_fake_SAT.ravel().tolist()
            inliers_fake_SAT = [(matches[i]) for i, inl in enumerate(mask_fake_SAT) if inl == 1]
            inliers_fake_SAT_idx = [[m[0], m[1]] for m in inliers_fake_SAT]

            matched_fake_SAT_draw = draw_custom_matches(
                fake_SAT_img_rgb, fake_SAT_keypoints_np,
                real_SAT_img_rgb, real_SAT_keypoints_np,
                inliers_fake_SAT_idx
            )

            matched_images.append([matched_real_UAV_draw, matched_fake_SAT_draw])
            match_counts.append([len(inliers_real_UAV), len(inliers_fake_SAT)])
            # Store feature counts: left pair (UAV->SAT), right pair (Fake->SAT)
            feature_counts.append([
                (num_uav_kpts, num_sat_kpts), 
                (num_fake_kpts, num_sat_kpts)
            ])

    # --------------------
    # SIFT BRANCH
    # --------------------
    elif method == 'sift':
        print("Using SIFT + BF(KNN) method.")
        sift = cv2.SIFT_create()
        bf_sift = cv2.BFMatcher(cv2.NORM_L2)

        for i in range(num_images):
            real_UAV_img = cv2.imread(real_UAV_paths[i], cv2.IMREAD_COLOR)
            real_SAT_img = cv2.imread(real_SAT_paths[i], cv2.IMREAD_COLOR)
            fake_SAT_img = cv2.imread(fake_SAT_paths[i], cv2.IMREAD_COLOR)

            if real_UAV_img is None or real_SAT_img is None or fake_SAT_img is None:
                print(f"Could not read:\n  {real_UAV_paths[i]}\n  {real_SAT_paths[i]}\n  {fake_SAT_paths[i]}")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (0,0), (0,0) ])
                continue

            gray_real_UAV = cv2.cvtColor(real_UAV_img, cv2.COLOR_BGR2GRAY)
            gray_real_SAT = cv2.cvtColor(real_SAT_img, cv2.COLOR_BGR2GRAY)
            gray_fake_SAT = cv2.cvtColor(fake_SAT_img, cv2.COLOR_BGR2GRAY)

            # Detect & compute SIFT
            kp_real_UAV, des_real_UAV = sift.detectAndCompute(gray_real_UAV, None)
            kp_real_SAT, des_real_SAT = sift.detectAndCompute(gray_real_SAT, None)
            kp_fake_SAT, des_fake_SAT = sift.detectAndCompute(gray_fake_SAT, None)

            # Keypoint counts
            num_uav_kpts  = len(kp_real_UAV) if kp_real_UAV else 0
            num_sat_kpts  = len(kp_real_SAT) if kp_real_SAT else 0
            num_fake_kpts = len(kp_fake_SAT) if kp_fake_SAT else 0

            if des_real_UAV is None or des_real_SAT is None or des_fake_SAT is None:
                print(f"SIFT: No features found in at least one image:\n  {real_UAV_paths[i]}")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue

            knn_real_UAV = bf_sift.knnMatch(des_real_UAV, des_real_SAT, k=2)
            knn_fake_SAT = bf_sift.knnMatch(des_fake_SAT, des_real_SAT, k=2)

            # Lowe's ratio test
            good_real_UAV = []
            for m, n in knn_real_UAV:
                if m.distance < 0.75 * n.distance:
                    good_real_UAV.append(m)

            good_fake_SAT = []
            for m, n in knn_fake_SAT:
                if m.distance < 0.75 * n.distance:
                    good_fake_SAT.append(m)

            if len(good_real_UAV) < 4 or len(good_fake_SAT) < 4:
                print("SIFT: Not enough good matches (need >=4).")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue

            # Coordinates
            src_real_UAV = np.float32([kp_real_UAV[m.queryIdx].pt for m in good_real_UAV]).reshape(-1, 1, 2)
            dst_real_UAV = np.float32([kp_real_SAT[m.trainIdx].pt for m in good_real_UAV]).reshape(-1, 1, 2)

            src_fake_SAT = np.float32([kp_fake_SAT[m.queryIdx].pt for m in good_fake_SAT]).reshape(-1, 1, 2)
            dst_fake_SAT = np.float32([kp_real_SAT[m.trainIdx].pt for m in good_fake_SAT]).reshape(-1, 1, 2)

            # Homography
            H_real_UAV, mask_real_UAV = cv2.findHomography(src_real_UAV, dst_real_UAV, cv2.RANSAC, 5.0)
            H_fake_SAT, mask_fake_SAT = cv2.findHomography(src_fake_SAT, dst_fake_SAT, cv2.RANSAC, 5.0)

            if H_real_UAV is None or mask_real_UAV is None or H_fake_SAT is None or mask_fake_SAT is None:
                print("SIFT: Homography failed.")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue

            mask_real_UAV = mask_real_UAV.ravel().tolist()
            mask_fake_SAT = mask_fake_SAT.ravel().tolist()

            inliers_real_UAV = [m for (m, inl) in zip(good_real_UAV, mask_real_UAV) if inl == 1]
            inliers_fake_SAT = [m for (m, inl) in zip(good_fake_SAT, mask_fake_SAT) if inl == 1]

            sift_matched_real_UAV = cv2.drawMatches(
                real_UAV_img, kp_real_UAV,
                real_SAT_img, kp_real_SAT,
                inliers_real_UAV, None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            sift_matched_fake_SAT = cv2.drawMatches(
                fake_SAT_img, kp_fake_SAT,
                real_SAT_img, kp_real_SAT,
                inliers_fake_SAT, None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )

            # Convert BGR->RGB for plotting
            sift_matched_real_UAV_rgb = cv2.cvtColor(sift_matched_real_UAV, cv2.COLOR_BGR2RGB)
            sift_matched_fake_SAT_rgb = cv2.cvtColor(sift_matched_fake_SAT, cv2.COLOR_BGR2RGB)

            matched_images.append([sift_matched_real_UAV_rgb, sift_matched_fake_SAT_rgb])
            match_counts.append([len(inliers_real_UAV), len(inliers_fake_SAT)])
            feature_counts.append([
                (num_uav_kpts, num_sat_kpts),
                (num_fake_kpts, num_sat_kpts)
            ])

    # --------------------
    # ORB BRANCH
    # --------------------
    else:  # method == 'orb'
        print("Using ORB + BF(crossCheck) method.")
        orb = cv2.ORB_create()
        bf_orb = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        for i in range(num_images):
            real_UAV_img = cv2.imread(real_UAV_paths[i], cv2.IMREAD_COLOR)
            real_SAT_img = cv2.imread(real_SAT_paths[i], cv2.IMREAD_COLOR)
            fake_SAT_img = cv2.imread(fake_SAT_paths[i], cv2.IMREAD_COLOR)

            if real_UAV_img is None or real_SAT_img is None or fake_SAT_img is None:
                print(f"Could not read:\n  {real_UAV_paths[i]}\n  {real_SAT_paths[i]}\n  {fake_SAT_paths[i]}")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (0,0), (0,0) ])
                continue

            gray_real_UAV = cv2.cvtColor(real_UAV_img, cv2.COLOR_BGR2GRAY)
            gray_real_SAT = cv2.cvtColor(real_SAT_img, cv2.COLOR_BGR2GRAY)
            gray_fake_SAT = cv2.cvtColor(fake_SAT_img, cv2.COLOR_BGR2GRAY)

            kp_real_UAV, des_real_UAV = orb.detectAndCompute(gray_real_UAV, None)
            kp_real_SAT, des_real_SAT = orb.detectAndCompute(gray_real_SAT, None)
            kp_fake_SAT, des_fake_SAT = orb.detectAndCompute(gray_fake_SAT, None)

            # Keypoint counts
            num_uav_kpts  = len(kp_real_UAV) if kp_real_UAV else 0
            num_sat_kpts  = len(kp_real_SAT) if kp_real_SAT else 0
            num_fake_kpts = len(kp_fake_SAT) if kp_fake_SAT else 0

            if des_real_UAV is None or des_real_SAT is None or des_fake_SAT is None:
                print(f"No features found in at least one image:\n  {real_UAV_paths[i]}")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue

            matches_real_UAV = bf_orb.match(des_real_UAV, des_real_SAT)
            matches_fake_SAT = bf_orb.match(des_fake_SAT, des_real_SAT)

            # Sort by distance
            matches_real_UAV = sorted(matches_real_UAV, key=lambda x: x.distance)
            matches_fake_SAT = sorted(matches_fake_SAT, key=lambda x: x.distance)

            # Coordinates
            src_pts_real_UAV = np.float32([kp_real_UAV[m.queryIdx].pt for m in matches_real_UAV]).reshape(-1, 1, 2)
            dst_pts_real_SAT_UAV = np.float32([kp_real_SAT[m.trainIdx].pt for m in matches_real_UAV]).reshape(-1, 1, 2)

            src_pts_fake_SAT = np.float32([kp_fake_SAT[m.queryIdx].pt for m in matches_fake_SAT]).reshape(-1, 1, 2)
            dst_pts_real_SAT_fake = np.float32([kp_real_SAT[m.trainIdx].pt for m in matches_fake_SAT]).reshape(-1, 1, 2)

            if len(src_pts_real_UAV) < 4 or len(src_pts_fake_SAT) < 4:
                print("ORB: not enough matches for homography.")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue

            H_real_UAV, mask_real_UAV = cv2.findHomography(src_pts_real_UAV, dst_pts_real_SAT_UAV, cv2.RANSAC, 5.0)
            H_fake_SAT, mask_fake_SAT = cv2.findHomography(src_pts_fake_SAT, dst_pts_real_SAT_fake, cv2.RANSAC, 5.0)

            if H_real_UAV is None or mask_real_UAV is None or H_fake_SAT is None or mask_fake_SAT is None:
                print("ORB: homography not found.")
                matched_images.append(None)
                match_counts.append([0, 0])
                feature_counts.append([ (num_uav_kpts, num_sat_kpts), (num_fake_kpts, num_sat_kpts) ])
                continue

            mask_real_UAV = mask_real_UAV.ravel().tolist()
            mask_fake_SAT = mask_fake_SAT.ravel().tolist()

            inliers_real_UAV = [m for (m, inl) in zip(matches_real_UAV, mask_real_UAV) if inl == 1]
            inliers_fake_SAT = [m for (m, inl) in zip(matches_fake_SAT, mask_fake_SAT) if inl == 1]

            orb_matched_real_UAV = cv2.drawMatches(
                real_UAV_img, kp_real_UAV,
                real_SAT_img, kp_real_SAT,
                inliers_real_UAV, None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            orb_matched_fake_SAT = cv2.drawMatches(
                fake_SAT_img, kp_fake_SAT,
                real_SAT_img, kp_real_SAT,
                inliers_fake_SAT, None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )

            orb_matched_real_UAV_rgb = cv2.cvtColor(orb_matched_real_UAV, cv2.COLOR_BGR2RGB)
            orb_matched_fake_SAT_rgb = cv2.cvtColor(orb_matched_fake_SAT, cv2.COLOR_BGR2RGB)

            matched_images.append([orb_matched_real_UAV_rgb, orb_matched_fake_SAT_rgb])
            match_counts.append([len(inliers_real_UAV), len(inliers_fake_SAT)])
            feature_counts.append([
                (num_uav_kpts, num_sat_kpts),
                (num_fake_kpts, num_sat_kpts)
            ])

    # =============== Interactive Plot with Arrow Keys ===============
    current_index = 0
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    def show_image(idx):
        """Display the two matched images side by side, 
           including the total number of detected keypoints in each image 
           and the inlier count for RANSAC."""
        for ax in axes:
            ax.clear()

        left_img, right_img = matched_images[idx]
        left_inliers, right_inliers = match_counts[idx]
        # feature_counts[idx] = [ (UAV_kpts, SAT_kpts), (Fake_kpts, SAT_kpts) ]
        (uav_kpts, sat_kpts_left) = feature_counts[idx][0]
        (fake_kpts, sat_kpts_right) = feature_counts[idx][1]

        # Show left subplot: Real UAV vs. Real SAT
        axes[0].imshow(left_img)
        axes[0].set_title(
            f"Real UAV vs. Real SAT\n"
            # f"All features --> UAV: {uav_kpts} features, SAT: {sat_kpts_left} features\n"
            f"Inlier Matches: {left_inliers}",
            fontsize=24
        )
        axes[0].axis('off')

        # Show right subplot: Fake SAT vs. Real SAT
        axes[1].imshow(right_img)
        axes[1].set_title(
            f"GEN SAT vs. Real SAT\n"
            # f"All features --> GEN: {fake_kpts} features, SAT: {sat_kpts_right} features\n"
            f"Inlier Matches: {right_inliers}",
            fontsize=24
        )
        axes[1].axis('off')

        fig.suptitle(f"Index {idx+1}/{num_images}", fontsize=14)
        fig.canvas.draw()

    def on_key(event):
        nonlocal current_index
        if event.key == 'right':
            current_index = min(current_index + 1, num_images - 1)
            if matched_images[current_index] is not None:
                show_image(current_index)
        elif event.key == 'left':
            current_index = max(current_index - 1, 0)
            if matched_images[current_index] is not None:
                show_image(current_index)

    fig.canvas.mpl_connect('key_press_event', on_key)

    # Display first result if available
    if num_images > 0 and matched_images[0] is not None:
        show_image(current_index)
    else:
        print("No valid image pairs to show.")

    plt.show()

if __name__ == "__main__":
    # You can pass 'sift', 'orb', or anything else for superpoint
    # Example:
    #   main('sift')
    #   main('orb')
    #   main('superpoint')
    main('adas')
