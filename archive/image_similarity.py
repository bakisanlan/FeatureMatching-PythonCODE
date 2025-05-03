import cv2
import numpy as np
from Timer import Timer
from utils import rotate_image

# Function to compute cross-correlation
def cross_correlation(image1, image2):
    # Convert images to grayscale
    # gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    # gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    
    # Perform cross-correlation
    result = cv2.matchTemplate(image1, image2, cv2.TM_CCORR_NORMED)
    return result

def cross_correlation_color(image1, image2):
    # Ensure both images are of the same size
    if image1.shape != image2.shape:
        raise ValueError("Images must be of the same size and channels")
    
    # Initialize an array to store the correlation results for each channel
    correlations = []
    
    # Iterate over each color channel
    for i in range(3):  # Assuming RGB
        channel1 = image1[:, :, i]
        channel2 = image2[:, :, i]
        
        # Compute normalized cross-correlation for the current channel
        result = cv2.matchTemplate(channel1, channel2, cv2.TM_CCORR_NORMED)
        max_corr = np.max(result)
        correlations.append(max_corr)
    
    # Average the correlation values across all channels
    average_corr = np.mean(correlations)
    return average_corr

data = '000094'
# data_folder = 'cycleganCUTv1'
data_folder = 'turbo'


real_uav = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_real_uav.png')
fake_sat = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_fake_sat.png')
real_sat = cv2.imread('data/cyclegan/' + data_folder + '/'+ data +'/'+data+'_real_sat.jpg')

# fake_sat = fake_sat[1:3,:4:6]

# real_uav = cv2.cvtColor(real_uav ,cv2.COLOR_BGR2RGB)
# fake_sat = cv2.cvtColor(fake_sat ,cv2.COLOR_BGR2RGB)
# real_sat = cv2.cvtColor(real_sat ,cv2.COLOR_BGR2RGB)

# Compute cross-correlation
corr_result_1 = cross_correlation(real_sat, real_uav)
corr_result_2 = cross_correlation(real_sat, fake_sat)

# Get the maximum value of the cross-correlation result
max_corr = np.max(corr_result_1)
print("Maximum correlation:", max_corr)

max_corr = np.max(corr_result_2)
print("Maximum correlation:", max_corr)


import cv2
import numpy as np

def compute_normalized_color_histogram(image, bins=(8, 8, 8), exclude_black=True):
    """
    Compute a normalized color histogram for an image in the HSV color space.
    Optionally excludes pixels that are fully black (0,0,0 in BGR).
    
    Args:
        image (numpy.ndarray): Input image in BGR format.
        bins (tuple): Number of bins for each channel (H, S, V).
        exclude_black (bool): Whether to exclude full black pixels from the histogram.
    
    Returns:
        numpy.ndarray: Flattened and normalized histogram.
    """
    # Create a mask to exclude full black pixels if needed
    mask = None
    if exclude_black:
        # This mask will be 255 (white) for pixels that are NOT full black
        # and 0 for pixels that are exactly (0,0,0) in BGR.
        mask = cv2.inRange(image, np.array([1, 1, 1], dtype=np.uint8),
                           np.array([255, 255, 255], dtype=np.uint8))
    
    # Convert the image to the HSV color space.
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Compute the histogram for the HSV image using the mask (if provided)
    hist = cv2.calcHist([image], channels=[0, 1, 2], mask=mask,
                        histSize=bins, ranges=[0, 180, 0, 256, 0, 256])
    
    # Normalize the histogram so that the sum is 1.
    hist = cv2.normalize(hist, hist).flatten()
    
    return hist

def histogram_intersection(hist1, hist2):
    """
    Compute the histogram intersection between two normalized histograms.
    
    Args:
        hist1, hist2 (numpy.ndarray): Normalized histograms.
     
    Returns:
        float: Intersection score between 0 and 1 (1 means identical histograms).
    """
    # return np.sum(np.minimum(hist1, hist2))
    return cv2.compareHist(hist1, hist2, 0)

def color_similarity(image1, image2, bins=(16, 16, 16), exclude_black=True):
    """
    Compute a similarity metric between two images based solely on their color distribution,
    while excluding full black pixels. This metric is rotation invariant since it relies on
    the global color histogram.
    
    Args:
        image1, image2 (numpy.ndarray): Input images in BGR format.
        bins (tuple): Number of bins for each HSV channel.
        exclude_black (bool): Whether to exclude full black pixels from the histogram.
    
    Returns:
        float: Similarity score between 0 and 1.
    """
    hist1 = compute_normalized_color_histogram(image1, bins, exclude_black=exclude_black)
    hist2 = compute_normalized_color_histogram(image2, bins, exclude_black=exclude_black)
    similarity = histogram_intersection(hist1, hist2)
    return similarity

# if __name__ == '__main__':
#     import argparse

# # Set up the argument parser to accept image file paths.
# parser = argparse.ArgumentParser(
#     description="Compute color similarity between two images based on color histograms, excluding full black pixels."
# )
# parser.add_argument("image1", help="Path to the first image")
# parser.add_argument("image2", help="Path to the second image")
# args = parser.parse_args()

# Load images using OpenCV
# Load the image
org_sat = real_sat
image = real_sat# Get image dimensions
(h, w) = image.shape[:2]

# Calculate the center of the image
center = (w / 2, h / 2)

# Calculate the rotation matrix using cv2.getRotationMatrix2D
angle = 10
scale = 1.0
M = cv2.getRotationMatrix2D(center, angle, scale)

# Calculate the new bounding dimensions of the image
cos = np.abs(M[0, 0])
sin = np.abs(M[0, 1])

# New width and height
new_w = int((h * sin) + (w * cos))
new_h = int((h * cos) + (w * sin))

# Adjust the rotation matrix to consider translation
M[0, 2] += (new_w / 2) - center[0]
M[1, 2] += (new_h / 2) - center[1]

# Perform the actual rotation and show the image
real_sat = cv2.warpAffine(image, M, (new_w, new_h))
cv2.imshow('Rotated Image', real_sat)
cv2.waitKey(0)

img1 = real_uav
img2 = real_sat

# Check if images are loaded successfully.
if img1 is None or img2 is None:
    print("Error: One or both images could not be loaded.")
    exit(1)

# Compute the color similarity score.
similarity_score = color_similarity(img1, img2)
print("Color similarity score: {:.4f}".format(similarity_score))


# Load images using OpenCV
img1 = fake_sat
img2 = real_sat

# Check if images are loaded successfully.
if img1 is None or img2 is None:
    print("Error: One or both images could not be loaded.")
    exit(1)


similarity_score = color_similarity(img1, img2)
print("Color similarity score: {:.4f}".format(similarity_score))
