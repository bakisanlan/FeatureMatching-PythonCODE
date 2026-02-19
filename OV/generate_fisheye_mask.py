#!/usr/bin/env python3
"""
Fisheye Mask Generator for OpenVINS

This script generates a circular mask for fisheye cameras to exclude
distorted outer regions from feature tracking in OpenVINS.

The mask is a binary image where (OpenVINS convention):
- White (255) = Masked/excluded region (features ignored)
- Black (0) = Valid region for feature tracking

Usage:
    python generate_fisheye_mask.py [--radius_ratio 0.9] [--output mask.png] [--preview]
"""

import cv2
import numpy as np
import argparse
import os


def generate_fisheye_mask(
    resolution: tuple,
    principal_point: tuple,
    radius_ratio: float = 0.9,
    inner_radius_ratio: float = 0.0,
    output_path: str = "mask.png",
    preview: bool = False
):
    """
    Generate a circular mask for fisheye camera.
    
    Args:
        resolution: (width, height) of the image
        principal_point: (cx, cy) optical center coordinates
        radius_ratio: Ratio of mask radius to minimum half-dimension (0.0 to 1.0)
                     1.0 = full circle to image edge, 0.9 = 90% of that radius
        inner_radius_ratio: Optional inner radius ratio for ring mask (0.0 = no inner hole)
        output_path: Path to save the mask image
        preview: If True, display the mask before saving
    
    Returns:
        mask: The generated binary mask (numpy array)
    """
    width, height = resolution
    cx, cy = principal_point
    
    # Calculate the maximum radius that fits within the image
    # Consider distance from principal point to all edges
    dist_to_left = cx
    dist_to_right = width - cx
    dist_to_top = cy
    dist_to_bottom = height - cy
    
    # Use the minimum distance to ensure the circle stays within bounds
    max_radius = min(dist_to_left, dist_to_right, dist_to_top, dist_to_bottom)
    
    # Apply the radius ratio
    radius = int(max_radius * radius_ratio)
    inner_radius = int(max_radius * inner_radius_ratio)
    
    print(f"Image resolution: {width} x {height}")
    print(f"Principal point: ({cx:.2f}, {cy:.2f})")
    print(f"Maximum radius: {max_radius:.2f}")
    print(f"Mask outer radius: {radius} pixels (ratio: {radius_ratio})")
    if inner_radius_ratio > 0:
        print(f"Mask inner radius: {inner_radius} pixels (ratio: {inner_radius_ratio})")
    
    # Create white mask (all masked initially - OpenVINS convention)
    mask = np.ones((height, width), dtype=np.uint8) * 255
    
    # Draw black filled circle for valid region
    center = (int(cx), int(cy))
    cv2.circle(mask, center, radius, 0, -1)
    
    # Optionally cut out inner circle (for ring mask)
    if inner_radius > 0:
        cv2.circle(mask, center, inner_radius, 255, -1)
    
    # Preview if requested
    if preview:
        # Create a visualization with the mask overlaid on a sample pattern
        preview_img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Draw grid pattern to visualize mask
        for i in range(0, width, 50):
            cv2.line(preview_img, (i, 0), (i, height), (50, 50, 50), 1)
        for j in range(0, height, 50):
            cv2.line(preview_img, (0, j), (width, j), (50, 50, 50), 1)
        
        # Apply mask as semi-transparent overlay
        # OpenVINS: white (255) = masked, black (0) = valid
        masked_region = mask == 255
        preview_img[masked_region] = [0, 0, 100]  # Red tint for masked areas
        preview_img[~masked_region] = [0, 100, 0]  # Green for valid areas
        
        # Draw mask boundary
        cv2.circle(preview_img, center, radius, (255, 255, 255), 2)
        if inner_radius > 0:
            cv2.circle(preview_img, center, inner_radius, (255, 255, 255), 2)
        
        # Draw principal point
        cv2.drawMarker(preview_img, center, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
        
        # Add text info
        cv2.putText(preview_img, f"Resolution: {width}x{height}", (10, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(preview_img, f"Radius: {radius}px ({radius_ratio*100:.0f}%)", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(preview_img, f"Principal Point: ({cx:.1f}, {cy:.1f})", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Calculate valid area percentage
        valid_pixels = np.sum(mask == 0)
        total_pixels = width * height
        valid_percentage = (valid_pixels / total_pixels) * 100
        cv2.putText(preview_img, f"Valid Area: {valid_percentage:.1f}%", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow("Fisheye Mask Preview (Green=Valid, Red=Masked)", preview_img)
        cv2.imshow("Binary Mask", mask)
        print("\nPress any key to save and continue, or 'q' to quit without saving...")
        key = cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        if key == ord('q'):
            print("Mask generation cancelled.")
            return None
    
    # Save the mask
    cv2.imwrite(output_path, mask)
    print(f"\nMask saved to: {output_path}")
    
    # Print statistics
    valid_pixels = np.sum(mask == 0)
    total_pixels = width * height
    print(f"Valid region: {valid_pixels} pixels ({valid_pixels/total_pixels*100:.1f}% of image)")
    
    return mask


def main():
    parser = argparse.ArgumentParser(
        description="Generate a circular mask for fisheye cameras (OpenVINS compatible)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate mask with default settings (90% radius)
  python generate_fisheye_mask.py
  
  # Generate mask with 85% radius and preview
  python generate_fisheye_mask.py --radius_ratio 0.85 --preview
  
  # Generate ring mask (exclude center and outer regions)
  python generate_fisheye_mask.py --radius_ratio 0.9 --inner_ratio 0.1
  
  # Custom output path
  python generate_fisheye_mask.py --output /path/to/cam0_mask.png
        """
    )
    
    # Camera parameters from your calibration
    # Default values from the provided YAML
    parser.add_argument("--width", type=int, default=960,
                        help="Image width (default: 960)")
    parser.add_argument("--height", type=int, default=540,
                        help="Image height (default: 540)")
    parser.add_argument("--cx", type=float, default=482.97225685931204,
                        help="Principal point x (default: 482.97)")
    parser.add_argument("--cy", type=float, default=274.74159496622894,
                        help="Principal point y (default: 274.74)")
    
    # Mask parameters
    parser.add_argument("--radius_ratio", type=float, default=0.90,
                        help="Outer radius ratio (0.0-1.0, default: 0.90)")
    parser.add_argument("--inner_ratio", type=float, default=0.0,
                        help="Inner radius ratio for ring mask (0.0-1.0, default: 0.0 = no inner hole)")
    
    # Output options
    parser.add_argument("--output", type=str, default="cam0_mask.png",
                        help="Output mask file path (default: cam0_mask.png)")
    parser.add_argument("--preview", action="store_true",
                        help="Preview the mask before saving")
    
    args = parser.parse_args()
    
    # Validate parameters
    if not 0.0 < args.radius_ratio <= 1.0:
        print("Error: radius_ratio must be between 0.0 and 1.0")
        return
    
    if not 0.0 <= args.inner_ratio < args.radius_ratio:
        print("Error: inner_ratio must be between 0.0 and less than radius_ratio")
        return
    
    print("=" * 50)
    print("Fisheye Mask Generator for OpenVINS")
    print("=" * 50)
    
    # Generate the mask
    mask = generate_fisheye_mask(
        resolution=(args.width, args.height),
        principal_point=(args.cx, args.cy),
        radius_ratio=args.radius_ratio,
        inner_radius_ratio=args.inner_ratio,
        output_path=args.output,
        preview=args.preview
    )
    
    if mask is not None:
        print("\n" + "=" * 50)
        print("OpenVINS Configuration:")
        print("=" * 50)
        print(f"""
Add the following to your OpenVINS config YAML:

cam0:
  mask: "{os.path.abspath(args.output)}"
  
Or use relative path if preferred:
cam0:
  mask: "{args.output}"
""")


if __name__ == "__main__":
    main()
