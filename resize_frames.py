import cv2
from pathlib import Path
from utils import resize_image
import argparse

def resize_frames_in_folder(input_folder, output_folder, snap_dim=(256, 256)):
    """
    Load frames from input folder, resize them, and save to output folder.
    
    Args:
        input_folder (str or Path): Path to folder containing input images
        output_folder (str or Path): Path to folder where resized images will be saved
        snap_dim (tuple): Target dimensions (width, height) for resizing
    """
    # Convert to Path objects
    input_path = Path(input_folder)
    output_path = Path(output_folder)
    
    # Create output folder if it doesn't exist
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Supported image extensions
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff']
    
    # Get all image files from input folder
    image_files = []
    for ext in image_extensions:
        image_files.extend(input_path.glob(ext))
        image_files.extend(input_path.glob(ext.upper()))  # Also check uppercase extensions
    
    # Sort files by name
    image_files = sorted(image_files)
    
    if not image_files:
        print(f"No image files found in {input_path}")
        return
    
    print(f"Found {len(image_files)} images to resize")
    print(f"Target dimensions: {snap_dim}")
    print(f"Output folder: {output_path}")
    print("-" * 50)
    
    # Process each image
    success_count = 0
    for img_file in image_files:
        try:
            # Read the image
            frame = cv2.imread(str(img_file))
            
            if frame is None:
                print(f"Warning: Could not read {img_file.name}")
                continue
            
            # Resize the image using the utils function
            resized_frame = resize_image(frame, snapDim=snap_dim)
            
            # Create output filename (keep original name)
            output_file = output_path / img_file.name
            
            # Save the resized image
            cv2.imwrite(str(output_file), resized_frame)
            
            success_count += 1
            print(f"Processed [{success_count}/{len(image_files)}]: {img_file.name}")
            
        except Exception as e:
            print(f"Error processing {img_file.name}: {str(e)}")
    
    print("-" * 50)
    print(f"Successfully resized {success_count}/{len(image_files)} images")
    print(f"Resized images saved to: {output_path}")


if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Resize images in a folder')
    parser.add_argument('--input', type=str, default='./captured_frames/540/',
                        help='Input folder containing images (default: ./captured_frames/540/)')
    parser.add_argument('--output', type=str, default='./captured_frames/256/',
                        help='Output folder for resized images (default: ./captured_frames/256/)')
    parser.add_argument('--width', type=int, default=256,
                        help='Target width (default: 256)')
    parser.add_argument('--height', type=int, default=256,
                        help='Target height (default: 256)')
    
    args = parser.parse_args()
    
    # Run the resize function
    snap_dim = (args.width, args.height)
    resize_frames_in_folder(args.input, args.output, snap_dim)