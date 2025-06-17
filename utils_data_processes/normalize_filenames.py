
import os
import re


def normalize_filenames(folder_path):
    files = [f for f in os.listdir(folder_path) if f.endswith('.jpg')]
    
    # Extract timestamp strings from filenames
    timestamps = []
    for f in files:
        match = re.search(r'frame_(\d+\.\d+)\.jpg', f)
        if match:
            timestamps.append(match.group(1))

    # Find the maximum length of the timestamp string
    max_len = max(len(ts) for ts in timestamps)

    for f in files:
        match = re.search(r'frame_(\d+\.\d+)\.jpg', f)
        if match:
            old_timestamp = match.group(1)
            padded_timestamp = old_timestamp.ljust(max_len, '0')
            new_name = f'frame_{padded_timestamp}.jpg'
            old_path = os.path.join(folder_path, f)
            new_path = os.path.join(folder_path, new_name)
            os.rename(old_path, new_path)
            print(f'Renamed: {f} -> {new_name}')