import pandas as pd
import os
import sys
from pathlib import Path

# add parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# --- User parameters ---
INPUT_CSV  = "data/camera-001/27052025/130/timestamps.csv"    # your source CSV
OUTPUT_CSV = "data/camera-001/27052025/130/timestamps2.csv"   # where to save the modified CSV
CONSTANT   = 1746433752875790000                   # your constant value

# select csv file that will copied
select_csv = "data/camera-001/27052025/130/imu_raw_20250527_132719_846291.csv"

# --- Load the data ---
df = pd.read_csv(INPUT_CSV)

selected_df = pd.read_csv(select_csv)

# --- Modify the timestamp column ---
# row_index is 0 for the first row, 1 for the second, etc.
df['timestamp_ns'] = (selected_df['timestamp_px'] * 1e3).round().astype('uint64') # Convert to nanoseconds

# --- Save back out ---
df.to_csv(OUTPUT_CSV, index=False)

print(f"Processed {len(df)} rows. Saved modified CSV to {OUTPUT_CSV}.")
