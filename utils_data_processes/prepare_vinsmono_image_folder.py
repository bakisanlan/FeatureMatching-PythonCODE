import sys
import os
import csv
import shutil
from pathlib import Path
from decimal import Decimal, getcontext

# add parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# — set enough precision to cover integer+7-decimal digits
getcontext().prec = 20

# --- User parameters ---
src_folder      = Path("data/camera-001/output_20250505_112658")
output_csv      = Path("data/camera-001/ts/timestamps.csv")
filtered_folder = src_folder.parent / "filtered_images"

# the exact filename at which to start saving
reference_frame = "frame_1746433752.8757975.jpg"

# --- Gather and sort files by their embedded timestamp string ---
files = sorted(
    [f for f in src_folder.iterdir() if f.name.startswith("frame_")],
    key=lambda f: Decimal(f.stem.split("_")[1])
)

if not files:
    raise RuntimeError(f"No 'frame_' files found in {src_folder}")

# --- Prepare a list of (ns_timestamp, filename) for every file ---
rows = []
for f in files:
    ts_dec = Decimal(f.stem.split("_")[1])
    # no baseline subtraction → preserves exact digits
    ts_ns = int((ts_dec * Decimal("1e9")).to_integral_value())
    rows.append((ts_ns, f.name))

# --- Find where the reference_frame lands ---
try:
    start_idx = next(i for i, (_, name) in enumerate(rows) if name == reference_frame)
except StopIteration:
    raise RuntimeError(f"Reference frame '{reference_frame}' not found in folder.")

# Slice out only the rows from reference onward
filtered_rows = rows[start_idx:]

# --- Write filtered CSV if missing ---
output_csv.parent.mkdir(parents=True, exist_ok=True)
if not output_csv.exists():
    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp_ns", "filename"])
        writer.writerows(filtered_rows)
    print(f"Created and wrote {len(filtered_rows)} entries (from '{reference_frame}') to {output_csv}")
else:
    print(f"CSV '{output_csv}' already exists; skipping creation.")

# --- Copy images from reference_frame onward ---
filtered_folder.mkdir(exist_ok=True)
copied = 0
for _, fname in filtered_rows:
    src = src_folder / fname
    dst = filtered_folder / fname
    shutil.copy2(src, dst)
    copied += 1

print(f"Copied {copied} images from '{reference_frame}' onward into {filtered_folder}")
