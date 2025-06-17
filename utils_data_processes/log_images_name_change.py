#!/usr/bin/env python3

import os
import sys
import csv
import shutil
from pathlib import Path
from decimal import Decimal, getcontext

import pandas as pd
import numpy as np
import cv2
from scipy.interpolate import interp1d

from normalize_filenames import normalize_filenames

def main():
    # -----------------------------------------------------------------------------
    # 1. Configuration & Paths
    # -----------------------------------------------------------------------------
    # Ensure parent directory is on PYTHONPATH for imports
    sys.path.append(str(Path(__file__).resolve().parent.parent))

    # Base folder for this run
    rel_path = Path("data/camera-001/29052025/50-30_noyaw")
    src_folder = rel_path / "output_19700101_020135_544881"
    select_csv  = rel_path / "imu_raw_19700101_020135_545336.csv"
    IMU_CSV     = rel_path / "imudata_matlab.csv"
    OUTPUT_CSV  = rel_path / "timestamps.csv"
    MERGED_IMU_CSV = rel_path / "imu_merged.csv"

    # -----------------------------------------------------------------------------
    # 2. Normalize existing image filenames
    # -----------------------------------------------------------------------------
    normalize_filenames(src_folder)

    # -----------------------------------------------------------------------------
    # 3. Gather image-based timestamps & write initial VINS CSV
    # -----------------------------------------------------------------------------
    getcontext().prec = 20  # high precision for Decimal → nanosecond conversion

    # find and sort all files named "frame_<timestamp>"
    frame_files = sorted(
        [f for f in src_folder.iterdir() if f.name.startswith("frame_")],
        key=lambda f: Decimal(f.stem.split("_")[1])
    )
    if not frame_files:
        raise RuntimeError(f"No 'frame_' files found in {src_folder}")

    # build rows of (ns_timestamp, filename)
    rows = []
    for f in frame_files:
        ts_dec = Decimal(f.stem.split("_")[1])
        ts_ns  = int((ts_dec * Decimal("1e9")).to_integral_value())
        rows.append((ts_ns, f.name))

    # write to CSV if not already present
    output_csv = src_folder.parent / "timestamps.csv"
    output_csv.parent.mkdir(exist_ok=True, parents=True)
    if not output_csv.exists():
        with open(output_csv, "w", newline="") as cf:
            writer = csv.writer(cf)
            writer.writerow(["timestamp_ns", "filename"])
            writer.writerows(rows)
        print(f"Created initial timestamps CSV: {output_csv}")
    else:
        print(f"Initial CSV already exists, skipping creation: {output_csv}")

    # -----------------------------------------------------------------------------
    # 4. Load & synchronize df with selected_df (crop + drop NaNs)
    # -----------------------------------------------------------------------------
    df          = pd.read_csv(output_csv)
    selected_df = pd.read_csv(select_csv)

    # crop to equal length
    min_len = min(len(df), len(selected_df))
    df = df.iloc[:min_len].reset_index(drop=True)
    selected_df = selected_df.iloc[:min_len].reset_index(drop=True)

    # drop any rows with NaNs
    combined = pd.concat([df, selected_df], axis=1).dropna().reset_index(drop=True)
    df          = combined[df.columns]
    selected_df = combined[selected_df.columns]

    # compute new timestamp_ns from selected_df and overwrite
    df["timestamp_ns"] = (
        (selected_df["timestamp_px"] * 1e3)
        .round()
        .astype("uint64")
    )
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"Synchronized and saved {len(df)} rows to {OUTPUT_CSV}")

    # -----------------------------------------------------------------------------
    # 5. Deduplicate timestamps
    # -----------------------------------------------------------------------------
    before = len(df)
    df_unique = df.drop_duplicates(subset="timestamp_ns", keep="first").reset_index(drop=True)
    after = len(df_unique)
    dup_count = before - after
    if dup_count:
        dup_vals = df["timestamp_ns"][df["timestamp_ns"].duplicated()].unique()
        print(f"Removed {dup_count} duplicate timestamp(s): {dup_vals.tolist()}")
    else:
        print("No duplicate timestamps found.")

    # -----------------------------------------------------------------------------
    # 6. Trim images to match unique timestamps
    # -----------------------------------------------------------------------------
    image_files = sorted(
        [f for f in src_folder.iterdir() if f.name.startswith("frame_") and f.suffix == ".jpg"],
        key=lambda f: Decimal(f.stem.split("_")[1])
    )

    # delete excess images
    if len(image_files) > len(df_unique):
        excess = len(image_files) - len(df_unique)
        print(f"Deleting {excess} extra image(s).")
        for f in image_files[-excess:]:
            f.unlink()
        image_files = image_files[:-excess]
    # or warn if too few
    elif len(image_files) < len(df_unique):
        print(f"⚠️ {len(image_files)} images vs {len(df_unique)} timestamps. Truncating timestamps.")
        df_unique = df_unique.iloc[: len(image_files)]

    # -----------------------------------------------------------------------------
    # 7. Convert, resize, and save images as grayscale PNGs
    # -----------------------------------------------------------------------------
    for img_path, ts_ns in zip(image_files, df_unique["timestamp_ns"]):
        gray = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"❌ Corrupt or unreadable: {img_path}. Deleting.")
            img_path.unlink(missing_ok=True)
            continue

        # resize and save
        resized = cv2.resize(gray, (960, 540), interpolation=cv2.INTER_AREA)
        out_path = img_path.with_name(f"{ts_ns}.png")
        cv2.imwrite(str(out_path), resized)
        img_path.unlink()  # remove original .jpg

    print(f"Processed {len(image_files)} images → grayscale 960×540 PNGs.")

    # -----------------------------------------------------------------------------
    # 8. Clean CSV to match actual PNGs
    # -----------------------------------------------------------------------------
    existing_pngs = {f.stem for f in src_folder.iterdir() if f.suffix == ".png"}
    df_synced = df_unique[df_unique["timestamp_ns"].astype(str).isin(existing_pngs)].reset_index(drop=True)
    removed = len(df_unique) - len(df_synced)
    if removed:
        print(f"Removed {removed} timestamp row(s) without a PNG.")
    else:
        print("All timestamps have matching PNGs.")
    df_synced.to_csv(OUTPUT_CSV, index=False)
    print(f"Cleaned CSV saved to {OUTPUT_CSV}")

    # -----------------------------------------------------------------------------
    # 9. Synchronize start times between images and IMU
    # -----------------------------------------------------------------------------
    imu_df = pd.read_csv(IMU_CSV)
    imu_df["timestamp_ns"] = imu_df["timestamp_ns"].astype("uint64")
    df_synced["timestamp_ns"] = df_synced["timestamp_ns"].astype("uint64")

    first_img = df_synced["timestamp_ns"].iat[0]
    first_imu = imu_df["timestamp_ns"].iat[0]
    print(f"First image ts: {first_img}, first IMU ts: {first_imu}")

    if first_imu > first_img:
        # IMU starts later → drop early images
        df_synced = df_synced[df_synced["timestamp_ns"] >= first_imu].reset_index(drop=True)
        cnt = 0
        for f in list(src_folder.iterdir()):
            stem = f.stem
            if f.suffix == ".png" and int(stem) < first_imu:
                f.unlink(missing_ok=True)
                cnt += 1
        print(f"Deleted {cnt} early PNG(s) before IMU data.")
    elif first_img > first_imu:
        # Images start later → drop early IMU rows
        imu_df = imu_df[imu_df["timestamp_ns"] >= first_img].reset_index(drop=True)
        print(f"Trimmed IMU to {len(imu_df)} rows starting at image timestamp.")

    # save synchronized files
    df_synced.to_csv(OUTPUT_CSV, index=False)
    imu_df.to_csv(IMU_CSV, index=False)
    print(f"Synchronized CSVs saved: {OUTPUT_CSV}, {IMU_CSV}")

    # -----------------------------------------------------------------------------
    # 10. Merge & interpolate IMU for image timestamps
    # -----------------------------------------------------------------------------
    img_ts = df_synced["timestamp_ns"].values
    imu_ts = imu_df["timestamp_ns"].values
    imu_cols = imu_df.columns.drop("timestamp_ns")

    # build interpolators
    interpolators = {
        col: interp1d(imu_ts, imu_df[col].values, kind="linear",
                      fill_value="extrapolate", bounds_error=False)
        for col in imu_cols
    }

    # find missing image timestamps
    missing_ts = [ts for ts in np.unique(img_ts) if ts not in set(imu_ts)]

    # interpolate only missing timestamps
    new_rows = []
    for ts in missing_ts:
        row = {"timestamp_ns": ts}
        for col, f in interpolators.items():
            row[col] = float(f(ts))
        new_rows.append(row)

    interp_df = pd.DataFrame(new_rows)
    merged_imu = pd.concat([imu_df, interp_df], ignore_index=True)
    merged_imu.sort_values("timestamp_ns", inplace=True)
    merged_imu.reset_index(drop=True, inplace=True)
    merged_imu.to_csv(MERGED_IMU_CSV, index=False)
    print(f"Merged IMU: {len(imu_df)} original + {len(interp_df)} interpolated = {len(merged_imu)} rows")

    # -----------------------------------------------------------------------------
    # 11. Finalize OUTPUT_CSV with filename column
    # -----------------------------------------------------------------------------
    final = df_synced[["timestamp_ns"]].copy()
    final["filename"] = final["timestamp_ns"].astype(str) + ".png"
    final.to_csv(OUTPUT_CSV, index=False)
    print(f"Final timestamps CSV written with filenames: {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
