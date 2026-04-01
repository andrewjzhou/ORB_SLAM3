"""
Fix kalibr_bagcreater compatibility: remove camera frames whose nanosecond
timestamps have fewer than 10 digits (i.e. t < 1 second from SDK start).

kalibr_bagcreater splits image filenames with timestamp_nsecs[0:-9] to get
seconds, which yields an empty string for 9-digit timestamps, crashing int().

Typically only the first ~10 frames are affected. Removing them is safe for
calibration (3300+ frames remain).

Also offsets imu0.csv timestamps if needed (same 9-digit issue applies there).

Usage:
    python3 fix_timestamps.py <dataset_dir>
"""
import os, sys

MIN_DIGITS = 10  # rospy.Time split requires >= 10 digit ns timestamp


def fix_camera(cam_dir):
    times_path = os.path.join(cam_dir, 'times.txt')
    with open(times_path) as f:
        timestamps = [line.strip() for line in f if line.strip()]

    kept, removed = [], 0
    for ts in timestamps:
        if len(ts) < MIN_DIGITS:
            img = os.path.join(cam_dir, ts + '.png')
            if os.path.exists(img):
                os.remove(img)
                removed += 1
        else:
            kept.append(ts)

    with open(times_path, 'w') as f:
        f.write('\n'.join(kept) + '\n')
    print(f"  {os.path.basename(cam_dir)}: removed {removed} frames, kept {len(kept)}")


def fix_imu_csv(imu_csv_path):
    """imu0.csv has ns timestamps in column 0 (after header). Same < 10 digit issue."""
    with open(imu_csv_path) as f:
        lines = f.readlines()

    header = lines[0]
    data = []
    fixed = 0
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split(',')
        ts = parts[0]
        if len(ts) < MIN_DIGITS:
            fixed += 1
            continue  # drop this IMU sample (it's before t=1s; cameras won't use it anyway)
        data.append(line)

    with open(imu_csv_path, 'w') as f:
        f.write(header)
        f.write('\n'.join(data) + '\n')
    print(f"  imu0.csv: dropped {fixed} early samples, kept {len(data)}")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 fix_timestamps.py <dataset_dir>")
        sys.exit(1)

    d = sys.argv[1]
    print(f"Fixing {d} ...")
    fix_camera(os.path.join(d, 'cam0'))
    fix_camera(os.path.join(d, 'cam1'))

    imu_csv = os.path.join(d, 'imu0.csv')
    if os.path.exists(imu_csv):
        fix_imu_csv(imu_csv)
    else:
        print("  imu0.csv not found — run process_imu.py first")

    print("Done.")
