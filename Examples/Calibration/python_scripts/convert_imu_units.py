"""
Convert INDEMIND IMU files from native SDK units (g, deg/s) to SI units (m/s², rad/s).

Usage:
    python3 convert_imu_units.py <dataset_dir>

Where <dataset_dir> contains:
    IMU/acc.txt   -- format: timestamp_s,acc_x,acc_y,acc_z   (in g)
    IMU/gyro.txt  -- format: timestamp_s,gyro_x,gyro_y,gyro_z (in deg/s)

Converts in-place (overwrites originals). Run once per dataset.
"""

import sys
import math
import os

G = 9.81          # m/s² per g
DEG2RAD = math.pi / 180.0


def convert_file(path, scale):
    with open(path, 'r') as f:
        lines = f.readlines()

    out = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split(',')
        ts = parts[0]
        vals = [str(float(v) * scale) for v in parts[1:]]
        out.append(ts + ',' + ','.join(vals))

    with open(path, 'w') as f:
        f.write('\n'.join(out) + '\n')

    print(f"Converted {path}  (×{scale:.6f})")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 convert_imu_units.py <dataset_dir>")
        sys.exit(1)

    d = sys.argv[1]
    acc_path  = os.path.join(d, 'IMU', 'acc.txt')
    gyro_path = os.path.join(d, 'IMU', 'gyro.txt')

    for p in [acc_path, gyro_path]:
        if not os.path.exists(p):
            print(f"ERROR: {p} not found")
            sys.exit(1)

    convert_file(acc_path,  G)
    convert_file(gyro_path, DEG2RAD)
    print("Done. acc is now m/s², gyro is now rad/s.")
