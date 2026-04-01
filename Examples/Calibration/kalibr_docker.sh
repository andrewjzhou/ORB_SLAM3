#!/usr/bin/env bash
# Run any kalibr command via Docker, with ORB_SLAM3 repo mounted at /ws.
#
# Usage examples:
#   ./Examples/Calibration/kalibr_docker.sh kalibr_bagcreater --folder /ws/data/... --output-bag /ws/...
#   ./Examples/Calibration/kalibr_docker.sh kalibr_calibrate_cameras --bag /ws/... ...
#   ./Examples/Calibration/kalibr_docker.sh kalibr_calibrate_imu_camera --bag /ws/... ...
#
# All paths passed to kalibr commands must use /ws/ (maps to the repo root).
set -e
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
sudo docker run --rm \
  -v "$REPO_ROOT":/ws \
  -e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 \
  --entrypoint bash \
  kalibr \
  -c 'source /catkin_ws/devel/setup.bash && export PATH="/catkin_ws/devel/lib/kalibr:$PATH" && exec "$@"' \
  -- "$@"
