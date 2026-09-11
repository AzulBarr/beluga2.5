#!/usr/bin/env bash
# Run from a shell with the rebuilt ROS workspace sourced. Does not modify the bag.
set -euo pipefail
beluga_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
beluga_bag="${1:-$beluga_root/belugaslam_example/bags/mit_rosbag/mit_bag_ros2}"
beluga_gt="${2:-$beluga_root/belugaslam_example/bags/mit_rosbag/gt.txt}"
# This states which physical body the GT describes. Change to base_footprint ONLY
# if the GT was already converted to the robot base. No lever arm is guessed.
beluga_reference_frame="${3:-base_laser_link}"
if ! command -v ros2 >/dev/null; then
  echo 'ROS 2 is not sourced. Source /opt/ros/humble/setup.bash and ~/ros2_ws/install/setup.bash.' >&2
  exit 1
fi
if [[ ! -d "$beluga_bag" || ! -f "$beluga_gt" ]]; then
  echo 'Usage: bash tools/run_mit_frame_test.sh [MIT_bag_directory] [GT_TUM_file] [GT_body_frame]' >&2
  echo 'The real MIT bag and GT must exist locally.' >&2
  exit 1
fi
# Validate the actual bag files, not just a metadata.yaml left by an incomplete ZIP.
python3 - "$beluga_bag" <<'PY'
from pathlib import Path
import sys
import yaml
bag = Path(sys.argv[1])
meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
files = meta['relative_file_paths']
if not files or any(not (bag/name).is_file() for name in files):
    raise SystemExit('MIT bag payload is missing: metadata.yaml alone cannot be played.')
PY
mkdir -p "$beluga_root/mit_frame_runs"
beluga_run="$(mktemp -d "$beluga_root/mit_frame_runs/run_XXXXXXXX")"
echo "Run outputs: $beluga_run"
echo "Declared GT body frame: $beluga_reference_frame"
ros2 launch belugaslam_example mit_rosbag_belugaslam.xml \
  bag_path:="$beluga_bag" record_bag:=false bag_timeout:=0 \
  final_trajectory_path:="$beluga_run/final.csv" \
  performance_diagnostics_path:="$beluga_run/performance.csv" \
  2>&1 | tee "$beluga_run/launch.log"
python3 "$beluga_root/tools/evaluate_frame_trajectory.py" \
  --reference "$beluga_gt" --reference-frame "$beluga_reference_frame" \
  --estimate "$beluga_run/final.csv" --output-dir "$beluga_run/evaluation"
echo "Finished: $beluga_run"
