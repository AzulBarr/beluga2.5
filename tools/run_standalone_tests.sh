#!/usr/bin/env bash
# Dependency-free numerical checks; this does NOT replace a ROS/Ceres build.
set -euo pipefail
beluga_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
beluga_test_dir="$(mktemp -d)"
trap 'rm -rf "$beluga_test_dir"' EXIT
for beluga_test_name in odometry_tracking_prior probability_matching hierarchical_bayes tracking_accuracy motion_filter loop_belief output_selection pose_graph_residual proposal_pose robust_tracking late_run grid_update loop_search; do
  "${CXX:-g++}" -std=c++17 -O2 -pthread -I "$beluga_source_root/belugaslam_core/include" \
    "$beluga_source_root/belugaslam_core/test/${beluga_test_name}_test.cpp" \
    -o "$beluga_test_dir/$beluga_test_name"
  "$beluga_test_dir/$beluga_test_name"
done
python3 -m unittest discover -s "$beluga_source_root/tools" -p 'test_*.py'
