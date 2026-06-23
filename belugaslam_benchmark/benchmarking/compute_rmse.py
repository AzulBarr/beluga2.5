#!/usr/bin/env python3

import argparse
import re
import subprocess
from pathlib import Path

from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions

from rclpy.serialization import deserialize_message

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from rosidl_runtime_py.utilities import get_message
import sys


BEST_POSE_TOPIC = "/best_pose"
GT_TOPIC = "/odometry/ground_truth"


def write_tum_line(f, stamp, pos, quat):
    f.write(
        f"{stamp:.9f} "
        f"{pos.x} {pos.y} {pos.z} "
        f"{quat.x} {quat.y} {quat.z} {quat.w}\n"
    )


def extract_trajectories(bag_dir, output_dir, gt_file_external=None):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    est_file = output_dir / "estimated.txt"
    gt_file = output_dir / "gt.txt"

    storage_options = StorageOptions(uri=str(bag_dir), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics}

    with open(est_file, "w") as est_f:
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != BEST_POSE_TOPIC:
                continue
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            write_tum_line(est_f, stamp, msg.pose.pose.position, msg.pose.pose.orientation)

    if gt_file_external is not None:
        import shutil
        shutil.copy(gt_file_external, gt_file)

    return gt_file, est_file


def compute_evo_rmse(gt_file, est_file):

    cmd = [
        "evo_ape",
        "tum",
        str(gt_file),
        str(est_file),
        "-a",
        "-r",
        "trans_part",
        "--t_max_diff",
        "0.5",
    ]

    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True
    )

    output = result.stdout + "\n" + result.stderr

    rmse_match = re.search(
        r"rmse\s+([0-9.eE+-]+)",
        output,
        re.IGNORECASE
    )

    if rmse_match is None:
        raise RuntimeError(
            "Could not extract RMSE from evo output:\n"
            + output
        )

    rmse = float(rmse_match.group(1))

    return rmse, output

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir")
    parser.add_argument("output_dir")
    parser.add_argument("--gt-file", type=Path, default=None)
    args = parser.parse_args()
    args.output_dir = Path(args.output_dir)

    gt_file, est_file = extract_trajectories(
        args.bag_dir,
        args.output_dir,
        gt_file_external=args.gt_file
    )

    rmse, evo_output = compute_evo_rmse(gt_file, est_file)

    (args.output_dir / "rmse.txt").write_text(f"{rmse:.6f}\n")
    (args.output_dir / "rmse_full.txt").write_text(evo_output)
    print(f"RMSE = {rmse:.6f} m")

if __name__ == "__main__":
    main()