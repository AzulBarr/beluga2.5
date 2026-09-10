#!/usr/bin/env python3
"""Extract timestamped online poses and evaluate them with evo (no scale fitting)."""
import argparse
import math
from pathlib import Path
import re
import shutil
import subprocess

BEST_POSE_TOPIC = '/best_pose'
GT_TOPIC = '/odometry/ground_truth'


def write_tum_line(stream, stamp_ns, position, quaternion):
    sign = "-" if stamp_ns < 0 else ""
    seconds, nanoseconds = divmod(abs(stamp_ns), 1_000_000_000)
    values = (position.x, position.y, position.z, quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite pose in recorded trajectory')
    stream.write(f'{sign}{seconds}.{nanoseconds:09d} ' + ' '.join(f'{v:.17g}' for v in values) + '\n')


def storage_identifier(bag_path):
    bag_path = Path(bag_path)
    metadata = bag_path / 'metadata.yaml'
    if metadata.exists():
        import yaml
        data = yaml.safe_load(metadata.read_text())
        value = data['rosbag2_bagfile_information']['storage_identifier']
        if not isinstance(value, str) or not value:
            raise ValueError('Invalid bag storage identifier')
        return value
    if bag_path.suffix == '.mcap':
        return 'mcap'
    if bag_path.suffix == '.db3':
        return 'sqlite3'
    raise ValueError('Bag metadata is missing; specify --storage-id explicitly if appropriate')


def write_decoded_trajectories(records, estimate_stream, reference_stream=None):
    """Shared extraction logic for decoded (topic, message) records."""
    counts = {BEST_POSE_TOPIC: 0, GT_TOPIC: 0}
    previous = {}
    for topic, message in records:
        stream = estimate_stream if topic == BEST_POSE_TOPIC else reference_stream if topic == GT_TOPIC else None
        if stream is None:
            continue
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        if topic in previous and stamp <= previous[topic]:
            raise ValueError(f'{topic}: repeated/out-of-order acquisition timestamp')
        previous[topic] = stamp
        pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        write_tum_line(stream, stamp, pose.position, pose.orientation)
        counts[topic] += 1
    if counts[BEST_POSE_TOPIC] == 0:
        raise ValueError(f'No estimates on {BEST_POSE_TOPIC}')
    if reference_stream is not None and counts[GT_TOPIC] == 0:
        raise ValueError(f'No reference on {GT_TOPIC}; supply --gt-file with verified reference data')
    return counts


def extract_trajectories(bag_dir, output_dir, gt_file_external=None, storage_id=None):
    # Lazy imports permit tests of timestamps/storage/failed-tool handling without ROS.
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    estimate, reference = output_dir/'estimated.txt', output_dir/'gt.txt'
    if gt_file_external is not None and Path(gt_file_external).resolve() == estimate.resolve():
        raise ValueError('Reference path conflicts with the estimated trajectory output')
    if gt_file_external is not None and not Path(gt_file_external).is_file():
        raise ValueError('External reference file does not exist')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id=storage_id or storage_identifier(bag_dir)),
                ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    def records():
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == BEST_POSE_TOPIC or (gt_file_external is None and topic == GT_TOPIC):
                yield topic, deserialize_message(data, get_message(types[topic]))
    with estimate.open('w') as estimate_stream:
        if gt_file_external is None:
            with reference.open('w') as reference_stream:
                write_decoded_trajectories(records(), estimate_stream, reference_stream)
        else:
            write_decoded_trajectories(records(), estimate_stream)
    if gt_file_external is not None and Path(gt_file_external).resolve() != reference.resolve():
        shutil.copyfile(gt_file_external, reference)
    return reference, estimate


def compute_evo_rmse(gt_file, est_file, max_time_difference=.05, estimate_time_offset=0.0):
    if not math.isfinite(max_time_difference) or max_time_difference <= 0 or not math.isfinite(estimate_time_offset):
        raise ValueError('Invalid timestamp association settings')
    command = ['evo_ape', 'tum', str(gt_file), str(est_file), '-a', '-r', 'trans_part',
               '--t_max_diff', str(max_time_difference), '--t_offset', str(estimate_time_offset)]
    result = subprocess.run(command, capture_output=True, text=True)
    output = result.stdout + '\n' + result.stderr
    if result.returncode != 0:
        raise RuntimeError(f'evo_ape failed ({result.returncode}):\n{output}')
    match = re.search(r'\brmse\s+([0-9.eE+-]+)', output, re.IGNORECASE)
    if match is None:
        raise RuntimeError('Could not extract RMSE from evo output:\n'+output)
    rmse = float(match.group(1))
    if not math.isfinite(rmse) or rmse < 0:
        raise RuntimeError('Invalid RMSE returned by evo')
    return rmse, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_dir'); parser.add_argument('output_dir', type=Path)
    parser.add_argument('--gt-file', type=Path)
    parser.add_argument('--storage-id')
    parser.add_argument('--t-max-diff', type=float, default=.05, help='Maximum timestamp mismatch in seconds')
    parser.add_argument('--t-offset', type=float, default=0, help='Known seconds ADDED to estimate timestamps; not fitted')
    args = parser.parse_args()
    try:
        reference, estimate = extract_trajectories(args.bag_dir, args.output_dir, args.gt_file, args.storage_id)
        rmse, output = compute_evo_rmse(reference, estimate, args.t_max_diff, args.t_offset)
        (args.output_dir/'rmse.txt').write_text(f'{rmse:.9g}\n')
        (args.output_dir/'rmse_full.txt').write_text(output)
        print(f'RMSE = {rmse:.6f} m; max timestamp difference = {args.t_max_diff:g} s')
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
