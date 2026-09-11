#!/usr/bin/env python3
"""Evaluate a planar trajectory in an explicitly selected physical body frame.

Uses the node's per-scan TF sidecar, never an offset fitted to ground truth.
Example: --estimate mit_final.csv --reference gt.txt
         --reference-frame base_laser_link --output-dir mit_evaluation
The frame name is a user assertion about the provenance of the reference file.
It cannot be inferred from the eight numeric columns of a TUM trajectory.
"""
import argparse
import csv
import json
import math
from pathlib import Path

from evaluate_trajectory import (
    Pose, compare, load_estimate, load_tum, nanoseconds, split_column, validate,
    wrap, write_tum,
)


def frame_name(value):
    value = value.strip().lstrip('/')
    if not value or any(ch.isspace() for ch in value):
        raise ValueError('Invalid or empty body frame')
    return value


def load_extrinsics(path):
    required = {'stamp_ns', 'base_frame', 'scan_frame', 'base_from_scan_x',
                'base_from_scan_y', 'base_from_scan_yaw', 'deskewed'}
    records, frames = {}, None
    previous = None
    with Path(path).open() as stream:
        reader = csv.DictReader(stream)
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError('Expected the trajectory .frames.csv sidecar produced by the corrected node')
        for row in reader:
            stamp = int(row['stamp_ns'])
            if previous is not None and stamp <= previous:
                raise ValueError('TF timestamps must be strictly increasing')
            previous = stamp
            current = (frame_name(row['base_frame']), frame_name(row['scan_frame']))
            if frames is not None and current != frames:
                raise ValueError('Body/scan frame changed during the run; split the recording explicitly')
            frames = current
            xyz = tuple(float(row[name]) for name in
                        ('base_from_scan_x', 'base_from_scan_y', 'base_from_scan_yaw'))
            if not all(math.isfinite(value) for value in xyz):
                raise ValueError('Nonfinite scan extrinsics')
            if row['deskewed'] not in ('0', '1'):
                raise ValueError('Invalid deskew status')
            if current[0] == current[1] and any(abs(v) > 1e-10 for v in xyz):
                raise ValueError('Same-frame TF must be identity')
            records[stamp] = (*xyz, row['deskewed'] == '1')
    if not records:
        raise ValueError('No TF records; replay with final_trajectory_path enabled')
    return frames, records


def convert_to_frame(poses, frames, extrinsics, target):
    """Right-compose at EXACT scan stamps, before aligning the world frames."""
    validate(poses)
    target = frame_name(target)
    base, scan = frames
    if target not in frames:
        raise ValueError(f'Reference frame {target!r} is neither recorded base {base!r} nor scan {scan!r}')
    output = []
    for pose in poses:
        if pose.stamp not in extrinsics:
            raise ValueError(f'No TF at trajectory stamp {pose.stamp}; use the sidecar from this exact run')
        dx, dy, dyaw, _ = extrinsics[pose.stamp]
        if target == base:
            output.append(pose)
        else:
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            output.append(Pose(pose.stamp, pose.x + c*dx - s*dy,
                               pose.y + s*dx + c*dy, wrap(pose.yaw + dyaw)))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--estimate', required=True, help='Final CSV (optionally #online/#optimized) or TUM; CSV defaults to optimized')
    parser.add_argument('--extrinsics', type=Path, help='Defaults to <estimate path>.frames.csv')
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--reference-frame', required=True, help='Physical body frame of the reference, e.g. base_laser_link; NOT its world frame')
    parser.add_argument('--reference-time-offset', default='0', help='Known offset added to reference timestamps; never fitted')
    parser.add_argument('--max-diff', type=float, default=.05)
    parser.add_argument('--alignment', choices=('se2', 'origin', 'none'), default='se2')
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    try:
        path, column = split_column(args.estimate)
        sidecar = args.extrinsics or Path(str(path) + '.frames.csv')
        frames, extrinsics = load_extrinsics(sidecar)
        estimate = load_estimate(path, column)
        target = frame_name(args.reference_frame)
        converted = convert_to_frame(estimate, frames, extrinsics, target)
        reference = load_tum(args.reference, nanoseconds(args.reference_time_offset))
        report = compare(reference, {'beluga': converted}, args.max_diff, args.alignment)
        report.update({'estimated_body_frame': frames[0], 'recorded_scan_frame': frames[1],
                       'reference_body_frame_asserted': target,
                       'estimate': str(path), 'estimate_column': column or 'optimized_if_final_csv',
                       'extrinsics_file': str(sidecar),
                       'reference_time_offset_s': args.reference_time_offset,
                       'deskewed_estimate_scans': sum(extrinsics[p.stamp][3] for p in estimate)})
        report['notes'].append('Body TFs are measured from the bag at exact scan timestamps. No extrinsics are fitted to the reference.')
        report['notes'].append('The declared reference body frame must be verified from the source of the GT file; numeric TUM data alone does not identify it.')
        outputs = [args.output_dir/name for name in ('estimated_base.tum', 'estimated_reference_frame.tum', 'metrics.json')]
        inputs = {Path(path).resolve(), args.reference.resolve(), sidecar.resolve()}
        if any(output.resolve() in inputs for output in outputs):
            raise ValueError('Output would overwrite an input; choose another output directory')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        write_tum(outputs[0], estimate)
        write_tum(outputs[1], converted)
        outputs[2].write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        metrics = report['runs']['beluga']
        print(f'Body frame: {frames[0]} -> {target}; TF from {sidecar}')
        print(f'XY APE RMSE: {metrics["position_ape_m"]["rmse"]:.6f} m')
        print(f'Yaw APE RMSE: {metrics["yaw_ape_rad"]["rmse"]:.6f} rad')
        print(f'Associated reference poses: {report["common_reference_poses"]}/{report["reference_poses"]}')
        print(f'Outputs: {args.output_dir}')
    except (ValueError, OSError, KeyError, ArithmeticError, TypeError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
