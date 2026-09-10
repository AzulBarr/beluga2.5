#!/usr/bin/env python3
"""Planar trajectory RMSE and fixed-time RPE on common reference timestamps.

Input estimates: TUM files, the recovery revision's performance CSV, or the node's
final-trajectory CSV, which holds the online and the pose-graph-optimized trajectory.
No scale fitting, automatic clock shifting, or per-segment realignment.
"""
import argparse
from bisect import bisect_left, bisect_right
import csv
from dataclasses import dataclass
from decimal import Decimal
import json
import math
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Pose:
    stamp: int
    x: float
    y: float
    yaw: float


def nanoseconds(value):
    value = Decimal(str(value))
    if not value.is_finite():
        raise ValueError('Timestamp/offset must be finite')
    return int((value * 1_000_000_000).to_integral_value())


def validate(poses):
    if not poses:
        raise ValueError('Trajectory contains no poses')
    if any(not all(math.isfinite(v) for v in (p.x, p.y, p.yaw)) for p in poses):
        raise ValueError('Nonfinite pose')
    if any(a.stamp >= b.stamp for a, b in zip(poses, poses[1:])):
        raise ValueError('Timestamps must be strictly increasing; duplicate/out-of-order poses are not silently removed')
    return poses


def load_tum(path, offset=0):
    poses = []
    with Path(path).open() as stream:
        for line in stream:
            fields = line.partition('#')[0].split()
            if not fields:
                continue
            if len(fields) != 8:
                raise ValueError(f'{path}: expected eight TUM columns')
            values = [float(v) for v in fields[1:]]
            if not all(math.isfinite(v) for v in values):
                raise ValueError(f'{path}: nonfinite TUM values')
            x, y, _, qx, qy, qz, qw = values
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if abs(norm-1) > .01:
                raise ValueError(f'{path}: invalid unit quaternion')
            qx, qy, qz, qw = [v/norm for v in (qx, qy, qz, qw)]
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            poses.append(Pose(nanoseconds(fields[0])+offset, x, y, yaw))
    return validate(poses)


def load_performance(path):
    poses = []
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not {'status', 'stamp_ns', 'output_x', 'output_y', 'output_yaw'} <= set(reader.fieldnames or []):
            raise ValueError('CSV lacks published poses; use the recovery revision or extract /best_pose from the recorded bag')
        for row in reader:
            if row['status'] == 'processed':
                poses.append(Pose(int(row['stamp_ns']), *(float(row[k]) for k in ('output_x', 'output_y', 'output_yaw'))))
    return validate(poses)


FINAL_COLUMNS = ('online', 'optimized')


def load_final(path, column='optimized'):
    """Read one trajectory out of the node's final_trajectory_path CSV."""
    if column not in FINAL_COLUMNS:
        raise ValueError(f'Column must be one of {FINAL_COLUMNS}')
    poses = []
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        needed = {'sequence', 'stamp_ns'} | {f'{column}_{k}' for k in ('x', 'y', 'yaw')}
        if not needed <= set(reader.fieldnames or []):
            raise ValueError(f'CSV lacks the {column} trajectory; expected a final_trajectory_path file')
        for row in reader:
            poses.append(Pose(int(row['stamp_ns']),
                              *(float(row[f'{column}_{k}']) for k in ('x', 'y', 'yaw'))))
    return validate(poses)


def split_column(path):
    """A trailing #online or #optimized picks one trajectory of a final CSV."""
    head, separator, column = path.rpartition('#')
    return (head, column) if separator and column in FINAL_COLUMNS else (path, None)


def load_estimate(path, column=None):
    with Path(path).open() as stream:
        fields = [f.strip() for f in stream.readline().split(',')]
    if 'optimized_x' in fields:
        return load_final(path, column or 'optimized')
    if column:
        raise ValueError(f'{path} is not a final-trajectory CSV, so #{column} does not apply')
    return load_performance(path) if 'stamp_ns' in fields else load_tum(path)


def write_tum(path, poses):
    with Path(path).open('w') as stream:
        for p in validate(poses):
            # Decimal preserves nanosecond stamps even for Unix epochs.
            stamp = Decimal(p.stamp)/Decimal(1_000_000_000)
            stream.write(f'{stamp:.9f} {p.x:.17g} {p.y:.17g} 0 0 0 {math.sin(p.yaw/2):.17g} {math.cos(p.yaw/2):.17g}\n')


def associate(reference, estimate, max_difference):
    """Greedy minimum-time-difference, one-to-one matching; no pose interpolation."""
    times = [p.stamp for p in estimate]
    candidates = []
    for i, p in enumerate(reference):
        lo = bisect_left(times, p.stamp-max_difference)
        hi = bisect_right(times, p.stamp+max_difference)
        candidates.extend((abs(times[j]-p.stamp), i, j) for j in range(lo, hi))
    candidates.sort()
    used, matches = set(), {}
    for _, i, j in candidates:
        if i not in matches and j not in used:
            matches[i] = j
            used.add(j)
    return matches


def align(reference, estimate, mode):
    angle = tx = ty = 0.0
    if mode == 'origin':
        angle = wrap(reference[0].yaw-estimate[0].yaw)
        c, s = math.cos(angle), math.sin(angle)
        tx = reference[0].x-c*estimate[0].x+s*estimate[0].y
        ty = reference[0].y-s*estimate[0].x-c*estimate[0].y
    elif mode == 'se2':
        ax = sum(p.x for p in estimate)/len(estimate)
        ay = sum(p.y for p in estimate)/len(estimate)
        bx = sum(p.x for p in reference)/len(reference)
        by = sum(p.y for p in reference)/len(reference)
        dot = sum((p.x-ax)*(q.x-bx)+(p.y-ay)*(q.y-by) for p, q in zip(estimate, reference))
        cross = sum((p.x-ax)*(q.y-by)-(p.y-ay)*(q.x-bx) for p, q in zip(estimate, reference))
        if math.hypot(dot, cross) < 1e-12:
            raise ValueError('SE(2) positional alignment is degenerate; specify origin/none if appropriate')
        angle = math.atan2(cross, dot)
        c, s = math.cos(angle), math.sin(angle)
        tx, ty = bx-c*ax+s*ay, by-s*ax-c*ay
    elif mode != 'none':
        raise ValueError('Alignment must be se2, origin or none')
    c, s = math.cos(angle), math.sin(angle)
    return [Pose(p.stamp, c*p.x-s*p.y+tx, s*p.x+c*p.y+ty, wrap(p.yaw+angle)) for p in estimate], dict(x=tx, y=ty, yaw=angle, scale=1)


def stats(values):
    if not values:
        return {'count': 0}
    values = sorted(values)
    position = (len(values)-1)*.95
    lo, hi = int(position), min(int(position)+1, len(values)-1)
    return {'count': len(values), 'rmse': math.sqrt(sum(v*v for v in values)/len(values)),
            'p95': values[lo]+(values[hi]-values[lo])*(position-lo), 'max': values[-1]}


def relative(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    x, y = b.x-a.x, b.y-a.y
    return c*x+s*y, -s*x+c*y, wrap(b.yaw-a.yaw)


def compare(reference, estimates, max_diff_s=.05, alignment='se2', rpe_delta_s=1.0):
    if not estimates:
        raise ValueError('At least one estimate is required')
    if not math.isfinite(max_diff_s) or not 0 < max_diff_s <= 2 or not math.isfinite(rpe_delta_s) or rpe_delta_s <= 0:
        raise ValueError('Require 0 < max-diff <= 2 seconds and rpe-delta > 0')
    validate(reference)
    for trajectory in estimates.values():
        validate(trajectory)
    tolerance = nanoseconds(max_diff_s)
    maps = {name: associate(reference, poses, tolerance) for name, poses in estimates.items()}
    common = sorted(set.intersection(*(set(mapping) for mapping in maps.values())))
    if len(common) < 3:
        raise ValueError(f'Fewer than three common timestamps. Check clock origins, units and coverage. Per-run matches: {dict((k,len(v)) for k,v in maps.items())}')
    truth = [reference[i] for i in common]
    times = [p.stamp for p in truth]
    pairs = []
    for i, stamp in enumerate(times):
        target = stamp+nanoseconds(rpe_delta_s)
        j = bisect_left(times, target)
        choices = [k for k in (j-1, j) if i < k < len(times)]
        if choices:
            k = min(choices, key=lambda k: abs(times[k]-target))
            if abs(times[k]-target) <= tolerance:
                pairs.append((i, k))
    report = {'reference_poses': len(reference), 'common_reference_poses': len(common),
              'common_reference_fraction': len(common)/len(reference), 'alignment': alignment,
              'max_timestamp_difference_s': max_diff_s, 'rpe_delta_s': rpe_delta_s,
              'common_time_range_s': [str(Decimal(times[0])/1_000_000_000), str(Decimal(times[-1])/1_000_000_000)], 'runs': {},
              'notes': ['All runs use the same reference indices; missing poses and coverage are reported separately.',
                        'Metrics are planar XY translation and wrapped yaw. No scale fitting or per-segment realignment.',
                        'These are online estimates if /best_pose is used. Compare the same trajectory type and body frame for every method.',
                        'The optimized column of a final-trajectory CSV is the trajectory after pose graph optimisation, read at the end of the run; it is not what the robot could have known online.',
                        'Time synchronization is explicit; reference provenance and independent ground-truth accuracy must be established.',
                        'Low matched-pose RMSE does not excuse missing coverage or establish map accuracy.']}
    for name, poses in estimates.items():
        matched = [poses[maps[name][i]] for i in common]
        aligned, transform = align(truth, matched, alignment)
        errors = [math.hypot(a.x-b.x, a.y-b.y) for a, b in zip(aligned, truth)]
        angles = [abs(wrap(a.yaw-b.yaw)) for a, b in zip(aligned, truth)]
        rpe_t, rpe_a = [], []
        for i, j in pairs:
            ax, ay, aa = relative(aligned[i], aligned[j])
            bx, by, ba = relative(truth[i], truth[j])
            rpe_t.append(math.hypot(ax-bx, ay-by)); rpe_a.append(abs(wrap(aa-ba)))
        used = len(maps[name])
        worst = sorted(range(len(errors)), key=lambda i: errors[i], reverse=True)[:10]
        report['runs'][name] = {
            'estimate_poses': len(poses), 'matched_reference_poses': used,
            'unmatched_reference_poses': len(reference)-used, 'unmatched_estimate_poses': len(poses)-used,
            'alignment_transform': transform,
            'association_dt_s': stats([abs(a.stamp-b.stamp)/1e9 for a, b in zip(matched, truth)]),
            'position_ape_m': stats(errors), 'yaw_ape_rad': stats(angles),
            'translation_rpe_m': stats(rpe_t), 'rotation_rpe_rad': stats(rpe_a),
            'position_ape_by_third_m': {label: stats(errors[k*len(errors)//3:(k+1)*len(errors)//3]) for k, label in enumerate(('early', 'middle', 'late'))},
            'worst_reference_samples': [{'reference_index': common[i], 'stamp_ns': truth[i].stamp, 'position_error_m': errors[i], 'yaw_error_rad': angles[i]} for i in worst]}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('export-csv')
    export.add_argument('csv'); export.add_argument('tum')
    final = commands.add_parser('export-final')
    final.add_argument('csv'); final.add_argument('tum')
    final.add_argument('--column', choices=FINAL_COLUMNS, default='optimized')
    evaluate = commands.add_parser('compare')
    evaluate.add_argument('--reference', required=True)
    evaluate.add_argument('--estimate', action='append', required=True, help='Unique name=path (TUM, performance CSV, or final-trajectory CSV; append #online or #optimized to pick one of its two trajectories)')
    evaluate.add_argument('--reference-time-offset', default='0', help='Known offset in seconds ADDED to reference stamps; never fitted')
    evaluate.add_argument('--max-diff', type=float, default=.05)
    evaluate.add_argument('--alignment', choices=('se2', 'origin', 'none'), default='se2')
    evaluate.add_argument('--rpe-delta', type=float, default=1.0)
    evaluate.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        if args.command == 'export-csv':
            poses = load_performance(args.csv); write_tum(args.tum, poses)
            print(f'Exported {len(poses)} online poses to {args.tum}')
        elif args.command == 'export-final':
            poses = load_final(args.csv, args.column); write_tum(args.tum, poses)
            print(f'Exported {len(poses)} {args.column} poses to {args.tum}')
        else:
            estimates = {}
            for item in args.estimate:
                name, separator, path = item.partition('=')
                if not separator or not name or name in estimates:
                    raise ValueError('Estimates require unique name=path entries')
                estimates[name] = load_estimate(*split_column(path))
            report = compare(load_tum(args.reference, nanoseconds(args.reference_time_offset)), estimates,
                             args.max_diff, args.alignment, args.rpe_delta)
            report['reference_time_offset_s'] = args.reference_time_offset
            text = json.dumps(report, indent=2, allow_nan=False)+'\n'
            if args.output:
                args.output.write_text(text)
            print(text, end='')
    except (ValueError, OSError, KeyError, ArithmeticError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
