#!/usr/bin/env python3
"""Diagnose and correct a body-frame offset between a reference and an estimate.

An SE(2) alignment removes a rigid transform in the WORLD. It cannot remove a
lever arm, which is fixed in the BODY and therefore rotates with the robot. A
trajectory compared against a reference expressed at a different point on the
robot keeps that offset in every metric, constant from the first sample, with no
relation to SLAM quality.

`diagnose` reports the offset without changing anything. Its fitted value is a
DIAGNOSTIC, never a correction: fitting the offset that minimises the error also
absorbs genuine SLAM error, so reporting the fitted residual as a score is
circular. `apply` takes the lever arm you measured from the robot's extrinsics
(`ros2 run tf2_ros tf2_echo <base_frame> <laser_frame>`) and rewrites the
trajectory into the other frame, which is a real change of representation.
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_trajectory import (  # noqa: E402
    Pose, align, associate, load_estimate, load_tum, nanoseconds, split_column, wrap,
    write_tum)


def transform(poses, dx, dy, dyaw):
    """Move every pose along its own body axes: p * (dx, dy, dyaw)."""
    out = []
    for p in poses:
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        out.append(Pose(p.stamp, p.x + c * dx - s * dy, p.y + s * dx + c * dy, wrap(p.yaw + dyaw)))
    return out


def paired(reference, estimate, max_diff_s, alignment):
    matches = associate(reference, estimate, nanoseconds(max_diff_s))
    if len(matches) < 3:
        raise SystemExit('Fewer than 3 associated poses; check the clocks and --max-diff')
    ref = [reference[i] for i in sorted(matches)]
    est = [estimate[matches[i]] for i in sorted(matches)]
    aligned, _ = align(ref, est, alignment)
    return ref, aligned


def residuals(ref, est):
    """Error of each estimate expressed in the reference's body frame."""
    forward, lateral, yaw = [], [], []
    for r, e in zip(ref, est):
        ex, ey = e.x - r.x, e.y - r.y
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        forward.append(c * ex + s * ey)
        lateral.append(-s * ex + c * ey)
        yaw.append(wrap(e.yaw - r.yaw))
    return forward, lateral, yaw


def rmse(forward, lateral):
    return math.sqrt(sum(f * f + l * l for f, l in zip(forward, lateral)) / len(forward))


def summarize(ref, est):
    forward, lateral, yaw = residuals(ref, est)
    mean = (statistics.fmean(forward), statistics.fmean(lateral), statistics.fmean(yaw))
    spread = tuple(statistics.pstdev(v) if len(v) > 1 else 0.0 for v in (forward, lateral, yaw))
    centred = rmse([f - mean[0] for f in forward], [l - mean[1] for l in lateral])
    return {
        'poses': len(ref),
        'mean_forward_m': mean[0], 'mean_lateral_m': mean[1], 'mean_yaw_rad': mean[2],
        'stdev_forward_m': spread[0], 'stdev_lateral_m': spread[1], 'stdev_yaw_rad': spread[2],
        'offset_magnitude_m': math.hypot(mean[0], mean[1]),
        'position_rmse_m': rmse(forward, lateral),
        'position_rmse_without_mean_offset_m': centred,
    }


def report(summary, note):
    print('poses asociadas: %d' % summary['poses'])
    print('\nerror en el marco del cuerpo:')
    print('  adelante : %+.4f m   (desv %.4f)' % (summary['mean_forward_m'], summary['stdev_forward_m']))
    print('  lateral  : %+.4f m   (desv %.4f)' % (summary['mean_lateral_m'], summary['stdev_lateral_m']))
    print('  yaw      : %+.4f rad (desv %.4f)' % (summary['mean_yaw_rad'], summary['stdev_yaw_rad']))
    print('  |offset| : %.4f m' % summary['offset_magnitude_m'])
    print('\nRMSE de posicion            : %.4f m' % summary['position_rmse_m'])
    print('RMSE sin el offset medio    : %.4f m  <-- DIAGNOSTICO, no un resultado' % summary['position_rmse_without_mean_offset_m'])
    if note:
        print('\n' + note)


VERDICT = """Un offset cuya desviacion es pequena frente a su media es un brazo de
palanca: la estimacion y la referencia estan en puntos distintos del robot.
Medi la extrinseca real con `ros2 run tf2_ros tf2_echo <base> <laser>` y usala
en `apply`. NO uses la media ajustada de arriba como correccion: fue elegida
para minimizar el error, asi que tambien absorbe error real de SLAM."""


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('diagnose', 'apply'):
        p = sub.add_parser(name)
        p.add_argument('--reference', required=True)
        p.add_argument('--estimate', required=True,
                       help='TUM, performance CSV or final-trajectory CSV (append #online or #optimized)')
        p.add_argument('--reference-time-offset', type=float, default=0.0)
        p.add_argument('--max-diff', type=float, default=0.05)
        p.add_argument('--alignment', choices=('se2', 'origin', 'none'), default='se2')
        p.add_argument('--output')
    a = sub.choices['apply']
    a.add_argument('--lever-x', type=float, required=True, help='Measured extrinsic, metres along the body x axis')
    a.add_argument('--lever-y', type=float, default=0.0)
    a.add_argument('--lever-yaw', type=float, default=0.0, help='radians')
    a.add_argument('--export-tum', help='Write the transformed estimate here')
    args = parser.parse_args()

    reference = load_tum(args.reference, nanoseconds(args.reference_time_offset))
    path, column = split_column(args.estimate)
    estimate = load_estimate(path, column)

    if args.command == 'apply':
        estimate = transform(estimate, args.lever_x, args.lever_y, args.lever_yaw)
        if args.export_tum:
            write_tum(args.export_tum, estimate)

    ref, est = paired(reference, estimate, args.max_diff, args.alignment)
    summary = summarize(ref, est)
    if args.command == 'apply':
        summary['applied_lever'] = {'x': args.lever_x, 'y': args.lever_y, 'yaw': args.lever_yaw}
    report(summary, VERDICT if args.command == 'diagnose' else None)
    if args.output:
        Path(args.output).write_text(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
