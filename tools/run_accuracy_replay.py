#!/usr/bin/env python3
"""Replay every Intel scan through the installed production C++ core, then zip diagnostics.

No ROS transport, no omitted scans and no reference input to SLAM. Optional
reference evaluation happens only after the executable exits. Standard library.
"""
import argparse
import csv
import datetime
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'belugaslam_example/bags/intel'))
from carmen_reader import load_ordered_flaser
from evaluate_trajectory import load_tum, load_performance, compare, nanoseconds, write_tum
from summarize_loop_verification import summarize


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def logger_offset(path):
    """Read the clock relation from measurements, never estimate it from poses."""
    offsets = set()
    with Path(path).open() as stream:
        for line in stream:
            t = line.split()
            if t and t[0] == 'FLASER':
                n = int(t[1])
                offsets.add(Decimal(t[n+8]) - Decimal(t[n+10]))
    if len(offsets) != 1 or not next(iter(offsets)).is_finite():
        raise ValueError('Acquisition-minus-logger time is not a single constant; supply a reference in acquisition time')
    return next(iter(offsets))


def write_input(dataset, target, limit=None):
    records, backwards = load_ordered_flaser(dataset)
    if limit:
        records = records[:limit]
    with Path(target).open('w') as stream:
        for r in records:
            values = (r.timestamp_ns, *r.odometry, len(r.ranges), *r.ranges)
            stream.write(' '.join(str(x) for x in values) + '\n')
    return records, backwards


def check_coverage(run, stamps):
    online = load_performance(run / 'performance.csv')
    optimized = load_tum(run / 'optimized_trajectory.tum')
    if [p.stamp for p in online] != stamps or [p.stamp for p in optimized] != stamps:
        raise ValueError('Trajectories do not contain exactly every input acquisition stamp')
    return online, optimized


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=ROOT/'belugaslam_example/bags/intel/intel.clf')
    parser.add_argument('--binary', type=Path, help='Defaults to the installed belugaslam_core executable')
    parser.add_argument('--particles', type=int, default=300)
    parser.add_argument('--hypotheses', type=int, default=4)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--loops', choices=['belief', 'map', 'geometry', 'off'], default='belief')
    parser.add_argument('--loop-update-mode', choices=['bayes', 'heuristic'], default='bayes')
    parser.add_argument('--frontend-pose-mode', choices=['frontend', 'proposal_seed', 'proposal_mean'], default='proposal_seed')
    parser.add_argument('--effective-beams', type=float, default=20.)
    parser.add_argument('--prior-information-scale', type=float, default=1.,
                        help='Matcher regularization: 1 preserves the prior; .05 is the experimental 20-beam MAP interpretation')
    parser.add_argument('--submap-scans', type=int, default=15)
    parser.add_argument('--max-scans', type=int, help='Partial smoke test ONLY; omit for RMSE claims')
    parser.add_argument('--reference', type=Path, help='Optional TUM reference; used AFTER replay only')
    parser.add_argument('--reference-clock', choices=['logger', 'acquisition'], default='acquisition')
    parser.add_argument('--output-root', type=Path, default=Path.home()/'beluga_accuracy_runs')
    parser.add_argument('--suite', action='store_true', help='Three separate fresh runs: submaps without loops, belief N30, belief at requested N')
    args = parser.parse_args()
    if args.loop_update_mode == 'bayes' and args.loops not in ('belief', 'off'):
        parser.error('Use --loop-update-mode heuristic for the legacy MAP/geometry ablations')
    if not 5 <= args.particles <= 10000 or not 1 <= args.hypotheses <= args.particles or not 1 <= args.seed < 2**32:
        parser.error('Require 5..10000 particles, 1..particles hypotheses, positive uint32 seed')
    if (not math.isfinite(args.effective_beams) or args.effective_beams <= 0 or args.submap_scans < 1 or
            not math.isfinite(args.prior_information_scale) or args.prior_information_scale <= 0):
        parser.error('Require positive effective beams and submap scan count')
    if args.max_scans is not None and args.max_scans < 3:
        parser.error('--max-scans must be at least 3')
    if not args.dataset.is_file() or (args.reference and not args.reference.is_file()):
        parser.error('Dataset or reference does not exist')
    if args.suite and (args.particles == 30 or args.hypotheses > 30):
        parser.error('For --suite use a particle count different from 30 and at most 30 hypotheses')
    try:
        if args.binary is None:
            prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'belugaslam_core'],
                                    capture_output=True, text=True, check=True).stdout.strip()
            args.binary = Path(prefix)/'lib/belugaslam_core/intel_accuracy_replay'
        args.binary = args.binary.resolve()
        if not args.binary.is_file():
            parser.error('Replay executable is missing; rebuild belugaslam_core and source install/setup.bash')
        args.output_root.mkdir(parents=True, exist_ok=True)
        folder = Path(tempfile.mkdtemp(prefix='intel_accuracy_', dir=args.output_root.resolve()))
        input_path = folder/'scans.input'
        records, backwards = write_input(args.dataset, input_path, args.max_scans)
        stamps = [r.timestamp_ns for r in records]
        status = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  'input_scans': len(records), 'partial_recording': args.max_scans is not None,
                  'backward_file_transitions': backwards, 'ground_truth_used_by_slam': False,
                  'dataset_sha256': sha(args.dataset), 'executable_sha256': sha(args.binary),
                  'source_hashes': {}, 'runs': {}}
        for base in ('belugaslam_core/include', 'tools'):
            for p in sorted((ROOT/base).rglob('*')):
                if p.is_file() and p.suffix in ('.hpp', '.cpp', '.py'):
                    status['source_hashes'][str(p.relative_to(ROOT))] = sha(p)
        settings = [('requested', args.particles, args.hypotheses, args.loops)]
        if args.suite:
            settings = [('submaps_no_lc', args.particles, 1, 'off'),
                        ('belief_n30', 30, args.hypotheses, 'belief'),
                        (f'belief_n{args.particles}', args.particles, args.hypotheses, 'belief')]
        estimates = {}
        print(f'Diagnostics: {folder}\nInput scans per run: {len(records)}', flush=True)
        try:
            for name, particles, hypotheses, loops in settings:
                run = folder/name; run.mkdir()
                command = [str(args.binary), str(input_path), str(run), str(particles), str(hypotheses),
                           str(args.seed), loops, args.frontend_pose_mode, str(args.effective_beams), str(args.submap_scans),
                           str(args.prior_information_scale), args.loop_update_mode]
                info = {'command': command, 'complete': False, 'particles': particles, 'hypotheses': hypotheses,
                        'loops': loops, 'loop_update_mode': args.loop_update_mode, 'frontend_pose_mode': args.frontend_pose_mode,
                        'effective_beams': args.effective_beams, 'submap_scans': args.submap_scans,
                        'prior_information_scale': args.prior_information_scale,
                        'seed': args.seed, 'range_max': 30., 'worker_threads': 2,
                        'alpha1': .1, 'alpha2': .05, 'alpha3': .1, 'alpha4': .05}
                status['runs'][name] = info
                (run/'parameters.json').write_text(json.dumps(info, indent=2)+'\n')
                print(f'\nStarting {name}: N={particles}, H={hypotheses}, loops={loops}', flush=True)
                with (run/'terminal.log').open('w', buffering=1) as log:
                    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                               text=True, errors='replace')
                    try:
                        for line in process.stdout:
                            log.write(line); print(line, end='', flush=True)
                        returncode = process.wait()
                    except BaseException:
                        process.terminate()
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill(); process.wait(timeout=5)
                        raise
                    finally:
                        process.stdout.close()
                info['returncode'] = returncode
                if returncode:
                    raise RuntimeError(f'{name}: replay failed with code {returncode}; retaining diagnostics')
                online, optimized = check_coverage(run, stamps)
                info['loop_audit'] = summarize(run/'loops.csv', .25)
                info['complete'] = True
                estimates[name+'/optimized'] = optimized
                estimates[name+'/online'] = online
            if args.reference:
                offset = logger_offset(args.dataset) if args.reference_clock == 'logger' else Decimal(0)
                reference = load_tum(args.reference, nanoseconds(offset))
                if args.max_scans:
                    reference = [p for p in reference if stamps[0] <= p.stamp <= stamps[-1]]
                report = compare(reference, estimates, max_diff_s=.01, alignment='se2')
                report.update(reference_clock=args.reference_clock, reference_offset_s=str(offset),
                              reference_sha256=sha(args.reference),
                              reference_provenance='User-supplied reference; independent ground-truth accuracy not established')
                (folder/'rmse_comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
                write_tum(folder/'reference_acquisition.tum', reference)
                for name, metrics in report['runs'].items():
                    print(f'{name}: APE XY RMSE = {metrics["position_ape_m"]["rmse"]:.6f} m', flush=True)
                print(f'Common reference coverage: {report["common_reference_fraction"]:.2%}', flush=True)
        except (Exception, KeyboardInterrupt) as error:
            status['error'] = str(error) or 'Interrupted'
            print(f'Diagnostic error: {status["error"]}', file=sys.stderr)
        status['complete'] = bool(status['runs']) and all(r['complete'] for r in status['runs'].values()) and not status.get('error')
        status['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        (folder/'run_status.json').write_text(json.dumps(status, indent=2, allow_nan=False)+'\n')
        archive = folder.with_suffix('.zip')
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob('*')):
                if path.is_file() and path != input_path:
                    z.write(path, Path(folder.name)/path.relative_to(folder))
        input_path.unlink()
        print(f'\nSEND THIS ZIP:\n{archive}\nComplete: {status["complete"]}', flush=True)
        return 0 if status['complete'] else 1
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    sys.exit(main())
