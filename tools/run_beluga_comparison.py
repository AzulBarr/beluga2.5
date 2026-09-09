#!/usr/bin/env python3
"""Run one Intel verifier ablation using the already-built Beluga installation.

No SLAM source edits, rebuilding, reference trajectory, or tuning search.
Requires a sourced ROS2 environment. Python standard library only.
"""
import argparse
import collections
import csv
import datetime
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import zipfile


def launch_command(run, verifier, max_hypotheses, seed, analytic=True, polish=True,
                   particles=30, frontend_pose_mode='proposal_mean', workers=2, replay_rate=1.0):
    return [
        'ros2', 'launch', 'belugaslam_example', 'intel_dataset_belugaslam.xml',
        'use_sim_time:=true', 'record_bag:=false', f'replay_rate:={replay_rate}',
        f'random_seed:={seed}', f'worker_threads:={workers}', 'min_particles:=5',
        f'max_particles:={particles}', f'max_hypotheses:={max_hypotheses}',
        f'frontend_pose_mode:={frontend_pose_mode}',
        'enable_loop_closure:=true', 'enable_pgo:=true',
        f'pgo_analytic_jacobians:={str(analytic).lower()}',
        f'loop_robust_polish:={str(polish).lower()}',
        f'loop_verifier_mode:={verifier}', 'output_selection_mode:=pose_risk',
        f'performance_diagnostics_path:={run / "performance.csv"}',
        f'tracking_diagnostics_path:={run / "tracking.csv"}',
        f'loop_diagnostics_path:={run / "loops.csv"}',
        f'final_trajectory_path:={run / "final_trajectory.csv"}',
        f'optimized_trajectory_path:={run / "optimized_trajectory.tum"}',
    ]


def validate_map(path):
    """Validate the expanded ROS CLI OccupancyGrid YAML, without a YAML dependency.

    Accept only the block layout emitted by ros2 topic echo. In particular,
    an ellipsis is not map data, even if the CLI exits successfully.
    """
    result = {'complete': False, 'problems': []}
    try:
        lines = path.read_text().splitlines()
        start = lines.index('info:')
        data_start = lines.index('data:', start)
        info = lines[start + 1:data_start]
        def scalar(name):
            matches = [line.split(':', 1)[1].strip() for line in info
                       if line.startswith('  ' + name + ':')]
            if len(matches) != 1:
                raise ValueError('missing or duplicate map ' + name)
            return matches[0]
        width, height = int(scalar('width')), int(scalar('height'))
        resolution = float(scalar('resolution'))
        if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid map dimensions or resolution')
        result.update(width=width, height=height, resolution=resolution,
                      expected_cells=width * height, recorded_cells=0)
        for line in lines[data_start + 1:]:
            if line.strip() in ('', '---'):
                continue
            if not re.fullmatch(r'- (?:-1|[0-9]+)', line):
                raise ValueError('truncated or invalid map array entry: ' + line[:80])
            value = int(line[2:])
            if not -1 <= value <= 100:
                raise ValueError('map occupancy outside [-1, 100]')
            result['recorded_cells'] += 1
        if result['recorded_cells'] != width * height:
            raise ValueError('map cell count does not equal width times height')
    except (OSError, ValueError) as error:
        result['problems'].append(str(error))
    result['complete'] = not result['problems']
    return result


def validate_parameters(path, expected):
    """Check that the running node actually received the requested experiment."""
    problems = []
    try:
        lines = path.read_text().splitlines()
        for name, value in expected.items():
            matches = [line.split(':', 1)[1].strip().strip("'\"") for line in lines
                       if line.strip().startswith(name + ':')]
            if matches != [str(value).lower()]:
                problems.append(f'{name}: expected {value}, found {matches}')
    except OSError as error:
        problems.append(str(error))
    return {'complete': not problems, 'problems': problems}


def validate_optimized_trajectory(run):
    result = {'complete': False, 'trajectory_kind': 'retrospective_graph', 'problems': []}
    try:
        lines = (run / 'optimized_trajectory.tum').read_text().splitlines()
        header = re.fullmatch(r'# retrospective_graph selected_hypothesis=(\d+)', lines[0]) if lines else None
        if not header:
            raise ValueError('Missing selected-graph trajectory header')
        result['selected_hypothesis'] = int(header.group(1))
        stamps = []
        for line in lines[1:]:
            if not line.strip() or line.startswith('#'):
                continue
            fields = line.split()
            if len(fields) != 8:
                raise ValueError('Malformed TUM pose')
            stamp = Decimal(fields[0]) * 1_000_000_000
            if not stamp.is_finite() or stamp != stamp.to_integral_value():
                raise ValueError('Invalid nanosecond timestamp')
            values = [float(x) for x in fields[1:]]
            if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[3:])-1) > 1e-6:
                raise ValueError('Invalid TUM pose/quaternion')
            stamps.append(int(stamp))
        with (run / 'performance.csv').open(newline='') as stream:
            processed = [r for r in csv.DictReader(stream) if r['status']=='processed']
        if not stamps or stamps != [int(row['stamp_ns']) for row in processed]:
            raise ValueError('Retrospective timestamps do not match online scan coverage')
        if result['selected_hypothesis'] != int(processed[-1]['selected_hypothesis']):
            raise ValueError('Retrospective graph differs from the final selected hypothesis')
        result['poses'] = len(stamps)
    except (OSError, ValueError, KeyError, InvalidOperation) as error:
        result['problems'].append(str(error))
    result['complete'] = not result['problems']
    return result


def capture(command, output, timeout=30):
    """Bounded CLI capture. Preserve errors separately from YAML output."""
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        output.write_text(result.stdout)
        output.with_suffix(output.suffix + '.stderr.txt').write_text(result.stderr)
        return {'command': command, 'returncode': result.returncode,
                'nonempty_stdout': bool(result.stdout.strip())}
    except (OSError, subprocess.TimeoutExpired) as error:
        output.with_suffix(output.suffix + '.stderr.txt').write_text(str(error) + '\n')
        return {'command': command, 'error': str(error)}


def stop_launch(process):
    """Give ros2 launch time to stop its nodes and close diagnostic streams."""
    forced = False
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            forced = True
            # This is the new session created ONLY for this runner's launch.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)
    return {'launch_returncode': process.returncode, 'forced_termination': forced}


def validate_csvs(run, expected, verifier, particles=None):
    """Validate full Intel scan coverage after shutdown; never discard bad rows."""
    result = {'expected_input_scans': expected, 'problems': []}
    tables = {}
    for filename in ('performance.csv', 'tracking.csv', 'loops.csv'):
        try:
            with (run / filename).open(newline='') as stream:
                reader = csv.DictReader(stream)
                rows = list(reader)
                if not reader.fieldnames:
                    raise ValueError('missing CSV header')
            bad = [i + 2 for i, row in enumerate(rows)
                   if None in row or any(value is None for value in row.values())]
            if bad:
                result['problems'].append(f'{filename}: malformed rows {bad[:8]}')
            result[filename + '_rows'] = len(rows)
            tables[filename] = rows
        except (OSError, ValueError) as error:
            result['problems'].append(f'{filename}: {error}')
    if result['problems']:
        result['complete'] = False
        return result
    try:
        perf = tables['performance.csv']
        if expected is None or expected <= 0 or len(perf) != expected:
            result['problems'].append('Performance row count does not match dataset completion count')
        for i, row in enumerate(perf):
            if (row['status'] != 'processed' or int(row['received']) != i + 1 or
                    int(row['processed']) != i + 1 or row['output_selection_mode'] != 'pose_risk'):
                result['problems'].append(f'Unexpected callback, counter or output mode at scan {i}')
                break
            if particles is not None and int(row['particles']) != particles:
                result['problems'].append(f'Unexpected particle count at scan {i}')
                break
        stamps = [int(row['stamp_ns']) for row in perf]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            result['problems'].append('Non-increasing processed timestamps')
        masses = collections.defaultdict(list)
        ids = collections.defaultdict(set)
        population = collections.defaultdict(int)
        for row in tables['tracking.csv']:
            seq, hid = int(row['sequence']), int(row['hypothesis'])
            if hid in ids[seq]:
                result['problems'].append(f'Duplicate tracking hypothesis at scan {seq}')
            ids[seq].add(hid)
            mass = float(row['mass'])
            if not math.isfinite(mass) or mass < 0:
                result['problems'].append(f'Invalid tracking mass at scan {seq}')
            masses[seq].append(mass)
            if particles is not None:
                population[seq] += int(row['particles'])
        if expected is None or sorted(masses) != list(range(expected)):
            result['problems'].append('Tracking scan sequences do not match full dataset')
        if any(not math.isclose(math.fsum(v), 1, abs_tol=1e-6, rel_tol=0) for v in masses.values()):
            result['problems'].append('Incomplete or unnormalized tracking belief')
        if particles is not None and any(n != particles for n in population.values()):
            result['problems'].append('Tracking particle counts do not match the requested population')
        if any(row['verifier_mode'] != verifier for row in tables['loops.csv']):
            result['problems'].append('Logged verifier differs from requested mode')
        result['processed_scans'] = len(perf)
        result['tracking_snapshots'] = len(masses)
    except (KeyError, TypeError, ValueError) as error:
        result['problems'].append(f'CSV validation failed: {error}')
    result['complete'] = not result['problems']
    return result


def record_source_identity(workspace, run):
    source = workspace / 'src/beluga2.5'
    paths = [
        'belugaslam_core/include/belugaslam_core/fastslam_oc_grid_core.hpp',
        'belugaslam_core/include/belugaslam_core/output_selection.hpp',
        'belugaslam_core/include/belugaslam_core/proposal_pose.hpp',
        'belugaslam_core/include/belugaslam_core/submap.hpp',
        'belugaslam_core/include/belugaslam_core/pose_graph_cost.hpp',
        'belugaslam_core/include/belugaslam_core/pose_graph_residual.hpp',
        'belugaslam_core/include/belugaslam_core/loop_belief.hpp',
        'belugaslam_node/src/fastslam_oc_grid_node.cpp',
        'belugaslam_node/launch/fastslam_oc_grid.launch.py',
        'belugaslam_example/example/launch/intel_dataset_belugaslam.xml',
        'belugaslam_example/bags/intel/intel.clf',
    ]
    hashes = {}
    for name in paths:
        path = source / name
        hashes[name] = None
        if path.is_file():
            with path.open('rb') as stream:
                digest = hashlib.sha256()
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
                hashes[name] = digest.hexdigest()
    (run / 'source_and_input_hashes.json').write_text(json.dumps(hashes, indent=2) + '\n')
    if shutil.which('git') and source.is_dir():
        capture(['git', '-C', str(source), 'rev-parse', 'HEAD'], run / 'source_commit.txt')
        capture(['git', '-C', str(source), 'diff', '--stat'], run / 'source_changes.txt')
    if shutil.which('lscpu'):
        capture(['lscpu'], run / 'cpu.txt')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verifier', choices=['belief', 'map'], default='belief')
    parser.add_argument('--pgo-analytic-jacobians', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--loop-robust-polish', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--max-hypotheses', type=int, choices=[1, 4], default=4)
    parser.add_argument('--particles', type=int, default=30)
    parser.add_argument('--frontend-pose-mode', choices=['frontend', 'proposal_mean'], default='proposal_mean')
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--replay-rate', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'ros2_ws')
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'beluga_runs')
    parser.add_argument('--drain-seconds', type=float, default=15.0,
                        help='Wait after dataset completion before capture; coverage is checked after shutdown')
    args = parser.parse_args()
    if args.seed <= 0 or args.seed > 2**32 - 1 or not math.isfinite(args.drain_seconds) or args.drain_seconds < 0:
        parser.error('Use a positive uint32 seed and a finite nonnegative drain time')
    if not 5<=args.particles<=10000 or not 1<=args.workers<=64:
        parser.error('Use 5..10000 particles and 1..64 worker threads')
    if not math.isfinite(args.replay_rate) or not 0<args.replay_rate<=10:
        parser.error('Use a finite replay rate in (0, 10]')
    if not args.workspace.is_dir() or not shutil.which('ros2'):
        parser.error('Workspace missing or ROS2 not sourced; see README.md')
    # Never stop or reuse another running SLAM instance.
    try:
        existing = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired) as error:
        parser.error(f'Cannot check running ROS nodes: {error}')
    if existing.returncode != 0:
        parser.error('ros2 node list failed: ' + existing.stderr.strip())
    if '/belugaslam' in existing.stdout.splitlines():
        parser.error('An existing /belugaslam node is running. Stop the preceding launch first.')
    args.workspace = args.workspace.resolve()
    args.output_root.mkdir(parents=True, exist_ok=True)
    prefix = (f'{args.verifier}_h{args.max_hypotheses}_n{args.particles}_{args.frontend_pose_mode}_seed{args.seed}_'
              f'a{int(args.pgo_analytic_jacobians)}p{int(args.loop_robust_polish)}_')
    run = Path(tempfile.mkdtemp(prefix=prefix, dir=args.output_root.resolve()))
    command = launch_command(run, args.verifier, args.max_hypotheses, args.seed,
                             args.pgo_analytic_jacobians, args.loop_robust_polish,
                             args.particles, args.frontend_pose_mode, args.workers, args.replay_rate)
    meta = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'command': command, 'workspace': str(args.workspace), 'captures': {},
            'drain_seconds': args.drain_seconds, 'ground_truth_used': False}
    (run / 'launch_command.txt').write_text(shlex.join(command) + '\n')
    record_source_identity(args.workspace, run)
    print(f'Run directory: {run}\nThe full replay takes about {45/args.replay_rate:g} minutes. '
          'This runner captures the final map and parameters, then stops and zips automatically.\n', flush=True)
    done = threading.Event()
    state = {'expected': None}
    process = None
    reader = None
    terminal = (run / 'terminal.log').open('w', buffering=1)
    try:
        process = subprocess.Popen(command, cwd=args.workspace, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                   text=True, errors='replace', start_new_session=True)

        def copy_output():
            for line in process.stdout:
                terminal.write(line)
                print(line, end='', flush=True)
                match = re.search(r'Dataset complete: published (\d+) scans\.', line)
                if match:
                    state['expected'] = int(match.group(1))
                    done.set()

        reader = threading.Thread(target=copy_output, daemon=True)
        reader.start()
        while not done.wait(0.25):
            if process.poll() is not None:
                raise RuntimeError('Launch exited before the dataset completion message')
        print(f'\nDataset finished. Allowing {args.drain_seconds:g} seconds for queued processing...', flush=True)
        deadline = time.monotonic() + args.drain_seconds
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Launch exited before final capture')
            time.sleep(min(0.25, max(0, deadline - time.monotonic())))
        meta['captures']['parameters'] = capture(
            ['ros2', 'param', 'dump', '/belugaslam'], run / 'parameters.yaml')
        meta['captures']['map'] = capture(
            ['ros2', 'topic', 'echo', '/map', '--once', '--full-length',
             '--qos-durability', 'transient_local'], run / 'final_map_raw.yaml', timeout=60)
    except KeyboardInterrupt:
        meta['error'] = 'User interrupted the replay; archive may be incomplete'
        print('\nStopping early and retaining the available diagnostics.', flush=True)
    except Exception as error:
        meta['error'] = str(error)
        print(f'\nRun error: {error}', file=sys.stderr, flush=True)
    finally:
        if process is not None:
            meta.update(stop_launch(process))
        if reader is not None:
            reader.join(timeout=5)
            if reader.is_alive():
                meta['error'] = 'Launch output reader did not finish; inspect terminal.log'
        if reader is None or not reader.is_alive():
            terminal.close()
    meta['csv_validation'] = validate_csvs(run, state['expected'], args.verifier, args.particles)
    meta['map_validation'] = validate_map(run / 'final_map_raw.yaml')
    meta['optimized_trajectory_validation'] = validate_optimized_trajectory(run)
    meta['parameter_validation'] = validate_parameters(run / 'parameters.yaml', {
        'loop_verifier_mode': args.verifier, 'output_selection_mode': 'pose_risk',
        'pgo_analytic_jacobians': args.pgo_analytic_jacobians,
        'loop_robust_polish': args.loop_robust_polish,
        'max_hypotheses': args.max_hypotheses, 'max_particles': args.particles,
        'frontend_pose_mode': args.frontend_pose_mode, 'worker_threads': args.workers,
        'random_seed': args.seed, 'enable_pgo': True, 'enable_loop_closure': True,
    })
    captures_ok = all(meta['captures'].get(name, {}).get('returncode') == 0 and
                      meta['captures'].get(name, {}).get('nonempty_stdout')
                      for name in ('parameters', 'map'))
    meta['complete_capture'] = bool(meta['csv_validation']['complete'] and captures_ok and
                                    meta['map_validation']['complete'] and meta['parameter_validation']['complete'] and
                                    meta['optimized_trajectory_validation']['complete'] and
                                    not meta.get('error') and not meta.get('forced_termination'))
    meta['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    (run / 'run_status.json').write_text(json.dumps(meta, indent=2) + '\n')
    archive = run.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as zipped:
        for path in sorted(run.rglob('*')):
            if path.is_file():
                zipped.write(path, Path(run.name) / path.relative_to(run))
    print(f'\nUpload this file:\n{archive}\n'
          f'Complete capture: {meta["complete_capture"]}', flush=True)
    if not meta['complete_capture']:
        print('Keep and upload the ZIP even if incomplete; run_status.json records the failures.')
    return 0 if meta['complete_capture'] else 1


if __name__ == '__main__':
    sys.exit(main())
