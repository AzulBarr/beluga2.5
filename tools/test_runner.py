import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from run_beluga_comparison import launch_command, validate_csvs, validate_map, validate_parameters, validate_optimized_trajectory


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name)
        self.write('performance.csv', [
            {'received': i + 1, 'processed': i + 1, 'status': 'processed',
             'stamp_ns': 1000 + i, 'output_selection_mode': 'pose_risk', 'selected_hypothesis': 0, 'particles': 30} for i in range(2)])
        self.write('tracking.csv', [
            {'sequence': i, 'hypothesis': h, 'mass': mass, 'particles': 15}
            for i in range(2) for h, mass in [(0, .6), (1, .4)]])
        self.write('loops.csv', [{'verifier_mode': 'map'}])

    def write(self, name, rows):
        with (self.run / name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)

    def test_complete(self):
        self.assertTrue(validate_csvs(self.run, 2, 'map')['complete'])

    def test_actual_particle_counts(self):
        self.assertTrue(validate_csvs(self.run, 2, 'map', 30)['complete'])
        self.assertFalse(validate_csvs(self.run, 2, 'map', 300)['complete'])

    def test_hundreds_of_particles_and_pose_mode_forwarded(self):
        command=launch_command(self.run,'belief',4,42,particles=300,
                               frontend_pose_mode='proposal_mean',workers=4,replay_rate=.5)
        for arg in ('max_particles:=300','worker_threads:=4','replay_rate:=0.5','frontend_pose_mode:=proposal_mean'):
            self.assertIn(arg,command)
        a=launch_command(self.run,'belief',4,42,frontend_pose_mode='frontend')
        b=launch_command(self.run,'belief',4,42,frontend_pose_mode='proposal_mean')
        self.assertEqual([(x,y) for x,y in zip(a,b) if x!=y],
                         [('frontend_pose_mode:=frontend','frontend_pose_mode:=proposal_mean')])

    def trajectory(self, body=None, hypothesis=0):
        path=self.run/'optimized_trajectory.tum'
        if body is None:
            body='0.000001000 0 0 0 0 0 0 1\n0.000001001 1 0 0 0 0 0 1\n'
        path.write_text(f'# retrospective_graph selected_hypothesis={hypothesis}\n'+body)
        return validate_optimized_trajectory(self.run)

    def test_retrospective_trajectory_coverage_and_identity(self):
        self.assertTrue(self.trajectory()['complete'])
        self.assertFalse(self.trajectory(hypothesis=5)['complete'])

    def test_bad_retrospective_trajectory(self):
        for body in ('', '0.000001000 0 0 0 0 0 0 1\n',
                     '0.000001000 nan 0 0 0 0 0 1\n',
                     '0.000001000 0 0 0 0 0 0 2\n',
                     '0.0000010001 0 0 0 0 0 0 1\n'):
            with self.subTest(body=body):
                self.assertFalse(self.trajectory(body)['complete'])

    def test_missing_dataset_tail(self):
        self.assertFalse(validate_csvs(self.run, 3, 'map')['complete'])

    def test_unknown_expected_count(self):
        self.assertFalse(validate_csvs(self.run, None, 'map')['complete'])

    def test_truncated_csv(self):
        with (self.run / 'performance.csv').open('a') as stream:
            stream.write('3,3,processed')
        self.assertFalse(validate_csvs(self.run, 3, 'map')['complete'])

    def test_missing_hypothesis_row(self):
        self.write('tracking.csv', [{'sequence': 0, 'hypothesis': 0, 'mass': .6},
                                    {'sequence': 1, 'hypothesis': 0, 'mass': 1}])
        self.assertFalse(validate_csvs(self.run, 2, 'map')['complete'])

    def test_wrong_verifier(self):
        self.assertFalse(validate_csvs(self.run, 2, 'belief')['complete'])

    def test_one_variable_verifier_ablation(self):
        belief = launch_command(self.run, 'belief', 4, 42)
        map_only = launch_command(self.run, 'map', 4, 42)
        self.assertEqual([(a, b) for a, b in zip(belief, map_only) if a != b],
                         [('loop_verifier_mode:=belief', 'loop_verifier_mode:=map')])
        self.assertIn('max_particles:=30', map_only)
        self.assertIn('max_hypotheses:=4', map_only)
        self.assertIn('output_selection_mode:=pose_risk', map_only)

    def test_independent_backend_ablations(self):
        enabled = launch_command(self.run, 'belief', 4, 42)
        disabled = launch_command(self.run, 'belief', 4, 42, polish=False)
        self.assertEqual([(a, b) for a, b in zip(enabled, disabled) if a != b],
                         [('loop_robust_polish:=true', 'loop_robust_polish:=false')])
        disabled = launch_command(self.run, 'belief', 4, 42, analytic=False)
        self.assertEqual([(a, b) for a, b in zip(enabled, disabled) if a != b],
                         [('pgo_analytic_jacobians:=true', 'pgo_analytic_jacobians:=false')])

    def map_result(self, data, width=2, height=2, resolution=.05):
        path = self.run / 'map.yaml'
        path.write_text(f'header:\n  frame_id: map\ninfo:\n  width: {width}\n'
                        f'  height: {height}\n  resolution: {resolution}\ndata:\n' + data + '\n---\n')
        return validate_map(path)

    def test_full_map(self):
        result = self.map_result('- -1\n- 0\n- 50\n- 100')
        self.assertTrue(result['complete'], result)
        self.assertEqual(result['recorded_cells'], 4)

    def test_truncated_map_ellipsis(self):
        result = self.map_result("- -1\n- 0\n- '...'")
        self.assertFalse(result['complete'])
        self.assertEqual(result['recorded_cells'], 2)

    def test_short_long_invalid_and_empty_maps(self):
        for data in ('- 0', '- 0\n' * 5, '- -2\n- 0\n- 0\n- 0',
                     '- 101\n- 0\n- 0\n- 0', '- 0.5\n- 0\n- 0\n- 0', ''):
            with self.subTest(data=data):
                self.assertFalse(self.map_result(data)['complete'])

    def test_invalid_map_geometry_and_missing_file(self):
        for kw in ({'width': 0}, {'height': -1}, {'resolution': float('nan')}, {'resolution': 0}):
            self.assertFalse(self.map_result('- 0\n' * 4, **kw)['complete'])
        self.assertFalse(validate_map(self.run / 'missing.yaml')['complete'])

    def test_missing_or_stale_runtime_parameters(self):
        path = self.run / 'params.yaml'
        path.write_text('/belugaslam:\n  ros__parameters:\n    loop_robust_polish: true\n')
        self.assertTrue(validate_parameters(path, {'loop_robust_polish': True})['complete'])
        self.assertFalse(validate_parameters(path, {'loop_robust_polish': False})['complete'])
        self.assertFalse(validate_parameters(path, {'pgo_analytic_jacobians': True})['complete'])

    def test_automatic_capture_archives_after_shutdown_flush(self):
        # Simulate ROS CLI/process behavior only; this does not validate real ROS.
        bin_dir = self.run / 'bin'; bin_dir.mkdir()
        workspace = self.run / 'ws'; workspace.mkdir()
        fake = bin_dir / 'ros2'
        fake.write_text('#!' + sys.executable + '\n' + '''
import csv,signal,sys,time
from pathlib import Path
a=sys.argv[1:]
if a[:2]==['node','list']:sys.exit(0)
if a[:2]==['param','dump']:
 print('/belugaslam:\\n  ros__parameters:\\n    output_selection_mode: pose_risk\\n    loop_verifier_mode: map\\n    pgo_analytic_jacobians: true\\n    loop_robust_polish: true\\n    max_hypotheses: 4\\n    max_particles: 30\\n    random_seed: 42\\n    enable_pgo: true\\n    enable_loop_closure: true\\n    frontend_pose_mode: proposal_mean\\n    worker_threads: 2');sys.exit(0)
if a[:2]==['topic','echo']:
 assert '--full-length' in a
 print('info:\\n  width: 1\\n  height: 1\\n  resolution: 0.05\\ndata:\\n- 0');sys.exit(0)
if a[0]!='launch':sys.exit(2)
p=dict(s.split(':=',1) for s in a if ':=' in s)
f=Path(p['performance_diagnostics_path']).open('w',buffering=1)
t=Path(p['tracking_diagnostics_path']).open('w',buffering=1)
l=Path(p['loop_diagnostics_path']).open('w',buffering=1)
f.write('received,processed,status,stamp_ns,output_selection_mode,selected_hypothesis,particles\\n')
t.write('sequence,hypothesis,mass,particles\\n')
l.write('verifier_mode\\n'+p['loop_verifier_mode']+'\\n')
for i in range(2):
 f.write(f'{i+1},{i+1},processed,{1000+i},pose_risk,0,30\\n')
 t.write(f'{i},0,1,30\\n')
def stop(signum,frame):
 f.write('3,3,processed,1002,pose_risk,0,30\\n')
 t.write('2,0,1,30\\n')
 Path(p['optimized_trajectory_path']).write_text('# retrospective_graph selected_hypothesis=0\\n'+''.join(f'0.{1000+i:09d} 0 0 0 0 0 0 1\\n' for i in range(3)))
 for s in (f,t,l):s.close()
 print('Clean shutdown',flush=True)
 sys.exit(0)
signal.signal(signal.SIGINT,stop)
print('Dataset complete: published 3 scans.',flush=True)
while True:time.sleep(.1)
''')
        fake.chmod(0o755)
        env = dict(os.environ, PATH=str(bin_dir))
        result = subprocess.run(
            [sys.executable, str(Path(__file__).with_name('run_beluga_comparison.py')),
             '--workspace', str(workspace), '--output-root', str(self.run / 'runs'),
             '--drain-seconds', '0', '--verifier', 'map'],
            env=env, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        archives = list((self.run / 'runs').glob('*.zip'))
        self.assertEqual(len(archives), 1)
        import zipfile
        with zipfile.ZipFile(archives[0]) as zipped:
            status_name = next(n for n in zipped.namelist() if n.endswith('/run_status.json'))
            status = json.loads(zipped.read(status_name))
            self.assertTrue(status['complete_capture'])
            self.assertTrue(status['map_validation']['complete'])
            self.assertEqual(status['map_validation']['recorded_cells'], 1)
            self.assertTrue(status['parameter_validation']['complete'])
            self.assertTrue(status['optimized_trajectory_validation']['complete'])
            self.assertEqual(status['optimized_trajectory_validation']['poses'], 3)
            self.assertEqual(status['csv_validation']['processed_scans'], 3)
            terminal_name = next(n for n in zipped.namelist() if n.endswith('/terminal.log'))
            self.assertIn(b'Clean shutdown', zipped.read(terminal_name))


if __name__ == '__main__':
    unittest.main()
