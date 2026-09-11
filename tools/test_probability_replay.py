"""Exercise the A/B driver with a fake executable; this is not a SLAM test."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile
from run_beluga_comparison import launch_command

SCRIPT = Path(__file__).with_name('run_accuracy_replay.py')


class ProbabilityReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.data = self.root/'scans.clf'
        self.data.write_text(''.join(f'FLASER 2 1 2 0 0 0 {i*.01} 0 0 {1000+i*.1:.1f} host {i*.1:.1f}\n'
                                     for i in range(3)))
        self.binary = self.root/'fake_replay'
        self.binary.write_text('''#!/usr/bin/env python3
import json,sys
from pathlib import Path
from decimal import Decimal
assert len(sys.argv)==19 and sys.argv[18] in ("on","off"), sys.argv
source,out=Path(sys.argv[1]),Path(sys.argv[2])
stamps=[int(row.split()[0]) for row in source.read_text().splitlines()]
(out/'arguments.json').write_text(json.dumps(sys.argv[3:]))
(out/'performance.csv').write_text('stamp_ns,status,output_x,output_y,output_yaw\\n'+''.join(f'{t},processed,{i},0,0\\n' for i,t in enumerate(stamps)))
(out/'optimized_trajectory.tum').write_text(''.join(f'{Decimal(t)/Decimal(10**9):.9f} {i} 0 0 0 0 0 1\\n' for i,t in enumerate(stamps)))
(out/'loops.csv').write_text('candidate_id\\n')
''')
        self.binary.chmod(0o755)

    def invoke(self, *extra):
        return subprocess.run([sys.executable, str(SCRIPT), '--dataset', str(self.data),
                               '--binary', str(self.binary), '--output-root', str(self.root/'runs'),
                               '--loops', 'off', '--frontend-pose-mode', 'frontend', *extra],
                              text=True, capture_output=True, timeout=20)

    def test_two_complete_runs_differ_only_in_matcher_and_output(self):
        result = self.invoke('--matcher-comparison')
        self.assertEqual(result.returncode, 0, result.stderr+result.stdout)
        status_path = next((self.root/'runs').glob('*/run_status.json'))
        status = json.loads(status_path.read_text())
        self.assertTrue(status['complete'])
        self.assertEqual(set(status['runs']), {'distance', 'probability_ceres'})
        commands = [status['runs'][m]['command'] for m in ('distance', 'probability_ceres')]
        self.assertEqual([i for i,(a,b) in enumerate(zip(*commands)) if a!=b], [2,12])
        for mode in ('distance','probability_ceres'):
            self.assertEqual(status['runs'][mode]['tracking_matcher'],mode)
        with zipfile.ZipFile(status_path.parent.with_suffix('.zip')) as archive:
            self.assertEqual(sum(n.endswith('parameters.json') for n in archive.namelist()),2)
            self.assertFalse(any(n.endswith('scans.input') for n in archive.namelist()))

    def test_scan_proposal_mode_is_forwarded_and_recorded(self):
        result = self.invoke('--scan-informed-proposal', 'off')
        self.assertEqual(result.returncode, 0, result.stderr+result.stdout)
        status = json.loads(next((self.root/'runs').glob('*/run_status.json')).read_text())
        run = status['runs']['requested']
        self.assertEqual(run['command'][18], 'off')
        self.assertEqual(run['scan_informed_proposal'], 'off')

    def test_comparison_rejects_a_confounded_pose_readout(self):
        result = self.invoke('--matcher-comparison','--frontend-pose-mode','proposal_mean')
        self.assertEqual(result.returncode,2)
        self.assertIn('requires --frontend-pose-mode frontend',result.stderr)

    def test_invalid_voxel_size_is_rejected_before_starting(self):
        result = self.invoke('--tracking-voxel-size','nan')
        self.assertEqual(result.returncode,2)
        self.assertFalse((self.root/'runs').exists())

    def test_ros_runner_forwards_selected_matcher(self):
        command = launch_command(self.root,'belief',4,42,tracking_matcher='probability_ceres')
        self.assertIn('tracking_matcher:=probability_ceres',command)

    def test_prior_comparison_preserves_step_one_and_changes_only_prior_mode(self):
        result = self.invoke('--prior-comparison','--tracking-matcher','probability_ceres')
        self.assertEqual(result.returncode,0,result.stderr+result.stdout)
        status = json.loads(next((self.root/'runs').glob('*/run_status.json')).read_text())
        self.assertTrue(status['complete'])
        runs = [status['runs'][m] for m in ('ceres_fixed','ceres_odometry')]
        commands = [run['command'] for run in runs]
        self.assertEqual([i for i,(a,b) in enumerate(zip(*commands)) if a!=b],[2,15])
        self.assertEqual([run['tracking_prior_mode'] for run in runs],['fixed','odometry'])
        self.assertTrue(all(run['tracking_matcher']=='probability_ceres' for run in runs))

    def test_prior_comparison_rejects_removing_step_one(self):
        result = self.invoke('--prior-comparison','--tracking-matcher','distance')
        self.assertEqual(result.returncode,2)
        self.assertIn('requires --tracking-matcher probability_ceres',result.stderr)

    def test_prior_sigma_is_validated_before_replay(self):
        result = self.invoke('--tracking-odom-translation-sigma','0')
        self.assertEqual(result.returncode,2)
        self.assertFalse((self.root/'runs').exists())

    def test_ros_runner_keeps_both_changes_enabled(self):
        command = launch_command(self.root,'belief',4,42,tracking_matcher='probability_ceres',
                                 tracking_prior_mode='odometry')
        self.assertIn('tracking_matcher:=probability_ceres',command)
        self.assertIn('tracking_prior_mode:=odometry',command)


if __name__ == '__main__':
    unittest.main()
