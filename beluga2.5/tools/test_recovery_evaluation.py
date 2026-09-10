#!/usr/bin/env python3
import csv
import importlib.util
import io
import math
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import patch
from evaluate_trajectory import Pose, associate, compare, load_performance, load_tum, write_tum, wrap
from inspect_slam_run import inspect
from summarize_performance import STAGES

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rmse_extractor', ROOT/'belugaslam_benchmark/benchmarking/compute_rmse.py')
extractor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extractor)


def trajectory(count=8):
    return [Pose(976052857_000000123+i*1_000_000_000, float(i), .2*(i%3), .1*i) for i in range(count)]


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def test_rigid_alignment_recovers_only_gauge(self):
        truth = trajectory(); angle = .7; c, s = math.cos(angle), math.sin(angle)
        estimate = [Pose(p.stamp, c*p.x-s*p.y+10, s*p.x+c*p.y-7, wrap(p.yaw+angle)) for p in truth]
        run = compare(truth, {'pf': estimate})['runs']['pf']
        self.assertLess(run['position_ape_m']['rmse'], 1e-10)
        self.assertLess(run['yaw_ape_rad']['rmse'], 1e-10)

    def test_scale_error_is_not_fitted_away(self):
        truth = trajectory(); estimate = [Pose(p.stamp, 2*p.x, 2*p.y, p.yaw) for p in truth]
        self.assertGreater(compare(truth, {'pf': estimate})['runs']['pf']['position_ape_m']['rmse'], 1)

    def test_common_support_and_missing_coverage_are_explicit(self):
        truth = trajectory(); report = compare(truth, {'pf': truth[:-2], 'graph': truth})
        self.assertEqual(report['common_reference_poses'], 6)
        self.assertEqual(report['runs']['pf']['unmatched_reference_poses'], 2)
        self.assertEqual(report['runs']['graph']['position_ape_m']['count'], 6)

    def test_clock_origin_mismatch_fails(self):
        truth = trajectory(); relative = [Pose(p.stamp-truth[0].stamp, p.x, p.y, p.yaw) for p in truth]
        with self.assertRaisesRegex(ValueError, 'clock origins'):
            compare(truth, {'pf': relative})

    def test_tum_roundtrip_preserves_epoch_nanoseconds(self):
        poses = trajectory(); path = self.root/'poses.tum'; write_tum(path, poses)
        recovered = load_tum(path)
        self.assertEqual([p.stamp for p in recovered], [p.stamp for p in poses])
        self.assertAlmostEqual(recovered[-1].yaw, poses[-1].yaw)

    def test_known_reference_offset_is_explicit(self):
        path = self.root/'ref.tum'; write_tum(path, trajectory())
        self.assertEqual(load_tum(path, 17)[0].stamp, trajectory()[0].stamp+17)

    def test_duplicate_timestamps_rejected(self):
        path = self.root/'duplicate.tum'; path.write_text('1 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n')
        with self.assertRaisesRegex(ValueError, 'strictly increasing'):
            load_tum(path)

    def test_wrapped_yaw_error(self):
        truth = trajectory(); estimate = [Pose(p.stamp, p.x, p.y, p.yaw+2*math.pi-.01) for p in truth]
        error = compare(truth, {'pf': estimate}, alignment='none')['runs']['pf']['yaw_ape_rad']['rmse']
        self.assertAlmostEqual(error, .01)

    def test_rpe_detects_translation_drift(self):
        truth = [Pose(i*1_000_000_000, i, 0, 0) for i in range(8)]
        estimate = [Pose(p.stamp, 2*p.x, 0, 0) for p in truth]
        result = compare(truth, {'pf': estimate})['runs']['pf']['translation_rpe_m']
        self.assertEqual(result['count'], 7)
        self.assertAlmostEqual(result['rmse'], 1)

    def test_degenerate_alignment_does_not_invent_rotation(self):
        poses = [Pose(i*1_000_000_000, 0, 0, 0) for i in range(4)]
        with self.assertRaisesRegex(ValueError, 'degenerate'):
            compare(poses, {'pf': poses})
        self.assertEqual(compare(poses, {'pf': poses}, alignment='origin')['runs']['pf']['position_ape_m']['rmse'], 0)

    def test_one_to_one_association(self):
        reference = [Pose(i, i, 0, 0) for i in range(4)]
        estimate = [Pose(1, 1, 0, 0), Pose(3, 3, 0, 0)]
        mapping = associate(reference, estimate, 2)
        self.assertEqual(len(mapping), 2)
        self.assertEqual(len(set(mapping.values())), 2)

    def test_online_csv_skips_rejected_callbacks(self):
        path = self.root/'perf.csv'
        path.write_text('stamp_ns,status,output_x,output_y,output_yaw\n976052857000000123,processed,1,2,.3\n976052857100000123,tf_error,,,\n976052857200000123,processed,2,3,.4\n')
        result = load_performance(path)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].stamp, 976052857000000123)

    def test_old_csv_cannot_be_misread_as_published_pose(self):
        path = self.root/'perf.csv'; path.write_text('stamp_ns,status\n1,processed\n')
        with self.assertRaisesRegex(ValueError, 'lacks published poses'):
            load_performance(path)

    def test_late_error_is_reported_without_realigning_each_third(self):
        truth = trajectory(12); estimate = [Pose(p.stamp, p.x, p.y+(1 if i>=8 else 0), p.yaw) for i,p in enumerate(truth)]
        thirds = compare(truth, {'pf': estimate}, alignment='none')['runs']['pf']['position_ape_by_third_m']
        self.assertEqual(thirds['early']['rmse'], 0)
        self.assertAlmostEqual(thirds['late']['rmse'], 1)


class ExtractionTests(unittest.TestCase):
    @staticmethod
    def message(ns):
        pose = NS(position=NS(x=1., y=2., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
        return NS(header=NS(stamp=NS(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)), pose=NS(pose=pose))

    def test_ground_truth_topic_is_actually_written(self):
        estimate, reference = io.StringIO(), io.StringIO()
        message = self.message(976052857000000123)
        counts = extractor.write_decoded_trajectories([(extractor.BEST_POSE_TOPIC,message),(extractor.GT_TOPIC,message)], estimate, reference)
        self.assertEqual(counts[extractor.GT_TOPIC], 1)
        self.assertTrue(reference.getvalue().startswith('976052857.000000123 '))
        self.assertEqual(estimate.getvalue(), reference.getvalue())

    def test_negative_nanosecond_stamp_is_formatted_correctly(self):
        stream = io.StringIO(); message = self.message(-1)
        extractor.write_decoded_trajectories([(extractor.BEST_POSE_TOPIC, message)], stream)
        self.assertTrue(stream.getvalue().startswith('-0.000000001 '))

    def test_missing_gt_requires_external_reference(self):
        with self.assertRaisesRegex(ValueError, 'No reference'):
            extractor.write_decoded_trajectories([(extractor.BEST_POSE_TOPIC,self.message(1))], io.StringIO(), io.StringIO())

    def test_external_reference_path_can_skip_gt_stream(self):
        extractor.write_decoded_trajectories([(extractor.BEST_POSE_TOPIC,self.message(1))], io.StringIO())

    def test_failed_evo_cannot_return_a_plausible_rmse(self):
        with patch.object(extractor.subprocess, 'run', return_value=NS(returncode=1, stdout='rmse 0.01', stderr='failed')):
            with self.assertRaisesRegex(RuntimeError, 'failed'):
                extractor.compute_evo_rmse('gt', 'est')

    def test_evo_time_settings_are_forwarded_without_scale_correction(self):
        with patch.object(extractor.subprocess, 'run', return_value=NS(returncode=0, stdout='rmse 0.123', stderr='')) as run:
            self.assertEqual(extractor.compute_evo_rmse('gt','est', .025, 10)[0], .123)
            command = run.call_args.args[0]
            self.assertIn('0.025', command)
            self.assertNotIn('--correct_scale', command)
            self.assertEqual(command[command.index('--t_offset')+1], '10')

    def test_storage_from_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory); (path/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n')
            self.assertEqual(extractor.storage_identifier(path), 'sqlite3')


class InspectionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(); self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.perf, self.tracking, self.loops = [self.root/name for name in ('perf.csv','tracking.csv','loops.csv')]
        self.rows = []
        for i in range(6):
            self.rows.append(dict.fromkeys(STAGES, '1'))
            self.rows[-1].update(status='processed', stamp_ns=str(i*1_000_000_000), received=str(i+1), processed=str(i+1),
                map_publications='0',last_map_ms='0',visualization_ticks='0',last_visualization_ms='0',
                age_ms='1',output_innovation_m='1' if i==3 else '0',output_innovation_rad='0', tracking_status='tracked',
                selection_changed='1' if i==3 else '0',selected_hypothesis='1' if i>=3 else '0',local_only_pgo_skips='0',
                loop_cache_bytes='0',trials='2' if i==3 else '0',baseline_solves='0',weak_scans='0')
        with self.tracking.open('w') as stream:
            stream.write('sequence,hypothesis,usable,overlap,mass,status,pf_frontend_distance_m\n')
            for i in range(6): stream.write(f'{i},0,1,1,1,tracked,0\n')
        self.loops.write_text('candidate_id,query_sequence,reference_sequence,source_hypothesis,hypothesis,prior_weight,compatibility,trial_usable,belief_score,map_score,uniform_score,geometry_score,eligible,selected\n0,3,0,0,0,1,1,1,1,1,1,1,1,1\n')

    def write(self):
        with self.perf.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=self.rows[0]);writer.writeheader();writer.writerows(self.rows)

    def test_first_correction_is_joined_by_sequence(self):
        self.write(); result=inspect(self.perf,self.tracking,self.loops)
        event=result['first_notable_correction'];self.assertEqual(event['sequence'],3)
        self.assertEqual(event['pre_backend_tracking'][0]['hypothesis'],'0')
        self.assertEqual(event['selected_hypothesis'],'1')
        self.assertEqual(event['candidates_with_this_query_sequence'][0]['candidate_id'],'0')

    def test_missing_counter_prevents_false_join(self):
        self.rows[3]['processed']='9';self.write()
        with self.assertRaisesRegex(ValueError,'processed counters'):
            inspect(self.perf,self.tracking,self.loops)

    def test_no_large_correction_is_not_called_failure(self):
        self.rows[3]['output_innovation_m']='0';self.write()
        self.assertIsNone(inspect(self.perf,self.tracking,self.loops)['first_notable_correction'])


if __name__ == '__main__':
    unittest.main()
