import math
from pathlib import Path
import tempfile
import unittest

from evaluate_trajectory import Pose, compare
from evaluate_frame_trajectory import convert_to_frame, load_extrinsics


class FrameTrajectoryTests(unittest.TestCase):
    def test_lever_arm_rotates_and_yaw_is_composed(self):
        poses = [Pose(123, 1., 2., math.pi/2)]
        result = convert_to_frame(poses, ('base', 'laser'), {123: (.3, .1, .2, True)}, '/laser')[0]
        self.assertAlmostEqual(result.x, .9)
        self.assertAlmostEqual(result.y, 2.3)
        self.assertAlmostEqual(result.yaw, math.pi/2 + .2)
        self.assertEqual(result.stamp, 123)

    def test_time_varying_extrinsics_use_exact_timestamps(self):
        poses = [Pose(100, 0., 0., 0.), Pose(200, 0., 0., 0.)]
        result = convert_to_frame(poses, ('base', 'laser'),
                                  {100: (.3, 0., 0., False), 200: (.4, .1, .2, True)}, 'laser')
        self.assertAlmostEqual(result[1].x-result[0].x, .1)
        self.assertAlmostEqual(result[1].yaw, .2)

    def test_missing_tf_is_not_replaced_with_nearest_or_identity(self):
        with self.assertRaisesRegex(ValueError, 'No TF'):
            convert_to_frame([Pose(100, 0., 0., 0.)], ('base', 'laser'), {101: (.3, 0., 0., False)}, 'laser')

    def test_base_output_is_unchanged(self):
        poses = [Pose(100, 1., 2., .2)]
        self.assertEqual(convert_to_frame(poses, ('base', 'laser'), {100: (.3, 0., .1, False)}, 'base'), poses)

    def test_unknown_reference_frame_fails(self):
        with self.assertRaisesRegex(ValueError, 'neither'):
            convert_to_frame([Pose(0, 0., 0., 0.)], ('base', 'laser'), {}, 'camera')

    def test_recorded_tf_removes_physical_offset_before_world_alignment(self):
        # Non-circular motion: a body lever cannot be hidden in a world alignment.
        base = [Pose(i*1_000_000_000, float(i), .1*i*i, .23*i) for i in range(12)]
        tf = {p.stamp: (.3, -.05, .1, False) for p in base}
        sensor = convert_to_frame(base, ('base', 'laser'), tf, 'laser')
        # Independent world transform and directly computed sensor coordinates.
        ref = []
        for p in base:
            x = p.x + .3*math.cos(p.yaw) + .05*math.sin(p.yaw)
            y = p.y + .3*math.sin(p.yaw) - .05*math.cos(p.yaw)
            ref.append(Pose(p.stamp, math.cos(.4)*x-math.sin(.4)*y+5,
                            math.sin(.4)*x+math.cos(.4)*y-3, p.yaw+.1+.4))
        report = compare(ref, {'raw': base, 'sensor': sensor})
        self.assertGreater(report['runs']['raw']['position_ape_m']['rmse'], .02)
        self.assertLess(report['runs']['sensor']['position_ape_m']['rmse'], 1e-10)
        self.assertLess(report['runs']['sensor']['yaw_ape_rad']['rmse'], 1e-10)

    def test_sidecar_validation(self):
        header = 'stamp_ns,base_frame,scan_frame,base_from_scan_x,base_from_scan_y,base_from_scan_yaw,deskewed\n'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'tf.csv'
            path.write_text(header + '123,/base,/laser,.3,0,.1,1\n')
            frames, records = load_extrinsics(path)
            self.assertEqual(frames, ('base', 'laser'))
            self.assertEqual(records[123], (.3, 0., .1, True))
            for rows in ('123,base,laser,nan,0,0,0\n',
                         '123,base,laser,.3,0,0,0\n123,base,laser,.3,0,0,0\n',
                         '123,base,laser,.3,0,0,0\n124,base,camera,.3,0,0,0\n',
                         '123,base,base,.3,0,0,0\n', ''):
                path.write_text(header + rows)
                with self.assertRaises(ValueError):
                    load_extrinsics(path)


if __name__ == '__main__':
    unittest.main()
