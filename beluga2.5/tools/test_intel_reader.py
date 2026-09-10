#!/usr/bin/env python3
"""Run without ROS: python3 tools/test_intel_reader.py [--dataset path/to/intel.clf]."""
import argparse
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'belugaslam_example/bags/intel'))
from carmen_reader import load_ordered_flaser, parse_flaser, scan_angles
import math


def flaser(stamp='976052857.337530', logger='0.000246', pose='1 2 0.3'):
    return f'FLASER 2 3 4 0 0 0 {pose} {stamp} host {logger}\n'


class IntelReaderTest(unittest.TestCase):
    def load(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'data.clf'
            path.write_text(content)
            return load_ordered_flaser(path)

    def test_acquisition_timestamp_and_odometry(self):
        record = parse_flaser(flaser(), 17)
        self.assertEqual(record.timestamp_ns, 976052857337530000)
        self.assertEqual(record.ranges, (3.0, 4.0))
        self.assertEqual(record.odometry, (1.0, 2.0, 0.3))
        self.assertEqual(record.line_number, 17)

    def test_other_messages_ignored(self):
        self.assertIsNone(parse_flaser('# format documentation'))
        self.assertIsNone(parse_flaser('ODOM 0 0 0'))
        self.assertIsNone(parse_flaser(''))

    def test_order_preserves_stamp_and_measurement_association(self):
        records, backwards = self.load(flaser('12', pose='12 0 0') + flaser('10', pose='10 0 0') + flaser('11', pose='11 0 0'))
        self.assertEqual(backwards, 1)
        self.assertEqual([r.timestamp_ns for r in records], [10000000000, 11000000000, 12000000000])
        self.assertEqual([r.odometry[0] for r in records], [10, 11, 12])
        self.assertEqual([r.line_number for r in records], [2, 3, 1])

    def test_invalid_record_fails_with_source_line(self):
        for line in ['FLASER', 'FLASER 180 1 2', flaser('nan'), flaser('-1'), flaser(pose='nan 0 0')]:
            with self.subTest(line=line):
                with self.assertRaisesRegex(ValueError, 'line 7'):
                    parse_flaser(line, 7)

    def test_duplicate_times_not_silently_clamped_or_discarded(self):
        with self.assertRaisesRegex(ValueError, 'duplicate acquisition time'):
            self.load(flaser('1') + flaser('1', pose='2 0 0'))

    def test_no_scans_is_explicit_error(self):
        with self.assertRaisesRegex(ValueError, 'no FLASER'):
            self.load('# empty file\n')

    def test_intel_beam_angles_without_stretching(self):
        start, end, increment = scan_angles(180)
        self.assertAlmostEqual(math.degrees(start), -90)
        self.assertAlmostEqual(math.degrees(end), 89)
        self.assertAlmostEqual(math.degrees(increment), 1)
        self.assertAlmostEqual(start + 179 * increment, end)
        self.assertAlmostEqual(math.degrees(scan_angles(180, -90, 180/179)[1]), 90)
        with self.assertRaises(ValueError):
            scan_angles(180, -90, 0)

    def test_long_gap_is_preserved(self):
        records, _ = self.load(flaser('1') + flaser('5.000001'))
        self.assertEqual(records[1].timestamp_ns - records[0].timestamp_ns, 4000001000)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--write-stamps', type=Path, help='Write original integer acquisition nanoseconds for scan accounting')
    args = parser.parse_args()
    if args.write_stamps and not args.dataset:
        parser.error('--write-stamps requires --dataset')
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(IntelReaderTest))
    if not result.wasSuccessful():
        raise SystemExit(1)
    if args.dataset:
        ordered, backwards = load_ordered_flaser(args.dataset)
        original = sorted(ordered, key=lambda r: r.line_number)
        latest = -1
        rejected = 0
        for record in original:
            if record.timestamp_ns <= latest:
                rejected += 1
            else:
                latest = record.timestamp_ns
        print(f'Dataset: {len(ordered)} records; {backwards} adjacent backward transitions; '
              f'{rejected} would fail a last-accepted timestamp guard in file order.')
        assert all(a.timestamp_ns < b.timestamp_ns for a, b in zip(ordered, ordered[1:]))
        print(f'PASS: all {len(ordered)} records retained with strictly increasing, unchanged acquisition timestamps.')
        if args.write_stamps:
            args.write_stamps.write_text(''.join(f'{record.timestamp_ns}\n' for record in ordered))
