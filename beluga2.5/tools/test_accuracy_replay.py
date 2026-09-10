import csv
from pathlib import Path
import tempfile
import unittest
from run_accuracy_replay import logger_offset, write_input, check_coverage


class AccuracyReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)

    def data(self, logger_second='0.2'):
        p = self.folder/'test.clf'
        p.write_text('FLASER 2 1 2 0 0 0 0 0 0 1000.1 host 0.1\n'
                     f'FLASER 2 3 4 0 0 0 0.001 0 0 1000.2 host {logger_second}\n')
        return p

    def test_clock_offset_is_read_from_recording(self):
        self.assertEqual(str(logger_offset(self.data())), '1000.0')

    def test_changing_clock_relation_is_not_fitted(self):
        with self.assertRaisesRegex(ValueError, 'not a single constant'):
            logger_offset(self.data('0.3'))

    def test_input_preserves_acquisition_and_measurement(self):
        output = self.folder/'input'
        records, _ = write_input(self.data(), output)
        lines = output.read_text().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[1].split(), ['1000200000000', '0.001', '0.0', '0.0', '2', '3.0', '4.0'])
        self.assertEqual(records[1].timestamp_ns, 1000200000000)

    def test_coverage_rejects_missing_or_shifted_scans(self):
        (self.folder/'performance.csv').write_text('stamp_ns,status,output_x,output_y,output_yaw\n1000,processed,0,0,0\n2000,processed,1,0,0\n')
        (self.folder/'optimized_trajectory.tum').write_text('0.000001000 0 0 0 0 0 0 1\n0.000002000 1 0 0 0 0 0 1\n')
        self.assertEqual(len(check_coverage(self.folder, [1000,2000])[0]), 2)
        for stamps in ([1000], [1001,2000], [1000,2000,3000]):
            with self.assertRaisesRegex(ValueError, 'exactly every'):
                check_coverage(self.folder, stamps)


if __name__ == '__main__':
    unittest.main()
