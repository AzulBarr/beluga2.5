import csv
import math
from pathlib import Path
import tempfile
import unittest
from summarize_bayesian_loops import summarize


class BayesianDiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'bayes.csv'

    def rows(self, event=1, status='pending', attempts=0, age=0, probability=.5):
        return [dict(event_id=event, hypothesis=h, association=a, posterior_mass=w,
                     log_mass=math.log(w), evidence_scans=age, attempted_scans=attempts,
                     loop_probability=probability, status=status)
                for h, a, w in [(0, 'no_loop', 1-probability), (1, 'loop', probability)]]

    def write(self, rows):
        with self.path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.rows()[0].keys())
            writer.writeheader(); writer.writerows(rows)

    def test_tentative_installation_is_not_acceptance(self):
        self.write(self.rows())
        result = summarize(self.path)
        self.assertEqual(result['pending'], 1)
        self.assertEqual(result['accepted'], 0)

    def test_one_decision_per_event_not_per_particle_or_scan(self):
        self.write(self.rows()+self.rows(status='accepted', attempts=10, age=10, probability=.96))
        result = summarize(self.path)
        self.assertEqual(result['events'], 1)
        self.assertEqual(result['accepted'], 1)

    def test_unknown_coverage_timeout_is_undecided(self):
        self.write(self.rows(status='undecided', attempts=30))
        self.assertEqual(summarize(self.path)['undecided'], 1)

    def test_incomplete_population_is_rejected(self):
        self.write(self.rows()[:1])
        with self.assertRaises(ValueError): summarize(self.path)

    def test_duplicate_hypothesis_is_rejected(self):
        rows = self.rows(); self.write(rows+[rows[0]])
        with self.assertRaises(ValueError): summarize(self.path)

    def test_inconsistent_probability_is_rejected(self):
        rows = self.rows()
        for row in rows: row['loop_probability'] = .9
        self.write(rows)
        with self.assertRaises(ValueError): summarize(self.path)

    def test_empty_header_only_recording_has_no_events(self):
        self.write([])
        self.assertEqual(summarize(self.path)['events'], 0)


if __name__ == '__main__': unittest.main()
