#!/usr/bin/env python3
"""Regression fixtures for the frozen-belief and pruning audit (no ROS needed)."""
import csv
from pathlib import Path
import tempfile
import unittest
from summarize_loop_verification import summarize


class LoopDiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'loops.csv'

    def rows(self, candidate=0, event=0, query=10):
        return [dict(candidate_id=candidate, event_id=event, query_sequence=query,
                     hypothesis=h, prior_weight=w, compatibility=e, trial_usable=1,
                     belief_score=.4, map_score=0, uniform_score=.5, geometry_score=.8,
                     eligible=1, selected=1, query_consumed=1, retained_branch_mass=.8)
                for h, w, e in ((0, .6, 0), (1, .4, 1))]

    def write(self, rows):
        with self.path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def test_secondary_mode_support_and_pruning_bound(self):
        self.write(self.rows())
        report = summarize(self.path, .25)
        self.assertEqual(report['belief_accept_map_reject'], [0])
        self.assertAlmostEqual(report['event_audit']['maximum_discarded_branch_mass'], .2)
        self.assertEqual(report['event_audit']['events_with_pruning'], 1)
        self.assertAlmostEqual(report['largest_score_disagreements'][0]['non_MAP_mass_bound'], .4)

    def test_same_event_can_contain_multiple_geometric_alternatives(self):
        self.write(self.rows() + self.rows(candidate=1))
        self.assertEqual(summarize(self.path, .25)['event_audit']['consumed_queries'], 1)

    def test_repeated_consumption_is_rejected(self):
        self.write(self.rows() + self.rows(candidate=1, event=1))
        with self.assertRaisesRegex(ValueError, 'consumed in multiple events'):
            summarize(self.path, .25)

    def test_event_prior_must_be_frozen(self):
        rows = self.rows() + self.rows(candidate=1, query=11)
        rows[2]['hypothesis'] = 99
        self.write(rows)
        with self.assertRaisesRegex(ValueError, 'prior changed'):
            summarize(self.path, .25)

    def test_wrong_MAP_or_uniform_score_is_rejected(self):
        rows = self.rows()
        for row in rows:
            row['map_score'] = .4
        self.write(rows)
        with self.assertRaisesRegex(ValueError, 'MAP/uniform'):
            summarize(self.path, .25)

    def test_invalid_retained_mass_is_rejected(self):
        rows = self.rows()
        for row in rows:
            row['retained_branch_mass'] = float('nan')
        self.write(rows)
        with self.assertRaisesRegex(ValueError, 'retained branch mass'):
            summarize(self.path, .25)

    def test_old_diagnostics_remain_readable(self):
        rows = self.rows()
        for row in rows:
            for name in ('event_id', 'query_consumed', 'retained_branch_mass'):
                row.pop(name)
        self.write(rows)
        report = summarize(self.path, .25)
        self.assertEqual(report['belief_accept_map_reject'], [0])
        self.assertFalse(report['event_audit']['available'])


if __name__ == '__main__':
    unittest.main()
