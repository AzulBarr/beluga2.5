#!/usr/bin/env python3
"""Score logged PRE-backend frontend beliefs. No ground truth or full SLAM replay.

Compile score_output_snapshots.cpp as documented in POSTERIOR_OUTPUT_REVIEW.md.
Only an explicitly allowed, malformed trailing CSV row may be excluded.
"""
import argparse
import csv
import io
import itertools
import json
import math
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tracking', type=Path)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-truncated-tail', action='store_true')
    args = parser.parse_args()
    with args.tracking.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'sequence', 'hypothesis', 'mass', 'x', 'y'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('Missing required tracking fields')
        rows = list(reader)
    if not rows:
        raise ValueError('Empty tracking log')
    malformed = [i for i, row in enumerate(rows)
                 if None in row or any(value in (None, '') for value in row.values())]
    excluded = 0
    if malformed:
        if not args.allow_truncated_tail or malformed != [len(rows)-1]:
            raise ValueError('Malformed CSV rows; only an explicitly allowed final row may be excluded')
        # Remove the whole final sequence: its other rows may be only a partial belief.
        last_sequence = rows[-1].get('sequence')
        while rows and rows[-1].get('sequence') == last_sequence:
            rows.pop(); excluded += 1
    groups = [(int(seq), list(group)) for seq, group in
              itertools.groupby(rows, key=lambda row: row['sequence'])]
    if not groups or any(groups[i][0] >= groups[i+1][0] for i in range(len(groups)-1)):
        raise ValueError('Empty, repeated or unordered snapshot groups')
    lines = []
    beliefs = {}
    for seq, group in groups:
        points = [(int(r['hypothesis']), float(r['mass']), float(r['x']), float(r['y'])) for r in group]
        if len({p[0] for p in points}) != len(points):
            raise ValueError(f'Duplicate hypothesis ID at sequence {seq}')
        if any(not all(math.isfinite(v) for v in p[1:]) or p[1] < 0 for p in points):
            raise ValueError(f'Invalid belief at sequence {seq}')
        total = math.fsum(p[1] for p in points)
        if not math.isclose(total, 1.0, abs_tol=1e-6, rel_tol=0):
            raise ValueError(f'Incomplete or unnormalized belief at sequence {seq}')
        beliefs[seq] = points
        lines.append(f'{seq} {len(points)}\n')
        lines.extend(f'{hid} {mass:.17g} {x:.17g} {y:.17g}\n' for hid, mass, x, y in points)
    output = subprocess.run([str(args.executable.resolve())], input=''.join(lines),
                            capture_output=True, text=True, check=True).stdout
    decisions = list(csv.DictReader(io.StringIO(output)))
    if len(decisions) != len(groups):
        raise ValueError('C++ output count mismatch')
    gains, changed = [], 0
    for (seq, _), decision in zip(groups, decisions):
        if int(decision['sequence']) != seq:
            raise ValueError('C++ output sequence mismatch')
        points = beliefs[seq]
        total = math.fsum(p[1] for p in points)
        risks = {p[0]: math.fsum(q[1]/total*((p[2]-q[2])**2+(p[3]-q[3])**2) for q in points)
                 for p in points if p[1] > 0}
        map_id, risk_id = int(decision['map_hypothesis']), int(decision['risk_hypothesis'])
        expected_map = min((p for p in points if p[1] > 0), key=lambda p: (-p[1], p[0]))[0]
        map_loss = float(decision['map_position_risk_m2'])
        risk_loss = float(decision['minimum_position_risk_m2'])
        if (map_id != expected_map or risk_id not in risks or
            not math.isclose(risks[map_id], map_loss, abs_tol=1e-10, rel_tol=1e-10) or
            not math.isclose(risks[risk_id], risk_loss, abs_tol=1e-10, rel_tol=1e-10) or
            not math.isclose(min(risks.values()), risk_loss, abs_tol=1e-10, rel_tol=1e-10) or
            risk_loss > map_loss + 1e-10):
            raise ValueError(f'Independent reference check failed at sequence {seq}')
        gains.append(map_loss-risk_loss)
        changed += map_id != risk_id
    report = {
        'snapshot_stage': 'pre_backend_frontend_point_belief',
        'snapshots': len(groups), 'excluded_trailing_rows': excluded,
        'different_hypothesis_decisions': changed,
        'internal_risk_reductions_greater_than_1e_minus_6_m2': sum(g > 1e-6 for g in gains),
        'maximum_internal_risk_reduction_m2': max(gains),
        'mean_internal_risk_reduction_m2': math.fsum(gains)/len(gains),
        'cpp_against_independent_python_check': 'passed',
        'ground_truth_used': False, 'full_slam_replay': False,
        'measured_rmse_improvement': None,
        'limits': 'Snapshots precede PGO and branching; these are not counterfactual ROS outputs. '
                  'Internal position loss does not measure actual position, heading or map accuracy.'
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'snapshot_decisions.csv').write_text(output)
    (args.output/'snapshot_summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
