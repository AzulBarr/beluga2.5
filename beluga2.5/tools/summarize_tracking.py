#!/usr/bin/env python3
"""Summarize registration acceptance and the retained hypothesis weights."""
import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from summarize_performance import percentiles


def summarize(path):
    modes, scans = defaultdict(list), defaultdict(list)
    with Path(path).open(newline='') as source:
        for row in csv.DictReader(source):
            sequence, hypothesis = int(row['sequence']), int(row['hypothesis'])
            mass, overlap = float(row['mass']), float(row['overlap'])
            if not math.isfinite(mass) or mass < 0 or not math.isfinite(overlap) or not 0 <= overlap <= 1:
                raise ValueError('Invalid tracking mass or overlap')
            if row['usable'] not in ('0', '1'):
                raise ValueError('Invalid registration usability flag')
            modes[hypothesis].append(row)
            scans[sequence].append((hypothesis, mass))
    if not scans:
        raise ValueError('No tracking rows')
    effective, nonmap = [], []
    for rows in scans.values():
        if len({h for h, _ in rows}) != len(rows):
            raise ValueError('Duplicate hypothesis within a scan')
        total = sum(w for _, w in rows)
        if abs(total-1) > 1e-6:
            raise ValueError('Hypothesis masses do not sum to one')
        weights = [w/total for _, w in rows]
        effective.append(1/sum(w*w for w in weights))
        nonmap.append(1-max(weights))
    return {
        'scans': len(scans),
        'scans_with_multiple_hypotheses': sum(len(rows) > 1 for rows in scans.values()),
        'effective_hypothesis_count': percentiles(effective),
        'non_MAP_mass': percentiles(nonmap),
        'hypotheses': {h: {
            'observations': len(rows),
            'usable_fraction': sum(int(row['usable']) for row in rows)/len(rows),
            'overlap': percentiles([float(row['overlap']) for row in rows]),
            'mass': percentiles([float(row['mass']) for row in rows]),
            'status_counts': dict(Counter(row.get('status','unavailable') for row in rows)),
            'max_consecutive_weak_scans': max(int(row.get('consecutive_weak_scans',0)) for row in rows),
            'pf_frontend_distance_m': percentiles([float(row['pf_frontend_distance_m']) for row in rows if 'pf_frontend_distance_m' in row]),
            'pose_source_counts': dict(Counter(row.get('pose_source','unavailable') for row in rows)),
            'proposal_pose_decisions': dict(Counter(row.get('proposal_pose_decision','unavailable') for row in rows)),
            'proposal_ess': percentiles([float(row['proposal_ess']) for row in rows if 'proposal_ess' in row]),
            'proposal_mean_offset_m': percentiles([float(row['proposal_mean_offset_m']) for row in rows if 'proposal_mean_offset_m' in row]),
        } for h, rows in sorted(modes.items())},
        'notes': [
            'Usability gates insertion; high rejection can reduce coverage.',
            'Bootstrap rows are usable with zero overlap because no reference exists.',
            'Weights are recorded before the current backend event and resampling.',
            'Effective hypothesis count measures weight concentration, not independent trajectories or accuracy.'],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    args = parser.parse_args()
    try:
        print(json.dumps(summarize(args.csv), indent=2, allow_nan=False))
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
