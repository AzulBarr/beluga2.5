#!/usr/bin/env python3
"""Summarize bayes.csv by EVENT, distinguishing tentative installation from a decision."""
import argparse
import collections
import csv
import json
import math
from pathlib import Path


def summarize(path):
    groups = collections.defaultdict(list)
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'event_id', 'hypothesis', 'association', 'posterior_mass', 'log_mass',
                    'evidence_scans', 'attempted_scans', 'loop_probability', 'status'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('Missing Bayesian evidence columns')
        for row in reader:
            groups[int(row['event_id']), int(row['attempted_scans'])].append(row)
    events = {}
    for (event, attempts), rows in sorted(groups.items()):
        if len({r['hypothesis'] for r in rows}) != len(rows):
            raise ValueError('Duplicate hypothesis row in one event scan')
        statuses = {r['status'] for r in rows}
        ages = {int(r['evidence_scans']) for r in rows}
        probabilities = {float(r['loop_probability']) for r in rows}
        if len(statuses) != 1 or len(ages) != 1 or len(probabilities) != 1:
            raise ValueError('Inconsistent event-wide decision fields')
        status, age, probability = statuses.pop(), ages.pop(), probabilities.pop()
        if status not in {'pending', 'accepted', 'rejected', 'undecided'} or not 0 <= age <= attempts:
            raise ValueError('Invalid event decision or evidence age')
        masses = [float(r['posterior_mass']) for r in rows]
        logs = [float(r['log_mass']) for r in rows]
        if (not math.isfinite(probability) or not 0 <= probability <= 1 or
                any(not math.isfinite(w) or w < 0 for w in masses) or
                any(not math.isfinite(l) for l in logs) or
                not math.isclose(sum(masses), 1., abs_tol=1e-8)):
            raise ValueError('Invalid or incomplete hypothesis posterior')
        for row, weight, log_weight in zip(rows, masses, logs):
            if row['association'] not in {'loop', 'no_loop'}:
                raise ValueError('Unknown loop association')
            if not math.isclose(weight, math.exp(log_weight), abs_tol=1e-10):
                raise ValueError('Linear and log graph mass disagree')
        loop_mass = sum(w for r, w in zip(rows, masses) if r['association'] == 'loop')
        if not math.isclose(loop_mass, probability, abs_tol=1e-8):
            raise ValueError('Loop posterior disagrees with summed association mass')
        if event in events and events[event]['status'] != 'pending':
            raise ValueError('Evidence continues after a final event decision')
        events[event] = {'status': status, 'evidence_scans': age, 'attempted_scans': attempts,
                         'loop_probability': probability, 'hypotheses': len(rows)}
    counts = collections.Counter(e['status'] for e in events.values())
    return {'events': len(events),
            **{s: counts[s] for s in ('accepted', 'rejected', 'undecided', 'pending')},
            'event_details': events,
            'note': 'Model posterior, not an empirically calibrated correctness probability. Pending includes runs ending before a decision.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(summarize(args.csv), indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'Invalid Bayesian diagnostics: {error}\n')


if __name__ == '__main__':
    main()
