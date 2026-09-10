#!/usr/bin/env python3
"""Locate notable online corrections without changing the SLAM runtime.

Loop candidates are associated by their query scan, not by an assumed execution
time. Coincidence in these logs does not prove that a loop or pose is incorrect.
"""
import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
from summarize_performance import summarize as summarize_performance
from summarize_tracking import summarize as summarize_tracking
from summarize_loop_verification import summarize as summarize_loops


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def inspect(performance, tracking, loops, translation=.35, rotation=.15):
    if not all(math.isfinite(v) and v > 0 for v in (translation, rotation)):
        raise ValueError('Correction thresholds must be finite and positive')
    summary = summarize_performance(performance)
    tracking_summary = summarize_tracking(tracking)
    loop_summary = summarize_loops(loops, .25)
    rows = read_csv(performance)
    processed = [row for row in rows if row['status'] == 'processed']
    count = 0
    for row in rows:
        count += row['status'] == 'processed'
        if int(row['processed']) != count:
            raise ValueError('Incomplete processed counters: cannot safely join scan sequences')
    if not processed:
        raise ValueError('No processed scans')
    by_sequence = defaultdict(list)
    for row in read_csv(tracking):
        by_sequence[int(row['sequence'])].append(row)
    candidates = defaultdict(list)
    for row in read_csv(loops):
        candidates[int(row['query_sequence'])].append(row)
    for row in processed:
        for key in ('output_innovation_m', 'output_innovation_rad'):
            if not math.isfinite(float(row[key])) or float(row[key]) < 0:
                raise ValueError(f'Invalid {key}')
    origin = int(processed[0]['stamp_ns'])
    def describe(sequence):
        row = processed[sequence]
        tracking_rows = by_sequence.get(sequence, [])
        local = [{key: item[key] for key in ('hypothesis', 'usable', 'overlap', 'mass', 'status', 'pf_frontend_distance_m')} for item in tracking_rows]
        query = {}
        for item in candidates.get(sequence, []):
            key = int(item['candidate_id'])
            if key not in query:
                query[key] = {k: item[k] for k in ('candidate_id', 'reference_sequence', 'query_sequence', 'belief_score', 'map_score', 'eligible', 'selected')}
        return {'sequence': sequence, 'elapsed_s': (int(row['stamp_ns'])-origin)/1e9,
                **{key: row[key] for key in ('stamp_ns', 'selected_hypothesis', 'selection_changed', 'tracking_status',
                   'weak_scans', 'output_innovation_m', 'output_innovation_rad', 'baseline_solves', 'trials', 'backend_ms', 'total_ms', 'age_ms')},
                'pre_backend_tracking': local, 'candidates_with_this_query_sequence': list(query.values())}
    notable = [i for i, row in enumerate(processed) if float(row['output_innovation_m']) >= translation or float(row['output_innovation_rad']) >= rotation]
    first = notable[0] if notable else None
    episodes, start = [], None
    for i in range(len(processed)+1):
        weak = i < len(processed) and processed[i]['tracking_status'] in ('weak', 'rejected', 'recovery_pending')
        if weak and start is None:
            start = i
        if not weak and start is not None:
            if i-start >= 3:
                episodes.append({'start_sequence': start, 'end_sequence': i-1, 'length': i-start,
                                 'start_elapsed_s': (int(processed[start]['stamp_ns'])-origin)/1e9})
            start = None
    return {'performance': summary, 'tracking': tracking_summary, 'loop_verification': loop_summary,
            'correction_threshold_m': translation, 'correction_threshold_rad': rotation,
            'notable_correction_count': len(notable), 'first_notable_correction': describe(first) if first is not None else None,
            'first_correction_context': [describe(i) for i in range(max(0, first-2), min(len(processed), first+3))] if first is not None else [],
            'first_ten_notable_corrections': [describe(i) for i in notable[:10]],
            'weak_or_rejected_episode_count': len(episodes), 'first_ten_weak_or_rejected_episodes': episodes[:10],
            'tracking_sequences_without_processed_callback': sorted(set(by_sequence)-set(range(len(processed)))),
            'processed_sequences_without_tracking': sorted(set(range(len(processed)))-set(by_sequence)),
            'notes': ['Use files from the same uninterrupted run. Counter consistency cannot establish file provenance.',
                      'Core scan sequence equals the zero-based processed callback counter in this node.',
                      'Tracking rows precede backend branching: a newly selected child may not yet appear there.',
                      'Candidate query time is not necessarily verification execution time; trials and baseline_solves describe current callback work.',
                      'A large correction can be valid. These flags locate evidence; reference trajectories are required to determine error.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('performance'); parser.add_argument('tracking'); parser.add_argument('loops')
    parser.add_argument('--translation', type=float, default=.35)
    parser.add_argument('--rotation', type=float, default=.15)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        report = inspect(args.performance, args.tracking, args.loops, args.translation, args.rotation)
        text = json.dumps(report, indent=2, allow_nan=False)+'\n'
        if args.output:
            args.output.write_text(text)
        print(text, end='')
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
