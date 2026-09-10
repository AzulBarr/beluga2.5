#!/usr/bin/env python3
"""Summarize measured scan latency; exact missing-scan counts need source stamps."""
import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

STAGES = ('total_ms', 'tf_convert_ms', 'motion_ms', 'matching_ms', 'insertion_ms',
          'backend_ms', 'resample_ms', 'pose_publish_ms', 'baseline_pgo_ms',
          'retrieval_ms', 'verification_ms')

def percentiles(values):
    values = sorted(values)
    if not values:
        return {}
    def percentile(q):
        position = (len(values) - 1) * q
        lo = int(position)
        hi = min(lo + 1, len(values) - 1)
        return values[lo] + (values[hi] - values[lo]) * (position - lo)
    return {'count': len(values), 'mean': sum(values) / len(values),
            'p50': percentile(.5), 'p95': percentile(.95), 'p99': percentile(.99), 'max': values[-1]}

def summarize(path, budget_ms=None, expected_stamps=None):
    with Path(path).open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError('No diagnostic rows')
    required = set(STAGES) | {'status', 'stamp_ns', 'received', 'map_publications', 'last_map_ms', 'visualization_ticks', 'last_visualization_ms'}
    if not required <= rows[0].keys():
        raise ValueError('Missing diagnostic columns')
    for row in rows:
        for column in STAGES + ('last_map_ms', 'last_visualization_ms'):
            if not math.isfinite(float(row[column])) or float(row[column]) < 0:
                raise ValueError(f'Invalid {column}')
        if row['status'] not in {'processed', 'tf_error', 'empty_scan', 'out_of_order'}:
            raise ValueError('Unknown scan status')
    if [int(row['received']) for row in rows] != list(range(1, len(rows) + 1)):
        raise ValueError('Received counters are incomplete, duplicated or out of order')
    processed = [row for row in rows if row['status'] == 'processed']
    report = {'callbacks': len(rows), 'status_counts': dict(Counter(row['status'] for row in rows)),
              'processed_scan_stage_ms': {stage: percentiles([float(row[stage]) for row in processed]) for stage in STAGES},
              'all_callback_ms': percentiles([float(row['total_ms']) for row in rows])}
    if processed:
        thirds = {}
        for index,name in enumerate(('early','middle','late')):
            part=processed[index*len(processed)//3:(index+1)*len(processed)//3]
            thirds[name]={stage:percentiles([float(row[stage]) for row in part]) for stage in ('total_ms','matching_ms','backend_ms','age_ms')}
        report['run_thirds']=thirds
    if processed and 'output_innovation_m' in processed[0]:
        report['tracking_status_counts']=dict(Counter(row['tracking_status'] for row in processed))
        report['selected_hypothesis_changes']=sum(int(row['selection_changed']) for row in processed)
        report['local_only_pgo_skips']=sum(int(row['local_only_pgo_skips']) for row in processed)
        report['retained_loop_cache_peak_bytes']=max(int(row['loop_cache_bytes']) for row in processed)
        largest=sorted(processed,key=lambda row:float(row['output_innovation_m']),reverse=True)[:10]
        report['largest_output_corrections']=[{key:row[key] for key in (
            'stamp_ns','selected_hypothesis','selection_changed','tracking_status','output_innovation_m','output_innovation_rad','backend_ms','trials')} for row in largest]
    loop_rows = [row for row in processed if int(row.get('trials', 0)) > 0]
    report['loop_event_callback_ms'] = percentiles([float(row['total_ms']) for row in loop_rows])
    if processed and 'polish_solves' in processed[0]:
        report['robust_polish'] = {
            'solves': sum(int(row['polish_solves']) for row in processed),
            'sum_trial_elapsed_ms': sum(float(row['polish_work_ms']) for row in processed),
            'note': 'Sum of per-trial elapsed times; parallel workers may overlap. Use backend_ms for stage wall time.'}
    seen_maps = {}
    for row in rows:
        count = int(row['map_publications'])
        if count > 0:
            seen_maps[count] = float(row['last_map_ms'])
    report['observed_map_work_ms'] = percentiles(list(seen_maps.values()))
    seen_ticks = {}
    for row in rows:
        count = int(row['visualization_ticks'])
        if count > 0:
            seen_ticks[count] = float(row['last_visualization_ms'])
    report['observed_visualization_work_ms'] = percentiles(list(seen_ticks.values()))
    report['notes'] = [
        'Callback latency excludes timer work and diagnostic writing; map timings are reported separately.',
        'Only timer timings observed by a later scan callback are available; visualization includes map work.',
        'Source timestamp gaps alone do not prove dropped scans; variable scan rates and replay clocks matter.',
        'age_ms is meaningful only when ROS time and scan timestamps use the same clock.']
    if budget_ms is not None:
        if not math.isfinite(budget_ms) or budget_ms <= 0:
            raise ValueError('budget-ms must be finite and positive')
        report['budget_ms'] = budget_ms
        report['callbacks_exceeding_budget'] = sum(float(row['total_ms']) > budget_ms for row in rows)
        report['observed_map_ticks_exceeding_budget'] = sum(value > budget_ms for value in seen_maps.values())
    if expected_stamps is not None:
        expected = [int(line.strip()) for line in Path(expected_stamps).read_text().splitlines() if line.strip()]
        if len(expected) != len(set(expected)):
            raise ValueError('Source stamp list must contain unique integer nanoseconds')
        source, received, accepted = set(expected), {int(row['stamp_ns']) for row in rows}, {int(row['stamp_ns']) for row in processed}
        report['source_scan_accounting'] = {
            'expected': len(source), 'not_received': len(source - received),
            'received_but_not_processed': len((source & received) - accepted),
            'unexpected_received': len(received - source)}
    return report

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--budget-ms', type=float)
    parser.add_argument('--expected-stamps', help='One unique source scan timestamp per line, integer nanoseconds')
    args = parser.parse_args()
    try:
        result = summarize(args.csv, args.budget_ms, args.expected_stamps)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False))

if __name__ == '__main__':
    main()
