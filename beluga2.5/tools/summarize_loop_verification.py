#!/usr/bin/env python3
"""Compare all scorer decisions on the SAME recorded candidates.

This is an offline score ablation, not a replacement for independent SLAM runs.
Optional labels CSV: candidate_id,label (label is 0 or 1, for this one run).
Unlabelled candidates never count as negatives.
"""
import argparse
import csv
from collections import Counter, defaultdict
import json
import math
from pathlib import Path


def summarize(path, threshold, labels_path=None):
    groups = defaultdict(list)
    with Path(path).open(newline="") as stream:
        for row in csv.DictReader(stream):
            groups[int(row["candidate_id"])].append(row)
    labels = {}
    if labels_path:
        with Path(labels_path).open(newline="") as stream:
            for row in csv.DictReader(stream):
                key, label = int(row["candidate_id"]), int(row["label"])
                if label not in (0, 1) or key in labels:
                    raise ValueError("Labels must be unique candidate IDs with label 0 or 1")
                labels[key] = label
        unknown = set(labels) - set(groups)
        if unknown:
            raise ValueError(f"Labels refer to absent candidate IDs: {sorted(unknown)}")
    result = {"candidates": len(groups), "labelled": len(labels), "hypothesis_rows": 0,
              "belief_accept_map_reject": [], "map_accept_belief_reject": [], "modes": {}}
    all_rows = [row for rows in groups.values() for row in rows]
    if all_rows and 'verification_status' in all_rows[0]:
        result['trial_status_counts'] = dict(Counter(row['verification_status'] for row in all_rows))
        result['polish_solves'] = sum(int(row['polish_attempted']) for row in all_rows)
        result['polish_sum_trial_elapsed_ms'] = sum(float(row['polish_ms']) for row in all_rows)
        result['installed_trials'] = sum(int(row['trial_installed']) for row in all_rows)
    predictions = {mode: {} for mode in ("belief", "map", "uniform", "geometry")}
    events = {}
    consumed = {}
    disagreements = []
    event_available = bool(all_rows) and all('event_id' in row for row in all_rows)
    for key, rows in groups.items():
        result["hypothesis_rows"] += len(rows)
        if len({row["hypothesis"] for row in rows}) != len(rows):
            raise ValueError(f"Repeated hypothesis rows in candidate {key}")
        weights = [float(row["prior_weight"]) for row in rows]
        compatibility = [float(row["compatibility"]) for row in rows]
        if not all(math.isfinite(x) and 0 <= x <= 1 for x in weights + compatibility):
            raise ValueError(f"Invalid weights/compatibilities in candidate {key}")
        if abs(sum(weights) - 1.0) > 1e-6:
            raise ValueError(f"Incomplete belief rows for candidate {key}")
        first = rows[0]
        if abs(sum(w * e for w, e in zip(weights, compatibility)) - float(first["belief_score"])) > 1e-6:
            raise ValueError(f"Inconsistent marginalized score for candidate {key}")
        map_index = max(range(len(weights)), key=lambda i: weights[i])
        expected_map = compatibility[map_index]
        expected_uniform = sum(compatibility)/len(compatibility)
        if (not math.isclose(expected_map, float(first['map_score']), abs_tol=1e-6) or
                not math.isclose(expected_uniform, float(first['uniform_score']), abs_tol=1e-6)):
            raise ValueError(f'Inconsistent MAP/uniform score for candidate {key}')
        disagreements.append({'candidate_id': key,
                              'belief_minus_MAP': float(first['belief_score'])-expected_map,
                              'non_MAP_mass_bound': 1-weights[map_index]})
        if event_available:
            event = int(first['event_id'])
            prior = {int(row['hypothesis']): float(row['prior_weight']) for row in rows}
            retained = float(first['retained_branch_mass'])
            if not math.isfinite(retained) or not 0 < retained <= 1:
                raise ValueError(f'Invalid retained branch mass in event {event}')
            previous = events.get(event)
            if previous and (previous['prior'].keys() != prior.keys() or
                             any(abs(previous['prior'][h]-w)>1e-6 for h,w in prior.items())):
                raise ValueError(f'Frozen prior changed in event {event}')
            if previous and abs(previous['retained']-retained)>1e-6:
                raise ValueError(f'Inconsistent retained branch mass in event {event}')
            events[event] = {'prior': prior, 'retained': retained}
            for row in rows:
                if (int(row['event_id']) != event or
                        not math.isclose(float(row['retained_branch_mass']), retained, abs_tol=1e-6)):
                    raise ValueError(f'Inconsistent retained branch mass/event in candidate {key}')
                if int(row['query_consumed']):
                    query = int(row['query_sequence'])
                    if query in consumed and consumed[query] != event:
                        raise ValueError(f'Query {query} consumed in multiple events')
                    consumed[query] = event
        usable = any(int(row["trial_usable"]) for row in rows)
        for mode in ("belief", "map", "uniform"):
            predictions[mode][key] = usable and float(first[f"{mode}_score"]) >= threshold
        # Recorded candidates have already passed the geometric matcher. This
        # baseline keeps the numerical fit guard but ignores trajectory distortion.
        predictions["geometry"][key] = usable
        if predictions["belief"][key] and not predictions["map"][key]:
            result["belief_accept_map_reject"].append(key)
        if predictions["map"][key] and not predictions["belief"][key]:
            result["map_accept_belief_reject"].append(key)
    for mode, decisions in predictions.items():
        metrics = {"accepted": sum(decisions.values()), "rejected": len(decisions) - sum(decisions.values())}
        if labels:
            tp = sum(decisions[key] and label == 1 for key, label in labels.items())
            fp = sum(decisions[key] and label == 0 for key, label in labels.items())
            fn = sum(not decisions[key] and label == 1 for key, label in labels.items())
            tn = sum(not decisions[key] and label == 0 for key, label in labels.items())
            metrics.update(tp=tp, fp=fp, fn=fn, tn=tn,
                           precision=tp / (tp + fp) if tp + fp else None,
                           recall=tp / (tp + fn) if tp + fn else None)
        result["modes"][mode] = metrics
    result['largest_score_disagreements'] = sorted(disagreements,
        key=lambda item: abs(item['belief_minus_MAP']), reverse=True)[:20]
    result['event_audit'] = {'available': event_available,
        'events': len(events), 'consumed_queries': len(consumed),
        'events_with_pruning': sum(e['retained']<1-1e-9 for e in events.values()),
        'maximum_discarded_branch_mass': max((1-e['retained'] for e in events.values()), default=0.)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--threshold", type=float, default=0.25)
    parser.add_argument("--labels", help="CSV with candidate_id,label; unknown candidates remain unlabelled")
    args = parser.parse_args()
    if not 0 <= args.threshold <= 1:
        parser.error("threshold must be in [0,1]")
    print(json.dumps(summarize(args.csv, args.threshold, args.labels), indent=2))


if __name__ == "__main__":
    main()
