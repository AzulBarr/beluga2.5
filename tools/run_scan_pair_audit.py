#!/usr/bin/env python3
"""Run the production matcher on scan pairs without ground truth; not full SLAM."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--include-directory", type=Path,
                        default=root / "belugaslam_core/include")
    parser.add_argument("--prior-information-scale", type=float, default=1.0,
                        help="Explicit experimental matcher regularization; default preserves legacy behavior")
    args = parser.parse_args()
    if not math.isfinite(args.prior_information_scale) or args.prior_information_scale<=0:
        parser.error('prior information scale must be positive')
    sys.path.insert(0, str(root / "belugaslam_example/bags/intel"))
    from carmen_reader import load_ordered_flaser
    records, backwards = load_ordered_flaser(args.recording)
    if len(records) < 2:
        raise ValueError("At least two scans are required")
    destination = args.output_directory.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    csv_path = destination / "scan_pairs.csv"
    with tempfile.TemporaryDirectory(prefix="beluga_pair_audit_") as directory:
        temporary = Path(directory)
        frames = temporary / "frames.txt"
        with frames.open("w", encoding="utf-8") as output:
            for record in records:
                points = [(distance * math.cos(math.radians(i - 90)),
                           distance * math.sin(math.radians(i - 90)))
                          for i, distance in enumerate(record.ranges)
                          if math.isfinite(distance) and .1 < distance < 25.0]
                values = [f"{(record.timestamp_ns-records[0].timestamp_ns)/1e9:.9f}",
                          *(format(v, ".17g") for v in record.odometry), str(len(points)),
                          *(format(v, ".17g") for point in points for v in point)]
                output.write(" ".join(values) + "\n")
        executable = temporary / "scan_pairs"
        source = root / "tools/audit_scan_pairs.cpp"
        subprocess.run(["g++", "-std=c++17", "-O3", "-Wall", "-Wextra",
                        "-I" + str(args.include_directory.resolve()), str(source),
                        "-o", str(executable)], check=True)
        subprocess.run([str(executable), str(frames), str(csv_path), str(args.prior_information_scale)], check=True)
    with csv_path.open(newline="", encoding="utf-8") as source:
        rows = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(source)]
    if len(rows) != len(records) - 1:
        raise RuntimeError("Incomplete pair audit")
    both = [row for row in rows if row["forward_accepted"] and row["reverse_accepted"]]

    def metric(key, selected):
        if not selected:
            return None
        values = sorted(row[key] for row in selected)
        return {"median": statistics.median(values),
                "p95": values[int(.95 * (len(values)-1))],
                "p99": values[int(.99 * (len(values)-1))], "max": values[-1]}

    summary = {
        "scope": "Production matcher, consecutive single-scan endpoint fields. "
                 "Two-thirds fit points; one-third held out. No accumulated submaps, "
                 "PF, recovery, PGO, reference trajectory, absolute pose RMSE, or map-quality metric.",
        "recording_sha256": hashlib.sha256(args.recording.read_bytes()).hexdigest(),
        "matcher_sha256": hashlib.sha256((args.include_directory /
                           "belugaslam_core/robust_tracking.hpp").read_bytes()).hexdigest(),
        "frames": len(records), "pairs": len(rows),
        "prior_information_scale": args.prior_information_scale,
        "out_of_order_before_sort": backwards,
        "forward_accepted": sum(int(row["forward_accepted"]) for row in rows),
        "both_directions_accepted": len(both),
        "held_out_mean_log_gain": statistics.mean(
            row["matched_held_log"]-row["prior_held_log"] for row in rows),
        "cycle_m_for_accepted_pairs": metric("cycle_m", both),
        "cycle_rad_for_accepted_pairs": metric("cycle_rad", both),
        "pair_matching_ms_excluding_field_build": metric("match_ms", rows),
        "worst_accepted_cycles": sorted(both, key=lambda row: row["cycle_m"], reverse=True)[:10],
    }
    (destination / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key != "worst_accepted_cycles"}, indent=2))


if __name__ == "__main__":
    main()
