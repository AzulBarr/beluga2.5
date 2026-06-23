#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import shutil
import sys

if shutil.which("evo_ape") is None:
    print("ERROR: evo_ape not found in PATH")
    sys.exit(1)

UPDATE_RE = re.compile(
    r"Particle filter update iteration stats: .* - ([0-9.]+)ms"
)


def parse_time_format(t):
    parts = t.split(':')

    if len(parts) == 3:
        return float(parts[0]) * 3600 + float(parts[1]) * 60 + float(parts[2])

    elif len(parts) == 2:
        return float(parts[0]) * 60 + float(parts[1])

    return float(t)


def parse_time_file(file_path):

    metrics = {}

    if not file_path.exists():
        return metrics

    with open(file_path, "r") as f:
        for line in f:

            if "Percent of CPU" in line:
                metrics["cpu_percent"] = float(
                    line.split(":")[1].replace("%", "")
                )

            elif "Elapsed (wall clock) time" in line:
                metrics["elapsed"] = parse_time_format(
                    line.strip().split(": ")[1]
                )

    return metrics


def parse_console_log(log_path):

    updates = []

    if not log_path.exists():
        return {}

    with open(log_path, "r") as f:

        for line in f:

            m = UPDATE_RE.search(line)

            if m:
                updates.append(float(m.group(1)))

    if len(updates) == 0:
        return {}

    updates = np.array(updates)

    return {
        "mean_update_ms": np.mean(updates),
        "n_updates": len(updates)
    }


def parse_rmse(folder):

    rmse_file = folder / "rmse.txt"

    if not rmse_file.exists():
        return None

    try:
        return float(rmse_file.read_text().strip())
    except:
        return None


def extract_particles(folder_name):

    m = re.search(r"bench_output_(\d+)_particles", folder_name)

    if m:
        return int(m.group(1))

    return None


def plot_metric(df, column, ylabel, output_file):

    plt.figure(figsize=(8, 5))

    plt.plot(
        df["particles"],
        df[column],
        marker="o"
    )

    plt.xlabel("Maximum number of particles")
    plt.ylabel(ylabel)
    plt.grid(True)

    plt.tight_layout()

    plt.savefig(output_file, dpi=300)
    plt.close()


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "dir",
        type=Path,
        help="Benchmark output directory"
    )

    args = parser.parse_args()

    rows = []

    folders = sorted(
        args.dir.glob("bench_output_*_particles"),
        key=lambda p: extract_particles(p.name)
    )

    for folder in folders:

        particles = extract_particles(folder.name)

        if particles is None:
            continue

        time_metrics = parse_time_file(
            folder / "time_stats.txt"
        )

        update_metrics = parse_console_log(
            folder / "console_output.log"
        )

        rmse = parse_rmse(folder)

        row = {
            "particles": particles,
            "cpu_percent": time_metrics.get("cpu_percent"),
            "elapsed_s": time_metrics.get("elapsed"),
            "rmse_m": rmse,
        }

        row.update(update_metrics)

        rows.append(row)

    if len(rows) == 0:
        print("No benchmark results found.")
        return

    df = pd.DataFrame(rows)

    df = df.sort_values("particles")

    print("\nBenchmark summary\n")
    print(df)

    csv_file = args.dir / "benchmark_summary.csv"

    df.to_csv(csv_file, index=False)

    print(f"\nSaved {csv_file}")

    plot_metric(
        df,
        "mean_update_ms",
        "Mean update time [ms]",
        args.dir / "mean_update_vs_particles.png"
    )

    plot_metric(
        df,
        "cpu_percent",
        "CPU usage [%]",
        args.dir / "cpu_vs_particles.png"
    )

    if df["rmse_m"].notna().any():

        plot_metric(
            df,
            "rmse_m",
            "RMSE [m]",
            args.dir / "rmse_vs_particles.png"
        )

        plt.figure(figsize=(8, 5))

        plt.plot(
            df["mean_update_ms"],
            df["rmse_m"],
            marker="o"
        )

        for _, row in df.iterrows():
            plt.annotate(
                str(int(row["particles"])),
                (row["mean_update_ms"], row["rmse_m"])
            )

        plt.xlabel("Mean update time [ms]")
        plt.ylabel("RMSE [m]")
        plt.grid(True)

        plt.tight_layout()

        plt.savefig(
            args.dir / "rmse_vs_update_time.png",
            dpi=300
        )

        plt.close()


if __name__ == "__main__":
    main()