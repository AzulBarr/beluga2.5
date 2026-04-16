#!/usr/bin/env python3
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker 

def parse_time_file(file_path: Path):
    metrics = {}
    if not file_path.exists():
        return None

    with open(file_path, "r") as f:
        for line in f:
            if "User time" in line:
                metrics["user_time"] = float(line.split(":")[1])
            elif "System time" in line:
                metrics["system_time"] = float(line.split(":")[1])
            elif "Percent of CPU" in line:
                metrics["cpu_percent"] = float(line.split(":")[1].replace("%", ""))
            elif "Elapsed (wall clock) time" in line:
                time_str = line.strip().split(": ")[1] 
                metrics["elapsed"] = parse_time_format(time_str)
            elif "Maximum resident set size" in line:
                metrics["memory_kb"] = int(line.split(":")[1])

    return metrics

def parse_time_format(t):
    """Convierte h:mm:ss o mm:ss a segundos totales."""
    parts = t.split(':')
    if len(parts) == 3:  # h:mm:ss
        return float(parts[0]) * 3600 + float(parts[1]) * 60 + float(parts[2])
    elif len(parts) == 2:  # mm:ss
        return float(parts[0]) * 60 + float(parts[1])
    return float(t)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dir", type=Path, help="Benchmark output folder")
    args = parser.parse_args()

    base_dir = Path(args.dir)
    folders = sorted(list(base_dir.glob("bench_output_*")), 
                    key=lambda x: int(x.name.split("_")[2]))

    if folders:
        particles = []
        times = []

        for folder in folders:
            try:
                N = int(folder.name.split("_")[2])
                m = parse_time_file(folder / "time_stats.txt")
                
                if m and "elapsed" in m:
                    particles.append(N)
                    times.append(m["elapsed"])
                    print(f"Parsed {N} particles: {m['elapsed']:.2f}s")
            except Exception as e:
                print(f"Skipping {folder.name}: {e}")

        if not times:
            print("No data found to plot.")
            return

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(particles, times, marker="o", linestyle='-', color='b', label='Execution Time')
        
        ax.set_xlabel("Number of Particles")
        ax.set_ylabel("Time (seconds)")
        ax.set_title("FastSLAM Execution Time vs Particles")
        
        ax.grid(True, which="both", ls="-", alpha=0.5)
        ax.legend()
        
        plt.tight_layout()
        plt.show()

    else:
        print(f"No folders matching 'bench_output_*' found in {base_dir}")

if __name__ == "__main__":
    main()