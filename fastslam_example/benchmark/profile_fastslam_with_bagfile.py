#!/usr/bin/env python3

import os
import subprocess
import argparse
import sys
from ament_index_python.packages import get_package_share_directory

def setup_kernel():
    """Ensures the kernel allows perf usage without sudo privileges."""
    print("Configuring kernel parameters for profiling...")
    os.system('sudo sysctl -w kernel.perf_event_paranoid=-1')
    os.system('sudo sysctl -w kernel.kptr_restrict=0')

def main():
    parser = argparse.ArgumentParser(description="Replicating Beluga's profiler for FastSLAM.")
    parser.add_argument('--bag_path', type=str, 
                        default='/home/azul/ros2_ws/src/fastslam_oc_grid/fastslam_example/bags/beluga_rosbag',
                        help='Path to the rosbag file.')
    parser.add_argument('--rate', type=float, default=1.0, help='Playback rate.')
    args = parser.parse_args()

    # 1. Kernel setup
    setup_kernel()

    # 2. Output path for PERF data
    output_file = os.path.join(os.getcwd(), "perf.data")

    # 3. Define the PERF prefix (matching Beluga's configuration)
    # -g: enables call-graph recording
    # --call-graph dwarf: use DWARF to unwind the stack
    # -F 99: sampling frequency in Hz
    perf_prefix = f"perf record -e cycles -F 99 -g --call-graph dwarf -o {output_file} --"

    # 4. Locate the package share directory to find the XML launch file
    try:
        pkg_share = get_package_share_directory('fastslam_example')
    except Exception as e:
        print(f"Error: Could not find package 'fastslam_example'. Did you source the workspace?")
        sys.exit(1)

    launch_file = os.path.join(pkg_share, 'example', 'launch', 'beluga_rosbag_fastslam.xml')

    # 5. Build the final command
    cmd = [
        'ros2', 'launch', 'fastslam_example', 'beluga_rosbag_fastslam.xml',
        f'bag_path:={args.bag_path}',
        f'slam_prefix:={perf_prefix}',
        f'bag_rate:={args.rate}',
        'use_sim_time:=true'
    ]

    print(f"\n--- Starting Beluga-style profiling ---")
    print(f"Output file: {output_file}\n")

    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass
    
    if os.path.exists(output_file):
        print(f"\n--- Profiling finished. Data saved in {output_file} ---")
    else:
        print(f"\n--- Profiling finished. Warning: {output_file} was not generated. ---")

if __name__ == '__main__':
    main()