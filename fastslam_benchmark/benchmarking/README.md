# How to benchmark Beluga 2.5

## Run rosbag benchmark parameterizing the number of particles

This script will run the benchmark with the specified number of particles and record:

- Execution time and CPU usage statistics.
- Memory usage metrics.
- Full console output of the SLAM execution.
- The final map generated.
- A rosbag with the reference and estimated trajectories.

To run, use:

```bash
ros2 run fastslam_benchmark parameterized_run.sh <NUMBER_OF_PARTICLES_EXPERIMENT_1> <NUMBER_OF_PARTICLES_EXPERIMENT_2>
```

The results of the different runs will be stored in folders named `benchmark_${N_PARTICLES}_particles_output`, where `N_PARTICLES` are the numbers specified in the above command.

For other options, e.g. using a different launch file, see:

```bash
ros2 run fastslam_benchmark parameterized_run.sh --help
```
---
## Visualizing results
Use the following command:
```bash
ros2 run fastslam_benchmark time_results.py <PATH_TO_OUTPUT_DIR_OF_RUN>
```
>**Note:** If **<PATH_TO_OUTPUT_DIR_OF_RUN>** is a folder containing the outputs of varius benchmarking runs, and only that, the command allows to compare their times in a plot.

