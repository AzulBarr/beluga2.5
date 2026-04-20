# How to profile Beluga 2.5

## Generating a flamegraph from recorded perf events

A flamegraph is a convenient tool for understanding how CPU time is being used.

1. Run the following script to generate the profiling data:
    ```bash
    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws
    colcon build --packages-select fastslam_core fastslam_node
    source install/local_setup.bash
    ros2 run fastslam_benchmark profile_fastslam_with_bagfile
    ```
    `perf` will generate a `perf.data` file in the folder it was run.

2. To generate a flamegraph from the recorded data, run:
    ```bash
    source install/local_setup.bash
    ros2 run fastslam_benchmark flamegraph
    ```
    To visualize the flamegraph and be able to zoom it in or out, open the generated `svg` file in a web-browser.

## References

- https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- https://www.brendangregg.com/perf.html
- https://github.com/Ekumen-OS/beluga/tree/main/beluga_benchmark

## Profiling (Flamegraph)

[FastSLAM Flamegraph](../../docs/images/flamegraph.svg)

