# How to profile Beluga 2.5

## Generating a flamegraph from recorded perf events

A flamegraph is a convenient tool for understanding how CPU time is being used.

1. Run the following script to generate the profiling data:
    ```bash
    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws
    colcon build --packages-select belugaslam_core belugaslam_node belugaslam_benchmark belugaslam_example
    source install/local_setup.bash
    ros2 run belugaslam_benchmark profile_belugaslam_with_bagfile
    ```
    >**Note:** Set **env=3** in the configuration file.

    `perf` will generate a `perf.data` file in the folder it was run.

2. To generate a flamegraph from the recorded data, run:
    ```bash
    source install/local_setup.bash
    ros2 run belugaslam_benchmark flamegraph
    ```
    To visualize the flamegraph and be able to zoom it in or out, open the generated `svg` file in a web-browser.

## References

- https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- https://www.brendangregg.com/perf.html
- https://github.com/Ekumen-OS/beluga/tree/main/beluga_benchmark

## Profiling (Flamegraph)

[BelugaSLAM Flamegraph example](../../docs/images/flamegraph.svg)

