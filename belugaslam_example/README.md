# Usage
## Running BelugaSLAM with a ROS bag

For ROS 2 Jazzy, open three separate terminals and follow the steps below.

1. **Build the packages and start RViz**.

    ```bash
    cd ~/ros2_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark
    source install/setup.bash
    rviz2
    ```

2. **Launch the BelugaSLAM node**

    ```bash
    cd ~/ros2_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch belugaslam_node fastslam_oc_grid.launch.py
    ```
    > **Note**: You may specify custom parameters when launching the node:
    ```bash
    ros2 launch belugaslam_node fastslam_oc_grid.launch.py \ 
    min_particles:=5 \ 
    max_particles:=30 \ 
    base_frame:=base_footprint \ 
    odom_frame:=odom_combined \ 
    scan_topic:=/base_scan \ 
    range_max:=60.0
    ```

3. **Play the ROSBag**.

    ```bash
    cd ~/ros2_ws/bag_files
    source /opt/ros/jazzy/setup.bash
    ros2 bag play ros2_bag --clock
    ```

---

## Run an example using the Intel dataset (Quick Start).

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark 
source install/setup.bash
chmod +x src/fastslam_oc_grid/belugaslam_example/bags/intel/intel_clf_to_ros2.py
ros2 launch belugaslam_example intel_dataset_belugaslam.xml record_bag:=true
```
>**Note:** Set **env=2** in the configuration file.

---
## Configuration
Modify the config.py file located in [belugaslam_core/config/grid_config.py](../belugaslam_core/config/grid_config.py) to decide the size and resolution of the map.