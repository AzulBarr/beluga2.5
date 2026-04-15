# Usage
## Run the node using any ROS bag
    For ROS 2 distributions, in three separate terminals run:
1. **Build the packages and open RViz**.

    ```bash
    cd ~/ros2_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select fastslam_core fastslam_node fastslam_example fastslam_benchmark
    source install/setup.bash
    rviz2
    ```

2. **Launch the fastslam node**.

    ```bash
    cd ~/ros2_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch fastslam_node fastslam_oc_grid.launch.py
    ```
    > **Note for Beluga ROS bag:** If you are using the Beluga dataset, you must append the specific frame and topic arguments to the launch command:
    ```bash
    ros2 launch fastslam_node fastslam_oc_grid.launch.py num_particles:=800 base_frame:=base odom_frame:=odom scan_topic:=/scan
    ```

3. **Play the ros bag**.

    ```bash
    cd ~/ros2_ws/bag_files
    source /opt/ros/jazzy/setup.bash
    ros2 bag play ros2_bag --clock
    ```

## Run an example using a Beluga ROS bag (Quick Start).

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fastslam_core fastslam_node fastslam_example fastslam_benchmark 
source install/setup.bash
ros2 launch fastslam_example beluga_rosbag_fastslam.xml
```

## Configuration
Modify the config.py file located in [fastslam_core/config/grid_config.py](../../fastslam_core/config/grid_config.py) to decide the size and resolution of the map.