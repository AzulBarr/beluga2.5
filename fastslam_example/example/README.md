# How to use
1. **Run the node using a ROS bag**.

    For ROS 2 distributions, in two separate terminals run:
    ```bash
    cd /ros2_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select fastslam_core fastslam_node
    source install/setup.bash
    ros2 launch fastslam_node fastslam_oc_grid.launch.py
    ```

    ```bash
    cd /ros2_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    rviz2
    ```