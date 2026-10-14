# Beluga-MH
An open-source **multi-hypothesis 2D LiDAR SLAM system** built on **ROS 2** that preserves uncertainty across both local trajectory modes and competing loop-closure interpretations beyond the single MAP (Maximum A Posteriori) trajectory.

---

## Overview

In 2D LiDAR SLAM, ambiguous observations (e.g., in repetitive corridors, symmetric rooms, or open halls) often produce multiple plausible trajectories and competing loop-closure associations. Standard RBPF-SLAM and pose-graph methods prematurely collapse the belief to the single most probable (MAP) estimate, permanently discarding secondary alternatives that might later turn out to be correct.

**Beluga-MH** addresses this problem by maintaining a **hierarchical belief representation** that preserves alternatives at two levels:

1. **Frontend (Trajectory Modes):** Persistent clusters in the conditional particle population give rise to separate trajectory hypotheses, allowing initially low-mass modes to survive and dominate if subsequent LiDAR observations support them.
2. **Backend (Delayed Loop Decisions):** Ambiguous loop closures generate competing **loop** and **no-loop** graph hypotheses. Rather than committing immediately, Beluga-MH delays decisions, gathering subsequent LiDAR scan evidence over frozen validation maps until sequential Bayesian thresholds accept or reject the candidate.
3. **Memory & Real-Time Bounds:** Shared immutable submaps and **copy-on-write active maps** prevent memory explosion, enabling deterministic bounds suitable for online execution on robotic hardware.

---

## Key Features

| Component | Description |
|---|---|
| **Sensor Input** | 2D LiDAR scans (`sensor_msgs/LaserScan`) and odometry (`nav_msgs/Odometry`) |
| **Hierarchical Belief** | Hypotheses maintain their own pose graph, active submaps, and conditional particles |
| **Motion Model** | Differential drive with noise parameters |
| **Scan Matching** | Distance-Field Scan Matching + Ceres probability field tracking |
| **Particle Proposal** | Scan-informed proposals combining frontend matcher alignment with odometric priors |
| **Submaps** | Cartographer-style overlapping submaps (50% overlap, 60 keyframes per finished submap) |
| **Place Recognition** | 2D Scan Context++ descriptor (ring key + sector key + circular yaw seed) |
| **Loop Verification** | SE(2) trial PGO trajectory deformation compatibility gate |
| **Delayed Decisions** | Sequential Bayesian evidence testing on frozen validation submaps |
| **Backend Optimization** | Ceres Solver pose-graph optimization with exact analytic SE(2) Jacobians and Huber robust loss |
| **Resampling** | Conditional ESS-triggered resampling with largest-remainder allocation of particle budget |
| **Memory Efficiency** | Shared immutable submaps + copy-on-write (COW) active grid detachment |

---

## Architecture

```
beluga2.5/
├── belugaslam_core/        # Core SLAM algorithms (ROS-independent C++17 library)
│   ├── include/
│   │   ├── belugaslam_core/ # Multi-hypothesis tracking, submaps, Scan Context 2D, PGO, Bayes evidence
│   │   └── beluga/          # Vendored Beluga 2.0 primitives (MCL, motion & sensor models)
│   ├── src/
│   └── config/
│       └── grid_config.py   # Predefined environment geometry presets (generates grid_config.hpp)
│
├── belugaslam_node/         # ROS 2 node wrapper
│   ├── src/
│   │   ├── belugaslam_node.cpp            # Node entrypoint
│   │   └── fastslam_oc_grid_node.cpp      # Parameter registration, ROS 2 pub/sub, TF broadcast
│   └── launch/
│       └── fastslam_oc_grid.launch.py     # Launch file with 70+ configurable parameters
│
├── belugaslam_example/      # Dataset replays, example XML launch files, and RViz configs
│   ├── example/launch/      # Presets for Intel and MIT datasets (with/without pure frontend)
│   ├── bags/                # Datasets (Intel, MIT, and HQ simulation)
│   └── RVIZFiles/           # RViz visualization configuration
│
├── belugaslam_benchmark/    # Benchmarking and profiling toolsuite
│   ├── benchmarking/        # Automated sweeps, EVO RMSE scripts, map comparison
│   └── profiling/           # CPU flamegraph profiling via perf
│
└── docs/images/             # Maps, EVO trajectory plots, flamegraphs
```

### ROS 2 Interface

| Direction | Topic | Type | Description |
|---|---|---|---|
| **Subscribe** | `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR range measurements |
| **Subscribe** | `/odom` | `nav_msgs/Odometry` | Wheel or visual odometry increments |
| **Publish** | `/map` | `nav_msgs/OccupancyGrid` | Map from the selected consistent hypothesis |
| **Publish** | `/particle_cloud` | `geometry_msgs/PoseArray` | Multi-hypothesis particle population |
| **Publish** | `/estimated_pose` | `geometry_msgs/PoseStamped` | Current estimated robot pose |
| **Publish** | `/loop_closure_markers`| `visualization_msgs/MarkerArray` | Active and trial loop constraints |
| **TF** | `map` → `odom` | `tf2_ros` | Broadcasts map-to-odom transform |

---

## Dependencies

### Core Build Dependencies

- **ROS 2** (rclcpp, nav_msgs, sensor_msgs, tf2_ros, visualization_msgs)
- **C++17** compiler
- **Eigen3** (linear algebra)
- **Sophus** (SE(2) Lie groups)
- **Ceres Solver** (scan matching and pose-graph optimization)
- **Intel TBB** (parallel particle processing and loop candidate evaluations)
- **CMake ≥ 3.16** and **Python 3**

### Optional (Evaluation & Profiling)

- [**EVO**](https://github.com/MichaelGrupp/evo) (trajectory error analysis: APE, RPE)
<!-- - **perf** + [**FlameGraph**](https://github.com/brendangregg/FlameGraph) (CPU profiling) TODO: add profiling -->

---

## Usage Instructions

For detailed usage instructions and examples, please refer to the [Example README](belugaslam_example/README.md).

---
<!--
## Results

Some representative results of the FastSLAM implementation are shown in this [README](RESULTS.md), including occupancy grid maps generated from different datasets.


-->
---

## Acknowledgments & Bibliography

- **Beluga:** Inspired by and builds upon the [Beluga](https://github.com/ekumenlabs/beluga) library for modern Monte Carlo Localization in ROS 2.
- **Theoretical Foundations:** Based on probabilistic robotics concepts originally introduced in *Probabilistic Robotics* (Thrun, Burgard, Fox) and extended with multi-hypothesis belief modeling and delayed Bayesian decision-making.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).  
