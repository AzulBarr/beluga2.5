<!--# Usage
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
chmod +x src/beluga2.5/belugaslam_example/bags/intel/intel_clf_to_ros2.py 
ros2 launch belugaslam_example intel_dataset_belugaslam.xml record_bag:=true
```
>**Note:** Set **env=2** in the configuration file.

-->

## Build 

```bash
# 1. Source your ROS 2 installation
source /opt/ros/${ROS_DISTRO}/setup.bash

# 3. Build workspace
cd ~/ros2_ws
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example
source install/setup.bash
```
<!-- # 2. Select environment preset in belugaslam_core/config/grid_config.py:
#    ENV = 1  →  MIT Stata Center
#    ENV = 2  →  Intel Research Lab
#    ENV = 3  →  Beluga HQ (profiling)
#    ENV = 4  →  Beluga HQ Simulation


> **Note:** `grid_config.py` runs during CMake build to produce `grid_config.hpp`. Changing `ENV` requires rebuilding `belugaslam_core`.
-->
---

## Usage

### Quick Start — Intel Research Lab Dataset

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash

# Ensure dataset converter script is executable
chmod +x src/beluga2.5/belugaslam_example/bags/intel/intel_clf_to_ros2.py

# Launch Beluga-MH, RViz, and synchronized bag replay
ros2 launch belugaslam_example intel_dataset_belugaslam.xml record_bag:=true
```

<!-- *(Ensure `ENV = 2` is set in `grid_config.py`).*-->

### Manual Launch (Three Terminals)

**Terminal 1 — RViz:**
```bash
cd ~/ros2_ws
source install/setup.bash
rviz2 -d src/beluga2.5/belugaslam_example/RVIZFiles/config.rviz
```

**Terminal 2 — Beluga-MH Node:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch belugaslam_node fastslam_oc_grid.launch.py \
  num_particles:=35 \
  base_frame:=base_footprint \
  odom_frame:=odom_combined \
  scan_topic:=/base_scan \
  range_max:=30.0
```

**Terminal 3 — Replay Rosbag:**
```bash
ros2 bag play <path_to_rosbag> --clock
```

---

## Configuration & Parameters
<!--
### Predefined Environments (`grid_config.py`)

| ENV | Dataset | Grid Dimensions | Resolution | Origin Offset |
|---|---|---|---|---|
| **1** | MIT Stata Center | 250 × 400 | 0.10 m | Centered offset |
| **2** | Intel Research Lab | 350 × 350 | 0.10 m | Centered offset |
| **3** | Beluga HQ (Profiling) | 111 × 111 | 0.05 m | Centered offset |
| **4** | HQ Simulation | 500 × 500 | 0.075 m | Centered offset |

*Submaps are decoupled from the global grid and sized to 12 m × 12 m.*
-->
### Key Launch Parameters

<details>
<summary><b>Multi-Hypothesis & Particle Budget</b></summary>

| Parameter | Default | Description |
|---|---|---|
| `max_hypotheses` | 4 | Maximum concurrent graph hypotheses ($H_{\max}$) |
| `num_particles` | 35 | Total particle budget across hypotheses ($N_{\max}$) |
| `split_min_mass` | 0.02 | Minimum mass required to fork a new spatial mode |
| `split_min_particles` | 2 | Minimum particles required to form a spatial cluster |
| `split_persistence` | 3 | Scans a spatial mode must persist before branching |
| `hypothesis_prune_mass` | 1e-06 | Prune hypotheses falling below this posterior mass |
| `random_seed` | 42 | Seed for reproducible execution (0 = random) |

</details>

<details>
<summary><b>Frontend Tracking & Proposals</b></summary>

| Parameter | Default | Description |
|---|---|---|
| `tracking_matcher` | `probability_ceres` | Frontend matcher (`distance` or `probability_ceres`) |
| `tracking_prior_mode` | `odometry` | Matcher motion prior (`odometry` or `fixed`) |
| `scan_informed_proposal` | true | Enable scan-informed particle proposal distribution |
| `motion_proposal_samples` | 8 | Number of proposal candidates ($K$) evaluated per particle |
| `deskew_scan` | true | Compensate within-scan robot motion |
| `tracking_effective_beams` | 20.0 | Effective LiDAR beams used for measurement evidence |
| `alpha1` – `alpha4` | 0.1, 0.05, 0.1, 0.05 | Differential drive odometry noise coefficients |

</details>

<details>
<summary><b>Submaps & Keyframes</b></summary>

| Parameter | Default | Description |
|---|---|---|
| `submap_num_range_data` | 30 | Keyframes before starting next overlapping submap |
| `keyframe_min_translation` | 0.15 m | Minimum displacement for keyframe insertion |
| `keyframe_min_rotation` | 0.087 rad | Minimum rotation for keyframe insertion |
| `keyframe_max_time` | 5.0 s | Max time between keyframes |
| `map_resolution` | 0.05 m | Published occupancy grid resolution |

</details>

<details>
<summary><b>Multi-Hypothesis Loop Closure & Verification</b></summary>

| Parameter | Default | Description |
|---|---|---|
| `enable_loop_closure` | true | Enable multi-hypothesis loop search and branching |
| `loop_use_scan_context_2d` | true | Enable 2D Scan Context++ place recognition |
| `loop_candidate_distance` | 10.0 m | Maximum pose prior search distance |
| `loop_min_score` | 0.55 | Minimum registration score for candidates |
| `loop_min_overlap` | 0.35 | Minimum scan overlap fraction with submap |
| `loop_update_mode` | `bayes` | `bayes` (sequential evidence) or `heuristic` (ablation) |
| `loop_verifier_mode` | `belief` | Verifier mode (`belief` required for `bayes`; `map`, `uniform`, `geometry` for heuristic ablation) |
| `loop_bayes_accept_probability` | 0.95 | Threshold $\tau_A$ to commit to the loop branch |
| `loop_bayes_reject_probability` | 0.05 | Threshold $\tau_R$ to discard the loop branch |
| `loop_bayes_min_scans` | 10 | Minimum future scans before deciding |
| `loop_undecided_max_scans` | 20 | Window before forcing decision on ambiguous events |

</details>

<details>
<summary><b>Pose Graph Optimization (PGO)</b></summary>

| Parameter | Default | Description |
|---|---|---|
| `enable_pgo` | true | Enable backend pose graph optimization |
| `pgo_every_n_nodes` | 20 | Trigger PGO every N inserted keyframes |
| `pgo_max_iterations` | 50 | Maximum Ceres iterations per solve |
| `pgo_analytic_jacobians` | true | Use exact analytic SE(2) Jacobians (faster than AutoDiff) |
| `pgo_huber_scale` | 1.0 | Robust loss Huber scale |
| `pgo_loop_translation_weight` | 10.0 | Information weight on loop translation constraints |
| `pgo_loop_rotation_weight` | 12.0 | Information weight on loop rotation constraints |

</details>

<details>
<summary><b>Diagnostics & Output Selection</b></summary>

| Parameter | Default | Description |
|---|---|---|
| `output_selection_mode` | `map` | `map` (highest mass) or `pose_risk` (minimum risk) |
| `final_trajectory_path` | `""` | Optional CSV export of final online + PGO trajectory |
| `detection_events_path` | `""` | Optional CSV export of hypothesis forking and loop events |
| `optimized_trajectory_path` | `""` | Write PGO trajectory in TUM format on clean shutdown |

</details>

---

## Experimental Results

Beluga-MH was evaluated against **SLAM Toolbox** (the default Nav2 SLAM system) and **Beluga-MAP** (an ablation of Beluga-MH that immediately collapses to the highest-mass hypothesis at every bifurcation).

### Trajectory Accuracy: APE RMSE (m)

*Values averaged across multiple independent runs:*

| System | Intel Research Lab | HQ Simulation | MIT Stata Center |
|---|:---:|:---:|:---:|
| **Beluga-MH** | **0.076** | **0.020** | **0.085** |
| Beluga-MAP (Ablation) | 0.082 | 0.022 | 0.088 |
| SLAM Toolbox | 0.266 | 0.023 | 0.086 |

*Beluga-MH achieves the lowest average APE RMSE across all datasets, with the most pronounced advantage on the longest and most ambiguous sequence (Intel Research Lab), where delaying loop commitments and maintaining alternative trajectory modes prevents catastrophic filter collapse.*

<!--
<table align="center">
  <tr>
    <td align="center">
      <img src="docs/images/mapa_belugaslam_hq.png" width="280px" alt="HQ Simulation map"/>
      <br><sub>HQ Simulation</sub>
    </td>
    <td align="center">
      <img src="docs/images/mapa_belugaslam_intel.png" width="280px" alt="Intel Research Lab map"/>
      <br><sub>Intel Research Lab</sub>
    </td>
    <td align="center">
      <img src="docs/images/mapa_belugaslam_mit.png" width="280px" alt="MIT Stata Center map"/>
      <br><sub>MIT Stata Center</sub>
    </td>
  </tr>
</table>

For additional trajectory plots and detailed metric breakdowns, see [RESULTS.md](RESULTS.md).

---

## Benchmarking & Profiling

### Automated Benchmarks

Execute multi-particle parameter sweeps:
```bash
ros2 run belugaslam_benchmark parameterized_run.sh <PARTICLES_MIN> <PARTICLES_MAX>
```

Compare execution time and trajectory statistics across runs:
```bash
ros2 run belugaslam_benchmark compare_results.py <PATH_TO_OUTPUT_DIR>
```

### CPU Flamegraph Profiling

Generate interactive SVG flamegraphs using Linux `perf`:
```bash
ros2 run belugaslam_benchmark profile_belugaslam_with_bagfile
ros2 run belugaslam_benchmark flamegraph
```

<p align="center">
  <img src="docs/images/flamegraph.svg" width="700px" alt="CPU flamegraph profile"/>
  <br>
  <sub>Flamegraph showing CPU execution breakdown of Beluga-MH.</sub>
</p>


---
## Configuration
Modify the config.py file located in [belugaslam_core/config/grid_config.py](../belugaslam_core/config/grid_config.py) to decide the size and resolution of the map.
-->