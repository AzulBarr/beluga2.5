> Latest update: [ADAPTIVE_PRIOR.md](ADAPTIVE_PRIOR.md) adds the motion-dependent full covariance prior (step 2) while keeping probability-grid Ceres matching (step 1). Includes a fixed-vs-odometry prior comparison and exact build/run commands.

> Latest frontend experiment: [PROBABILITY_MATCHER.md](PROBABILITY_MATCHER.md) adds `tracking_matcher:=probability_ceres`, spatial filtering and an automatic A/B replay. Read the compilation limits and test commands there. `distance` remains the default.

> Bayesian update: [BAYESIAN_UPDATE.md](BAYESIAN_UPDATE.md) implements hierarchical masses and delayed loop evidence. The Intel launch now defaults to `loop_update_mode:=bayes`. Read its build/test status and commands before running. Earlier review files describe historical revisions.

> Current pose-accuracy revision: see [RMSE_REVISION_20260909.md](RMSE_REVISION_20260909.md) for fixes, architecture limitations, validation and replay instructions. Earlier review files describe historical revisions.

# Beluga-SLAM

## Overview

**Beluga-SLAM** is a FastSLAM-based implementation for 2D simultaneous localization and mapping (SLAM) using occupancy grids and 2D LiDAR data.

The project builds on the design principles of particle filters and probabilistic robotics.

---

## Features

- ** algorithm** with per-particle maps  
- **Log-odds grid representation** for efficient occupancy updates  
- **Likelihood field measurement model** for robust sensor integration  
- **Differential drive motion model**  
- Modular design with clear separation between:
  - Core SLAM logic (`belugaslam_core`)
  - ROS 2 node interface (`belugaslam_node`)

---

## Architecture

The project is structured in two main components:

### `belugaslam_core`

Core SLAM implementation independent of ROS:
- Particle representation
- Motion and measurement models
- Occupancy grid mapping
- Resampling

### `belugaslam_node`

ROS 2 interface layer:
- Parameter handling
- Topic subscriptions (e.g., LiDAR, odometry)
- Publishing maps and poses

---
## Dependencies

- ROS 2
- C++
- Eigen
- Sophus
- Beluga library

---
## Usage Instructions

For detailed usage instructions and examples, please refer to the [Example README](belugaslam_example/README.md).

---
## Results

Some representative results of the FastSLAM implementation are shown in this [README](RESULTS.md), including occupancy grid maps generated from different datasets.

---
## Bibliography

The theoretical foundations of this project are primarily based on the book **Probabilistic Robotics** by Sebastian Thrun, Wolfram Burgard, and Dieter Fox.

All models, assumptions, and algorithmic approaches (including FastSLAM, motion models, and sensor models) follow the formulations presented in this book.

---
## Acknowledgments

This project is inspired by and builds upon the ideas and design of the Beluga 2.0 library:

👉 https://github.com/ekumenlabs/beluga

Beluga provides a modern and modular framework for Monte Carlo Localization in robotics.

---
