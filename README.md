# Beluga 2.5

## Overview

**Beluga 2.5** is a FastSLAM-based implementation for 2D simultaneous localization and mapping (SLAM) using occupancy grids and 2D LiDAR data.

The project builds on the design principles of particle filters and probabilistic robotics.

---

## Features

- **FastSLAM algorithm** with per-particle maps  
- **Log-odds grid representation** for efficient occupancy updates  
- **Likelihood field measurement model** for robust sensor integration  
- **Differential drive motion model**  
- Modular design with clear separation between:
  - Core SLAM logic (`fastslam_core`)
  - ROS 2 node interface (`fastslam_node`)

---

## Architecture

The project is structured in two main components:

### `fastslam_core`

Core SLAM implementation independent of ROS:
- Particle representation
- Motion and measurement models
- Occupancy grid mapping
- Resampling

### `fastslam_node`

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

For detailed usage instructions and examples, please refer to the [Example README](fastslam_example/example/README.md).

## Results

Some representative results of the FastSLAM implementation are shown in this [README](RESULTS.md), including occupancy grid maps generated from different datasets.

---
## Bibliography

The theoretical foundations of this project are primarily based on the book *Probabilistic Robotics* by :contentReference[oaicite:0]{index=0}, :contentReference[oaicite:1]{index=1}, and :contentReference[oaicite:2]{index=2}.

All models, assumptions, and algorithmic approaches (including FastSLAM, motion models, and sensor models) follow the formulations presented in this reference.

---
## Acknowledgments

This project is inspired by and builds upon the ideas and design of the Beluga 2.0 library:

👉 https://github.com/ekumenlabs/beluga

Beluga provides a modern and modular framework for Monte Carlo Localization in robotics.

---
