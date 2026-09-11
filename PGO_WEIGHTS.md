# Pose accuracy: what is left on the table at 0.27 m APE (MIT Stata Center)

Reference point: `2012-04-06-11-15-29`, PR2, `/base_scan`, `odom_combined`,
667 s, 13 331 scans, 12 453 reference poses in
`belugaslam_example/bags/mit_rosbag/gt.txt`.

This file ranks the identified gaps by expected effect and gives the exact
sweep commands. Nothing here is a measured improvement yet: every default in
the code is unchanged, the new knobs simply expose values that used to be
hardcoded so they can be swept on the real recording.

---

## 0. Measurement protocol — check this before tuning anything

Two protocol issues can each move the number more than any parameter.

**0.1 The evaluated trajectory is the online one.**
`belugaslam_benchmark/benchmarking/compute_rmse.py` reads `/best_pose`, which
is the estimate published at each scan. It never receives the retrospective
loop corrections. Cartographer and the other systems in the comparison are
scored on the *finalized* pose graph. The core already writes that trajectory,
in TUM format, ready for `evo_ape`:

```bash
ros2 launch belugaslam_example mit_rosbag_belugaslam.xml \
  optimized_trajectory_path:=/tmp/mit_opt.tum
evo_ape tum belugaslam_example/bags/mit_rosbag/gt.txt /tmp/mit_opt.tum -a
```

Report both. Comparing an online trajectory against other systems' optimized
trajectories understates this system by exactly the amount PGO corrects.

**0.2 The MIT launch used to stop the bag at 180 s.**
`timeout 180 ros2 bag play` covered 27 % of a 667 s recording, before most of
the large loop closures. It is now `bag_timeout`, default `0` (whole bag).
If the 0.27 came from a 180 s run, it is not comparable with published Stata
Center numbers in either direction.

---

## 1. Loop constraints barely bend the graph  *(highest expected effect)*

The weights are relative trust, they are the only thing telling the optimizer
which measurement class to believe. Ratios, before this revision:

| Constraint | translation | rotation | ratio rot/trans |
|---|---:|---:|---:|
| consecutive-node odometry | 3 | 5 | 1.7 |
| node ↔ active submap (intra) | 5 | 8 | 1.6 |
| inter-submap loop | 10 | 12 | 1.2 |

Cartographer 2D, for the same three roles:

| Constraint | translation | rotation | ratio rot/trans |
|---|---:|---:|---:|
| intra-submap | 5e2 | 1.6e3 | 3.2 |
| loop closure | 1.1e4 | 1e5 | 9.1 |

Two differences matter:

- **Loop vs intra-submap.** Here a loop constraint outranks an intra-submap
  one by 2× in translation and 1.5× in rotation. In Cartographer it is 22× and
  62×. A closed loop is supported by a handful of inter-submap edges against
  thousands of intra-submap edges; at these ratios the intra-submap edges win
  and the loop only partially closes. Residual global deformation of a few
  tens of cm is exactly the symptom.
- **Rotation vs translation.** Rotation is under-weighted relative to
  Cartographer in every class. A heading error acts through the lever arm of
  the rest of the trajectory, so in a building the size of Stata Center it is
  the dominant contributor to APE.

`pgo_huber_scale` must move with the loop weights: `HuberLoss` acts on the
*whitened* residual, so with `pgo_loop_translation_weight=10` and scale 1.0 it
already starts down-weighting a loop that disagrees by 10 cm. Raise the loop
weight without raising the Huber scale and the robustifier will simply cancel
the change.

Sweep (start with the ratio, not the absolute magnitude — the gauge is fixed
so only ratios matter):

```bash
for LT in 10 50 200 500; do
  ros2 launch belugaslam_example mit_rosbag_belugaslam.xml \
    optimized_trajectory_path:=/tmp/mit_lt${LT}.tum \
    pgo_loop_translation_weight:=${LT} \
    pgo_loop_rotation_weight:=$((LT*4)) \
    pgo_huber_scale:=$(python3 -c "print(${LT}*0.1)")
done
```

The `pgo_huber_scale = 0.1 * translation_weight` rule keeps the robustifier
knee at a constant 10 cm of raw disagreement while the weight moves.

## 2. Keyframes are 5× too coarse in rotation

`keyframe_min_rotation` is 0.0872665 rad (5°). Cartographer's 2D motion filter
uses 1° with a 0.2 m companion. Between two keyframes 5° apart the pose graph
has no node at all, so heading accumulated during a turn is constrained only
by the odometry edge that spans the turn. The PR2 turns in place repeatedly in
this recording.

```bash
keyframe_min_rotation:=0.0175   # 1 degree, Cartographer's value
```

Cost: roughly 2–3× more nodes and constraints, so more PGO time.

## 3. The frontend trusts the scan ~10× more than Cartographer does

With `tracking_prior_mode:=odometry` the effective weights are
`1/tracking_odom_translation_sigma = 10` and
`1/tracking_odom_rotation_sigma = 20`, against
`tracking_occupied_space_weight = 5.0`.
Cartographer's Ceres matcher uses `occupied_space_weight = 1.0`,
`translation_weight = 10`, `rotation_weight = 40`.

Normalising on the translation prior, the scan term here is
`5/10 = 0.5` versus Cartographer's `1/10 = 0.1`, and against rotation
`5/20 = 0.25` versus `1/40 = 0.025` — **ten times more scan-driven in
rotation**. That is a good trade in a feature-rich room and a bad one in a
corridor, where the scan is nearly invariant along the corridor axis and the
optimum in heading is shallow.

```bash
tracking_occupied_space_weight:=1.0 tracking_odom_rotation_sigma:=0.025
```

## 4. The probability grid the Ceres matcher interpolates is nearly binary

`insertion_l_occ = 1.2` and `insertion_l_free = -0.2`. `ProbabilityField`
maps log-odds through a sigmoid clamped to [0.1, 0.9]:

| hits on a cell | log-odds | probability |
|---:|---:|---:|
| 1 | 1.2 | 0.77 |
| 2 | 2.4 | 0.90 (clamped) |

So a wall cell saturates after two observations. The bicubic interpolation in
`ProbabilityField::sample` then has almost no sub-cell information to work
with: the field is a plateau at 0.9 with a one-cell cliff to 0.1, and its
gradient — the only thing driving the Ceres step — is concentrated in that
cliff. Cartographer's `hit_probability = 0.55` / `miss_probability = 0.49` are
+0.2 / −0.04 in log-odds, an order of magnitude softer, so a wall's
probability reflects *how many times* it was seen and the field varies
smoothly across it. That smooth variation is what lets the matcher resolve
below the 5 cm cell.

The aggressive value also means a single spurious return from a person walking
past writes a 0.77 cell that then needs six misses to erase. This recording is
full of pedestrians.

```bash
insertion_l_occ:=0.2 insertion_l_free:=-0.04   # Cartographer's 0.55/0.49
```

Expect this one to need `tracking_occupied_space_weight` retuned upward,
because the softer field produces smaller residuals.

## 5. Point budget and range

- `tracking_max_points = 180` and `max_points_per_scan_node = 180`. The
  UTM-30LX delivers ~1081 beams; `select_tracking_points` keeps a uniform
  index subsample. 180 is in the same range as Cartographer's adaptive voxel
  filter target of 200, so this is a second-order knob — but the loop-closure
  matcher uses the *same* 180-point budget, and that is where extra points buy
  the most.
- `range_max = 60.0` in the MIT launch is clamped by the message's own
  `range_max`, so the effective cutoff is the sensor's ~30 m. Endpoints at
  30 m have the largest lever arm on heading and the worst range noise
  indoors. Worth a sweep at 20 m.

```bash
tracking_max_points:=300 max_points_per_scan_node:=300 range_max:=20.0
```

---

## Suggested order

1. Re-measure with `optimized_trajectory_path` and the full bag (§0). Establish
   the real baseline before changing a single parameter.
2. Sweep the loop/intra weight ratio with the Huber scale tied to it (§1).
3. `keyframe_min_rotation:=0.0175` (§2).
4. Frontend scan/prior balance (§3).
5. Insertion log-odds, retuning `tracking_occupied_space_weight` alongside (§4).

Use `random_seed:=42` (already the default) throughout so the runs are
comparable, and record `loop_diagnostics_path` and `performance_diagnostics_path`
for each run — if a change alters how many loops are *accepted*, the APE
difference is a loop-detection effect and not the effect being tested.
