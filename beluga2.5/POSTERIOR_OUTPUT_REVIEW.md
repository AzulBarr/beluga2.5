# Diagnostic-driven output selection — experimental, opt-in

The uploaded `beluga_diag.zip` provides evidence of large published-pose changes
at hypothesis switches and graph optimization. It does not establish that the
local tracker is failing, that particular loops are false, or that the map or
trajectory has a known absolute error. No reference trajectory was used here.

This revision adds `output_selection_mode:=pose_risk` to test a specific output
decision problem. The default remains `map`. The two choices use the same
particle weights, loop verifier, branching, submaps and PGO implementation.
The previous consistency fixes remain included.

## What the uploaded run establishes

| Observation | Value / interpretation |
|---|---|
| Complete performance rows | 13,618, all processed |
| Input timestamp comparison | Exactly the first 13,618 acquisition-time-sorted Intel inputs; no missing timestamps in this prefix |
| Complete tracking rows | 40,698 across 13,616 complete hypothesis snapshots |
| Tracking status | Bootstrap once; all other complete rows marked tracked |
| Published corrections above 0.35 m | 108: 107 include a hypothesis switch; the other includes PGO |
| Largest correction | 2.571 m at scan 8073, a hypothesis switch without current backend solves |
| Largest correction without either a switch or backend solve | 0.09085 m |
| Median / p99 scan callback | 3.80 / 252.08 ms |
| Maximum scan callback / backend time | 2,043.31 / 2,041.07 ms |
| Maximum logged scan age | 3,641.90 ms |
| Baseline PGO / candidate trial solves | 810 / 3,249 |

“Correction” above is the logged output innovation relative to odometry
prediction, not ground-truth error. A graph correction or a mode switch may
be necessary. The tracker reporting success also does not prove localization
accuracy in repetitive geometry.

The CSVs end at different times. The final tracking row is cut off and the
terminal log extends beyond the CSVs, without a shutdown footer. Only that
incomplete final tracking row was excluded; the supplied files are unchanged.
The 13 Intel frames beyond the performance CSV cutoff are not counted as
dropped. The terminal's last periodic processed count is 13,630 and the dataset
publisher reports 13,631 inputs. Stop cleanly before copying the next logs.
No parameter dump, source hashes or build logs accompanied this run, so its
exact binary version is not independently verified.

## A concrete output decision problem

At scan 1694, the published pose switches approximately 1.59 m to H11:

| Hypothesis | Posterior mass | Frontend x (m) | Frontend y (m) |
|---|---:|---:|---:|
| H11 | 0.394746 | -5.112684 | 0.537458 |
| H14 | 0.224494 | -6.523181 | -0.195734 |
| H15 | 0.380760 | -6.523191 | -0.195662 |

H14 and H15 are only 0.000073 m apart at the current pose. Together they have
0.605254 mass, but choosing the largest individual hypothesis selects H11.
Their current poses being similar does not prove their historical graphs or
maps are identical. This patch therefore does not merge their maps or graphs.

For normalized masses w_j and current frontend positions p_j in the common
global frame, the optional output decision selects an existing hypothesis i:

    R(i) = sum_j w_j ||p_i - p_j||^2
    i* = argmin_{i: w_i > 0} R(i)

It publishes that hypothesis's own full pose, including its heading, and its
map. It never interpolates a robot pose between incompatible maps. At scan
1694 the actual C++ helper selects H15, reducing internal expected position
loss from 1.52949 to 0.99752 m². These numbers are NOT measured squared errors.

The decision costs O(H²) after the existing particle mass accumulation. It
does not use a history-dependent switch penalty, alter the RNG, redistribute
mass, modify particles, install constraints, or change a scan matching prior.
In the provided node, selection feeds publication, covariance and output
diagnostics; tracking uses per-hypothesis state and odometry-to-base input.
Core integration still needs compilation and replay on the ROS machine.

The position loss is a frontend point-mass approximation, not integration over
the full within-hypothesis PF distribution. Heading and map accuracy are not
in its loss function. This is why it is an opt-in experiment.

## Precise decision-theory proposition

For the finite position belief mu = sum_j w_j delta(p_j), with w_j >= 0 and
sum_j w_j = 1, let the admissible actions be the positive-mass retained positions.
Then the selected action satisfies R(i*) <= R(i_MAP), because the MAP
hypothesis's position is one of the admissible actions. Writing
p_bar = sum_j w_j p_j also gives:

    R(i) = ||p_i - p_bar||^2 + sum_j w_j ||p_j - p_bar||^2.

Thus the selected retained position is nearest the posterior mean under this
loss. Splitting mass across identical positions leaves mu, every risk value,
the minimum risk and the set of minimizing positions unchanged. With a unique
minimizing position, that position is unchanged as well. Exact ties retain the
MAP candidate when it is a minimizer, otherwise the first minimizer by stable
ID; heading/map identity may differ across tied or duplicated positions.

This is a standard minimum-risk decision fact, not a new theorem of SLAM
accuracy. It does not imply lower empirical RMSE if the retained belief is
miscalibrated or misses the correct trajectory. Nor does it imply smoother
output: with two distinct positive-mass positions, the majority position wins,
so crossing 50% can still produce a discontinuity. The logged switches at
10745 and 10753 remain switches under the recorded-snapshot decisions.

## Evidence for the paper's central mechanism

The title “Beyond the MAP Trajectory: Belief-Weighted Loop Closure Verification
for Multi-Hypothesis Particle-Filter SLAM” describes the intended verifier.
It is a mechanism claim; a performance claim requires a controlled comparison.

Of 1,131 evaluated candidates, two have belief score >= 0.25 while their
MAP-only score is below 0.25:

| Candidate | Query scan | MAP score | Belief score | Selected into a branch? |
|---|---:|---:|---:|---|
| 939 | 12032 | 0.194397 | 0.381048 | Yes |
| 1056 | 12942 | 0.124453 | 0.281242 | No |

The candidate rows' frozen mass sums and weighted score equations check out.
These are examples of different verification decisions. They do not label
either loop correct and only candidate 939 was selected into a branch.
Candidate 1056 passed the belief threshold but was not selected by the later
branch ranking/budget. The other 1,129 candidates agree on threshold eligibility.

A simple proposition directly supporting this mechanism is the following.
Let each compatibility c_h lie in [0,1] and B = sum_h w_h c_h. If the MAP
hypothesis has weight w_m < 1, compatibility c_m < threshold tau, and the
remaining hypotheses have normalized mean compatibility c_rest, then

    B >= tau  iff  c_rest >= (tau - w_m c_m) / (1 - w_m).

Proof: expand B = w_m c_m + (1-w_m)c_rest and rearrange. If every positive-mass
hypothesis has c_h < tau, their weighted average is below tau. These facts
justify how secondary support can change verification and why evidence is
required somewhere in the retained belief. They do not establish that the
trajectory-deformation compatibility is a calibrated loop probability.

To test the thesis, compare belief versus MAP verification with the SAME
output selector, particle/hypothesis budget, candidates, seed and dataset.
Separately compare `map` versus `pose_risk` output while fixing the verifier
to `belief`. Otherwise an output decision change could be mistaken for a loop
verification improvement. Multiple-hypothesis benefit is not proved by this
single unlabeled run or by the elementary propositions above.

## Validation and remaining work

- 315 standalone C++ selector checks pass, including the real scan-1694 case,
  duplicate-position mass splitting, rigid-frame invariance, mass scaling,
  zero-mass exclusion and invalid input rejection.
- The SAME C++ helper was evaluated on all 13,616 complete uploaded frontend
  snapshots. Independent Python loss computation agrees on every snapshot.
- Decisions differ from MAP in 2,692 snapshots; 2,573 reduce internal loss by
  more than 1e-6 m². Maximum internal loss reduction is 2.45712 m² and mean
  reduction is 0.003633 m². Neither statistic is empirical accuracy.
- Snapshot rows precede PGO, loop branching and resampling. This experiment
  is not a counterfactual ROS output trajectory or a full SLAM replay.
- Python/XML parameter wiring and existing submapping structural checks pass;
  the existing 24 Python trajectory-evaluation tests pass.
- Two core gtests were added for actual integration and invalid-mode rejection.
  ROS/Ceres development dependencies are unavailable here: these gtests and
  the full core/node build have NOT run. Build/test commands accompany the ZIP.

PGO and its trial verification remain synchronous and are unchanged. The
observed large latency spikes are not fixed by this output selector. Profiling
the long-running solves is the next performance task; loosening loop acceptance
or silently abandoning hypotheses to hide pauses would change the experiment.
The present evidence does not identify which large graph corrections should
be accepted, nor prove a better final map or lower absolute RMSE.

To reproduce the snapshot check from the repository root:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Ibelugaslam_core/include \
  tools/score_output_snapshots.cpp -o /tmp/beluga_score_output_snapshots
python3 tools/score_output_snapshots.py /path/to/beluga_diag/tracking.csv \
  --executable /tmp/beluga_score_output_snapshots \
  --output /tmp/beluga_snapshot_report --allow-truncated-tail
```

Use `--allow-truncated-tail` only for this incomplete upload, not to hide
interior CSV corruption. The utility removes the whole final sequence if that
sequence contains a partial row, and validates the remaining mass sums.

New performance CSV fields, appended after all previous columns:
`output_selection_mode`, `map_hypothesis`, `map_position_risk_m2`, and
`selected_position_risk_m2`. They reflect the post-resampling publication
decision. Rejected scan callbacks leave these and the output pose fields empty.
