# Particle pose estimator review — September 8, 2026

This revision addresses a disconnect between the particle update and the pose
used to construct the map. It is a test candidate, not a demonstrated RMSE
improvement. The baseline is `beluga_backend_consistency_fix.zip`; no newer
source or reference trajectory was supplied for this revision.

## What the pose audit found

The preceding frontend computes an odometry prediction for each hypothesis,
matches the scan in a native submap, and uses that matcher result for map
insertion, graph nodes and publication. This happens before the particle
proposal update. Particle likelihoods influence hypothesis masses and
bifurcation, but increasing the particle count does not directly refine this
within-hypothesis pose. The frontend can be good while the reported benefit
of increasing particles remains small.

The new `frontend_pose_mode=proposal_mean` option computes the conditional
weighted pose from **all scored motion proposals**, before choosing a proposal
per ancestor and before resampling. A concentrated cloud can replace a healthy
frontend pose if its mean passes the existing map-fit and motion checks.
The adopted pose feeds insertion, graph nodes, the next frontend prediction
and publication. It is not merely a filtered display pose.

The direct launch default remains `frontend` for compatibility. The comparison
runner defaults to `proposal_mean`. Use the supplied explicit command to test
the new estimator; merely rebuilding and repeating the old launch will not
activate it.

## Review scope and decisions

| Method/path | Finding and action |
|---|---|
| Intel parsing, acquisition stamps, scan conversion, TF | Preserve acquisition timestamps; propagate exact integer nanoseconds into stored trajectory samples. The Intel publisher does not supply per-beam timing, so this revision does not invent deskew timing. |
| Differential-drive sampling | Retain the existing prior-drawn proposals, stationary behavior and seeded random order. Do not push particles onto a scan-matcher optimum without an importance correction. |
| Proposal likelihood and evidence | Retain the original likelihood, log-domain evidence and stochastic proposal selection. The new readout includes ancestor weights and each ancestor's proposal count. No second sensor-weight update is applied. |
| Local scan matching and recovery | Retain native-grid matching and the existing recovery confirmation path. Only a healthy `tracked` result is eligible for replacement by the proposal mean. |
| Submapping, insertion and motion filtering | Feed the same accepted pose into insertion and graph construction. Existing two-active-submap lifecycle, local grids and insertion guards remain. |
| Spatial bifurcation | Initialize the child immediately from its weighted cluster mean. Previously `has_local_pose=false` allowed one-frame publication of a maximum-weight particle before the next tracking update. |
| Resampling and uncertainty | Cache the full proposal second moment before categorical selection/resampling. Recompute conditional particle moments when a spatial split changes membership. Rotate the cache when PGO changes the world frame. |
| Loop verification and graph branching | Preserve belief-weighted verification, loop/no-loop branches, branch masses, bounded population and separate graph hypotheses. |
| Pose graph optimization | Preserve the preceding analytic Jacobians and robust loop polishing. Add a retrospective export of the currently selected graph, using optimized node poses where available. |
| Global output selection | Preserve `pose_risk` in the experiment runner. Never average distinct graph hypotheses into a single pose for insertion. Generic uninitialized output fallback now uses a within-hypothesis weighted mean. |
| ROS diagnostics and evaluation | Record estimator decisions, ESS, spread and mean displacement. Validate requested particle count, exact scan coverage, selected graph identity and exported trajectory timestamps. |
| Performance | Expose particle count, worker count and replay rate in the runner; measure the proposal readout separately from likelihood scoring. No unsupported end-to-end speedup claim. |

This is a review of the SLAM execution and evaluation paths, not a line-by-line
certification of every vendored library or a replacement for a ROS build.

## The estimator and the proposition it supports

Fix one hypothesis and one proposal cloud C. Ancestor i has preceding weight
a_i and K_i proposals x_ik drawn from its motion prior. Let L_ik be the sensor
likelihood used by the existing particle update. Define

```math
Z = \sum_i \frac{a_i}{K_i}\sum_k L_{ik},\qquad
\beta_{ik}=\frac{a_i L_{ik}}{K_i Z},\qquad
\mu=\sum_{i,k}\beta_{ik}p_{ik}.
```

Here p_ik is the proposal's two-dimensional translation. Normalization is
within the hypothesis. The hypothesis posterior mass is not replaced by Z.
Log-sum-exp normalization avoids ordinary likelihood underflow.

**Finite-belief squared-loss proposition.** For this represented conditional
belief, let Sigma be the covariance about mu. For any translation estimate b,

```math
\sum_{i,k}\beta_{ik}\|p_{ik}-b\|^2
=\operatorname{tr}(\Sigma)+\|b-\mu\|^2.
```

Consequently mu minimizes expected squared translation error under this
finite belief. Proof: write p_ik-b=(p_ik-mu)+(mu-b), expand the square, and
use sum beta_ik(p_ik-mu)=0.

**Removal of proposal-selection noise.** The existing update chooses J_i
with probability L_i,J_i / sum_k L_ik. Its normalized ancestor weight after
the update is A_i=(a_i/K_i)sum_k L_ik/Z. If one estimates translation using
the selected representatives, mu_sel=sum_i A_i p_i,J_i, then

```math
\mathbb E[\hat\mu_{\mathrm{sel}}\mid C]=\mu,
```

and for a fixed true position p,

```math
\mathbb E[\|\hat\mu_{\mathrm{sel}}-p\|^2\mid C]
=\|\mu-p\|^2+
\operatorname{tr}\operatorname{Cov}(\hat\mu_{\mathrm{sel}}\mid C).
```

Thus integrating all proposals removes this additional selection variance.
This is a conditional-expectation argument, not a new general optimality
theorem for SLAM. In particular, the preceding frontend is a scan matcher,
not mu_sel: the proposition does **not** prove that this revision beats that
frontend. Once estimates feed future maps, future proposal clouds also change.

Yaw uses a circular mean, which minimizes circular chordal loss; the proof
above is about Euclidean translation, not a universal SE(2) loss. Gate
fallbacks and selection of a single global hypothesis further limit the claim.

## Why the mean is guarded

A mean between different modes can lie in a wall. The code therefore never
pools graph hypotheses and only accepts a local mean when all of these hold:

- The scan matcher reports `tracked`, not bootstrap, rejection or recovery.
- Proposal ESS is at least 5.
- At least 90% of cloud mass lies within the configured translation and yaw
  windows around the matcher pose.
- Translation RMS spread and yaw standard deviation are at most half their
  tracking windows; default bounds are 0.25 m and 0.125 rad.
- Mean displacement from the **original odometry prediction** is within the
  tracking bounds; the motion gate is not recentered on the matched pose.
- The mean meets the minimum inlier/overlap requirements, with overlap also
  at least the recovery threshold, and loses at most 0.02 in mean log scan
  likelihood relative to the matcher.

These are conservative heuristics, not a proof of unimodality or accuracy.
They choose the estimator; they do not truncate proposal tails or alter PF
weights. `tracking.csv` records `pose_source`, `proposal_pose_decision`, ESS,
local mass, position/yaw spread and mean-to-matcher displacement. If few poses
are accepted, this run cannot demonstrate a large benefit from the readout.

## Covariance and trajectory products

The covariance cache is the conditional empirical second moment about the
actual chosen pose, in world x/y and wrapped yaw coordinates. When the chosen
pose differs from the proposal mean this includes their offset. It is not a
calibrated estimate of map, loop or total trajectory uncertainty. Resampling
alone no longer changes the cached reported matrix; graph rotations transform
it consistently.

The runner records two distinct trajectory products:

- `performance.csv`: poses actually published online, at exact scan stamps.
  This history can switch between graph hypotheses.
- `optimized_trajectory.tum`: on clean shutdown, the final selected graph's
  retrospective trajectory. Retained keyframes use current graph node poses;
  other samples use their stored submap attachments transformed by the current
  submap pose. It uses one graph identity throughout and does not rewrite the
  online measurements.

The export does not perform an extra shutdown optimization. Its header gives
the current node count and node count at the last PGO; an unoptimized tail may
remain. Do not compare another system's final smoothed graph against this
system's online output and label the difference an estimator-only comparison.
Report online and retrospective errors separately, with the same timestamps,
robot frame, reference source and rigid SE(2) alignment convention. Do not use
scale fitting or discard failed intervals to improve the number.

## Validation actually completed here

- **50 Python tests passed**, covering the comparison runner, capture checks
  and the existing Python utilities, including corrupt/missing trajectory
  rejection and exact particle-count validation.
- Native standalone C++ suites passed: 47 loop-belief checks, 12 motion-filter
  checks, 315 output-selection checks, 24 tracking/proposal checks, 2,000 graph
  residual/Jacobian fixtures, and 36 new proposal-estimator checks.
- The new suite also ran **2,000 Gaussian cloud trials** with an analytically
  known posterior mean. With eight proposals per ancestor, mean-estimation
  RMSE was 0.0340406 at 30 particles and 0.0108376 at 300. Using one selected
  proposal per ancestor at 300 gave 0.0256874. **These are not trajectory RMSE
  measurements, not Intel results, and not a comparison against the frontend.**
- New tests cover unequal ancestor weights/proposal counts, log-weight shifts,
  angular wraparound, antipodal rejection, frame transformations, covariance,
  concentration, motion bounds, scan fit and a dangerous bimodal mean.
- Python launch syntax, XML declarations/forwarding and the structural
  submapping check passed.

An isolated native benchmark (180 synthetic scan points, eight proposals per
particle, serial scoring, 50 repetitions) measured:

| Particles | Scored proposals | Mean scoring time | Mean summary/gate time |
|---:|---:|---:|---:|
| 30 | 240 | 1.409 ms | 0.042 ms |
| 300 | 2,400 | 14.527 ms | 0.314 ms |
| 1,000 | 8,000 | 48.833 ms | 1.038 ms |

This fixture excludes ROS, actual core grouping/transforms, motion sampling,
map insertion, branching and PGO. It measures neither total runtime nor your
laptop's frame throughput. More particles still cost more work.

The full ROS/Ceres-dependent C++ build and integration tests could not run in
this environment. Integration tests have been added for unchanged PF weights,
pose/node/publication agreement, covariance stability, immediate spatial child
initialization, exact trajectory export and PGO covariance rotation. Run them
with the supplied `colcon test` command before the replay. No revised full
Intel replay or ground-truth trajectory RMSE was measured here.

## What this can and cannot establish for the paper

The central verifier remains posterior-weighted compatibility across the live
trajectory hypotheses. This revision changes conditional pose estimation and
diagnostics without removing loop/no-loop bifurcation or the MAP-verifier
ablation. The elementary proposition above can justify the readout; it should
not be presented as the novelty of belief-weighted loop verification.

The architecture shares one map/graph within each global hypothesis. Therefore
300 particles with four hypotheses means **four map/graph histories**, not 300
independent mapped histories. A spatial fork shares its pre-fork history; this
revision does not retrospectively reconstruct a different history for it.
This is an approximation to a full trajectory RBPF. For context, the original
[GMapping project](https://openslam-org.github.io/gmapping.html) describes
particle-specific maps, improved proposals and selective resampling; particle
count alone is not the whole estimator design.

Larger clouds can reduce Monte Carlo approximation error when the proposal
supports the correct region. They cannot remove model bias, overconfident
pseudo-likelihoods, extrinsic/time errors, a corrupted submap or a false loop.
There is no theorem here that guarantees better real RMSE than graph SLAM or
standard RBPF, or guarantees improvement for every larger particle count.

First compare 300-particle `proposal_mean` with 300-particle `frontend` using
the same seed and backend. Then compare 30 versus 300 with `proposal_mean`.
Check estimator acceptance, tracking failures, hypothesis survival, scan
coverage and runtime alongside RMSE. For the paper, separately compare
belief versus MAP verification at the same hypothesis budget and a single
hypothesis baseline, across seeds and datasets. A good result on one map is
not enough to claim state-of-the-art accuracy or predict acceptance.
