# Quality revision: robust tracking and stochastic multi-hypothesis SLAM

**Latest revision:** see [LATE_RUN_REVIEW.md](LATE_RUN_REVIEW.md) for confirmed
tracking recovery, immediate reference handover, cache retention and PGO scheduling.
The descriptions below document the preceding implementation.

This revision is based on `beluga_performance_fix.zip`. The supplied screenshot
shows overlapping wall traces, streaks and inconsistent alignment. It cannot
identify whether the cause was tracking, bad loops, calibration, missed scans or
a combination. No runtime logs, ground-truth trajectory or new source tree were
attached. The code review therefore uses the previous shared workspace version.

The research focus stays **Beyond the MAP Trajectory: Belief-Weighted Loop
Closure Verification for Multi-Hypothesis Particle-Filter SLAM**. Up to four
trajectory/map hypotheses share a configurable total particle budget, thirty in
the Intel launch. This is not thirty separately mapped trajectory histories.
The proposed mathematical justification is in `paper/BELIEF_PROPOSITION.md` and
`paper/belief_verification_note.tex`.

## Audit and implemented changes

### Local tracking and particle update

The previous tracker evaluated raw occupancy log odds on a three-level discrete
lattice. It provided no explicit odometry regularization or registration quality
gate. Each particle was then greedily moved to a local score maximum before
weighting, without correcting the proposal density. That can reduce diversity
and overstate evidence. It is not generally a valid bootstrap PF update.

The default online tracker now uses a native-submap distance field, bilinear
interpolation and a Gaussian/outlier mixture. It optimizes a continuous SE(2)
pose with a small seed lattice and damped Gauss-Newton steps. One fixed odometry
prediction regularizes all trials. The loss is normalized by endpoint count;
it does not change confidence simply because more endpoints are sampled.
The objective is

\[
J(T)=-\frac1n\sum_i\log\{\epsilon+(1-\epsilon)
\exp[-d(Tp_i)^2/(2\sigma^2)]\}
+\frac{\|t-t_0\|^2}{2s_t^2}
+\frac{\operatorname{wrap}(\theta-\theta_0)^2}{2s_\theta^2}.
\]

Here `T` is a candidate robot pose in the matching submap, `p_i` are scan points
in the robot frame, `d` is distance to an occupied cell, and `(t_0,theta_0)` is
the odometry-predicted pose. Defaults are sigma=0.15 m, epsilon=0.05, s_t=0.50 m
and s_theta=0.20 rad. These are starting settings, not fitted noise estimates.
The distance field uses the existing style of chamfer approximation, capped
at 1 m. Derivatives are analytic for bilinear interpolation; it is not an exact
Euclidean distance transform or Cartographer's Ceres matcher.

Only the oldest matching submap supplies online registration. Its field is built
once after grid changes and then shared read-only during matching. Grid writes
and cropping invalidate it. A failed match leaves the odometry prediction as
the online pose and skips insertion/node creation for that hypothesis. At least
12 inliers and 35% overlap within 0.20 m are required once a nonempty reference
map exists. An initial nonempty scan still bootstraps a map. Growing submaps,
scan-level hit priority and the overlapping lifecycle remain in place.

Particles use a **stochastic multiple-proposal update** instead of greedy
optimization. For ancestor `a` in hypothesis `h`, draw K poses independently from
the configured motion model, evaluate their sensor likelihoods L_k, select one
with probability L_k / sum_j L_j, and multiply the ancestor weight by
`sum_k L_k / K`. Default K=8; stationary motion uses one deterministic proposal.
The sensor factor is `exp(effective_beams * mean_log_likelihood)` with default
`effective_beams=20`, so it is a tempered composite likelihood. It is not a
calibrated probability that the whole scan or trajectory is correct.

For a fixed pre-insertion map and any test function f, this update satisfies

\[
\mathbb E\!\left[\frac{\sum_k L(X_k)}K f(X_I)\right]
=\mathbb E_{X\sim p_{motion}}[L(X)f(X)].
\]

Conditioning on the K proposals gives `(1/K) sum_k L(X_k) f(X_k)`, and averaging
independent motion draws gives the right side. This justifies this particle
update for its chosen sensor factor. It does not prove exact Bayesian inference
for the entire shared-map SLAM backend. K=1 is the corresponding bootstrap step.
No claim of novelty is made for importance sampling or scan-informed proposals.
[GMapping](https://openslam-org.github.io/gmapping.html) already uses improved
proposals and selective resampling.

### Motion, input and publication

The wrapper passed `alpha5` into a fifth motion-model field that is actually
`distance_threshold`. It now passes `motion_distance_threshold=0.01` explicitly;
alpha1 through alpha4 are the noise coefficients. `alpha5` remains declared for
launch compatibility and is ignored. The motion model also handles zero variance
without constructing an invalid normal distribution and avoids cached random
variates leaking between separate filter instances on the same thread.

The Intel FLASER reader now uses `tokens[num_scans + 8]`, the acquisition timestamp.
Its previous last-token choice was the logger timestamp. For example, the first
record has sensor time 976052857.337530 and logger time 0.000246. The supplied file's own header distinguishes these fields.

More significantly, its 13,631 FLASER records have **631 adjacent backward
timestamp transitions**. There are no duplicate acquisition timestamps. A guard
that rejects timestamps not newer than the last accepted timestamp would reject
**1,612 records (11.8%)** in file order, before considering any transport or
computation losses. This is a source-file measurement, not a measurement of the
user's previous run. The new dependency-free reader sorts the finite dataset
chronologically, retains every record and its scan/odometry association, and
converts decimal timestamps directly to integer nanoseconds. Ambiguous duplicate
timestamps in a different input fail explicitly rather than being silently
clamped or discarded.

Replay uses a steady timer and monotonic wall-time pacing, preserves gaps longer
than one second, and supports `replay_rate` and `replay_start_delay`. It publishes
`/clock` for `use_sim_time:=true`; the preceding publisher did not. Supporting TF
still publishes before the scan. DDS ordering across topics is not guaranteed,
so the consumer's TF lookup remains necessary. Slower replay helps distinguish
sustained compute overload from input/estimation problems; it does not establish
real-time performance at the original rate.

The reader also makes beam geometry explicit: `laser_angle_min_deg=-90` and
`laser_angle_increment_deg=1`. All supplied scans have 180 returns, giving a last
beam at +89 degrees; `angle_max` agrees with the last actual beam. This follows
the one-degree convention in this existing
[Intel converter implementation](https://gist.github.com/FabianSchurig/3a178272008d5a73be8c5e19c5d5d561).
FLASER itself does not contain an angular increment, so this is a dataset
convention, not calibration estimated from the screenshot. The preceding reader
stretched 180 readings across both -90 and +90 using 180/179 degrees. Set
`laser_angle_increment_deg:=1.005586592178771` to reproduce that prior convention
in an ablation. The one-degree endpoint difference amounts to about 0.52 m at
30 m range. Keep timestamp ordering, beam convention and reader version fixed
across verifier comparisons.

The ROS converter respects both configured and sensor-reported range limits.
Optional deskew, enabled by default, uses start/end odometry and constant-twist
SE(2) interpolation when `LaserScan.time_increment` is positive and end-of-scan
TF is available. Missing end TF produces a throttled warning and undeskewed
fallback. Intel's supplied reader has no beam timing, so it does not invent a
scan duration or deskew that dataset. Sensor extrinsic calibration remains a
user/dataset input; this revision does not infer it from the screenshot.

Map resolution is now a runtime parameter. ROS launch defaults to 0.05 m, while
the core constructor defaults to the previous compile-time resolution for API
compatibility. This provides finer representation but costs more cells. Use
`map_resolution:=0.1` to compare the frontend at the previous Intel resolution.
PGO still moves submaps rigidly; it cannot repair bad geometry already inserted
inside a submap. Replay from a fresh node to rebuild the map.

The earlier performance fixes remain: Release default, bounded task arena,
fast ray insertion, cached publication and timer-driven visualization. Published
pose uncertainty now reports the selected mode's second moment about its
published frontend pose, rather than a covariance about an unrelated global
mixture mean. It is a particle diagnostic, not calibrated pose-graph covariance.

### Population, branching and backend

The old single-hypothesis population could remain at `min_particles=5` forever.
Initialization now uses `max_particles`, thirty in the Intel launch. This is a
fixed bounded population policy, not KLD-adaptive particle allocation. Legacy
`kld_*` and spatial-resolution parameters are still accepted for compatibility;
they do not make the current resampler KLD-adaptive. The old
`likelihood_scaling_factor` no longer controls production scan evidence; use
`tracking_effective_beams`.

Resampling uses a systematic draw within each retained hypothesis, checks
conditional ESS as well as whole-population ESS, and retains each hypothesis's
continuous mass through weights `W_h / quota_h`. This avoids masking a depleted
small mode behind a large mode's healthy particle ESS. The allocation quota does
not create extra posterior probability.

Spatial branching remains enabled. A prospective secondary cluster must now
contain at least two particles, carry at least 2% of its parent's mass, and
persist for three consecutive scans before consuming a graph slot. Unconfirmed
samples keep their weights and remain in their parent. These defaults suppress
transient noise tails; they can also delay a true small mode. Set
`split_min_particles:=1 split_min_mass:=0.0 split_persistence:=1` to ablate the
new confirmation rule. A spatial child shares its past graph and grids, then
diverges through later insertion; it does not retroactively reconstruct a full
per-particle trajectory history.

Geometric loop retrieval now corroborates the same transform against up to three
neighboring query scans. Each checked scan must pass the geometric score and
overlap thresholds; neighbors are not independently realigned. This is a
conservative geometric filter and may reduce recall. Set
`loop_validation_scans:=1` to isolate its effect.

**After retrieval, every fixed candidate is still evaluated under every retained
trajectory hypothesis.** The core combines the resulting compatibilities with
the pre-event hypothesis weights. Unsupported modes retain zero compatibility
and their denominator mass. The MAP, uniform and belief verifier modes still
share the same candidate interface. Loop/no-loop alternatives remain bounded.

Trial PGO must now report convergence before its deformation is used as
compatibility evidence. The existing numerical checks, required loop fit,
full-strength test constraint, gauge fixing, rigid live-submap group, immutable
local priors, pose-only cloning, and transactional commits were retained. Live
baseline PGO still accepts numerically usable solutions; only trial verification
requires convergence. A failed solve is zero support under the current verifier
policy, which is conservative and must be counted in experiments. The full graph
still grows and verification still waits for all trial results.

### Scientific limits

The maintained weights are an approximate, bounded SLAM belief: map sharing,
finite proposals, correlated beams, hard graph pruning and heuristic loop branch
factors matter. Trial deformation compatibility is **not a calibrated Bayes
factor**. A current scan can influence both tracking and loop verification;
no independence claim removes that reuse. Multiple hypotheses help only when
useful alternatives survive and receive meaningful weights. The simple
proposition demonstrates information loss from MAP compression, not superior
SLAM accuracy or guaranteed loop-label correctness.

The new frontend, resampling, deskew and neighboring-scan checks are engineering
changes. The intended research contribution remains applying trajectory-prior
verification to the live weighted trajectory belief. [ROVER](https://arxiv.org/abs/2508.13488)
already verifies loops through trial PGO and a trajectory prior. This revision
is not evidence that the weighted extension is unprecedented or an ICRA-level
breakthrough. Its novelty and value must be assessed against related work and
controlled experiments.

## New controls

| Parameter | ROS default | Purpose |
|---|---:|---|
| tracking_sigma | 0.15 | Distance likelihood scale, m |
| tracking_outlier_probability | 0.05 | Robust likelihood floor |
| tracking_translation_prior_sigma | 0.50 | Translation regularizer scale, m |
| tracking_rotation_prior_sigma | 0.20 | Rotation regularizer scale, rad |
| tracking_max_translation | 0.50 | Maximum correction from prediction, m |
| tracking_max_rotation | 0.25 | Maximum correction from prediction, rad |
| tracking_min_overlap | 0.35 | Minimum inlier fraction for insertion |
| tracking_inlier_distance | 0.20 | Inlier distance, m |
| tracking_effective_beams | 20 | Normalized sensor evidence strength |
| tracking_min_points | 12 | Minimum inliers once a reference exists |
| tracking_max_points | 180 | Maximum registration/weighting endpoints |
| tracking_max_iterations | 20 | Damped local optimizer iterations |
| motion_proposal_samples | 8 | Motion draws per persistent particle |
| map_resolution | 0.05 | Native and publication grid resolution, m |
| split_min_mass | 0.02 | Minimum relative mass to create a spatial child |
| split_min_particles | 2 | Minimum support to create a spatial child |
| split_persistence | 3 | Consecutive confirmations before spatial branching |
| loop_validation_scans | 3 | Neighboring scans for fixed-transform corroboration |
| tracking_diagnostics_path | empty | Optional per-hypothesis tracking CSV |
| motion_distance_threshold | 0.01 | In-place rotation threshold, m |
| deskew_scan | true | Use beam timing and available end odometry |

All are startup settings and are forwarded by the Python and Intel XML launches.
The Intel XML launch additionally exposes `replay_rate=1.0`,
`replay_start_delay=2.0`, `laser_angle_min_deg=-90.0`, and
`laser_angle_increment_deg=1.0` to the publisher. Standalone publisher parameters
`dataset_path` and `publish_clock=true` are also available through `--ros-args`.
Do not run two `/clock` publishers for the same replay.

The tracking CSV records sequence, hypothesis, usability, overlap, mean log
likelihood, frontend pose, mass and particle count. Join sequence to processing
order; the existing performance CSV retains input timestamps. A large fraction
of unusable registrations should trigger inspection of input calibration,
matching windows and motion noise. Do not simply interpret fewer insertions as
a cleaner or more accurate map.

## What must be tested

1. Build and run the C++ suite on ROS 2 Humble. Full integration compilation is
   unavailable in the generation environment. The dependency-free matcher and
   proposal tests run there; the full-node tests do not.
2. Replay Intel with loops enabled and disabled from a fresh node. Compare wall
   alignment, loop events, coverage, rejected insertion counts and scan coverage.
   A clean-looking map produced by rejecting most scans is not success.
3. Compare this revision at 0.1 and 0.05 m to distinguish algorithm and resolution
   effects. Fix the reader, seed, frontend and budgets across verifier ablations.
4. Run `belief`, `map`, `uniform`, and `geometry` on the same fixed candidate set
   and ground-truth loop labels. `tools/summarize_loop_verification.py` performs
   a fixed-candidate score ablation on one recorded belief history; it does not
   simulate the different future maps that alternative decisions would create.
   Then run independent end-to-end experiments across multiple seeds. Include `max_hypotheses=1` at the same total particle budget.
5. Report loop precision/recall, GT ATE/RPE, map quality and coverage, input scan
   accounting, failed trial solves, p95/p99 latency, and peak memory. Report
   K motion proposals and persistent particle count separately when comparing cost.
   Choose thresholds on held-out data; do not infer ICRA acceptance from a screenshot.

`tools/test_intel_reader.py --dataset belugaslam_example/bags/intel/intel.clf
--write-stamps /tmp/beluga_source_stamps.txt` exports the exact input stamp list.
Pass that file to `tools/summarize_performance.py --expected-stamps` to distinguish
missing callbacks from callbacks rejected by the node. The tracking summary in
`tools/summarize_tracking.py` reports insertion usability, overlap, secondary mass
and effective hypothesis count. Always report scan coverage alongside map quality.
