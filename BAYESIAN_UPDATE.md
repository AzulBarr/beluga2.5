# Hierarchical loop evidence update — 2026-09-10

This revision implements explicit graph masses, conditional particle log weights,
and sequential verification of a loop against a frozen historical reference.
The Intel launch enables it by default (`loop_update_mode:=bayes`).

**Validation status:** the numerical implementation and dependency-free regression
programs were run successfully. The full BelugaSLAM/ROS executable and the new
production integration tests were **not compiled or run** in the editing environment:
ROS, Ceres, Sophus, range-v3, and the standard TBB installation are unavailable.
Do not interpret this package as a verified ROS release or evidence of improved
trajectory RMSE. Run the build and integration gate below before experiments.
`validation/BAYESIAN_VALIDATION.json` records exactly what was checked.

## Install, build, test, run

The ZIP contains a complete `beluga2.5/` source directory, including the supplied
Intel dataset. It contains no Git metadata or build outputs. Extract it into
`~/ros2_ws/src`, replacing the source files in your existing `beluga2.5` directory.
Your existing Git repository metadata stays in place.

In a fresh terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select belugaslam_core belugaslam_node belugaslam_example belugaslam_benchmark \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select belugaslam_core --event-handlers console_direct+
colcon test-result --verbose
```

Proceed only when the build succeeds and `colcon test-result` reports no failures.
Run the same Intel launch, with explicit Bayesian settings and diagnostic paths:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
beluga_run_dir="$(mktemp -d "$HOME/beluga_bayes_XXXXXX")"
ros2 launch belugaslam_example intel_dataset_belugaslam.xml \
  record_bag:=false \
  loop_update_mode:=bayes loop_verifier_mode:=belief \
  max_particles:=30 max_hypotheses:=4 \
  loop_bayes_min_scans:=10 loop_bayes_max_scans:=30 \
  loop_bayes_diagnostics_path:="$beluga_run_dir/bayes.csv" \
  loop_diagnostics_path:="$beluga_run_dir/loops.csv" \
  tracking_diagnostics_path:="$beluga_run_dir/tracking.csv" \
  performance_diagnostics_path:="$beluga_run_dir/performance.csv" \
  optimized_trajectory_path:="$beluga_run_dir/optimized_trajectory.tum"
```

When the run has ended, in that same terminal:

```bash
python3 ~/ros2_ws/src/beluga2.5/tools/summarize_bayesian_loops.py "$beluga_run_dir/bayes.csv"
```

For automatic replay/capture/shutdown/ZIP creation, use the existing runner:

```bash
python3 ~/ros2_ws/src/beluga2.5/tools/run_beluga_comparison.py \
  --loop-update-mode bayes --verifier belief --particles 30 --max-hypotheses 4
```

A transport-free replay of the production C++ core is also wired:

```bash
python3 ~/ros2_ws/src/beluga2.5/tools/run_accuracy_replay.py \
  --loop-update-mode bayes --loops belief --particles 30 --hypotheses 4 \
  --frontend-pose-mode frontend
```

The replay binary accepts an optional final `bayes|heuristic` argument; its default
is `bayes`. No reference trajectory enters the SLAM implementation.

## What changed

1. **Graph probability is explicit.** `Hypothesis::log_mass` holds normalized log
   mass. `hypothesis_masses()` reads these values; particle counts do not vote.
   Output selection and the ROS entropy publisher use the joint distribution.
2. **Particle probability is conditional.** The particle tuple retains the Beluga
   interface: element 0 is pose, 1 is a derived linear conditional weight, 2 is its
   hypothesis pointer, and 3 is the authoritative conditional log weight.
   `set_conditional_log_weight()` keeps the two weight representations synchronized.
   `joint_particle_weight()` is the public helper for `W_k * w_ki`.
3. **Two normalizers are retained.** Each ancestor's motion proposals are sampled
   from odometry. Their likelihoods are averaged before marginalizing over that
   hypothesis's conditional particle prior. The resulting log evidence updates
   graph mass, and conditional weights normalize separately. The scan-matcher
   optimum is never substituted for predictive evidence.
4. **Explicit branch priors.** Each geometrically feasible parent produces null
   and loop children with priors `1-loop_branch_prior` and `loop_branch_prior`.
   Their masses sum to the parent's mass. Compatibility is a hard gate, not a
   likelihood multiplier. If a parent's loop is impossible under that gate, its
   whole mass stays on its null alternative.
5. **Frozen historical validation.** Each child gets an immutable distance field,
   observed-cell mask, and world pose from the candidate's historical reference
   submap, positioned according to that child's trial graph. A reference that is
   unfinished or contains the query scan is refused. This avoids validating against
   an active submap which moves rigidly with the robot and may make loop/null
   predictions indistinguishable. This first implementation uses one historical
   target, not a full composite map.
6. **Future evidence only.** Branch creation occurs after the triggering insertion;
   the next processed scan is the first possible evidence scan. Known-space gating
   chooses a common beam subset across the whole competing population. Every branch
   is scored on the same subset. With insufficient common coverage, the measurement
   update is neutral at both hierarchy levels; odometry still propagates particles
   and the independent tracking frontend still runs.
7. **Bounded sequential decisions.** At least 10 usable future scans are required.
   Summed loop mass >= .95 retains loop alternatives; <= .05 retains null
   alternatives. The remaining alternatives are renormalized and their particle
   population is refilled. At 30 attempted scans without a decision, the result is
   `undecided`, including when there were too few usable scans.
8. **Resampling and spatial splitting preserve probability.** Resampling resets
   conditional weights to `1/N_k` while carrying graph log masses separately,
   including masses too small to represent as linear doubles. A spatial split
   transfers the appropriate fraction of the parent's mass to its new child.
9. **Validation does not alter its own target.** Live grids keep receiving accepted
   tracking scans. PGO and spatial splits pause during the evidence window so the
   frozen graph/coordinate interpretation stays fixed. PGO can resume after a final
   decision. Covariance reporting during validation uses the resulting conditional
   particle distribution, rather than the frontend's differently scored proposals.
10. **Heuristic ablation remains available.** `loop_update_mode:=heuristic` keeps
    the original deformation-weighted branch rules, with the explicit mass storage.
    `loop_verifier_mode:=map|uniform|geometry` belongs to that legacy mode. Bayesian
    mode requires `loop_verifier_mode:=belief`, so a MAP comparison cannot silently
    become the same algorithm with a different label.

## Deliberate limits

- One candidate is evaluated sequentially per event. A candidate is ranked by the
  detector's geometry, then expanded over all retained parent graphs for which the
  gate passes. Simultaneous categorical alternatives for several distinct loop
  candidates are not implemented in Bayesian mode. `loop_max_branches` still
  applies to heuristic mode.
- A new event starts only if **all current parents plus their feasible loop
  children fit** in `max_hypotheses` and the particle budget. Otherwise it is
  deferred with `bayes_deferred_budget`; no parent is dropped just to manufacture
  space or confidence. With four existing parents and four slots, no new event can
  start. Raising the cap costs additional graph, map and particle work.
- An undecided event retains both associations, releases its validation snapshots,
  and freezes graph masses and spatial splitting outside a subsequent validation
  event. Tracking and conditional localization continue. A later event may replace
  the evidence window if the budget and reference permit it. If the bank is full,
  it can remain undecided indefinitely. This conservative version does not silently
  label a resource-driven deletion as Bayesian rejection.
- MAP/pose-risk output still publishes one complete **tentative** hypothesis while
  a decision is pending. Its publication is not a declaration that the loop has
  passed the posterior threshold. `bayes.csv` is the acceptance ledger.
- The same LiDAR likelihood weights the particles and graph mass during validation;
  the live-map tracking score is not multiplied in again. Outside a loop window,
  ordinary spatial modes use pre-insertion tracking evidence, except for the
  frozen graph masses of an undecided bank.

## Parameters and diagnostics

| Parameter | Default | Meaning |
|---|---:|---|
| `loop_update_mode` | `bayes` | Sequential evidence or `heuristic` ablation |
| `loop_branch_prior` | 0.5 | Loop prior within an eligible parent |
| `loop_bayes_min_scans` | 10 | Minimum usable future evidence scans |
| `loop_bayes_max_scans` | 30 | Maximum future scan attempts |
| `loop_bayes_accept_probability` | 0.95 | Loop retention threshold |
| `loop_bayes_reject_probability` | 0.05 | Null retention threshold |
| `loop_bayes_beta` | 0.1 | Multiplies `tracking_effective_beams` once |
| `loop_bayes_min_known_fraction` | 0.35 | Common known beam fraction required |
| `loop_geometry_min_compatibility` | 0.01 | Hard deformation gate |
| `hypothesis_prune_mass` | 0.000001 | Generic pruning outside pending/undecided events |
| `loop_bayes_diagnostics_path` | empty | Sequential evidence CSV |

With the existing `tracking_effective_beams=20`, beta=.1 gives a validation exponent
of 2 times the mean beam log score. These are conservative starting values, not
calibrated settings. The gate also requires at least `tracking_min_points` common
beams. `loop_validation_scans=3` remains the **old retrieval check on stored scans**;
it is different from the new 10–30 **future-scan** window.

`bayes.csv` records initial masses, every attempted future update, usable evidence
count, common beam count, predictive log evidence, pre-decision graph mass, summed
loop probability and the event decision. Initial installation is `pending`, never
`accepted`. `loops.csv` still records candidate/trial geometry; its `selected` and
`trial_installed` fields mean **tentative installation**, not sequential acceptance.
Its `verification_status` also records budget/reference deferrals.

## Statistical interpretation and experiments

The hierarchical normalizations implement Bayesian algebra, but the existing
likelihood-field kernel is an unnormalized robust endpoint score, and beam/frame
correlations plus the data-quality gate are approximations. Consequently this is
**tempered generalized Bayes**, not an empirically calibrated 95% probability that
an arbitrary physical loop is correct. A normalized range sensor likelihood and
held-out calibration would be needed for the stronger claim.

A defensible implementation description is:

> Beluga-MH maintains explicit hypothesis masses and conditional particle weights,
> and updates competing loop-closure hypotheses using tempered, particle-marginalized
> predictive LiDAR evidence against frozen historical reference submaps.

For the new contribution, run otherwise identical `bayes` and `heuristic` trials
with `--verifier belief`. For the earlier MAP-versus-belief verifier experiment,
keep `--loop-update-mode heuristic` fixed and change only `--verifier`.
The historical comparison runner defaults to heuristic for that reason; the Intel
launch and offline accuracy replay default to Bayesian mode. Capture actual runtime
parameters and complete recordings for every comparison.

Measure wrong-loop acceptance/rejection, undecided/deferred frequency, time to
resolve, association calibration, online and optimized pose RMSE, and runtime.
Do not infer lower SLAM RMSE from the synthetic matcher or posterior tests.
