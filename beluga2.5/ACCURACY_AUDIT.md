# Accuracy audit — test candidate

This working revision was initially withheld pending stronger validation and is
now supplied for testing at the user’s request. It does **not** establish improved
pose accuracy or mapping.
No claim of reduced absolute RMSE, superior maps, a breakthrough, or paper acceptance
is supported. No reference trajectory was read or used in this iteration.

The starting point was verified byte-for-byte against all 46 files in
`beluga_regression_recovery.zip`. The numerical core in that package was the
previously user-tested late-run version, with the failed loop-search refinement
reverted.

## Two corrections in the working source

1. **Output selection after population changes.** `post_update` selected the
   largest hypothesis before `resample` performed spatial splitting/pruning.
   A mode with mass .55 can split into .275 and .275 while another retains .45;
   the old output still reports the original parent. `refresh_output_selection`
   now runs after either resampling path, including the high-ESS early return.
   Pose, map selection, selected-mode diagnostics and covariance then refer to
   the current population. Two core integration regressions exercise splitting
   without ESS resampling and pruning of the previously selected mode. These
   integration tests are **not executed** in this environment.

2. **Rejected-match scores describe the returned pose.** The production matcher
   returned the odometry prediction after rejecting its optimized pose, but kept
   the optimized pose's likelihood/overlap/cost. Recovery then compared against
   a score inconsistent with the state actually retained. Rejection now restores
   both the prediction and its corresponding scores. A new test deliberately
   rejects an otherwise movable match: it fails on the preceding header with
   `rejected likelihood describes discarded optimizer pose` and passes after the
   correction. Accepted-match optimization is unchanged. This can change recovery
   proposal admission; its complete mapping effect still requires a SLAM replay.

The fixed-candidate, frozen-weight, all-hypothesis loop verifier and the
loop/no-loop and spatial bifurcations remain in place. No loop thresholds,
information weights, hypothesis priors or particle budgets were retuned.

## Real-recording component experiment

The diagnostic compiles and calls the **production** `TrackingField`,
`match_tracking_scan` and `tracking_score` implementations. It processes all
13,631 Intel FLASER records in acquisition-time order, giving 13,630 scan pairs.
It uses the existing reader's -90 degree start and 1 degree beam spacing,
.1–25 m valid range and .05 m grid resolution. The Intel ROS launch uses a 30 m
maximum; this 25 m component experiment is not an exact replay of that launch.
Each pair uses a fresh endpoint
field from the other scan. Two thirds of query endpoints fit the pose; the
remaining third measure a held-out likelihood. Forward and backward fits use
their own odometry priors. No scan is inserted into its own matching field.

This deliberately isolated experiment has **no accumulated submaps, persistent
particles, recovery, graph optimization, reference path or integrated output
trajectory**. It cannot reproduce ROS drops, global map deformation, or the
posterior-selection correction. It is not a substitute for replaying BelugaSLAM.
Held-out beams remain spatially correlated with fitting beams; they are not an
independent accuracy reference.

The starting matcher accepted all 13,630 pairs in both directions. Its forward/
backward translation discrepancy had median .00855 m, 95th percentile .02987 m,
and maximum .42536 m. The worst pair was sequence 13,264, late in the recording;
both registrations passed the local acceptance criteria. The average held-out
log-score gain over the odometry seed was .02087. There were 259 pairs with a
held-out log-score reduction greater than .01. These figures demonstrate that
passing local overlap criteria does not guarantee consistent registration.

The ~43 cm cycle discrepancy is a diagnostic warning, **not pose RMSE**, and it
does not establish that this pair caused the user's final map distortion.
The revised matcher is replayed separately with the same experiment. Because
every original match was accepted and the correction changes only rejection
metadata, accepted poses/scores should be identical; CSV comparison verifies this.
No quality improvement is inferred from unchanged pair results.

Reproduction of this component experiment (standard C++17 compiler and Python):

```bash
python3 tools/run_scan_pair_audit.py belugaslam_example/bags/intel/intel.clf /tmp/beluga_pair_audit
```

The output records the recording and matcher SHA-256 hashes, all pairs, coverage,
held-out scores and cycle discrepancies. Pair timing excludes field construction,
parsing, ROS, PF and PGO and must not be reported as SLAM latency.

## Validation limits and release gate

Executed: 24 C++ matcher/proposal checks; 14 C++ recovery/cache checks; 24 Python
trajectory-evaluation tests; structural submap/ROS-parameter validation. The new
rejection regression also demonstrably fails on the preceding matcher.

The complete core/ROS build and the two new population integration tests remain
blocked by missing Ceres/range-v3/ROS development dependencies. Automatic network
approval cancelled the attempted external dependency downloads. Available Python
Ceres bindings contain no C++ headers and do not make the SLAM core runnable.
No substitute solver, mocked dependency, or separately implemented SLAM was used
to claim that the real pipeline passed.

Before release: compile/run the real core integration suite and replay the full
recording from a fresh process, comparing this checkpoint with the recovery
baseline under equal seeds, particle budgets and timing. Inspect actual loop
events, output poses, tracking losses, dropped frames and late-run map coherence.
Reference-free consistency tests can falsify reliability but cannot establish a
ranking in absolute RMSE against graph SLAM or a standard RBPF. A defensible
accuracy comparison still needs an independent accuracy reference or a controlled
experiment with known poses, and fair baseline runs.
