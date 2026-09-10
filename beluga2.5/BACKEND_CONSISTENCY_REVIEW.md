# Backend consistency revision — 8 September 2026

This revision addresses an objective mismatch demonstrated in the loop verifier
and replaces automatic differentiation of SE(2) graph edges with analytic
derivatives. It preserves the weighted multi-hypothesis PF, fixed particle
budget, loop/no-loop alternatives, spatial splitting, and submap lifecycle.
It is a test candidate. Full ROS compilation/replay and improved RMSE have not
been established here. No reference trajectory was used.

## What the uploaded run establishes

Input: `map_h4_seed42_228o5el2.zip`. All six recorded source/input hashes match
the preceding delivered version. Runtime parameters confirm MAP loop
verification, pose-risk output selection, 30 particles, at most four hypotheses,
seed 42, two workers, and enabled loop closure/PGO.

All 13,631 scans have processed performance rows and complete tracking groups.
There are 41,259 tracking rows and 3,208 loop trial rows for 1,109 candidates.
The observed scan coverage does not support a dropped-frame diagnosis for this
run. However, backend pauses reach 2.405 seconds and reported scan age reaches
2.800 seconds. These latency peaks deserve continued measurement.

The previous belief run has a shorter complete logged prefix. Compare published
poses on the common 13,615 timestamps:

| Measurement | Previous belief verifier | Uploaded MAP verifier |
|---|---:|---:|
| Output corrections greater than 0.35 m | 76 | 71 |
| Output corrections greater than 1 m | 25 | 24 |
| Largest output correction | 2.240 m | 2.704 m |

The first output divergence occurs at scan sequence 10,261. This is mixed
evidence: neither verifier is demonstrated superior. An output correction is
the deviation from the preceding output propagated by odometry; it is not
error against ground truth. Neither endpoint proximity nor a clean-looking
map measures trajectory RMSE. One seed and one environment cannot establish
the paper's empirical claim.

My preceding recorder incorrectly marked the map capture complete. Its ROS
CLI command truncated the array at 128 entries; this map requires
1,167 × 1,072 = 1,251,024 entries. The new recorder requests `--full-length`,
checks positive dimensions/resolution and the exact cell count, and rejects
ellipsis or values outside [-1,100]. It also verifies the running node's key
experiment parameters. The CLI's default array truncation and full-length
option are documented by [OSRF](https://osrf.github.io/ros2multirobotbook/ros2_cli.html).
The uploaded CSVs remain useful, but the truncated array cannot reconstruct
the final map.

## Fix 1: settle a proposed loop under its installed objective

Previously, a candidate's trial used a quadratic penalty for the new loop,
while older loops used Huber(1). Subsequent ordinary PGO applied Huber(1) to
all loops, including the newly installed loop. Forcing the candidate is useful:
otherwise a robust loss can suppress a contradictory loop and make its induced
trajectory deformation look harmless. The missing step was checking what
happens once the candidate receives the loss used during ordinary PGO.

For each candidate/hypothesis pair, the revision:

1. Runs the existing forced-candidate trial and requires convergence and fit.
2. Measures forced compatibility against the original prior trajectory.
3. If the candidate's weighted residual norm exceeds 1, runs a second trial
   solve with ordinary all-loop-Huber PGO. It requires convergence and checks
   the same 0.30 m / 0.12 rad fit limits again.
4. Uses the smaller of forced and settled trajectory compatibilities. Both
   are measured against the same original prior and sampled scan sequences.
5. Passes only the settled trial to the existing branch selection/installation.
   A failure contributes compatibility zero; it does not mutate the live prior.

The second solve is skipped when the weighted residual is within Huber's
quadratic region. At such a forced solution, the candidate's residual block
has the same local loss as ordinary PGO. This is a local consistency check,
not a guarantee of a global optimum or invariance to future graph additions.

The residual norm is `sqrt((10*t)^2 + (12*a)^2)` for translation fit `t` and
rotation fit `a`, using the actual stored edge weights in code. Huber acts on
the entire three-residual block. See [Ceres's residual/loss definition](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#lossfunction).

In the uploaded run, 522 of 2,401 usable trial rows lie outside this quadratic
region. Of 1,300 usable rows belonging to selected candidates, 154 lie outside.
The old `selected` flag is candidate-wide; these 154 rows do not prove that all
154 hypothesis branches survived the population cap. New `trial_installed`
diagnostics remove that ambiguity. These counts identify where a second check
would be relevant on the old recorded trials; they do not predict the new
run's acceptance count after trajectories and proposals diverge.

The option is `loop_robust_polish:=true` by default. It adds solve work and can
reject genuine loops that the current graph cannot reconcile within the fit
gate. At the current budget, an event has at most 24 candidate/hypothesis
trials and at most 24 extra solves. Every retained mode is still evaluated.
The next replay must check latency as well as accepted/rejected constraints.

## Fix 2: analytic SE(2) graph derivatives

`pose_graph_residual.hpp` implements the same weighted translation and wrapped
rotation residuals, including the fixed transform from the active-submap rigid
group to each local grid. It provides both 3×3 Jacobian blocks directly.
The constant offset is pre-rotated at construction. The Ceres adapter obeys
its [row-major and nullable Jacobian interface](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#costfunction).

This avoids Jet arithmetic in graph-edge differentiation. It does not change
edge weights, gauge anchoring, loss thresholds, or solver tolerances. The
original AutoDiff implementation is retained behind
`pgo_analytic_jacobians:=false` for comparison. Numerical equivalence does not
promise bitwise identical branching in a long nonconvex SLAM run. Overall
speedup has not been measured; sparse factorization and matching may dominate.

## Connection to the paper

The title **Beyond the MAP Trajectory: Belief-Weighted Loop Closure Verification
for Multi-Hypothesis Particle-Filter SLAM** continues to describe the method.
The verifier evaluates each fixed candidate under every retained trajectory
and combines its compatibility using weights frozen before candidate trials.
Missing or failed modes contribute zero, without renormalizing over supporters.
The score remains a posterior-weighted compatibility heuristic; the deformation
scores and branch factors are not established calibrated loop probabilities.

For fixed prior weights `w_h >= 0`, summing to one, define
`e'_h = min(e_forced,h, e_settled,h)` for passing trials and zero for failures.
Then `e'_h <= e_forced,h`, so

\[
\sum_h w_h e'_h \leq \sum_h w_h e_{\mathrm{forced},h}.
\]

Proof: multiply each pointwise inequality by its nonnegative weight and sum.
Thus changing to the installed robust objective cannot increase verification
evidence merely by making a loop easier to ignore. This statement is about
one fixed candidate and prior belief; it says nothing about future candidates,
loop recall, or RMSE after branching and pruning.

The existing `paper/BELIEF_PROPOSITION.md` gives the separate elementary bound
showing why the MAP compatibility does not generally determine the weighted
decision. It still applies because the new per-mode compatibilities remain
in [0,1]. Neither proposition proves that retaining more hypotheses always
improves SLAM. Novelty and ICRA acceptance require separate evaluation.

## Validation and next measurements

Completed here:

- 2,000 compiled C++ residual/Jacobian fixtures, including rigid offsets,
  partial Jacobians, wrapped angles, and nonfinite inputs. Maximum residual
  discrepancy from the original transform formula: 3.411e-13. Maximum
  discrepancy from finite-difference derivatives: 3.039e-7.
- Three native Ceres fixtures through an existing Python binding, calling the
  compiled production C++ kernel. A forced fit of 0.25 m becomes 0.40 m under
  robust PGO and is rejected by the settled-fit rule; a second fixture settles
  at 0.25 m and remains valid; a third stays inside the quadratic region.
  The fixtures use stricter solver tolerances for closed-form comparisons;
  the same three cases also passed their fit gates and 1e-3 position checks
  with Ceres's default tolerances.
- 47 existing loop-belief/mass checks, 12 motion-filter checks, 315 output
  selector checks, and 24 robust-tracking/proposal checks.
- 46 Python tests, including full/short/truncated/invalid map arrays, stale
  parameters, independent ablation flags, and a simulated CLI shutdown/capture.
- Python/XML launch syntax and parameter wiring checks.

The native Ceres fixtures exercise the residual kernel and loss mechanism,
not the full Beluga graph or ROS executable. C++ integration tests were added
for actual trial isolation, rejection/settling, the quadratic fast path, and
analytic-versus-AutoDiff costs. The full ROS/Ceres development toolchain is
absent here, so those integration tests and a full Intel replay remain to run
on your machine. Build/test commands in the package stop on failures.

`loops.csv` adds forced and settled compatibilities/fits, polish attempt/time,
verification status, and per-hypothesis installation. `performance.csv` adds
`polish_solves` and `polish_work_ms`; the latter sums elapsed trial times that
may overlap across workers, so use `backend_ms` for backend wall time. The
summary tools understand the appended columns and still read old CSVs.

First run the revised belief verifier with the same seed and budget. Then
isolate the new fit check using the same binary and
`--no-loop-robust-polish`; isolate differentiation using
`--no-pgo-analytic-jacobians`. Once this revision is stable, compare belief
versus MAP verification with both backend options held fixed, and compare
one versus four hypotheses at the same total particle budget. Use several
seeds and environments before drawing paper-level conclusions. Ground-truth
or another independent pose reference is necessary to report absolute RMSE.

This report supersedes earlier revision notes where implementation details
conflict. Those files remain in the package as historical records.
