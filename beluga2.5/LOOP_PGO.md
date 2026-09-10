# Belief-weighted loop verification and PGO

**Revision note:** this file documents the earlier revision. See
[QUALITY_REVIEW.md](QUALITY_REVIEW.md) for the current frontend, population policy,
loop corroboration and Intel replay changes. Earlier kernel timings do not
measure the current end-to-end SLAM pipeline.

Loop closure and PGO are enabled by default. This is a research implementation
to build and benchmark, not a claim of improved accuracy or publication readiness.

## Paper alignment

Recommended title:

**Beyond the MAP Trajectory: Belief-Weighted Loop Closure Verification for
Multi-Hypothesis Particle-Filter SLAM**

The uploaded implementation did not compute a belief-weighted verifier. It
performed geometric matching within one hypothesis, moved low-weight particles
into a loop child and optimized that child. The modified implementation evaluates
each fixed candidate against every retained trajectory hypothesis and marginalizes
the resulting compatibility scores.

Use this narrower central claim:

> We verify each geometrically proposed loop against every retained trajectory
> hypothesis and aggregate the resulting compatibilities using the current
> normalized hypothesis weights. This can retain loops supported by sufficient
> secondary-mode mass even when the MAP trajectory rejects them, and reject
> candidates that are incompatible with all retained modes.

“Can” and “retained” matter. A 1% mode alone cannot pass a 25% evidence threshold.
No method can recover support in a trajectory mode already pruned from the belief.
Nor does compatibility with a prior establish that a loop is physically correct.

I would avoid **RBPF** in the title for now. The implementation shares one map
and continuous insertion trajectory per hypothesis, deterministically scan-matches
particle poses, and uses heuristic likelihood scaling. It does not implement or
derive an exact Rao-Blackwellized posterior/proposal correction for those steps.
Particle-filter SLAM accurately describes its present structure.

## Same candidate, all hypotheses

Let a fixed candidate be `c = (r, q, Z_rq)`, where `r` and `q` are global input-scan
sequence numbers and `Z_rq` is a measured SE(2) transform from reference scan r
to query scan q. It is held fixed while comparing hypotheses.

Candidates are retrieved from the union of the live hypotheses' historical
submaps. Each source hypothesis supplies bounded distance/signature retrieval
and geometric matching. Search remains bounded by pose distance and a finite
correlative window, so it is not unrestricted global place recognition.

A submap records the sequence of its initial scan, whose pose defines that
submap's frame. A match in that grid therefore supplies `Z_rq`. Every hypothesis
records compact scan poses relative to its own submaps, including scans its
insertion filter rejected. Thus differing keyframe decisions and branch-local
numeric node/submap IDs cannot silently turn c into a different association.
Sequence metadata survives point-cloud trimming.

When a query was filtered in another hypothesis, only the temporary trial graph
gets an auxiliary query node attached to that hypothesis's existing submap.
An accepted child keeps the auxiliary node; no scan is reinserted into a grid.

## Compatibility and aggregation

For hypothesis h, let `w_h` be the sum of its normalized particle weights before
adding any candidate in the current event. These weights are frozen while all
candidates are evaluated. They are a maintained approximate belief, not proven
calibrated probabilities.

1. Start from that hypothesis's pre-candidate graph and trajectory.
2. Copy the graph state, add c and run trial PGO. Live particles and grids are
   untouched. The candidate edge has no robust loss during this trial: it must
   actually exert its constraint. Existing accepted loops retain Huber losses.
3. Require a usable finite solve and a sufficiently small residual on c. A trial
   that ignored or failed to fit the candidate supplies zero compatibility.
4. Sample up to 200 matching scan sequences between r and q. Rigidly align the
   trial positions to the prior positions in SE(2), then measure position RMSE
   `d_t,h` and heading RMSE `d_r,h` using that same alignment rotation. Scale is
   fixed because LiDAR is metric; SIM(2) would hide contraction or stretching.
5. Compute `e_h(c) = exp(-0.5 * ((d_t,h / s_t)^2 + (d_r,h / s_r)^2))`, with
   default scales `s_t = 0.30 m` and `s_r = 0.10 rad`.
6. Compute **`S(c) = sum_h w_h * e_h(c)`**. The default threshold is 0.25.

Missing samples, failed baseline/trial solves and unusable alignments contribute
zero. Their weight remains in the denominator; supporting modes are never
renormalized separately. Candidate fit limits default to 0.30 m and 0.12 rad.
These scales and thresholds require calibration on held-out validation data.
The scores are compatibility kernels, not normalized observation likelihoods.

The geometric score now averages over all scan endpoints; out-of-grid returns
cannot improve it by disappearing from the denominator. “Overlap” is the fraction
within 0.30 m of a wall, rather than merely the fraction inside the grid rectangle.
At least 30 query endpoints are required by default.

## Branch weights and finite budgets

For K accepted candidate alternatives in one event, unnormalized branch masses
are `m_h,0 = w_h (1-pi) kappa_0` for no-loop, and
`m_h,k = w_h pi e_h(c_k) / K` for loop alternative k. Defaults are `pi=0.5` and
`kappa_0=0.2`. Candidates are categorical alternatives, not independent likelihood
factors multiplied together. Geometry-only mode uses one instead of the trajectory
compatibility for numerically usable trial branches.

These are explicit heuristic association factors. They are not a calibrated
Bayes factor, and reusing the current scan in the frontend and verifier does not
come with a proven independence assumption. Include calibration/ablation results
before making posterior-probability claims in a paper.

Retain the strongest branches within `max_hypotheses`, reserving a no-loop branch
when at least two slots are allowed. The retained masses are normalized; this is
a truncated approximation. Particle quotas are allocated within `max_particles`,
sampling each child conditionally from its parent's weighted particles. Every
particle in h receives `w_h / quota_h`: a minimum quota never creates belief mass.
The previous hard-coded four-mode cap and 5% pruning cutoff were removed in favor
of the configured bound and numerical-zero pruning. If the budget is exhausted,
some spatial clusters still stay within their parent, as in the original design.

## PGO consistency

- Trajectory nodes and submaps keep immutable frontend local poses; global poses
  are separate. Future local edges use local-pose differences, not previously
  optimized global poses. A loop correction cannot rewrite its own local prior.
- Every live overlapping grid and the current matching reference use one shared
  SE(2) optimization variable with fixed local offsets. PGO can move their group
  but cannot tear the pair apart or deform the grid interiors.
- Historical submaps and scan nodes are optimized jointly. The first submap fixes
  the gauge. Existing inter-submap edges use Huber loss.
- Current particles, the online tracking pose and the insertion-filter reference
  move with the corrected local-to-global transform of the tracking submap. They
  no longer follow a potentially different latest-node correction.
- Solves use temporary parameter arrays and commit only after usability, cost and
  finite-value checks. Trial hypotheses never transport live particles. Pose-only
  submap clones share grids; subsequent pixel insertion detaches on write.
- All live hypotheses receive periodic PGO (default every 20 added nodes), and
  baseline optimization runs before a loop event. Trial solves are bounded by
  `loop_max_verifications * active_hypotheses`, each with at most 50 iterations.

The backend still waits for trial completion before committing the belief, and the full graph grows with the run. Independent trials now use bounded parallel work; see [PERFORMANCE.md](PERFORMANCE.md) for caching, publication, timing and build changes. This is not an incremental iSAM2 backend.

## Ablations and diagnostics

The Intel launch forwards these switches:

- `loop_verifier_mode:=belief`: weighted aggregation, default.
- `loop_verifier_mode:=map`: MAP compatibility gate in the same MH backend.
- `loop_verifier_mode:=uniform`: unweighted compatibility gate.
- `loop_verifier_mode:=geometry`: geometric candidates plus numerical fit guards.
- `enable_loop_closure:=false enable_pgo:=true`: local graph optimization without loops.
- `enable_loop_closure:=false enable_pgo:=false`: isolate submapping again.
- `random_seed:=42`: core sampling seed; use several seeds for experiments.
- `loop_diagnostics_path:=/tmp/beluga_loops.csv`: one row per candidate/hypothesis.

Both loop and PGO must be enabled for loop application. Diagnostics include
candidate sequence IDs/transform, prior weights, trial fitness, trajectory RMSEs,
compatibilities, weighted/MAP/uniform scores, eligibility and retained selection.

```bash
python3 tools/summarize_loop_verification.py /tmp/beluga_loops.csv --threshold 0.25
```

This locates belief/MAP disagreements on identical candidates from one recorded
run. With independent ground-truth labels (`candidate_id,label`, 0 or 1):

```bash
python3 tools/summarize_loop_verification.py /tmp/beluga_loops.csv --labels labels.csv
```

Unknown labels remain unknown. Without labels, acceptance counts and score
disagreements are not precision, recall, accuracy or evidence of improvement.
Separate end-to-end runs will generate different candidate pools as the maps
diverge; report that distinction from fixed-candidate scorer ablations.

## ICRA evidence still needed

1. Build and run all C++ tests, then replay real datasets. No full ROS/Ceres build
   or dataset result was available in the patch-generation environment.
2. Show at least one labelled secondary-mode rescue and one false geometric loop
   rejected by trajectory support. A nicer occupancy image alone is insufficient.
3. Report false-loop precision/recall or PR curves, trajectory error with GT,
   runtime, memory, number of hypotheses and several seeds at a fixed particle
   budget. Separate threshold tuning from held-out test sequences.
4. Compare geometry, MAP-only, uniform and weighted scorers on identical inputs;
   also compare end-to-end SLAM. This MAP ablation is not an implementation of
   the official ROVER algorithm and must not be labelled “ROVER.”
5. Evaluate sensitivity to weight/compatibility calibration and hypothesis count.
   Candidate retrieval bounds, PF collapse, repetitive geometry and incorrect
   priors can still make the verifier fail.

The simple observation motivating the title is information loss: two beliefs can
have the same MAP trajectory (weight 0.6) but put the remaining 0.4 on different
trajectories. If the MAP has compatibility 0, one secondary trajectory has 1 and
the other has 0, the marginalized scores are 0.4 and 0. A 0.25 threshold separates
them although the MAP input is identical. This is a basic mathematical example,
not a novel theorem or a guarantee of empirical improvement.

Related work that prevents a broad novelty claim:

- [ROVER](https://arxiv.org/abs/2508.13488) already verifies loops using changes
  between a prior trajectory and candidate-conditioned PGO. Our metric SE(2)
  position-and-heading kernel is a ROVER-inspired variant, not its SIM(3) scorer.
- [MH-iSAM2](https://www.cs.cmu.edu/~kaess/pub/Hsiao19icra.pdf) already provides
  multi-hypothesis inference including loop-existence alternatives.
- [LCPF](https://ieeexplore.ieee.org/document/8964341/) already combines particle
  filter LiDAR SLAM with loop detection and correction.

The defensible proposed contribution is the particular maintained-belief loop
verifier and its demonstrated benefit. This limited literature check does not
establish that the exact combination is unprecedented or predict acceptance.
