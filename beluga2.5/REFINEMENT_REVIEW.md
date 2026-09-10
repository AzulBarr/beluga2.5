# Loop refinement and evidence reuse revision

This revision starts from the exact 40-file manifest of `beluga_late_run_fix.zip`.
The latest attachment was a map screenshot; no newer source or diagnostic logs
were supplied. The screenshot suggests improved consistency, but cannot identify
whether its remaining errors come from local registration, calibration, loop
selection, PGO, dynamic returns, or rendering.

## Changes grounded in the source

**Close the loop search's refinement gaps.** The previous translation lattice
spacing was 0.50 m, followed by refinement radii 0.15, 0.05 and 0.015 m. Their sum
is 0.215 m, short of half the coarse spacing. The angular spacing/refinements
had the analogous gap. A synthetic pose near a cell midpoint reproduces this
limitation. The replacement covers zero and both search-window endpoints, then
uses seven halving levels in the fixed initial search frame. All evaluated poses
stay inside the original bounds. Bounds of zero evaluate the initial pose once.

**Refine separate geometric alternatives.** Each of up to eight separated coarse
seeds keeps its own refinement path. Nearly identical fine poses cannot occupy
another seed's slot. The search returns up to `loop_search_modes` separated final
alignments per reference (default 2, allowed 1..8). Every alignment must pass the
existing neighboring-scan corroboration with one fixed transform. The candidate
union is still capped at `loop_max_verifications` (default 6), and every surviving
candidate is trial-optimized under every retained hypothesis. This preserves
geometric ambiguity for the trajectory belief to evaluate. Bounded search can
still miss modes; this is not a global-optimum guarantee.

**Use continuous native-field scoring.** Loop matching now bilinearly interpolates
cached distance and Gaussian-score arrays at cell centers. The matcher computes
one sine/cosine pair per pose and scores endpoints directly, without constructing
an Eigen transform per endpoint or allocating another field. Interpolation
removes the old floor-index plateaus. Cache ownership and eviction are unchanged.
A tie in geometric score prefers the smaller displacement from the search seed;
a flat direction no longer chooses a window corner merely because it was visited
first. This tie rule does not modify the evidence score or the hypothesis weights.
It is not a substitute for a calibrated anisotropic loop-information model.

**Consume a loop decision once per raw query scan.** Previously, checking only
whether a particular graph already contained an identical edge was insufficient:
no-loop descendants lacked that edge, and an existing-loop descendant could get
compatibility zero for a rediscovered candidate. A belief-level query ledger now
records a query after its categorical loop/no-loop update. Later appearances of
that query do not rebuild trial graphs, reweight descendants or create duplicate
branches. Rejected queries can be evaluated later because no branch update was
committed. This fixes exact reuse, not statistical correlation between different
neighboring scans. The ledger stores one integer per consumed event and grows
with graph history; it is not a constant-memory claim.

**Expose the pruning approximation.** The CSV appends `event_id`, `query_consumed`
and `retained_branch_mass`. The last value is the retained fraction of the
constructed, unnormalized categorical branch pool after budget pruning and
no-loop reservation. It is not the posterior mass of a single selected trajectory.
The audit checks frozen priors within an event, repeated query consumption,
weighted/MAP/uniform score consistency and the bound on MAP disagreement. It also
reports discarded branch mass. The short paper note now proves the one-step
truncation bound `|S_full-S_keep| <= delta` for fixed bounded compatibilities.

A diagnostic loop marker also now uses its own selected candidate's branch pose;
previously multiple markers could all use the first selected branch's pose.

## What remains central to the algorithm

The user-tested tracking/recovery, submap lifecycle, insertion filter, motion
proposals, within-mode resampling and periodic PGO scheduling retain their previous
implementation and defaults. Loop closure and PGO remain enabled. Loop/no-loop
branching and persistent spatial clustering remain enabled. The total particle
and hypothesis budgets remain 30 and 4 in the Intel launch.

For each fixed candidate, all retained priors and their weights are frozen before
trial evaluation. Unsupported modes retain compatibility zero in the denominator.
The verifier still computes `sum_h w_h e_h`, and its default threshold remains
0.25. Loop-search modes are alternative measurements; SLAM hypotheses are weighted
trajectory/map states. These are different levels of ambiguity. The same bounded
set of measurements is evaluated under all live SLAM hypotheses.

“Beyond the MAP Trajectory: Belief-Weighted Loop Closure Verification for
Multi-Hypothesis Particle-Filter SLAM” remains a reasonable description of this
implementation. Describe it as a bounded, approximate shared-map PF architecture:
its particles do not each own an independent full trajectory and occupancy map,
and deformation scores/branch factors have not been established as calibrated
Bayesian likelihoods. The propositions justify information retained by the rule;
they do not prove that adding hypotheses always improves SLAM or establish novelty.

## Cost and validation limits

The default search has at most 1,859 coarse score evaluations plus 1,456 fine
ones (3,315 total) per reference. This is more search work than the previous
2,507 maximum evaluations. Direct scoring avoids per-endpoint transform setup,
but interpolation performs additional field reads. More returned modes can fill
more of the existing trial budget. Measure retrieval and trial PGO time on the
actual machine before claiming a throughput improvement. The default maximum
remains 24 trial solves per verification batch (6 candidates times 4 hypotheses).
Historical maps and pose graphs still grow.

Executed here: the new 50-check standard C++ search/evidence suite, including 12
synthetic wall-registration cases, and seven Python diagnostic fixtures. Address
and undefined-behavior sanitizers also exercise the new standalone suite. Previous
standard C++ suites and the Intel reader audit are rerun for regression coverage;
see the package's `VALIDATION.md` for exact results.

The full core/ROS node and three new integration gtests require dependencies not
installed in this environment. They are supplied for execution on the ROS machine.
No new end-to-end replay, ATE/RPE, loop precision/recall, latency distribution or
memory-growth result is claimed. A synthetic field with ideal walls is much easier
than the user's real sequence and its error figures must not be reported as Intel
accuracy.

## Controlled replay

Apply only the patch for your installed revision and rebuild Release with tests.
Replay from a fresh process with the same seed, input timing, beam angles, map
resolution, particle budget and hardware. Keep the three diagnostic CSVs.

1. Compare this revision with the preceding user-tested late-run revision.
2. Run this revision with `loop_search_modes:=1` to isolate retaining multiple
   geometric alternatives. This retains the new interpolation and refinement.
3. Compare `loop_verifier_mode:=belief`, `map`, and `uniform` with identical total
   particle/hypothesis budgets and geometry settings, across several seeds.
4. Use `max_hypotheses:=1` as a separate architecture ablation. It changes the
   retained belief and is not the same experiment as changing only the verifier.
5. Measure the entire recording, including late processing age, frame accounting,
   tracking rejection, output corrections, trial cost and discarded branch mass.
   Label loops or use defensible reference trajectories for accuracy comparisons.

Changing the verifier changes later trajectories and retrieval. Offline rescoring
of one run controls the candidate set but is not an independent SLAM replay. For a
paper, use both controlled scorer comparisons and independent end-to-end runs.
Do not use one cleaner map screenshot as the evidence that multiple hypotheses
outperform one trajectory.
