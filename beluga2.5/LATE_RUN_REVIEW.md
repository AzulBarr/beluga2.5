# Late-run recovery and resource revision

This revision starts from `beluga_quality_fix.zip`. The supplied screenshot is
more coherent than the preceding one, but it still contains overlapping walls
and inconsistent geometry. It cannot identify the exact failure time or distinguish
local tracking loss, a wrong loop, a hypothesis switch, or processing backlog.
No new runtime CSVs, trajectory ground truth or source archive accompanied it.
The latest shared source was checked against the previous delivery before editing.

## Findings and changes

### Tracking could fail without a route back

Ordinary tracking searched a small seed lattice and used a bounded local optimizer.
After an unsupported match, insertion stopped while odometry continued propagating
the pose. There was no wider recovery search. Moreover, the reference could remain
a finished submap until another insertion succeeded. That creates an avoidable
failure mode: the operation needed to advance the reference depends on matching
success against the old reference.

The matching reference now advances to the oldest remaining active submap when
its predecessor finishes. The two-submap insertion overlap and scan counts stay
the same. The reference handover no longer waits for another accepted insertion.
A regression case now checks that a rejected scan cannot pin it to the retired map.

A match is considered weak for recovery purposes if it fails the normal gate or
has overlap below `recovery_min_overlap` (0.55). The normal insertion threshold
remains 0.35. On the first weak scan, the new recovery path:

1. Searches at most 1 m and 0.35 rad around the odometry prediction, using at most
   11 positions per coordinate/angle, 90 coarse endpoints and six refined seeds.
2. Uses weaker odometry regularization for this bounded search, requires at least
   55% overlap, and checks separated competing solutions. It rejects candidates
   with similar scan support in sufficiently separated modes.
3. Can also evaluate one other mature active submap in its native coordinates.
   It never rasterizes the active maps into a combined tracking grid.
4. Requires a likelihood improvement over the ordinary result, then holds the
   proposal without inserting the scan. A subsequent scan must corroborate its
   odometry-propagated pose before insertion resumes. With defaults this requires
   two consecutive scans, including the proposal scan. Repeated evaluation of the
   same scan sequence does not satisfy confirmation.
5. Attempts another wide search at most once every three processed scans per
   hypothesis. Between attempts, ordinary tracking still runs.

Weak ordinary matches retain the previous insertion policy when recovery finds
no sufficiently supported alternative. Thus this is not a blanket rule discarding
all scans below 55% overlap. While a recovery proposal awaits confirmation,
insertion is suppressed. Unsupported or ambiguous recovery does not create a new
map to conceal tracking failure. Recovery beyond the bounded window or in repeated
geometry can still fail.

The recovery step changes the frontend pose used for insertion. It does not
teleport the particles or reset their weights. The existing stochastic motion
proposals and importance update remain in place. Large persistent separation
between a hypothesis's particles and its frontend therefore remains a measurable
risk; the new `pf_frontend_distance_m` diagnostic exposes it.

Pending recovery poses and spatial-cluster confirmations are transported with
PGO's coordinate correction. New spatial children discard their parent's pending
recovery state and seed their own frontend as before.

### Growing graphs were repeatedly solved without useful work

When a graph contains only the locally constructed constraints, all those
measurements come from the same local poses and already admit a zero-residual
solution. The backend nevertheless rebuilt and solved that graph periodically.
It now skips those baseline solves while there are no inter-submap loop edges.

Geometric retrieval now happens before deciding whether to force a baseline solve
for a loop event. If it produces candidates, graphs requiring optimization are
solved before the hypothesis priors and weights are frozen for verification.
Graphs with existing loop edges still receive periodic baseline PGO. An unchanged,
already usable graph does not need another solve merely because a candidate was
retrieved. Periodic retries after a failed baseline solve are scheduled by the
last attempted node count, avoiding an immediate retry on every following scan.
A new candidate can still trigger the required baseline attempt.

The geometric measurements remain fixed in native reference coordinates. Moving
retrieval before baseline PGO can change candidate ranking/search initialization
relative to the previous ordering; evaluate recall as well as speed.

### Derived map fields accumulated in memory

Every finished submap eagerly built a float distance field. Queried submaps also
kept a double likelihood table indefinitely. A retired tracking reference could
retain another full float field. At fine grid resolution, these additional arrays
become expensive as history grows.

Frozen distance/likelihood arrays are now built lazily and kept in a shared cache
with a default retained budget of **64 MiB across retained hypotheses**. Pose-only
clones share the same cache holder. Least recently used derived arrays are evicted
after backend work and reconstructed from the unchanged occupancy grid when
needed. Search workers retain shared ownership of their payload, so eviction
cannot invalidate a reader. Production searches acquire once outside their point
loops. The distance algorithm, thresholds and double likelihood values are unchanged.
Unused historical tracking fields are also released.

The budget applies to retained derived loop fields, not all SLAM memory. Active
readers/builders can temporarily exceed it. Native map grids, graph state,
trajectory samples and publication grids still consume memory and grow with the
explored environment. The full graph is retained for belief verification; this is
not a bounded-memory SLAM guarantee or an asynchronous backend.

## Research claim

Loop closure and PGO remain enabled by default. Each fixed candidate still receives
one trial under every retained trajectory hypothesis, and the verifier still uses
`sum_h w_h * compatibility_h` with the pre-event weights. The compatibility
formula, threshold, particle budgets, loop/no-loop branch mass calculation and
spatial branching objective are unchanged in this revision. Unsupported hypotheses
still contribute zero without losing their denominator weight.

The proposed paper remains **Beyond the MAP Trajectory: Belief-Weighted Loop Closure
Verification for Multi-Hypothesis Particle-Filter SLAM**. These changes improve
engineering failure handling and resource use; they are not evidence of a new
loop-verification contribution, better accuracy, or likely conference acceptance.
The proposition and its scope remain in `paper/BELIEF_PROPOSITION.md`. It justifies
why non-MAP compatibility can matter, not that more hypotheses always win.

## New controls

| Parameter | Default | Meaning |
|---|---:|---|
| tracking_recovery | true | Enable bounded recovery and later-scan confirmation |
| recovery_translation_window | 1.0 | Maximum recovery displacement from prediction, m; allowed up to 3 |
| recovery_rotation_window | 0.35 | Maximum recovery rotation, rad; allowed up to 1 |
| recovery_min_overlap | 0.55 | Recovery overlap requirement and weak-match trigger |
| recovery_ambiguity_margin | 0.05 | Minimum mean-log-likelihood separation for competing recovery modes |
| recovery_after_failures | 1 | Consecutive weak scans before wide-search eligibility |
| recovery_interval | 3 | Minimum processed-scan interval between wide searches per hypothesis |
| recovery_confirmations | 2 | Consecutive supporting scans, including the proposal scan; minimum 2 |
| loop_cache_budget_mb | 64 | Retained derived loop-field budget in MiB; zero discards after use |

All are forwarded by both launches. Ordinary tracking settings and Intel timing
and beam conventions are retained from the preceding revision.

## Diagnose the next replay

The tracking CSV adds status (`bootstrap`, `tracked`, `weak`, `rejected`,
`recovery_pending`, `recovered`), consecutive weak scans, reference submap ID,
frontend correction magnitude, PF/frontend separation, node count and submap count.
Its weights and poses are recorded before the backend event.

The performance CSV adds the selected hypothesis ID, selection changes,
post-backend tracking status, weak-scan count, retained loop-cache bytes, skipped
local-only baseline solves, and the output pose's translation/rotation innovation
relative to the preceding output propagated by odometry. A large innovation can
be a legitimate loop correction or recovery; it is not itself a failure label.

The summary tools now compare early/middle/late callback costs and scan age, report
recovery/weak states and list the ten largest output corrections with their loop
activity and hypothesis switches. Interpret them together:

- Many rejected scans, a long weak streak and no recovery: local tracking support
  is failing; inspect calibration, coverage and motion increments.
- Large output correction coinciding with a loop event: inspect that candidate's
  per-hypothesis trial scores and resulting geometry.
- A correction accompanied by a hypothesis ID change: inspect which mode became
  selected and whether its map is actually better supported.
- Rising late-run latency/scan age or missing source stamps: compute/transport
  backlog is plausible. Compare a slower replay at the same input timestamps.
- Large sustained PF/frontend separation: particle support and the insertion
  trajectory disagree; do not interpret frontend recovery as calibrated PF recovery.

Run the same dataset/seed at 1x and 0.5x, and compare loops enabled versus disabled.
To isolate this revision's recovery, use `tracking_recovery:=false`. To assess the
paper claim, keep this frontend, reader and resource policy fixed across `belief`,
`map`, `uniform`, and single-hypothesis comparisons. Report loop labels, trajectory
error, coverage, runtime and memory, not just a visually cleaner map.
