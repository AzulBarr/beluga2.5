# Recover the tested baseline and measure trajectory accuracy

The refinement revision improved some visible regions but substantially damaged
others in the user's replay. Its synthetic loop-search tests did not establish
end-to-end accuracy. The user has clarified that low pose RMSE and good mapping
are both required; map appearance alone is not an acceptance criterion.

## Runtime decision

The SLAM core, submapping, tracking/recovery, particle proposal, hypothesis
selection, loop search/scoring, loop event handling and PGO are restored to the
last user-tested `beluga_late_run_fix.zip` version. The changed refinement search,
extra alignments per reference and query ledger are removed together. Changing
`loop_search_modes` to 1 would not restore the previous scoring/search behavior.
The parameter and its experimental tests are removed as part of the rollback.

This preserves loop closure, PGO, loop/no-loop hypotheses and persistent spatial
branching. Every fixed candidate still undergoes verification under every live
hypothesis, followed by weighted compatibility aggregation. Restoring one returned
geometric match per reference does not reduce the system to one trajectory.
The central paper claim is unchanged.

There are three plausible regression paths in the reverted changes: different
geometric scores under unchanged gates, changed competition for a bounded branch
budget, and changed eligibility of later candidates from a consumed query.
Additional search cost could also change frame delivery. The supplied screenshot
cannot identify which occurred. No particular false loop is asserted as the cause.

The only new online code is diagnostic output: three optional performance-CSV
columns, `output_x`, `output_y`, and `output_yaw`, record the exact pose used for
`/best_pose` on processed callbacks. Rejected callbacks leave these columns empty.
They are written after the timed update. Logging adds some I/O cost; it does not
alter numerical inference, RNG draws, gates or pose selection.

This is a recovery to the strongest observed baseline, not a demonstrated RMSE
improvement over that baseline, graph SLAM, or standard RBPF. Full replay and
ROS compilation are not available in this environment.

## Evaluation defects corrected

The old `compute_rmse.py` declared a ground-truth topic but only extracted
`/best_pose`; without `--gt-file` it never populated the reference trajectory.
It also forced MCAP storage even though supplied bag metadata includes sqlite3,
used a fixed 0.5-second association tolerance, and did not check the evaluator's
exit status before parsing its output.

The extractor now writes both topics using acquisition timestamps, supports
metadata-selected storage, rejects nonfinite and duplicate/out-of-order records,
preserves integer nanoseconds in TUM output, and requires successful `evo_ape`
execution. The configurable tolerance defaults to 0.05 seconds. `--t-offset`
adds a documented, known offset to the estimate; it is never optimized to lower
RMSE. External-reference input remains supported. The ROS bag-reader wrapper
requires testing on the user's ROS installation; decoded-record extraction and
command/error handling were tested without ROS.

The new `tools/evaluate_trajectory.py` works without ROS, NumPy or evo. It reads
TUM trajectories or the new performance CSV, performs deterministic one-to-one
timestamp association, and compares methods on common reference indices. It
reports planar position/yaw APE and fixed-time RPE, timestamp mismatches, coverage,
worst samples, and early/middle/late errors. Default alignment is one SE(2)
rotation/translation over the common trajectory, without scale fitting.
`--alignment origin` and `none` are explicit alternatives. Segment reports use
the same global alignment; each segment is not separately aligned to hide drift.

This custom planar evaluator and the legacy evo command have different geometry:
`evo_ape -a` performs SE(3) alignment and reports 3D translation error. Do not mix
those figures as if they were the same metric. Use one protocol for every method.
Definitions and alignment conventions are described in the primary
[evo metrics documentation](https://github.com/MichaelGrupp/evo/wiki/Metrics),
and timestamp controls in the
[evo trajectory documentation](https://github.com/MichaelGrupp/evo/wiki/evo_traj).

For a scientific comparison, estimates must use the same robot body frame,
sensor extrinsics, dataset, interval and online/final-optimized trajectory type.
The exported `/best_pose` trajectory is the online output; it is not the final
optimized graph retrospectively applied to every past pose. Match it against
online graph-SLAM/RBPF outputs. Report coverage with RMSE because missing difficult
segments can improve a matched-only score. A common timestamp set alone does not
prove that supplied files share a valid frame, clock or independent reference.

## Supplied Intel reference audit

`belugaslam_example/bags/intel/corrected_gt.txt` contains 9,722 poses with timestamps
from 0.000246 to 2691.3 seconds. The current replay preserves FLASER acquisition
stamps from 976052857.337530 to 976055548.633890 seconds. Their time ranges do not
overlap without an explicit offset.

The first ODOM acquisition stamp in the supplied CARMEN file is
976052857.337284 seconds. Adding this value to the supplied reference timestamps
places all 9,722 within 0.005 seconds of a FLASER stamp. This is evidence for the
relative clock convention of this particular reference file, not evidence that
its poses are accurate independent ground truth.

Reference SHA-256:
`0bff16ffcf0b9302e1b6c86c7353ff2acb6d42531eedb9330d85a44e808797fe`.
Its measurement/optimization provenance is not established by the supplied files.
If it is another SLAM result, describe the metric as error relative to that
reference, not independently measured ground-truth RMSE. None of these findings
invalidate the user's observed degradation or establish better PF accuracy.

## Inspecting the first failure

`tools/inspect_slam_run.py` combines the existing performance, tracking and loop
CSVs. It finds the first large output innovation, includes nearby timesteps,
shows weak/rejected intervals and retains loop-query context. It validates
processed counters before joining core sequence numbers. Tracking rows precede
backend branching, so a newly selected child may not appear in those rows.
Loop query time need not equal verification execution time; the tool explicitly
avoids making that assumption. Large corrections can be valid and these flags
are evidence locations, not automatic declarations of a bad loop.

## Next acceptance gate

Run the restored PF and the baselines on the same recording and clock. Keep the
estimated trajectories, reference file with its provenance, three PF diagnostic
CSVs, and raw map PGM/YAML. Compare position/yaw error, RPE, coverage and late-run
behavior before changing matcher or loop thresholds. Inspect the first divergence
against loop events and tracking quality. Keep the existing weighted verifier
and bounded particle/hypothesis budgets fixed while testing one proposed change.

Good mapping must also be checked from maps with their resolution and origin;
annotated screenshots do not provide a ground-truth map or metric coordinates.
Lower pose RMSE and a cleaner map are related goals, but neither proves the other.
The paper's defensible experiment asks when the weighted belief improves loop
verification and SLAM under controlled budgets; it cannot assume that retaining
more hypotheses always improves RMSE.
