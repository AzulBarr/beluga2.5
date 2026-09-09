# Pose accuracy review — September 9, 2026

This revision fixes identifiable implementation problems in the supplied
`beluga2.5(2).zip` (HEAD `51e039e`). It is a candidate for full replay on the user's
ROS2 machine. It does **not** establish an end-to-end RMSE improvement or superiority
over ordinary RBPF, Cartographer, ROVER, or any other SLAM system.

## Is the pose estimator actually using the particle filter?

Only partly. The shipped architecture has N instantaneous pose particles assigned
to H map/graph hypotheses. Particles within a hypothesis share its map and historical
trajectory. A separate local scan matcher estimates the pose used for insertion,
graph nodes and output. Particle likelihoods update hypothesis masses and spatial
splitting, but N=300,H=4 does not mean 300 independently represented trajectories.
Loop verification performs trials on the H graphs, not N particle trajectories.

The default frontend is a valid *hybrid engineering design*, but it is not a full
trajectory-conditioned RBPF posterior estimator. Its graph deformation compatibility,
fixed loop/null factors and bounded pruning are also approximations, not calibrated
Bayesian model evidence. More particles cannot fix a biased shared map or restore
past alternative trajectories that were never stored.

Changing which particle supplies the output is not intrinsically incorrect. A true
RBPF can choose a different complete trajectory/map pair. Mixing instantaneous poses
from different histories into one persistent map is the inconsistent operation.

For this bounded revision, `proposal_seed` lets a concentrated per-hypothesis
particle proposal cloud provide an additional scan-matcher seed. The existing
solution and original odometry prior remain candidates; a replacement must pass
the tracking gates and lower the same regularized objective. We do not rescore
particle weights a second time or force every particle to the optimum. This avoids
blindly substituting a Monte Carlo mean for a better local registration. It remains
a hybrid approximation and does not turn the code into full RBPF SLAM.

A full RBPF conversion would require per-particle trajectory ancestry, conditional
map updates (with copy-on-write submaps), a well-defined proposal/importance ratio,
and loop corrections consistent with each history. That is a separate architectural
change, with substantial memory and runtime implications; it has not been silently
implemented or claimed here.

## Active fixes

1. **Small-motion direction.** Below `motion_distance_threshold`, the motion model
   replaced the initial rotation by zero while retaining a positive translation.
   A small backward or lateral measurement could become forward motion. The new
   decomposition preserves the exact deterministic odometry increment; the threshold
   now affects rotation-noise estimation only. The attached Intel log contains 2,780
   nonzero displacements <= 1 cm, including 986 backward relative to previous yaw.
   This is an input-level finding, not a measured accumulated pose error.

2. **Loop reference frame.** Retrieval measures a scan in the candidate's native
   submap, whose frame was born at an anchor sequence. Verification previously
   redirected this measurement through the submap used to track that anchor scan,
   often its predecessor. PGO can move those frozen submaps differently. Verification
   now uses the actual anchored submap when present, preserving the measured frame.
   For another hypothesis without a submap born at that sequence, the recorded local
   anchor transform remains the transport fallback. Numeric IDs are not copied
   between hypotheses.

3. **Dense corrected trajectory.** Keyframes previously used optimized node poses
   while intermediate scans used a rigid submap transform. A keyframe correction
   could therefore disappear at the next non-keyframe scan. Intermediate poses now
   use an SE(2) interpolation of adjacent node corrections, evaluated at acquisition
   timestamps and applied to the immutable local trajectory. Keyframes are unchanged.
   End samples use the nearest node correction. This improves consistency of the
   retrospective readout; it adds no independent sensor information.

4. **Final optimization.** A finite run now explicitly settles retained loop graphs
   before exporting its retrospective trajectory. Previously a tail shorter than
   the PGO scheduling interval could remain pending. Export metadata records final
   convergence and selected graph identity. Online history is not rewritten.

5. **Repeated evidence and diagnostics.** A loop query can update the branch weights
   only once. Diagnostics now record the verification event, consumed query and
   retained mass after branch pruning. The existing Python audit tests were ahead
   of their implementation: the validator now checks frozen priors, weighted/MAP/
   uniform scores, repeated consumption and retained mass.

6. **Particle-assisted frontend option.** `frontend_pose_mode=proposal_seed` is
   available throughout the core and ROS launch. The new offline runner uses it.
   Direct ROS launch keeps `frontend` as the compatibility default; select the new
   option explicitly. `proposal_mean` remains available for an ablation.

## Regularization experiment: explicitly NOT enabled by default

The original frontend averages beam log likelihoods but adds a full Gaussian-form
odometry penalty. The PF uses 20 effective beams. If the frontend penalty is meant
to be the Gaussian prior for that likelihood, its per-beam information scale should
be 1/20. However, the original expression also serves as useful regularization.

The revised `tracking_prior_information_scale` makes this choice explicit:

- `1.0` (default): preserves the original matcher regularization.
- `0.05`: experimental effective-20-beam interpretation with less odometry bias.

On 40 known-map synthetic registrations, with deterministic small measurement noise,
the second choice reduced XY registration RMSE from 0.034778 m to 0.00231819 m.
The accuracy regression rejects the former's bias in that controlled fixture.

But a real Intel *scan-pair component* experiment found a tradeoff:

| Statistic, 13,630 consecutive pairs | Original scale 1 | Experimental scale .05 |
|---|---:|---:|
| Pairs accepted in both directions | 13,630 | 13,630 |
| Median forward/backward translation discrepancy | .008555 m | .010561 m |
| P95 discrepancy | .029873 m | .042385 m |
| Maximum discrepancy | .425364 m | .458232 m |
| Mean held-out log-score gain | .020866 | .020975 |

The weaker prior slightly improved held-out fit but worsened cycle consistency.
It is therefore an experimental option, not a default accuracy claim. These pair
experiments use fresh single-scan endpoint fields, a 25 m range cutoff and held-out
beams; they have no persistent maps, PF, recovery or PGO. The offline full replay
uses the Intel ROS launch's 30 m cutoff. Neither pair metric is SLAM pose RMSE.

## Evaluation and diagnostics

`tools/intel_accuracy_replay.cpp` calls the production `BelugaSLAM` class in the same
order as the ROS callback: motion proposals, measurement update, insertion, backend,
resampling, output. It uses the Intel launch's motion noise, float32 laser conversion,
30 m cutoff and identity laser extrinsic. It bypasses ROS queues and TF, so it is an
algorithm diagnostic, not a demonstration of real-time ROS behavior. It processes
all 13,631 scans from the attached recording with original acquisition timestamps.

`tools/run_accuracy_replay.py` locates the installed executable, prepares chronological
scan inputs, records executable/source/input hashes, checks exact input/output scan
coverage, and archives trajectories, tracking/loop diagnostics, timing and final map.
A failure retains available diagnostics and exits nonzero. Reference poses are never
passed to the executable; optional evaluation happens after it exits.

The supplied `corrected_gt.txt` contains 9,722 reference poses with logger-relative
timestamps. Every Intel FLASER record has exactly
`acquisition_time - logger_time = 976052857.337284` seconds. `--reference-clock logger`
reads and validates this clock relation from the recording; it never optimizes an
offset using poses. Reference times have rounding, so association allows 10 ms.
The evaluator reports coverage and aligns each trajectory once in SE(2), without
scale fitting. The provenance/independent accuracy of this supplied reference is
not established by its filename. Compare other methods using the same reference,
body frame, timestamp set and online/retrospective trajectory type.

The three-run suite compares revised submaps without LC (H=1,N=300), revised belief
SLAM with N=30, and revised belief SLAM with N=300. The first is **not** the user's
original no-submap Beluga baseline. That implementation/output and other datasets
are not provided as executable benchmarks in this attachment.

The older ROS runner also remains available. Its captured map is the last map
published before shutdown; final trajectory export can include the additional
shutdown PGO. Use the new offline replay's map for a map from the finalized graph.

## Validation performed here and remaining gate

- 61 Python tests pass (reader, evaluation, archive/coverage, loop audit and runners).
- Nine dependency-free C++ component test executables pass, including the new
  40-fixture matcher regression and existing numerical/Jacobian/proposal tests.
- Full Intel scan-pair comparisons executed on original and experimental matcher.
- Structural ROS parameter wiring, Python compilation and patch whitespace checks pass.
- Added C++ integration tests for small reverse/lateral increments, timestamp-based
  dense correction, native loop reference and proposal-seed objective/weight behavior.

The full C++ core/ROS build and those integration tests have **not** run here: ROS2,
Sophus, Ceres, TBB and range-v3 development dependencies are unavailable. An attempt
to obtain system dependencies failed because this runtime cannot perform apt's
required user/group operations. No substitute solver or mock SLAM was used to claim
that the complete implementation passed. Compile, run the integration suite and
replay the actual pipeline on the ROS2 machine before drawing RMSE conclusions.

For the distinction between local tracking and global graph corrections, see
[Cartographer's terminology](https://google-cartographer.readthedocs.io/en/latest/terminology.html)
and [algorithm walkthrough](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html).
This review's implementation findings come from the supplied source itself.
