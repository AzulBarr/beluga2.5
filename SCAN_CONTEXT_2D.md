# 2-D Scan Context++-style loop retrieval

This revision replaces the production loop-candidate ranking based on a 50-bin radial histogram with a planar Scan Context++-style retrieval stage while leaving geometric verification, belief-weighted verification, loop branching and PGO unchanged.

## Descriptor

Each finished submap is converted into a 20 x 60 polar occupancy context centered at the submap origin. A cell stores occupied-cell density rather than 3-D point height. The ring mean is a rotation-invariant coarse key. Full matching circularly shifts sectors and uses cosine similarity. A bounded +/- one-ring tolerance makes the planar descriptor less brittle to modest lateral viewpoint changes.

This is intentionally a 2-D adaptation, not a claim that a planar laser can reproduce the height encoding of the original 3-D Scan Context++ descriptor.

## Retrieval

Default ranking is:

`full_context_distance + 0.35 * ring_key_distance + 0.02 * pose_distance_m`

The descriptor is therefore primary; pose distance is only a weak feasibility prior. The existing `loop_candidate_distance` hard bound is retained because the downstream correlative matcher is translation bounded.

For every retrieved candidate the ordinary graph-pose seed is evaluated. A second seed uses the circular-shift yaw estimated by the descriptor while retaining the graph translation. The better geometrically valid match continues into the unchanged multi-scan corroboration and belief verification.

## Parameters

- `loop_use_scan_context_2d:=true` enables the new retrieval. Set false for the old radial-signature ablation.
- `loop_scan_context_max_distance:=0.70` descriptor rejection threshold.
- `loop_scan_context_ring_key_weight:=0.35` coarse-key contribution to rank.
- `loop_scan_context_pose_weight:=0.02` weak graph-distance contribution.
- `loop_scan_context_lateral_rings:=1` radial tolerance in full matching.
- `loop_scan_context_yaw_seed:=true` enables the second yaw seed.

## Suggested ablation

Run the same datasets/seeds once with `loop_use_scan_context_2d:=false` and once with it true. Compare candidate recall before geometric rejection, false candidate rate, accepted-loop precision, end-to-end pose RMSE and retrieval/verification time. Do not attribute improvement from the posterior-weighted verifier to this retrieval change; they are separate stages.
