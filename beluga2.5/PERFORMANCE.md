# Performance changes and replay validation

**Revision note:** this file documents the earlier revision. See
[QUALITY_REVIEW.md](QUALITY_REVIEW.md) for the current frontend, population policy,
loop corroboration and Intel replay changes. Earlier kernel timings do not
measure the current end-to-end SLAM pipeline.

This revision removes repeated work while retaining the loop candidate budgets,
particle budgets, matching lattice, posterior-weighted verifier, and PGO iteration
limit from the preceding loop/PGO patch. Loop closure and PGO stay enabled.
It is a performance patch to test, not a guarantee of real-time operation.

## What was expensive

The uploaded `build/belugaslam_core/CMakeCache.txt` and
`build/belugaslam_node/CMakeCache.txt` both had an empty `CMAKE_BUILD_TYPE` and
empty `CMAKE_CXX_FLAGS`. Both packages now default to Release when no build type
was selected. An explicitly selected Debug build is respected. Both matter
because the implementation is largely header-only.

Previously, every valid scan also rasterized every submap into the global map,
converted the whole map to occupancy, returned full-grid copies, and published
large visualization messages. Ray insertion allocated a cell vector per beam,
then sorted and deduplicated all overlapping ray traversals. Loop matching
recomputed the same Gaussian exponential on every endpoint/candidate lookup.
Independent particle matches, candidate matches and trial PGO solves were serial.

## Changes

- **Ray insertion:** a reusable generation marker per grid cell replaces repeated
  vector allocation and sorting. All hits precede misses; a cell is updated at
  most once per insertion; hit priority, footprint clearing, growth, clamping,
  ray traversal and endpoint exclusion are preserved. The core reuses the same
  scratch storage across its sequential grid insertions. The public scratch
  hit/miss lists now contain unique cells in visitation order; misses exclude hits.
- **Loop scoring:** compute the previous double-precision exponential once per
  frozen grid cell, on first geometric retrieval. The score cache is initialized
  with `std::call_once` and shared by pose-only clones. It does not quantize scores
  or change the distance field, overlap, search window or matching thresholds.
- **Parallel work:** one bounded TBB arena runs independent particle matches,
  geometric candidate matches and candidate/hypothesis trial solves. Each task
  writes its own output slot. RNG sampling, posterior reductions, candidate IDs,
  candidate ordering and population commits remain serial. Each Ceres solve uses
  one Ceres worker. `worker_threads=1` is the serial comparison mode.
- **PGO scheduling:** retain periodic PGO and baseline solves for every hypothesis
  before possible loop events. Skip extra event-triggered solves before there
  can be an eligible historical submap. Solves are still transactional.
- **Map publication:** the core selects a pose every scan, and rasterizes only
  when a map consumer requests the map. Grid getters return const references.
  A frozen-history raster is reused when its bounds, submap identities and poses
  match; new frozen maps can be appended. Bounds changes, PGO and hypothesis
  switches invalidate the cache as needed. Active grids are drawn afresh.
- **ROS visualization:** a wall timer handles visualization in the node's existing
  mutually exclusive callback group. The default map period is 1 s and timer
  period is 0.2 s. Pose and TF publication still occur for every processed scan.
  Unsubscribed particles/entropy/uncertainty/path publications are avoided;
  persistent markers publish on change. The transient-local map still publishes
  without subscribers. The display path has a bounded 5,000-pose history; core
  trajectories and graphs are not trimmed by that setting.
- **Input and logs:** a configurable DDS queue of 50 replaces the default sensor
  queue depth of 5. Reliability stays best-effort unless explicitly requested.
  The Intel publisher now sends its supporting TF before publishing the scan.
  No newest-scan-only policy or age-based scan skipping was added. Duplicate or
  non-increasing processed timestamps are explicitly rejected and counted;
  restart the node before replaying a dataset from an earlier timestamp.
  Verbose per-scan console output is replaced with five-second summaries.
- **Diagnostics:** optional CSV separates TF/conversion, motion, matching,
  insertion, backend, resampling and pose publication. Backend timings separate
  baseline PGO, geometric retrieval and belief verification. Timer/map costs and
  processed, TF-error, empty-scan and timestamp-rejection counts are recorded.

## Defaults and controls

| Parameter | Default | Meaning |
|---|---:|---|
| `worker_threads` | 2 | Maximum concurrency in the SLAM task arena, including its caller |
| `map_publish_period` | 1.0 | Minimum wall seconds between map publications |
| `visualization_publish_period` | 0.2 | Wall seconds between visualization timer ticks |
| `trajectory_max_poses` | 5000 | Maximum path visualization samples |
| `scan_queue_depth` | 50 | DDS history depth for short processing bursts |
| `scan_reliable` | false | Set true only if the scan publisher offers reliable QoS |
| `verbose_backend` | false | Print individual PGO/loop/split events |
| `performance_diagnostics_path` | empty | Optional per-callback CSV path |
| `uncertainty_map_publish_interval` | 10 | Every N **map publications**, when subscribed; 0 disables |

Publication periods must be at least 0.001 s. Timer scheduling can delay a
publication; a 1 s map period is not a hard real-time deadline. Parameters are
startup settings. The new eight controls are forwarded by both launch files;
`uncertainty_map_publish_interval` was already in the Python launch.

A larger queue absorbs bursts; it cannot repair sustained overload and can
increase data age. A reliable subscriber cannot communicate with a best-effort
publisher. The [ROS 2 sensor QoS profile](https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h) specifies best-effort delivery and a default depth of five.
The task arena bounds this application's parallel tasks; external BLAS libraries
may have their own threads. For controlled experiments, use the same BLAS/OpenMP
thread settings across runs. See [oneTBB task-arena guidance](https://uxlfoundation.github.io/oneTBB/main/tbb_userguide/Guiding_Task_Scheduler_Execution.html)
and [Ceres solver options](https://ceres-solver.org/nnls_solving.html).

## What was actually verified here

The production standard-library ray updater passed **2,400 complete-grid,
bit-for-bit equivalence comparisons** against an independent copy of the prior
algorithm. Cases include overlapping rays, repeated endpoints, all octants,
clipping, zero-length rays, grid size changes, repeated scans and saturated cells.
AddressSanitizer and UndefinedBehaviorSanitizer also passed these cases.
LeakSanitizer could not operate in this environment and was disabled for that run.

A Release microbenchmark of 250 repeated insertions, with 1,080 rays in a
600-by-600-cell synthetic grid, measured:

| Ray update implementation | Wall time per scan |
|---|---:|
| Previous allocating/sorting implementation | 8.24942 ms |
| Generation-marker implementation | 0.601497 ms |

That is **13.7148x for this kernel on this synthetic workload**, excluding scan
coordinate conversion, grid growth, matching, freezing, PGO and ROS. It is not an
end-to-end speedup, and hardware and datasets will change the result.
The earlier 12 motion-filter and 47 belief/alignment/quota checks also passed.

Five new ROS/Ceres-dependent gtests cover serial/parallel particle agreement,
cached score equality, lazy map publication, history cache invalidation, and
serial/parallel trial agreement with prior immutability. These, and the earlier
54 dependency-based gtests, **have not run here**. This environment lacks ROS,
TBB, Ceres, Eigen, Sophus and gtest. Full translation-unit checking stopped at the
missing `tbb/parallel_for.h`; full integration compilation remains unverified.
Python/XML syntax and launch forwarding checks, the timing summarizer's synthetic
fixtures, and whitespace checks passed.

To reproduce the kernel tests without ROS:

```bash
g++ -std=c++17 -O3 -Wall -Wextra -Werror -pedantic -I belugaslam_core/include belugaslam_core/test/grid_update_test.cpp -o /tmp/grid_update_test
/tmp/grid_update_test --benchmark
```

## Replay and interpret results

Build Release and run all core tests before replaying (commands in the ZIP README).
For the included historical Intel publisher, use `use_sim_time:=false` unless you
also provide a matching `/clock`; that publisher does not publish `/clock`.
Its historic scan timestamps are still used for TF and the core motion filter.

```bash
ros2 launch belugaslam_example intel_dataset_belugaslam.xml use_sim_time:=false record_bag:=false worker_threads:=2 map_publish_period:=1.0 performance_diagnostics_path:=/tmp/beluga_perf.csv loop_diagnostics_path:=/tmp/beluga_loops.csv
```

Stop cleanly to flush CSV buffers. After replay, from the repository root:

```bash
python3 tools/summarize_performance.py /tmp/beluga_perf.csv --budget-ms 100
```

Use your actual sensor period for the budget: 100 ms at 10 Hz, 50 ms at 20 Hz,
25 ms at 40 Hz. The tool reports p50/p95/p99/max and loop-event timings. A scan
period budget is only a diagnostic comparison, not a scheduler guarantee.
Compare `worker_threads=1` and `2` with the same seed, input, hypothesis budget,
loop parameters, optimized build and visualization subscribers. On a larger CPU,
try `4` only after measuring `2`. Profile high percentile latency and peak memory,
not just average throughput. Existing verification CSVs allow comparing candidate
identities and compatibility values between serial and parallel runs.

`processed / received` counts callbacks handled by this node; it cannot count
scans discarded upstream or by DDS before a callback. To check exact source
coverage, provide a text file containing one unique source scan timestamp per
line, in integer nanoseconds:

```bash
python3 tools/summarize_performance.py /tmp/beluga_perf.csv --budget-ms 100 --expected-stamps expected_scan_stamps.txt
```

The tool separates not-received source scans from received-but-rejected scans.
Timestamp gaps alone are not labeled as dropped frames. `age_ms` is meaningful
only when the scan stamp and ROS clock share the same time base; the included
historical Intel replay with system time does not satisfy that condition.
Callback timing excludes CSV/log writing and visualization timer work. Separate
map/timer costs must also fit the CPU budget, and only timer samples observed by
a later scan callback appear in the CSV. Map cost is included in timer cost.

## Remaining performance limits

Loop verification still waits for all requested hypothesis trials before it
commits the posterior. It has bounded candidate and iteration counts, but no
hard wall-time bound. Moving it fully into the background requires a correct
protocol for associating delayed results with the evolving hypothesis posterior;
that is a separate algorithmic change, not just adding a detached thread.

The full graph and trajectory samples still grow. Per-solve graph construction,
sparse factorization, submap finishing, and replay TF availability can dominate
long runs. The new score cache adds 8 bytes per cell only for retrieved frozen
submaps; the history raster adds one float per publication cell. These trade
memory for repeated work. Independent trial graphs also consume memory.
This revision does not establish zero lost scans or the fastest possible
configuration on your robot. The timing file identifies the next bottleneck.
