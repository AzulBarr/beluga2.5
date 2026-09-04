#!/usr/bin/env python3
"""Dependency-free structural checks for the Cartographer-like graph changes."""

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "belugaslam_core/include/belugaslam_core/fastslam_oc_grid_core.hpp"
SUBMAP = ROOT / "belugaslam_core/include/belugaslam_core/submap.hpp"
NODE = ROOT / "belugaslam_node/src/fastslam_oc_grid_node.cpp"
LAUNCH = ROOT / "belugaslam_node/launch/fastslam_oc_grid.launch.py"


def without_disabled_blocks(text: str) -> str:
    output = []
    disabled_depth = 0
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#if 0"):
            disabled_depth += 1
            continue
        if disabled_depth and stripped.startswith("#if"):
            disabled_depth += 1
            continue
        if disabled_depth and stripped.startswith("#endif"):
            disabled_depth -= 1
            continue
        if not disabled_depth:
            output.append(line)
    return "\n".join(output)


core = without_disabled_blocks(CORE.read_text())
submap = SUBMAP.read_text()
node = NODE.read_text()

for symbol in (
    "TrajectoryNode",
    "NodeSubmapConstraint",
    "ConstraintTag::kIntraSubmap",
    "ConstraintTag::kInterSubmap",
    "match_scan_to_submap",
    "trim_scan_data_outside_active_submaps",
):
    assert symbol in core or symbol in submap, f"missing {symbol}"

for legacy in ("odometry_constraints", "loop_constraints", "align_submaps("):
    assert legacy not in core, f"active legacy graph reference: {legacy}"

for parameter in (
    "submap_num_range_data",
    "keyframe_min_translation",
    "max_points_per_scan_node",
    "loop_max_candidates",
    "max_hypotheses",
):
    assert f'declare_parameter("{parameter}"' in node, f"undeclared ROS parameter: {parameter}"
    assert f'get_parameter("{parameter}")' in node, f"unread ROS parameter: {parameter}"

assert core.count("{") == core.count("}"), "unbalanced braces in active core code"
assert submap.count("{") == submap.count("}"), "unbalanced braces in submap code"
py_compile.compile(str(LAUNCH), cfile="/tmp/beluga_mh_launch.pyc", doraise=True)
print("OK: bipartite submapping structure and ROS parameter wiring")
