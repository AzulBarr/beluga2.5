#!/usr/bin/env python3
"""Optional isolated Ceres validation using the compiled production residual kernel.

Requires numpy and pyceres (optional validation dependencies, not SLAM runtime
dependencies). Build tools/pose_graph_kernel_c_api.cpp as a shared library and
pass its absolute filename. This does not exercise ROS or the Beluga graph.
"""
import ctypes
import json
from pathlib import Path
import sys

import numpy as np
import pyceres


class Edge(pyceres.CostFunction):
    def __init__(self, kernel, measurement):
        super().__init__()
        self.set_num_residuals(3)
        self.set_parameter_block_sizes([3, 3])
        self.kernel = kernel
        self.measurement = np.array(measurement, dtype=np.float64)

    def Evaluate(self, parameters, residuals, jacobians):
        pointer = lambda a: a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        a, b = (np.asarray(p, dtype=np.float64) for p in parameters)
        r = np.empty(3)
        ja, jb = np.empty(9), np.empty(9)
        ok = self.kernel(pointer(self.measurement), pointer(a), pointer(b),
                         pointer(r), pointer(ja), pointer(jb))
        residuals[:] = r
        if jacobians is not None:
            for target, value in zip(jacobians, (ja, jb)):
                if target is not None:
                    target[:] = value
        return bool(ok)


def solve(kernel, prior_weight, target, robust, initial=0.0):
    anchor, query = np.zeros(3), np.array([initial, 0., 0.])
    problem = pyceres.Problem()
    prior = Edge(kernel, [0, 0, 0, prior_weight, 8, 0, 0, 0])
    loop = Edge(kernel, [target, 0, 0, 10, 12, 0, 0, 0])
    loss = pyceres.HuberLoss(1.0) if robust else None
    problem.add_residual_block(prior, None, [anchor, query])
    problem.add_residual_block(loop, loss, [anchor, query])
    problem.set_parameter_block_constant(anchor)
    options = pyceres.SolverOptions()
    options.linear_solver_type = pyceres.LinearSolverType.SPARSE_NORMAL_CHOLESKY
    options.max_num_iterations = 50
    options.num_threads = 1
    # Use stricter tolerances to compare against closed-form fixture solutions.
    options.function_tolerance = 1e-12
    options.gradient_tolerance = 1e-12
    options.parameter_tolerance = 1e-12
    summary = pyceres.SolverSummary()
    pyceres.solve(options, problem, summary)
    assert summary.termination_type == pyceres.TerminationType.CONVERGENCE, summary.BriefReport()
    assert summary.IsSolutionUsable(), summary.BriefReport()
    return float(query[0])


def main():
    library = ctypes.CDLL(str(Path(sys.argv[1]).resolve()))
    kernel = library.beluga_pose_graph_evaluate
    kernel.argtypes = [ctypes.POINTER(ctypes.c_double)] * 6
    kernel.restype = ctypes.c_int
    results = []
    # Same graph, different loss for the proposed loop. Its forced fit passes in
    # both conflicting fixtures. The settled fit rejects the stronger conflict.
    for name, weight, target, forced_expected, settled_expected in (
        ('settled_fit_passes', np.sqrt(40), .5, 5/14, .25),
        ('settled_fit_rejects', 10., .5, .25, .1),
        ('quadratic_fast_path', 10., .1, .05, .05),
    ):
        forced = solve(kernel, weight, target, False)
        settled = solve(kernel, weight, target, True, forced)
        repeated = solve(kernel, weight, target, True, settled)
        assert abs(forced - forced_expected) < 1e-6, (name, forced)
        assert abs(settled - settled_expected) < 1e-6, (name, settled)
        assert abs(repeated - settled) < 1e-6, (name, repeated, settled)
        forced_fit, settled_fit = abs(target-forced), abs(target-settled)
        assert forced_fit <= .3
        assert (settled_fit <= .3) == (name != 'settled_fit_rejects')
        assert ((10*forced_fit)**2 > 1) == (name != 'quadratic_fast_path')
        results.append(dict(case=name, forced_x=forced, settled_x=settled,
                            forced_fit=forced_fit, settled_fit=settled_fit,
                            repeated_solve_movement=abs(repeated-settled)))
    print(json.dumps({'passed': len(results), 'native_ceres_fixtures': results}, indent=2))


if __name__ == '__main__':
    main()
