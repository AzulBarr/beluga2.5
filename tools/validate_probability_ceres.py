#!/usr/bin/env python3
"""Native Ceres solves with the compiled production occupancy residual kernel.

Optional: numpy + pyceres. This validates the residuals in real Ceres, but does
not compile the C++ adapter, run ROS, or establish dataset RMSE improvement.
"""
import ctypes as ct
import json
from pathlib import Path
import sys
import numpy as np
import pyceres


def pointer(a):
    return a.ctypes.data_as(ct.POINTER(ct.c_double))


class OccupancyCost(pyceres.CostFunction):
    def __init__(self, lib, handle, n):
        super().__init__()
        self.lib, self.handle, self.n = lib, handle, n
        self.set_num_residuals(n+3)
        self.set_parameter_block_sizes([3])

    def Evaluate(self, parameters, residuals, jacobians):
        delta = np.asarray(parameters[0], dtype=np.float64)
        r, j = np.empty(self.n+3), np.empty((self.n+3)*3)
        ok = self.lib.beluga_probability_evaluate(self.handle, pointer(delta), pointer(r), pointer(j))
        residuals[:] = r
        if jacobians is not None and jacobians[0] is not None:
            jacobians[0][:] = j
        return bool(ok)


def run(lib, truth, prior, repeats=1, outliers=0, corridor=False):
    w, res, origin = 160, .05, -4.
    cells = np.zeros((w,w), dtype=np.float32)
    world = []
    for i in range(30,130):
        cells[i,125] = 5
        world.append([origin+125.5*res, origin+(i+.5)*res])
        if corridor:
            cells[i,35] = 5
            world.append([origin+35.5*res, origin+(i+.5)*res])
        else:
            cells[130,i] = 5
            world.append([origin+(i+.5)*res, origin+130.5*res])
    if corridor:
        # Wall ends stay outside the scan: the longitudinal axis is unobservable.
        cells[:,125] = 5
        cells[:,35] = 5
    c, s = np.cos(truth[2]), np.sin(truth[2])
    scan = (np.array(world)-truth[:2]) @ np.array([[c,-s],[s,c]])
    if outliers:
        scan = np.vstack((scan, np.random.default_rng(42).uniform(10,20,(outliers,2))))
    scan = np.ascontiguousarray(np.tile(scan,(repeats,1)))
    prior = np.array(prior, dtype=np.float64)
    handle = lib.beluga_probability_create(cells.ctypes.data_as(ct.POINTER(ct.c_float)),
                                          w,w,res,origin,origin,pointer(scan),len(scan),pointer(prior))
    assert handle
    try:
        warm = np.empty(3)
        assert lib.beluga_probability_warm_start(handle,pointer(warm))
        delta = warm-prior
        delta[2] = np.arctan2(np.sin(delta[2]),np.cos(delta[2]))
        cost = OccupancyCost(lib,handle,len(scan))
        problem = pyceres.Problem()
        problem.add_residual_block(cost,None,[delta])
        for i, bound in enumerate((.5,.5,.25)):
            problem.set_parameter_lower_bound(delta,i,-bound)
            problem.set_parameter_upper_bound(delta,i,bound)
        options = pyceres.SolverOptions()
        options.linear_solver_type = pyceres.LinearSolverType.DENSE_QR
        options.max_num_iterations = 20
        options.num_threads = 1
        options.use_nonmonotonic_steps = False
        summary = pyceres.SolverSummary()
        pyceres.solve(options,problem,summary)
        assert summary.IsSolutionUsable(), summary.BriefReport()
        assert summary.final_cost <= summary.initial_cost + 1e-10
        pose = prior+delta
        error = pose-truth
        error[2] = np.arctan2(np.sin(error[2]),np.cos(error[2]))
        if corridor:
            assert abs(pose[1]-prior[1]) < 1e-3, pose
        else:
            assert np.linalg.norm(error[:2]) < .03, (pose, error)
        assert abs(error[2]) < .008, (pose, error)
        return {'pose': pose.tolist(), 'translation_error_m': float(np.linalg.norm(error[:2])),
                'yaw_error_rad': float(abs(error[2])), 'initial_cost': summary.initial_cost,
                'final_cost': summary.final_cost, 'beams': len(scan)}
    finally:
        lib.beluga_probability_destroy(handle)


def main():
    lib = ct.CDLL(str(Path(sys.argv[1]).resolve()))
    dp = ct.POINTER(ct.c_double)
    lib.beluga_probability_create.argtypes = [ct.POINTER(ct.c_float),ct.c_int,ct.c_int,
                                              ct.c_double,ct.c_double,ct.c_double,dp,ct.c_int,dp]
    lib.beluga_probability_create.restype = ct.c_void_p
    lib.beluga_probability_evaluate.argtypes = [ct.c_void_p,dp,dp,dp]
    lib.beluga_probability_evaluate.restype = ct.c_int
    lib.beluga_probability_warm_start.argtypes = [ct.c_void_p,dp]
    lib.beluga_probability_warm_start.restype = ct.c_int
    lib.beluga_probability_destroy.argtypes = [ct.c_void_p]
    truth = np.array([.14,-.09,.023])
    baseline = run(lib,truth,[0,0,0])
    repeat = run(lib,truth,[0,0,0],repeats=4)
    assert np.max(np.abs(np.array(repeat['pose'])-baseline['pose'])) < 1e-7
    results = {'observable_walls': baseline, 'duplicated_beams': repeat,
               'outliers': run(lib,truth,[0,0,0],outliers=60),
               'yaw_branch': run(lib,np.array([.14,-.09,-3.12]),[0,0,3.13]),
               'corridor': run(lib,np.array([.04,0,0]),[0,.1,0],corridor=True)}
    print(json.dumps({'passed': len(results), 'native_ceres_fixtures': results}, indent=2))


if __name__ == '__main__':
    main()
