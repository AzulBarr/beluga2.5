#ifndef BELUGASLAM_CORE_CERES_PROBABILITY_TRACKING_HPP
#define BELUGASLAM_CORE_CERES_PROBABILITY_TRACKING_HPP

#include "probability_matching.hpp"
#include <ceres/ceres.h>

namespace belugaslam {

class ProbabilityTrackingCost final : public ceres::CostFunction {
 public:
  ProbabilityTrackingCost(const ProbabilityField& field, const ScanPoints& scan,
      const PoseSample2& prior, const TrackingOptions& tracking, const ProbabilityMatchingOptions& options)
      : field_(field), scan_(scan), prior_(prior), tracking_(tracking), options_(options) {
    set_num_residuals(static_cast<int>(scan.size()+3));
    mutable_parameter_block_sizes()->push_back(3);
  }
  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    return evaluate_probability_match(field_, scan_, prior_, tracking_, options_, parameters[0],
                                      residuals, jacobians ? jacobians[0] : nullptr);
  }
 private:
  const ProbabilityField& field_;
  const ScanPoints& scan_;
  PoseSample2 prior_;
  TrackingOptions tracking_;
  ProbabilityMatchingOptions options_;
};

// The distance matcher supplies a basin of attraction; Ceres then minimizes the
// occupancy objective with the original odometry prior. Recovery's broad search
// and loop-closure verification remain separate from this local refinement.
inline TrackingResult match_probability_scan(const ProbabilityField& probability,
    const TrackingField& distance, const ScanPoints& scan, const PoseSample2& prior,
    const TrackingOptions& tracking, const ProbabilityMatchingOptions& options,
    const PoseSample2* seed = nullptr) {
  options.validate();
  TrackingResult result;
  result.pose = prior;
  if (scan.empty() || distance.occupied_cells() == 0) return result;
  result.initial_cost = probability_tracking_objective(probability,scan,prior,prior,tracking,options);
  auto pose = probability_tracking_initial_pose(probability,distance,scan,prior,tracking,options,seed);
  double cost = probability_tracking_objective(probability,scan,pose,prior,tracking,options);
  const auto within_bounds = [&](const PoseSample2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw) &&
        std::hypot(p.x-prior.x,p.y-prior.y) <= tracking.max_translation &&
        std::abs(wrap_angle(p.yaw-prior.yaw)) <= tracking.max_rotation;
  };
  const auto consider = [&](const PoseSample2& p) {
    if (!within_bounds(p)) return;
    const auto score = tracking_score(distance,scan,p,tracking);
    if (score.inliers < tracking.min_points || score.overlap < tracking.min_overlap) return;
    const double candidate = probability_tracking_objective(probability,scan,p,prior,tracking,options);
    if (std::isfinite(candidate) && candidate < cost) { pose = p; cost = candidate; }
  };
  double delta[] = {pose.x-prior.x, pose.y-prior.y, wrap_angle(pose.yaw-prior.yaw)};
  ceres::Problem problem;
  problem.AddResidualBlock(new ProbabilityTrackingCost(probability,scan,prior,tracking,options), nullptr, delta);
  for (int a = 0; a < 3; ++a) {
    const double bound = a == 2 ? tracking.max_rotation : tracking.max_translation;
    problem.SetParameterLowerBound(delta,a,-bound);
    problem.SetParameterUpperBound(delta,a,bound);
  }
  ceres::Solver::Options solver;
  solver.linear_solver_type = ceres::DENSE_QR;
  solver.max_num_iterations = tracking.max_iterations;
  solver.num_threads = 1;
  solver.use_nonmonotonic_steps = false;
  solver.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(solver,&problem,&summary);
  if (summary.IsSolutionUsable())
    consider({prior.x+delta[0], prior.y+delta[1], wrap_angle(prior.yaw+delta[2])});
  result.pose = pose;
  result.score = tracking_score(distance,scan,pose,tracking);
  result.final_cost = cost;
  result.accepted = std::isfinite(cost) && result.score.inliers >= tracking.min_points &&
      result.score.overlap >= tracking.min_overlap;
  if (!result.accepted) {
    result.pose = prior;
    result.score = tracking_score(distance,scan,prior,tracking);
    result.final_cost = result.initial_cost;
  }
  return result;
}

}  // namespace belugaslam
#endif
