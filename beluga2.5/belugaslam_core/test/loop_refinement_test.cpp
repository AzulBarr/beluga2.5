#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {

Sophus::SE2d Pose(double x, double y, double angle) {
  return Sophus::SE2d{Sophus::SO2d{angle}, Eigen::Vector2d{x, y}};
}

std::unique_ptr<BelugaSLAM> MakeSlam(bool refine) {
  FastSLAMParams params;
  params.min_particles = 1;
  params.max_particles = 3;
  params.loop_refine = refine;
  const beluga::DifferentialDriveModelParam motion{0.0, 0.0, 0.0, 0.0};
  const beluga::LikelihoodFieldProbModelParam sensor{100.0, 2.0, 0.5, 0.5, 0.2, true};
  return std::make_unique<BelugaSLAM>(
      BelugaSLAM::MotionModel{motion},
      BelugaSLAM::MeasurementModel{sensor, GridTypeOC{}}, params);
}

// An L of walls constrains x, y and yaw simultaneously, so the matcher has a
// single well-defined optimum instead of a valley along a corridor.
std::vector<std::pair<double, double>> WallPoints() {
  std::vector<std::pair<double, double>> points;
  for (double t = 1.0; t <= 7.0; t += 0.05) {
    points.emplace_back(t, 1.0);
    points.emplace_back(1.0, t);
  }
  return points;
}

std::shared_ptr<Submap> MakeReferenceSubmap(double kResolution = 0.05) {
  const int side = static_cast<int>(std::lround(10.0 / kResolution));
  auto submap = std::make_shared<Submap>(0, Pose(0.0, 0.0, 0.0), side, side, kResolution);
  auto& grid = submap->mutable_grid();
  for (auto& value : grid.data()) value = -2.0F;
  for (const auto& [x, y] : WallPoints()) {
    const int ix = static_cast<int>(std::floor((x - grid.origin_x()) / kResolution));
    const int iy = static_cast<int>(std::floor((y - grid.origin_y()) / kResolution));
    if (ix >= 0 && iy >= 0 && ix < grid.width() && iy < grid.height()) grid.at(ix, iy) = 5.0F;
  }
  submap->finish();
  return submap;
}

// Endpoints of the same walls seen from `truth`, in the robot frame.
ScanNodeData ScanFrom(const Sophus::SE2d& truth) {
  ScanNodeData data;
  const auto inverse = truth.inverse();
  for (const auto& [x, y] : WallPoints()) {
    const Eigen::Vector2d local = inverse * Eigen::Vector2d{x, y};
    data.returns.emplace_back(local.x(), local.y());
  }
  return data;
}

}  // namespace

// Measured behaviour, not aspiration: the beam search already samples at 0.015 m,
// finer than the chamfer field's own accuracy, so the refinement recovers only a
// few millimetres and on this fixture leaves half the cases untouched. What it
// must never do is move away from the truth, and that is what is pinned here.
TEST(LoopRefinement, RefinementNeverMovesAwayFromTheTruth) {
  const auto submap = MakeReferenceSubmap();
  const std::vector<Sophus::SE2d> truths{
      Pose(3.2237, 2.8119, 0.0413), Pose(2.7654, 3.3081, -0.0327),
      Pose(3.5119, 2.5442, 0.0912), Pose(2.9008, 2.9931, -0.0044)};
  int improved = 0;
  for (const auto& truth : truths) {
    const auto data = ScanFrom(truth);
    const auto initial = Pose(3.0, 3.0, 0.0);
    const auto lattice = MakeSlam(false)->match_scan_to_submap(data, *submap, initial);
    const auto refined = MakeSlam(true)->match_scan_to_submap(data, *submap, initial);
    ASSERT_TRUE(lattice.valid);
    ASSERT_TRUE(refined.valid);
    const auto lattice_error = (truth.inverse() * lattice.T_submap_node).translation().norm();
    const auto refined_error = (truth.inverse() * refined.T_submap_node).translation().norm();
    EXPECT_LE(refined_error, lattice_error + 1.0e-12);
    if (refined_error < lattice_error - 1.0e-9) ++improved;
  }
  EXPECT_GT(improved, 0) << "the refinement is a no-op on every fixture case";
}

// The residual that survives refinement is the chamfer field's discretization
// bias, so it scales with the cell and not with the search step. This is the
// evidence for keeping loop_refine off and looking at map_resolution instead.
TEST(LoopRefinement, ResidualScalesWithCellSizeNotSearchStep) {
  const auto truth = Pose(3.5119, 2.5442, 0.0912);
  const auto data = ScanFrom(truth);
  const auto initial = Pose(3.0, 3.0, 0.0);
  const auto error_at = [&](double resolution) {
    const auto submap = MakeReferenceSubmap(resolution);
    const auto match = MakeSlam(true)->match_scan_to_submap(data, *submap, initial);
    EXPECT_TRUE(match.valid);
    return (truth.inverse() * match.T_submap_node).translation().norm();
  };
  const auto coarse = error_at(0.05);
  const auto fine = error_at(0.025);
  EXPECT_LT(fine, coarse * 0.75) << "halving the cell must dominate any search-step effect";
}

// The refinement minimizes the tracking field while the loop gates score on the
// loop field. Accepting a move the loop metric dislikes would silently loosen
// loop_min_score and loop_min_overlap, so the guard must never let that through.
TEST(LoopRefinement, NeverWorsensTheLoopMetric) {
  const auto submap = MakeReferenceSubmap();
  const std::vector<Sophus::SE2d> truths{
      Pose(3.2237, 2.8119, 0.0413), Pose(2.7654, 3.3081, -0.0327),
      Pose(3.5119, 2.5442, 0.0912), Pose(2.9008, 2.9931, -0.0044)};
  for (const auto& truth : truths) {
    const auto data = ScanFrom(truth);
    const auto initial = Pose(3.0, 3.0, 0.0);
    const auto lattice = MakeSlam(false)->match_scan_to_submap(data, *submap, initial);
    const auto refined = MakeSlam(true)->match_scan_to_submap(data, *submap, initial);
    EXPECT_GE(refined.score, lattice.score);
    EXPECT_GE(refined.overlap, lattice.overlap);
  }
}

// Disabling the parameter must restore the previous measurement exactly, so the
// ablation compares the refinement alone and nothing else.
TEST(LoopRefinement, DisabledReproducesTheLatticeOptimum) {
  const auto submap = MakeReferenceSubmap();
  const auto data = ScanFrom(Pose(3.2237, 2.8119, 0.0413));
  const auto initial = Pose(3.0, 3.0, 0.0);
  const auto first = MakeSlam(false)->match_scan_to_submap(data, *submap, initial);
  const auto second = MakeSlam(false)->match_scan_to_submap(data, *submap, initial);
  const auto delta = first.T_submap_node.inverse() * second.T_submap_node;
  EXPECT_NEAR(delta.translation().norm(), 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(first.score, second.score);
}
