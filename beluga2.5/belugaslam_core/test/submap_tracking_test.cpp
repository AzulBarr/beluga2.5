#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <set>

#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {

Sophus::SE2d Pose(double x = 0.0, double y = 0.0, double angle = 0.0) {
  return Sophus::SE2d{Sophus::SO2d{angle}, Eigen::Vector2d{x, y}};
}

std::unique_ptr<BelugaSLAM> MakeSlam(double max_time = 5.0, int scans = 2) {
  FastSLAMParams params;
  params.min_particles = 1;
  params.max_particles = 3;
  params.submap_num_range_data = scans;
  params.keyframe_max_time = max_time;
  const beluga::DifferentialDriveModelParam motion{0.0, 0.0, 0.0, 0.0};
  const beluga::LikelihoodFieldProbModelParam sensor{100.0, 2.0, 0.5, 0.5, 0.2, true};
  return std::make_unique<BelugaSLAM>(
      BelugaSLAM::MotionModel{motion},
      BelugaSLAM::MeasurementModel{sensor, GridTypeOC{}}, params);
}

std::shared_ptr<Hypothesis> HypothesisOf(BelugaSLAM& slam) {
  return std::get<2>(*slam.particles().begin());
}

void ExpectPose(const Sophus::SE2d& actual, const Sophus::SE2d& expected) {
  const auto error = expected.inverse() * actual;
  EXPECT_NEAR(error.translation().norm(), 0.0, 1.0e-9);
  EXPECT_NEAR(error.so2().log(), 0.0, 1.0e-9);
}

void SetCell(LogOddsGrid& grid, double x, double y, float value) {
  const int ix = static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution()));
  const int iy = static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()));
  ASSERT_GE(ix, 0);
  ASSERT_GE(iy, 0);
  ASSERT_LT(ix, grid.width());
  ASSERT_LT(iy, grid.height());
  grid.at(ix, iy) = value;
}

TEST(SubmapTrackingTest, BootstrapKeepsPredictionWithoutAMap) {
  auto slam = MakeSlam();
  const auto prediction = Pose(100.0, -80.0, 0.7);
  double score = -123.0;
  ExpectPose(slam->refine_pose_on_submap(prediction, {{1.0, 0.0}}, nullptr, score), prediction);
  EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(SubmapTrackingTest, StationaryOdometryPreservesParticlePoseAndWeight) {
  auto slam = MakeSlam();
  const auto initial_pose = Pose(2.0, 3.0, 0.7);
  auto particle = slam->particles().begin();
  std::get<0>(*particle) = initial_pose;
  const double initial_weight = static_cast<double>(std::get<1>(*particle));
  const auto odom = Pose(100.0, -80.0, 0.4);
  slam->sample_motion_model(std::make_tuple(odom, odom));
  ExpectPose(std::get<0>(*particle), initial_pose);
  EXPECT_DOUBLE_EQ(static_cast<double>(std::get<1>(*particle)), initial_weight);
}

TEST(SubmapTrackingTest, UsesGrownNativeGridFarOutsideOldWorldWindow) {
  auto slam = MakeSlam();
  auto hypothesis = HypothesisOf(*slam);
  const auto world_submap = Pose(100.0, -80.0, 0.7);
  auto submap = std::make_shared<Submap>(0, world_submap, 4, 4, 0.1);
  auto& grid = submap->mutable_grid();
  ASSERT_TRUE(grid.grow_to_include(0.0, 0.0, 10.05, 0.05));
  SetCell(grid, 10.05, 0.05, 5.0F);
  hypothesis->submaps.active_submaps.push_back(submap);
  hypothesis->local_pose = world_submap;
  hypothesis->has_local_pose = true;
  std::get<0>(*slam->particles().begin()) = world_submap;
  const auto original_grid = grid.data();

  double score = 0.0;
  ExpectPose(slam->refine_pose_on_submap(world_submap, {{10.05, 0.05}}, submap.get(), score), world_submap);
  EXPECT_DOUBLE_EQ(score, 5.0);
  slam->measurement_model_map({{10.05, 0.05}});
  ExpectPose(hypothesis->local_pose, world_submap);
  ExpectPose(std::get<0>(*slam->particles().begin()), world_submap);
  EXPECT_EQ(grid.data(), original_grid);  // matching never inserts its own scan
}

TEST(SubmapTrackingTest, YoungerAndHistoricalGridsDoNotBiasTracking) {
  auto slam = MakeSlam();
  auto hypothesis = HypothesisOf(*slam);
  auto older = std::make_shared<Submap>(0, Pose(), 40, 40, 0.1);
  auto younger = std::make_shared<Submap>(1, Pose(), 40, 40, 0.1);
  SetCell(older->mutable_grid(), 1.05, 0.05, 1.2F);
  // This stronger but displaced return would win in the old max-composite map.
  SetCell(younger->mutable_grid(), 1.15, 0.05, 5.0F);
  hypothesis->submaps.active_submaps = {older, younger};
  auto historical = younger->clone();
  historical->finish();
  hypothesis->submaps.history.push_back(historical);
  hypothesis->local_pose = Pose();
  hypothesis->has_local_pose = true;

  slam->measurement_model_map({{1.05, 0.05}});
  ExpectPose(hypothesis->local_pose, Pose());
  ExpectPose(std::get<0>(*slam->particles().begin()), Pose());
  EXPECT_EQ(hypothesis->submaps.matching_submap().get(), older.get());
}

TEST(SubmapTrackingTest, FilteredScansLeaveGridGraphAndLifecycleUntouched) {
  auto slam = MakeSlam();
  auto hypothesis = HypothesisOf(*slam);
  hypothesis->has_local_pose = true;
  const BelugaSLAM::measurement_type scan{{1.05, 0.05}, {1.05, 0.55}};
  EXPECT_TRUE(slam->update_occupancy_grid(scan, 0.0).empty());
  auto& graph = hypothesis->submaps;
  ASSERT_EQ(graph.trajectory_nodes.size(), 1U);
  const auto original_grid = graph.active_submaps.front()->grid().data();
  for (double timestamp : {1.0, 2.0, 5.0}) {
    EXPECT_TRUE(slam->update_occupancy_grid(scan, timestamp).empty());
    ASSERT_EQ(graph.active_submaps.size(), 1U);
    EXPECT_EQ(graph.active_submaps.front()->num_insertions(), 1);
    EXPECT_EQ(graph.active_submaps.front()->grid().data(), original_grid);
    EXPECT_EQ(graph.trajectory_nodes.size(), 1U);
    EXPECT_EQ(graph.node_submap_constraints.size(), 1U);
    EXPECT_EQ(graph.local_trajectory_constraints.size(), 0U);
    EXPECT_DOUBLE_EQ(graph.last_keyframe_time, 0.0);
  }
  slam->update_occupancy_grid(scan, 5.01);
  EXPECT_EQ(graph.active_submaps.front()->num_insertions(), 2);
  EXPECT_EQ(graph.trajectory_nodes.size(), 2U);
  EXPECT_DOUBLE_EQ(graph.last_keyframe_time, 5.01);
  // Rejected and empty data cannot consume an insertion or timestamp.
  slam->update_occupancy_grid({}, 20.0);
  slam->update_occupancy_grid(scan, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(graph.trajectory_nodes.size(), 2U);
  EXPECT_DOUBLE_EQ(graph.last_keyframe_time, 5.01);
}

TEST(SubmapTrackingTest, FilteringUsesLastInsertionAndWrappedAngle) {
  auto slam = MakeSlam();
  auto hypothesis = HypothesisOf(*slam);
  hypothesis->has_local_pose = true;
  const BelugaSLAM::measurement_type scan{{1.05, 0.05}};
  slam->update_occupancy_grid(scan, 0.0);
  hypothesis->local_pose = Pose(0.10);
  slam->update_occupancy_grid(scan, 1.0);
  EXPECT_EQ(hypothesis->submaps.trajectory_nodes.size(), 1U);
  hypothesis->local_pose = Pose(0.20);
  slam->update_occupancy_grid(scan, 2.0);
  EXPECT_EQ(hypothesis->submaps.trajectory_nodes.size(), 2U);

  auto& graph = hypothesis->submaps;
  graph.last_keyframe_pose = Pose(0.0, 0.0, Sophus::Constants<double>::pi() - 0.01);
  EXPECT_FALSE(slam->should_insert_scan(
      graph, Pose(0.0, 0.0, -Sophus::Constants<double>::pi() + 0.01), 2.5));
  EXPECT_TRUE(slam->should_insert_scan(graph, Pose(0.0, 0.0, 2.9), 2.5));
}

TEST(SubmapTrackingTest, OverlapCountsAcceptedScansAndHandoverCannotStallOnRejection) {
  auto slam = MakeSlam(0.0, 2);
  auto hypothesis = HypothesisOf(*slam);
  hypothesis->has_local_pose = true;
  const BelugaSLAM::measurement_type scan{{1.05, 0.05}};
  auto& graph = hypothesis->submaps;
  for (int i = 0; i < 4; ++i) slam->update_occupancy_grid(scan, i);
  ASSERT_EQ(graph.history.size(), 1U);
  ASSERT_EQ(graph.active_submaps.size(), 1U);
  EXPECT_EQ(graph.history.front()->num_insertions(), 4);
  EXPECT_EQ(graph.active_submaps.front()->num_insertions(), 2);
  ASSERT_TRUE(graph.matching_submap());
  EXPECT_EQ(graph.matching_submap()->id(), 1U);
  EXPECT_FALSE(graph.matching_submap()->is_finished());

  slam->update_occupancy_grid(scan, 3.0); // duplicate timestamp: rejected
  EXPECT_EQ(graph.matching_submap()->id(), 1U);
  slam->update_occupancy_grid(scan, 4.0);
  EXPECT_EQ(graph.matching_submap()->id(), 1U);
  ASSERT_EQ(graph.active_submaps.size(), 2U);
  EXPECT_EQ(graph.active_submaps.front()->num_insertions(), 3);
  EXPECT_EQ(graph.active_submaps.back()->num_insertions(), 1);
  EXPECT_EQ(graph.trajectory_nodes.size(), 5U);

  const auto ids0 = graph.insertion_nodes(0);
  const auto ids1 = graph.insertion_nodes(1);
  std::set<ScanNodeId> common;
  for (auto id : ids0) {
    if (std::find(ids1.begin(), ids1.end(), id) != ids1.end()) common.insert(id);
  }
  EXPECT_EQ(common.size(), 2U);
  slam->update_occupancy_grid(scan, 5.0);
  ASSERT_EQ(graph.history.size(), 2U);
  EXPECT_EQ(graph.history.back()->num_insertions(), 4);
}

TEST(SubmapTrackingTest, SingleScanHalfLifeHasAValidMatchingReference) {
  auto slam = MakeSlam(0.0, 1);
  auto hypothesis = HypothesisOf(*slam);
  hypothesis->has_local_pose = true;
  for (int i = 0; i < 5; ++i) {
    slam->update_occupancy_grid({{1.05, 0.05}}, i);
    ASSERT_TRUE(hypothesis->submaps.matching_submap());
    EXPECT_LE(hypothesis->submaps.active_submaps.size(), 2U);
  }
  EXPECT_EQ(hypothesis->submaps.history.size(), 4U);
  for (const auto& submap : hypothesis->submaps.history) {
    EXPECT_EQ(submap->num_insertions(), 2);
  }
}

TEST(SubmapTrackingTest, MatchingReferenceResolvesClonedHypothesisPoseById) {
  auto slam = MakeSlam();
  auto parent = HypothesisOf(*slam);
  parent->has_local_pose = true;
  slam->update_occupancy_grid({{1.05, 0.05}}, 7.0);
  Hypothesis child = *parent;
  child.submaps.make_active_unique();
  child.submaps.active_submaps.front()->set_global_pose(Pose(100.0, 200.0, 0.7));
  ExpectPose(child.submaps.matching_submap()->global_pose(), Pose(100.0, 200.0, 0.7));
  ExpectPose(parent->submaps.matching_submap()->global_pose(), Pose());
  EXPECT_DOUBLE_EQ(child.submaps.last_keyframe_time, 7.0);
  SetCell(child.submaps.active_submaps.front()->mutable_grid(), 1.05, 0.05, 5.0F);
  EXPECT_NE(child.submaps.matching_submap()->grid().data(), parent->submaps.matching_submap()->grid().data());
}

}  // namespace
