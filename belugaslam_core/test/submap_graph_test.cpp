#include <gtest/gtest.h>

#include "belugaslam_core/submap.hpp"

namespace {

Sophus::SE2d IdentityPose() {
  return Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d::Zero()};
}

TEST(SubmapTest, FrozenDistanceFieldAndIdSurviveClone) {
  auto submap = std::make_shared<Submap>(7, IdentityPose(), 20, 20, 0.1);
  submap->mutable_grid().at(10, 10) = 5.0F;
  submap->finish();

  EXPECT_EQ(submap->id(), 7U);
  EXPECT_NEAR(submap->distance_at(0.05, 0.05), 0.0, 1.0e-6);
  EXPECT_NEAR(submap->distance_at(0.25, 0.05), 0.2, 0.03);

  const auto clone = submap->clone();
  EXPECT_EQ(clone->id(), submap->id());
  EXPECT_NEAR(clone->distance_at(0.25, 0.05), 0.2, 0.03);
}

TEST(SubmapGraphTest, CountsInterConstraintsAndTrimsInactiveScanData) {
  SubmapList graph;
  auto active = std::make_shared<Submap>(1, IdentityPose(), 20, 20, 0.1);
  graph.active_submaps.push_back(active);

  auto active_data = std::make_shared<ScanNodeData>();
  active_data->returns.push_back({1.0, 0.0});
  auto old_data = std::make_shared<ScanNodeData>();
  old_data->returns.push_back({2.0, 0.0});
  graph.trajectory_nodes.push_back({3, active_data, IdentityPose()});
  graph.trajectory_nodes.push_back({4, old_data, IdentityPose()});

  graph.node_submap_constraints.push_back({
      1, 3, IdentityPose(), 1.0, 1.0, ConstraintTag::kIntraSubmap, 1.0, 1.0});
  graph.node_submap_constraints.push_back({
      0, 4, IdentityPose(), 1.0, 1.0, ConstraintTag::kInterSubmap, 0.8, 0.7});

  EXPECT_EQ(graph.inter_constraint_count(), 1U);
  graph.trim_scan_data_outside_active_submaps();
  EXPECT_NE(graph.trajectory_nodes[0].constant_data, nullptr);
  EXPECT_EQ(graph.trajectory_nodes[1].constant_data, nullptr);
}

}  // namespace
