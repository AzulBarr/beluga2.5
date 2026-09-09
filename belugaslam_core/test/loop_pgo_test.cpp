#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {
Sophus::SE2d Pose(double x = 0.0, double y = 0.0, double angle = 0.0) {
  return {Sophus::SO2d{angle}, Eigen::Vector2d{x, y}};
}
void SamePose(const Sophus::SE2d& actual, const Sophus::SE2d& expected) {
  const auto error = expected.inverse() * actual;
  EXPECT_NEAR(error.translation().norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(error.so2().log(), 0.0, 1.0e-8);
}
std::unique_ptr<BelugaSLAM> MakeSlam(bool polish = true) {
  FastSLAMParams params;
  params.min_particles = 1; params.max_particles = 30; params.submap_num_range_data = 2;
  params.keyframe_max_time = 0.0;
  params.loop_robust_polish = polish;
  return std::make_unique<BelugaSLAM>(
      BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{0.1, 0.1, 0.1, 0.1}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100, 2, 0.5, 0.5, 0.2, true}, GridTypeOC{}}, params);
}
std::shared_ptr<Hypothesis> BuildPrior(BelugaSLAM& slam) {
  auto h = std::get<2>(*slam.particles().begin()); h->has_local_pose = true;
  const std::vector<Sophus::SE2d> path{Pose(), Pose(1), Pose(2), Pose(2,1), Pose(2,2),
      Pose(1,2), Pose(0,2), Pose(0,1), Pose(0.4,0.1)};
  for (std::size_t i = 0; i < path.size(); ++i) {
    h->local_pose = path[i]; std::get<0>(*slam.particles().begin()) = path[i];
    slam.update_occupancy_grid({{1.05, 0.05}, {1.05, 1.05}}, static_cast<double>(i));
  }
  return h;
}

TEST(LoopPgoTest, DefaultsEnableBeliefVerificationAndPgo) {
  FastSLAMParams params;
  EXPECT_TRUE(params.enable_loop_closure); EXPECT_TRUE(params.enable_pgo);
  EXPECT_EQ(params.loop_verifier_mode, "belief");
  EXPECT_TRUE(params.loop_robust_polish);
  EXPECT_TRUE(params.pgo_analytic_jacobians);
  EXPECT_EQ(BELUGASLAM_ENABLE_LOOP_CLOSURE, 1);
}

TEST(LoopPgoTest, AnalyticCostMatchesAutoDiffIncludingRigidOffsets) {
  for (int k = 0; k < 32; ++k) {
    std::array<double, 3> a{0.7*k-9., -0.3*k+8., 0.09*k-1.4};
    std::array<double, 3> b{-0.4*k+2., 0.5*k-3., -0.08*k+0.2};
    const double* blocks[] = {a.data(), b.data()};
    std::array<double, 3> ra{}, rd{};
    std::array<double, 9> ja{}, jb{}, da{}, db{};
    double* analytic_j[] = {ja.data(), jb.data()};
    double* autodiff_j[] = {da.data(), db.data()};
    std::unique_ptr<ceres::CostFunction> analytic(PoseGraphEdgeError::Create(
        .3, -.8, .2, 5, 8, .1*k, -.2*k, .07*k, true));
    std::unique_ptr<ceres::CostFunction> autodiff(PoseGraphEdgeError::Create(
        .3, -.8, .2, 5, 8, .1*k, -.2*k, .07*k, false));
    ASSERT_TRUE(analytic->Evaluate(blocks, ra.data(), analytic_j));
    ASSERT_TRUE(autodiff->Evaluate(blocks, rd.data(), autodiff_j));
    for (std::size_t i = 0; i < ra.size(); ++i) EXPECT_NEAR(ra[i], rd[i], 1.e-10);
    for (std::size_t i = 0; i < ja.size(); ++i) {
      EXPECT_NEAR(ja[i], da[i], 1.e-10);
      EXPECT_NEAR(jb[i], db[i], 1.e-10);
    }
    analytic_j[0] = nullptr;
    ASSERT_TRUE(analytic->Evaluate(blocks, ra.data(), analytic_j));
    ASSERT_TRUE(analytic->Evaluate(blocks, ra.data(), nullptr));
  }
}

std::shared_ptr<Hypothesis> BuildConflictingPrior(BelugaSLAM& slam, double query_weight) {
  auto h = std::get<2>(*slam.particles().begin());
  h->submaps = SubmapList{};
  auto anchor = std::make_shared<Submap>(0, Pose(), 20, 20, .1);
  anchor->finish();
  h->submaps.history.push_back(anchor);
  for (std::size_t i = 0; i < 3; ++i) {
    const auto pose = i == 1 ? Pose(0, 1) : Pose();
    TrajectoryNode node;
    node.id = i; node.sequence = i; node.local_pose = pose; node.global_pose = pose;
    h->submaps.trajectory_nodes.push_back(node);
    h->submaps.trajectory_samples.push_back({i, 0, pose});
    h->submaps.node_submap_constraints.push_back({0, i, pose, i == 2 ? query_weight : 10., 8.});
  }
  h->submaps.next_node_id = 3; h->submaps.next_submap_id = 1;
  h->has_local_pose = true; h->local_pose = Pose();
  return h;
}

TEST(LoopPgoTest, RejectsLoopThatSlipsOutsideFitGateUnderInstalledObjective) {
  for (bool polish : {false, true}) {
    auto slam = MakeSlam(polish);
    const auto h = BuildConflictingPrior(*slam, 10.);
    ASSERT_TRUE(slam->optimize_pose_graph(h));
    const auto result = slam->evaluate_loop_candidate(h, {0, 2, h->id, 1., 1., Pose(.5)});
    EXPECT_NEAR(result.forced_fit_translation, .25, 1.e-3);
    EXPECT_EQ(result.polish_attempted, polish);
    EXPECT_EQ(result.usable, !polish);
    if (polish) {
      EXPECT_EQ(result.status, "settled_fit_failed");
      EXPECT_NEAR(result.fit_translation, .4, 1.e-3);
      EXPECT_DOUBLE_EQ(result.compatibility, 0.);
      EXPECT_FALSE(result.hypothesis);
    }
    EXPECT_EQ(h->submaps.inter_constraint_count(), 0U);
    SamePose(h->submaps.trajectory_nodes.back().global_pose, Pose());
  }
}

TEST(LoopPgoTest, PolishedTrialCannotGainCompatibilityAndRetainsItsFit) {
  auto slam = MakeSlam();
  const auto h = BuildConflictingPrior(*slam, std::sqrt(40.));
  ASSERT_TRUE(slam->optimize_pose_graph(h));
  const auto result = slam->evaluate_loop_candidate(h, {0, 2, h->id, 1., 1., Pose(.5)});
  ASSERT_TRUE(result.usable);
  EXPECT_TRUE(result.polish_attempted);
  EXPECT_NEAR(result.fit_translation, .25, 1.e-3);
  EXPECT_LE(result.compatibility, result.forced_compatibility);
  EXPECT_LE(result.compatibility, result.settled_compatibility);
  const auto before = result.hypothesis->submaps.trajectory_nodes.back().global_pose;
  ASSERT_TRUE(slam->optimize_pose_graph(result.hypothesis, false));
  EXPECT_LT((before.inverse() * result.hypothesis->submaps.trajectory_nodes.back().global_pose).translation().norm(), 1.e-3);
  EXPECT_EQ(h->submaps.inter_constraint_count(), 0U);
}

TEST(LoopPgoTest, SkipsSecondSolveInsideHuberQuadraticRegion) {
  auto slam = MakeSlam();
  const auto h = BuildConflictingPrior(*slam, 10.);
  ASSERT_TRUE(slam->optimize_pose_graph(h));
  const auto result = slam->evaluate_loop_candidate(h, {0, 2, h->id, 1., 1., Pose(.1)});
  ASSERT_TRUE(result.usable);
  EXPECT_FALSE(result.polish_attempted);
  EXPECT_DOUBLE_EQ(result.polish_ms, 0.);
  EXPECT_NEAR(result.fit_translation, .05, 1.e-3);
}

TEST(LoopPgoTest, TrialPgoDoesNotMutateLiveMapsParticlesOrConstraints) {
  auto slam = MakeSlam(); const auto h = BuildPrior(*slam);
  ASSERT_TRUE(slam->optimize_pose_graph(h));
  const auto live_pose = h->local_pose;
  const auto particle_pose = std::get<0>(*slam->particles().begin());
  const auto reference_pose = h->submaps.history.front()->global_pose();
  const auto active_grid = h->submaps.active_submaps.front()->grid().data();
  const auto node_count = h->submaps.trajectory_nodes.size();
  BelugaSLAM::LoopCandidate candidate{0, 8, h->id, 1.0, 1.0, Pose(0.1, 0.0)};
  const auto result = slam->evaluate_loop_candidate(h, candidate);
  ASSERT_TRUE(result.usable);
  EXPECT_GT(result.compatibility, 0.25);
  EXPECT_EQ(h->submaps.inter_constraint_count(), 0U);
  EXPECT_EQ(h->submaps.trajectory_nodes.size(), node_count);
  SamePose(h->local_pose, live_pose); SamePose(std::get<0>(*slam->particles().begin()), particle_pose);
  SamePose(h->submaps.history.front()->global_pose(), reference_pose);
  EXPECT_EQ(h->submaps.active_submaps.front()->grid().data(), active_grid);
  EXPECT_EQ(result.hypothesis->submaps.inter_constraint_count(), 1U);
}

TEST(LoopPgoTest, ImpossibleLoopCannotPassByBeingRobustlyIgnored) {
  auto slam = MakeSlam(); const auto h = BuildPrior(*slam);
  ASSERT_TRUE(slam->optimize_pose_graph(h));
  const auto result = slam->evaluate_loop_candidate(h, {0, 8, h->id, 1.0, 1.0, Pose(100, 100)});
  EXPECT_LT(result.compatibility, 0.25);
  EXPECT_EQ(h->submaps.inter_constraint_count(), 0U);
}

TEST(LoopPgoTest, ActiveGridsMoveRigidlyAndLocalPriorsRemainImmutable) {
  auto slam = MakeSlam(); const auto h = BuildPrior(*slam);
  ASSERT_EQ(h->submaps.active_submaps.size(), 2U);
  auto& graph = h->submaps;
  const auto reference = graph.matching_submap();
  const auto tracking_in_reference = reference->global_pose().inverse() * h->local_pose;
  const auto relative = graph.active_submaps[0]->global_pose().inverse() * graph.active_submaps[1]->global_pose();
  const auto local_prior = graph.local_trajectory_constraints.back().T_from_to;
  const auto node_local = graph.trajectory_nodes.back().local_pose;
  const auto pixels = graph.active_submaps[0]->grid().data();
  graph.node_submap_constraints.push_back({graph.history.front()->id(), graph.trajectory_nodes.back().id,
      Pose(0.1, 0.0), 10, 12, ConstraintTag::kInterSubmap, 1, 1, 0, 8});
  ASSERT_TRUE(slam->optimize_pose_graph(h, true, graph.node_submap_constraints.size() - 1));
  SamePose(graph.active_submaps[0]->global_pose().inverse() * graph.active_submaps[1]->global_pose(), relative);
  SamePose(graph.matching_submap()->global_pose().inverse() * h->local_pose, tracking_in_reference);
  SamePose(graph.local_trajectory_constraints.back().T_from_to, local_prior);
  SamePose(graph.trajectory_nodes.back().local_pose, node_local);
  EXPECT_EQ(graph.active_submaps[0]->grid().data(), pixels);
}

TEST(LoopPgoTest, InvalidGraphFailsWithoutCommittingState) {
  auto slam = MakeSlam(); const auto h = BuildPrior(*slam);
  const auto before = h->local_pose;
  h->submaps.node_submap_constraints.push_back({999999, 999999, Pose(), 1, 1});
  EXPECT_FALSE(slam->optimize_pose_graph(h));
  SamePose(h->local_pose, before);
}

TEST(LoopPgoTest, SequenceIdentitySurvivesCloudTrimmingAndFilteredQueries) {
  auto slam = MakeSlam(); const auto h = BuildPrior(*slam);
  h->submaps.trim_scan_data_outside_active_submaps();
  ASSERT_TRUE(h->submaps.find_node_by_sequence(0));
  EXPECT_FALSE(h->submaps.find_node_by_sequence(0)->constant_data);
  Sophus::SE2d pose;
  EXPECT_TRUE(h->submaps.pose_at_sequence(0, pose));
  // Simulate a hypothesis whose filter did not create the latest query node.
  const auto removed_id = h->submaps.trajectory_nodes.back().id;
  h->submaps.trajectory_nodes.pop_back();
  auto& edges = h->submaps.node_submap_constraints;
  edges.erase(std::remove_if(edges.begin(), edges.end(), [&](const auto& e) { return e.node_id == removed_id; }), edges.end());
  auto& prior_edges = h->submaps.local_trajectory_constraints;
  prior_edges.erase(std::remove_if(prior_edges.begin(), prior_edges.end(), [&](const auto& e) { return e.to_node_id == removed_id; }), prior_edges.end());
  const auto count = h->submaps.trajectory_nodes.size();
  auto trial_graph = h->submaps;
  const auto promoted_id = slam->ensure_query_node(trial_graph, 8);
  ASSERT_TRUE(trial_graph.find_node(promoted_id));
  EXPECT_EQ(trial_graph.find_node(promoted_id)->sequence, 8U);
  EXPECT_EQ(h->submaps.trajectory_nodes.size(), count);
  EXPECT_EQ(trial_graph.trajectory_nodes.size(), count + 1);
}

TEST(LoopPgoTest, HypothesisMassSurvivesMinimumParticleQuotas) {
  auto slam = MakeSlam(); auto parent = std::get<2>(*slam->particles().begin());
  auto child = std::make_shared<Hypothesis>(*parent); child->id = 99;
  slam->install_population({{parent, parent, 0.99, Pose(), true}, {child, parent, 0.01, Pose(), false}}, 30);
  const auto masses = slam->hypothesis_masses();
  EXPECT_NEAR(masses.at(parent->id), 0.99, 1.0e-12);
  EXPECT_NEAR(masses.at(child->id), 0.01, 1.0e-12);
  EXPECT_EQ(slam->particles().size(), 30U);
}

TEST(LoopPgoTest, PoseOnlyCloneDetachesBeforePixelInsertion) {
  auto submap = std::make_shared<Submap>(0, Pose(), 20, 20, 0.1);
  submap->mutable_grid().at(10,10) = 1.2F;
  auto trial = submap->clone_for_pose();
  EXPECT_EQ(&trial->grid(), &submap->grid());
  trial->set_global_pose(Pose(10,20));
  trial->mutable_grid().at(10,10) = 5.0F;
  EXPECT_FLOAT_EQ(submap->grid().at(10,10), 1.2F);
  SamePose(submap->global_pose(), Pose());
}

TEST(LoopPgoTest, FinalTrajectoryReportsOptimizedPosesAndKeepsTheOnlineOnes) {
  auto slam = MakeSlam(); const auto h = BuildPrior(*slam);
  auto& graph = h->submaps;
  const auto before = slam->final_trajectory();
  ASSERT_GE(before.size(), 2U);
  // Nothing has been optimized yet, so both trajectories still agree scan by scan.
  for (const auto& point : before) SamePose(point.optimized, point.online);

  graph.node_submap_constraints.push_back({graph.history.front()->id(), graph.trajectory_nodes.back().id,
      Pose(0.1, 0.0), 10, 12, ConstraintTag::kInterSubmap, 1, 1, 0, 8});
  ASSERT_TRUE(slam->optimize_pose_graph(h, true, graph.node_submap_constraints.size() - 1));

  const auto optimized = slam->final_trajectory();
  ASSERT_EQ(optimized.size(), before.size());
  for (std::size_t i = 0; i < optimized.size(); ++i) {
    EXPECT_EQ(optimized[i].sequence, before[i].sequence);
    // The online pose is the record of what the run published; nothing rewrites it.
    SamePose(optimized[i].online, before[i].online);
    Sophus::SE2d pose;
    ASSERT_TRUE(graph.pose_at_sequence(optimized[i].sequence, pose));
    SamePose(optimized[i].optimized, pose);
  }

  // A submap that moves carries every scan recorded against it. That is what lets a
  // loop closure correct poses the run had already published.
  const auto shift = Pose(3.0, -2.0, 0.5);
  for (auto& submap : graph.history) submap->set_global_pose(shift * submap->global_pose());
  for (auto& submap : graph.active_submaps) submap->set_global_pose(shift * submap->global_pose());
  for (auto& node : graph.trajectory_nodes) node.global_pose = shift * node.global_pose;
  const auto shifted = slam->final_trajectory();
  ASSERT_EQ(shifted.size(), optimized.size());
  for (std::size_t i = 0; i < shifted.size(); ++i) {
    SamePose(shifted[i].online, optimized[i].online);
    SamePose(shifted[i].optimized, shift * optimized[i].optimized);
  }
}
}  // namespace
