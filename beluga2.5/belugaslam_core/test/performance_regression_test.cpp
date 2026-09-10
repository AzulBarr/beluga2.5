#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {
Sophus::SE2d Pose(double x = 0, double y = 0, double angle = 0) {
  return {Sophus::SO2d{angle}, Eigen::Vector2d{x, y}};
}
std::unique_ptr<BelugaSLAM> Slam(int workers) {
  FastSLAMParams p;
  p.worker_threads = workers; p.min_particles = 12; p.max_particles = 30;
  p.enable_pgo = false; p.enable_loop_closure = false;
  p.submap_num_range_data = 2; p.keyframe_max_time = 0;
  return std::make_unique<BelugaSLAM>(
      BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{0.1, 0.1, 0.1, 0.1}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100, 2, 0.5, 0.5, 0.2, true}, GridTypeOC{}}, p);
}
void ExpectPose(const Sophus::SE2d& a, const Sophus::SE2d& b) {
  const auto delta = a.inverse() * b;
  EXPECT_NEAR(delta.translation().norm(), 0.0, 1e-10);
  EXPECT_NEAR(delta.so2().log(), 0.0, 1e-10);
}
}

TEST(PerformanceRegression, ParallelParticleMatchingPreservesStateAndWeights) {
  auto serial = Slam(1), parallel = Slam(2);
  BelugaSLAM::measurement_type z;
  for (int i = 0; i < 100; ++i) z.emplace_back(3.03, -2.01 + 0.04 * i);
  for (int scan = 0; scan < 12; ++scan) {
    const auto u = std::make_tuple(Pose(0.04 * scan, 0, 0.01 * scan),
                                  Pose(0.04 * std::max(0, scan - 1), 0, 0.01 * std::max(0, scan - 1)));
    for (auto* slam : {serial.get(), parallel.get()}) {
      slam->sample_motion_model(u); slam->measurement_model_map(z);
      const auto events = slam->update_occupancy_grid(z, scan);
      slam->post_update(z, events);
    }
    for (std::size_t i = 0; i < serial->particles().size(); ++i) {
      const auto a = *(serial->particles().begin() + i), b = *(parallel->particles().begin() + i);
      ExpectPose(std::get<0>(a), std::get<0>(b));
      EXPECT_DOUBLE_EQ(static_cast<double>(std::get<1>(a)), static_cast<double>(std::get<1>(b)));
    }
    ExpectPose(serial->best_pose(), parallel->best_pose());
  }
  EXPECT_EQ(serial->best_occupancy_grid().data(), parallel->best_occupancy_grid().data());
}

TEST(PerformanceRegression, CachedLoopScoresEqualOriginalExponential) {
  Submap submap(0, Pose(), 80, 60, 0.1);
  submap.mutable_grid().at(40, 30) = 5;
  submap.mutable_grid().at(25, 24) = 5;
  submap.finish(); submap.prepare_loop_matching();
  const auto& grid = submap.grid();
  for (int y = -1; y <= grid.height(); ++y) for (int x = -1; x <= grid.width(); ++x) {
    const double px = grid.origin_x() + (x + 0.5) * grid.resolution();
    const double py = grid.origin_y() + (y + 0.5) * grid.resolution();
    const auto [distance, score] = submap.loop_cell_at(px, py);
    EXPECT_FLOAT_EQ(distance, submap.distance_at(px, py));
    const double normalized = static_cast<double>(distance) / 0.2;
    EXPECT_DOUBLE_EQ(score, std::isfinite(distance) ? std::exp(-0.5 * normalized * normalized) : 0.0);
  }
}

TEST(PerformanceRegression, PublicationIsLazyAndReusedAcrossConsumers) {
  auto slam = Slam(1);
  const BelugaSLAM::measurement_type z{{2.01, 0.01}, {2.01, 1.01}};
  slam->measurement_model_map(z);
  const auto events = slam->update_occupancy_grid(z, 1);
  slam->post_update(z, events);
  EXPECT_EQ(slam->publication_rebuilds(), 0U);
  const auto* occupancy = &slam->best_occupancy_grid();
  EXPECT_EQ(slam->publication_rebuilds(), 1U);
  (void)slam->best_log_odds_grid();
  EXPECT_EQ(&slam->best_occupancy_grid(), occupancy);
  EXPECT_EQ(slam->publication_rebuilds(), 1U);
  slam->post_update(z, {});
  (void)slam->best_occupancy_grid();
  EXPECT_EQ(slam->publication_rebuilds(), 2U);
}

TEST(PerformanceRegression, HistoryCacheInvalidatesOnPoseChangeAndHypothesisSwitch) {
  auto slam = Slam(1);
  auto h = std::get<2>(*slam->particles().begin());
  auto map = std::make_shared<Submap>(0, Pose(), 60, 60, 0.1);
  map->mutable_grid().at(30, 30) = 5; map->finish();
  h->submaps.history.push_back(map);
  GridTypeLO cached, fresh;
  slam->compose_publication_view(h, cached);
  for (int change = 0; change < 3; ++change) {
    auto alternate = std::make_shared<Hypothesis>(*h);
    alternate->submaps.history[0] = map->clone_for_pose();
    alternate->submaps.history[0]->set_global_pose(Pose(change, 0.5, 0.3));
    slam->compose_publication_view(alternate, cached);
    fresh = cached;
    std::fill(fresh.data().begin(), fresh.data().end(), 0);
    slam->draw_submap_into_grid(alternate->submaps.history[0], fresh);
    EXPECT_EQ(cached.data(), fresh.data());
  }
}

TEST(PerformanceRegression, ParallelTrialSolvesMatchSerialAndLeavePriorUntouched) {
  auto slam = Slam(2);
  auto h = std::get<2>(*slam->particles().begin()); h->has_local_pose = true;
  for (int i = 0; i < 9; ++i) {
    h->local_pose = Pose(i < 5 ? i : 8-i, i < 5 ? 0 : 1);
    slam->update_occupancy_grid({{1.05, 0.05}, {1.05, 1.05}}, i);
  }
  ASSERT_TRUE(slam->optimize_pose_graph(h));
  const auto prior_pose = h->local_pose;
  const auto prior_edges = h->submaps.node_submap_constraints.size();
  const std::vector<BelugaSLAM::LoopCandidate> candidates{{0, 8, h->id, 1, 1, Pose(0, 1)},
                                                       {0, 8, h->id, 1, 1, Pose(20, 20)}};
  std::vector<BelugaSLAM::LoopTrial> serial(2), parallel(2);
  for (std::size_t i = 0; i < 2; ++i) serial[i] = slam->evaluate_loop_candidate(h, candidates[i]);
  tbb::task_arena arena(2);
  arena.execute([&] { tbb::parallel_for(std::size_t{0}, std::size_t{2}, [&](std::size_t i) {
    parallel[i] = slam->evaluate_loop_candidate(h, candidates[i]);
  }); });
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(serial[i].usable, parallel[i].usable);
    EXPECT_NEAR(serial[i].compatibility, parallel[i].compatibility, 1e-10);
  }
  ExpectPose(h->local_pose, prior_pose);
  EXPECT_EQ(h->submaps.node_submap_constraints.size(), prior_edges);
}
