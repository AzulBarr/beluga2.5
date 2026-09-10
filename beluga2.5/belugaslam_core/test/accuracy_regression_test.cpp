#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {
Sophus::SE2d P(double x=0, double y=0, double a=0) {
  return {Sophus::SO2d{a}, Eigen::Vector2d{x,y}};
}
void Near(const Sophus::SE2d& a, const Sophus::SE2d& b) {
  const auto d=a.inverse()*b;
  EXPECT_NEAR(d.translation().norm(),0,1e-9);
  EXPECT_NEAR(d.so2().log(),0,1e-9);
}
}

TEST(AccuracyRegression, ZeroNoisePreservesSmallReverseAndLateralOdometry) {
  BelugaSLAM::MotionModel model{beluga::DifferentialDriveModelParam{0,0,0,0,.01}};
  std::mt19937 rng(42);
  const auto odom=P(2,-3,.7), particle=P(-8,4,-1.2);
  for (double dx : {-.008,0.,.008}) for (double dy : {-.003,0.,.003}) {
    for (double angle : {-.2,0.,.2}) {
      const auto delta=P(dx,dy,angle);
      auto sampler=model(std::make_tuple(odom*delta,odom));
      Near(sampler(particle,rng),particle*delta);
    }
  }
}

TEST(AccuracyRegression, DenseTrajectoryUsesTimeInterpolatedNodeCorrections) {
  SubmapList graph;
  auto sm=std::make_shared<Submap>(0,P(),20,20,.1);
  graph.active_submaps.push_back(sm);
  // Local path x=t, optimized correction grows by 2 m over 10 seconds.
  graph.trajectory_nodes.push_back({0,nullptr,P(),P(),0});
  graph.trajectory_nodes.push_back({1,nullptr,P(12),P(10),3});
  const std::int64_t epoch=976052857000000000LL;
  for (const auto& pair : std::vector<std::pair<std::uint64_t,int>>{{0,0},{1,2},{2,9},{3,10}})
    graph.trajectory_samples.push_back({pair.first,0,P(pair.second),P(pair.second),epoch+pair.second*1000000000LL});
  Sophus::SE2d pose;
  ASSERT_TRUE(graph.pose_at_sequence(1,pose)); Near(pose,P(2.4));
  ASSERT_TRUE(graph.pose_at_sequence(2,pose)); Near(pose,P(10.8));
  ASSERT_TRUE(graph.pose_at_sequence(3,pose)); Near(pose,P(12));
  // A common rigid change of world coordinates must commute with readout.
  const auto world=P(4,-5,.8);
  for (auto& node:graph.trajectory_nodes) node.global_pose=world*node.global_pose;
  sm->set_global_pose(world*sm->global_pose());
  ASSERT_TRUE(graph.pose_at_sequence(2,pose)); Near(pose,world*P(10.8));
}

TEST(AccuracyRegression, LoopUsesRetrievedSubmapInsteadOfItsPredecessor) {
  FastSLAMParams params;
  params.min_particles=1;params.max_particles=5;params.submap_num_range_data=2;
  params.keyframe_max_time=0;
  BelugaSLAM slam{BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.05,.1,.05}},
    BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params};
  auto h=std::get<2>(*slam.particles().begin());h->has_local_pose=true;
  for (int i=0;i<9;++i) {
    h->local_pose=P(.1*i);
    slam.update_occupancy_grid({{1.05,.05},{1.05,1.05}},i);
  }
  const auto anchor=h->submaps.find_submap_by_anchor(2);
  ASSERT_TRUE(anchor);
  ASSERT_NE(anchor->id(),h->submaps.find_sample(2)->submap_id);
  const auto measurement=anchor->global_pose().inverse()*h->submaps.trajectory_nodes.back().global_pose;
  const auto trial=slam.evaluate_loop_candidate(h,{2,8,h->id,1,1,measurement});
  ASSERT_TRUE(trial.usable);
  const auto& edge=trial.hypothesis->submaps.node_submap_constraints.back();
  EXPECT_EQ(edge.submap_id,anchor->id());
  Near(edge.T_submap_node,measurement);
}

TEST(AccuracyRegression, ProposalSeedKeepsWeightsAndCannotWorsenFrontendObjective) {
  std::vector<double> baseline_weights;
  double baseline_cost=0;
  for (const std::string mode : {"frontend", "proposal_seed"}) {
    FastSLAMParams params;params.min_particles=5;params.max_particles=30;
    params.frontend_pose_mode=mode;params.enable_loop_closure=false;
    BelugaSLAM slam{BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.05,.1,.05}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params};
    auto h=std::get<2>(*slam.particles().begin());
    auto sm=std::make_shared<Submap>(0,P(),160,160,.05);
    BelugaSLAM::measurement_type scan;
    for(int y=20;y<140;++y) {
      sm->mutable_grid().at(120,y)=5;
      scan.emplace_back(2.025,-4+(y+.5)*.05);
    }
    h->submaps.active_submaps.push_back(sm);h->has_local_pose=true;
    for(auto&& particle:slam.particles()) std::get<0>(particle)=P(.02);
    slam.sample_motion_model({P(),P()});slam.measurement_model_map(scan);
    const auto pose=h->local_pose;
    const double cost=belugaslam::tracking_objective(*sm->tracking_field(),scan,
      {pose.translation().x(),pose.translation().y(),pose.so2().log()},{},params.tracking);
    std::vector<double> weights;
    for(const auto& p:slam.particles())weights.push_back(static_cast<double>(std::get<1>(p)));
    if(mode=="frontend") {baseline_weights=weights;baseline_cost=cost;}
    else {EXPECT_EQ(weights,baseline_weights);EXPECT_LE(cost,baseline_cost+1e-10);}
  }
}
