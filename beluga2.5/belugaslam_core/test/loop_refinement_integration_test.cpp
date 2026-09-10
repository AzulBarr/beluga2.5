#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"
namespace {
Sophus::SE2d Pose(double x=0,double y=0,double a=0) {return {Sophus::SO2d{a},Eigen::Vector2d{x,y}};}
std::unique_ptr<BelugaSLAM> Slam(FastSLAMParams p={}) {
  p.min_particles=1;p.max_particles=30;p.submap_num_range_data=2;p.keyframe_max_time=0;
  return std::make_unique<BelugaSLAM>(BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.1,.1,.1}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},p);
}
std::shared_ptr<Hypothesis> Prior(BelugaSLAM& slam) {
  auto h=std::get<2>(*slam.particles().begin());h->has_local_pose=true;
  const std::vector<Sophus::SE2d> path{Pose(),Pose(1),Pose(2),Pose(2,1),Pose(2,2),Pose(1,2),Pose(0,2),Pose(0,1),Pose(.4,.1)};
  for(std::size_t i=0;i<path.size();++i) {h->local_pose=path[i];slam.update_occupancy_grid({{1.05,.05},{1.05,1.05}},i);}
  return h;
}
}
TEST(LoopRefinementIntegration, ReplayedQueryCannotChangeParticlesWeightsOrGraphs) {
  auto slam=Slam();auto parent=Prior(*slam);ASSERT_TRUE(slam->optimize_pose_graph(parent));
  BelugaSLAM::LoopCandidate candidate{0,8,parent->id,1,1,Pose(.1)};
  slam->verify_loop_candidates({candidate});
  const auto mass=slam->hypothesis_masses();
  ASSERT_GT(mass.size(),1U);
  struct Snapshot {state_type pose;double weight;std::shared_ptr<Hypothesis> hypothesis;std::size_t edges;};
  std::vector<Snapshot> snapshot;
  bool loop=false,no_loop=false;
  for(const auto& p:slam->particles()) {
    const auto h=std::get<2>(p);const auto edges=h->submaps.inter_constraint_count();
    snapshot.push_back({std::get<0>(p),static_cast<double>(std::get<1>(p)),h,edges});
    loop=loop||edges>0;no_loop=no_loop||edges==0;
  }
  ASSERT_TRUE(loop);ASSERT_TRUE(no_loop);
  // Includes a changed geometric alternative from the same already consumed scan.
  auto different=candidate;different.T_reference_query=Pose(.15);
  slam->verify_loop_candidates({candidate,different});
  EXPECT_EQ(slam->hypothesis_masses(),mass);
  ASSERT_EQ(slam->particles().size(),snapshot.size());
  for(std::size_t i=0;i<snapshot.size();++i) {
    const auto& p=*(slam->particles().begin()+i);const auto& before=snapshot[i];
    EXPECT_EQ(std::get<2>(p),before.hypothesis);EXPECT_DOUBLE_EQ(static_cast<double>(std::get<1>(p)),before.weight);
    EXPECT_EQ(std::get<2>(p)->submaps.inter_constraint_count(),before.edges);
    EXPECT_LT((before.pose.inverse()*std::get<0>(p)).translation().norm(),1e-12);
  }
}
TEST(LoopRefinementIntegration, InterpolationUsesTheCachedNativeGridWithoutMutatingIt) {
  auto slam=Slam();Submap map(0,Pose(3,2,.4),100,100,.05);
  auto& grid=map.mutable_grid();for(int i=20;i<80;++i) {grid.at(80,i)=5;grid.at(i,80)=5;}
  map.finish();const auto pixels=map.grid().data();const auto cached=map.loop_matching_data();
  ScanNodeData scan;for(int i=20;i<80;++i) {scan.returns.emplace_back(1.525,-2.5+(i+.5)*.05);scan.returns.emplace_back(-2.5+(i+.5)*.05,1.525);}
  const auto aligned=slam->score_scan_in_submap(scan,map,Pose(),cached);
  const auto offset=slam->score_scan_in_submap(scan,map,Pose(.02,.02),cached);
  EXPECT_GT(aligned.score,offset.score);EXPECT_GT(offset.score,0.9);
  EXPECT_EQ(map.grid().data(),pixels);EXPECT_EQ(map.loop_matching_data(),cached);
  const auto modes=slam->match_scan_to_submap_modes(scan,map,Pose(.25,-.25,.07));
  ASSERT_FALSE(modes.empty());EXPECT_LT(modes.front().T_submap_node.translation().norm(),.04);
  EXPECT_LT(std::abs(modes.front().T_submap_node.so2().log()),.01);
}
TEST(LoopRefinementIntegration, NewSearchDefaultsKeepWeightedVerifierAndBothBranchMechanisms) {
  FastSLAMParams p;EXPECT_EQ(p.loop_search_modes,2U);EXPECT_EQ(p.loop_verifier_mode,"belief");
  EXPECT_TRUE(p.enable_loop_closure);EXPECT_TRUE(p.enable_pgo);EXPECT_GE(p.max_hypotheses,2U);
  EXPECT_GE(p.loop_max_branches,2U);EXPECT_GE(p.split_persistence,1U);
  p.loop_search_modes=9;EXPECT_THROW(Slam(p),std::invalid_argument);
  p.loop_search_modes=2;p.loop_search_rotation=4;EXPECT_THROW(Slam(p),std::invalid_argument);
}
