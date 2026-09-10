#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"
namespace {
Sophus::SE2d Pose(double x=0,double y=0,double a=0) {return {Sophus::SO2d{a},Eigen::Vector2d{x,y}};}
std::unique_ptr<BelugaSLAM> Slam(FastSLAMParams params={}) {
  params.enable_loop_closure=false;
  return std::make_unique<BelugaSLAM>(BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.05,.1,.05}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params);
}
BelugaSLAM::measurement_type Scene(Submap& map,const Sophus::SE2d& robot) {
  auto& grid=map.mutable_grid();BelugaSLAM::measurement_type scan;
  for(int i=20;i<180;++i) {
    grid.at(180,i)=5;grid.at(i,190)=5;
    for(const auto& p:{Eigen::Vector2d{3.025,-6+(i+.5)*.05},Eigen::Vector2d{-6+(i+.5)*.05,3.525}}) {
      const auto local=robot.inverse()*p;scan.emplace_back(local.x(),local.y());
    }
  }
  return scan;
}
}
TEST(LateRunIntegration, RecoveryWaitsForNextScanBeforeInsertion) {
  auto slam=Slam();const auto h=std::get<2>(*slam->particles().begin());
  auto map=std::make_shared<Submap>(0,Pose(),240,240,.05);
  const auto scan=Scene(*map,Pose(.78,-.24,.1));
  h->submaps.active_submaps.push_back(map);h->submaps.next_submap_id=1;h->has_local_pose=true;
  const auto initial=map->grid().data();
  slam->measurement_model_map(scan);
  ASSERT_EQ(h->tracking_status,"recovery_pending");EXPECT_FALSE(h->tracking_usable);
  slam->update_occupancy_grid(scan,0);
  EXPECT_TRUE(h->submaps.trajectory_nodes.empty());EXPECT_EQ(map->grid().data(),initial);
  slam->sample_motion_model({Pose(),Pose()});slam->measurement_model_map(scan);
  EXPECT_EQ(h->tracking_status,"recovered");EXPECT_TRUE(h->tracking_usable);
  EXPECT_LT((Pose(.78,-.24,.1).inverse()*h->local_pose).translation().norm(),.12);
  slam->update_occupancy_grid(scan,1);
  EXPECT_EQ(h->submaps.trajectory_nodes.size(),1U);
}
TEST(LateRunIntegration, AnotherMatureNativeSubmapCanRecoverStaleReference) {
  auto slam=Slam();const auto h=std::get<2>(*slam->particles().begin());
  auto old=std::make_shared<Submap>(0,Pose(),240,240,.05);
  old->mutable_grid().at(2,2)=5;
  auto recent=std::make_shared<Submap>(1,Pose(),240,240,.05);
  const auto scan=Scene(*recent,Pose());for(int i=0;i<5;++i) recent->add_insertion();
  h->has_local_pose=true;h->submaps.active_submaps={old,recent};h->submaps.next_submap_id=2;
  slam->measurement_model_map(scan);ASSERT_TRUE(h->has_pending_recovery);
  EXPECT_EQ(h->recovery_reference,1U);slam->update_occupancy_grid(scan,0);
  slam->measurement_model_map(scan);EXPECT_EQ(h->tracking_status,"recovered");
  EXPECT_EQ(h->submaps.matching_submap()->id(),1U);
}
TEST(LateRunIntegration, UnusableScanCannotCreateRecoveryOrNewMap) {
  auto slam=Slam();auto h=std::get<2>(*slam->particles().begin());
  auto map=std::make_shared<Submap>(0,Pose(),240,240,.05);Scene(*map,Pose());
  h->submaps.active_submaps={map};h->has_local_pose=true;
  BelugaSLAM::measurement_type scan;for(int i=0;i<40;++i) scan.emplace_back(100+i,100);
  for(int i=0;i<8;++i) {slam->measurement_model_map(scan);slam->update_occupancy_grid(scan,i);}
  EXPECT_FALSE(h->tracking_usable);EXPECT_FALSE(h->has_pending_recovery);
  EXPECT_TRUE(h->submaps.trajectory_nodes.empty());EXPECT_EQ(h->submaps.active_submaps.size(),1U);
}
TEST(LateRunIntegration, LocalOnlyGraphDoesNotScheduleRedundantPgo) {
  FastSLAMParams p;p.pgo_every_n_nodes=1;p.keyframe_max_time=0;auto slam=Slam(p);
  auto h=std::get<2>(*slam->particles().begin());h->has_local_pose=true;
  for(int i=0;i<20;++i) {
    h->local_pose=Pose(i*.1);auto events=slam->update_occupancy_grid({{1.05,.05}},i);
    slam->post_update({},events);EXPECT_EQ(slam->backend_timing().baseline_solves,0U);
    EXPECT_EQ(slam->backend_timing().local_only_skips,1U);
  }
  EXPECT_EQ(h->optimized_node_count,h->submaps.trajectory_nodes.size());
}
TEST(LateRunIntegration, FrozenCachesAreLazySharedEvictableAndReproducible) {
  auto map=std::make_shared<Submap>(0,Pose(),240,240,.05);Scene(*map,Pose());map->finish();
  EXPECT_EQ(map->loop_cache_statistics().first,0U);
  auto a=map->loop_matching_data();auto clone=map->clone_for_pose();
  EXPECT_EQ(a,clone->loop_matching_data());EXPECT_GT(map->loop_cache_statistics().first,0U);
  clone->release_loop_cache();EXPECT_EQ(map->loop_cache_statistics().first,0U);
  auto b=map->loop_matching_data();EXPECT_NE(a,b);EXPECT_EQ(a->distances,b->distances);EXPECT_EQ(a->scores,b->scores);
}
TEST(LateRunIntegration, ZeroBudgetEvictsDerivedDataWithoutRemovingHistory) {
  FastSLAMParams p;p.loop_cache_budget_mb=0;auto slam=Slam(p);
  auto h=std::get<2>(*slam->particles().begin());auto map=std::make_shared<Submap>(0,Pose(),240,240,.05);
  Scene(*map,Pose());map->finish();h->submaps.history.push_back(map);(void)map->loop_matching_data();
  const auto pixels=map->grid().data();slam->trim_derived_caches();
  EXPECT_EQ(map->loop_cache_statistics().first,0U);EXPECT_EQ(h->submaps.history.size(),1U);
  EXPECT_EQ(map->grid().data(),pixels);EXPECT_EQ(slam->backend_timing().loop_cache_bytes,0U);
}
