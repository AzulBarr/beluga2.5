#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"
namespace {
Sophus::SE2d Pose(double x=0,double y=0,double a=0) {return {Sophus::SO2d{a},Eigen::Vector2d{x,y}};}
std::unique_ptr<BelugaSLAM> Slam(FastSLAMParams params={}) {
  params.enable_loop_closure=false;params.enable_pgo=false;
  return std::make_unique<BelugaSLAM>(BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.05,.1,.05}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params);
}
}
TEST(ProbabilityFrontend, RejectsUnknownModeAndInvalidOptions) {
  FastSLAMParams p;p.tracking_matcher="typo";
  EXPECT_THROW(Slam(p),std::invalid_argument);
  p.tracking_matcher="probability_ceres";p.probability_matching.voxel_size=-.1;
  EXPECT_THROW(Slam(p),std::invalid_argument);
  p.probability_matching.voxel_size=.05;p.probability_matching.occupied_space_weight=0;
  EXPECT_THROW(Slam(p),std::invalid_argument);
}
TEST(ProbabilityFrontend, RejectsInvalidMotionPriorConfiguration) {
  FastSLAMParams p;p.tracking_prior_mode="wrong";
  EXPECT_THROW(Slam(p),std::invalid_argument);
  p.tracking_prior_mode="odometry";p.odometry_prior.translation_sigma=0;
  EXPECT_THROW(Slam(p),std::invalid_argument);
}
TEST(ProbabilityFrontend, MotionPriorUsesPreviousHeadingInMatchingFrame) {
  FastSLAMParams p;p.tracking_prior_mode="odometry";p.tracking_matcher="probability_ceres";
  auto slam=Slam(p);
  slam->sample_motion_model({Pose(.2,.02,.05),Pose()});
  const belugaslam::PoseSample2 predicted{1.,2.,.75};
  const auto expected=belugaslam::odometry_tracking_prior({.2,.02,.05},.70,p.odometry_prior);
  const auto actual=slam->frontend_motion_prior(predicted);
  for(int i=0;i<9;++i)EXPECT_NEAR(actual.covariance[i],expected.covariance[i],1e-12);
  const auto configured=slam->frontend_tracking_options(predicted,p.tracking);
  EXPECT_TRUE(configured.use_full_prior);EXPECT_FALSE(p.tracking.use_full_prior);
  for(int i=0;i<9;++i)EXPECT_NEAR(configured.prior_sqrt_information[i],expected.sqrt_information[i],1e-12);
}
TEST(ProbabilityFrontend, CacheInvalidatesOnWriteAndCropAndDetachesAcrossBranches) {
  Submap a(0,Pose(),80,80,.1);
  a.mutable_grid().at(40,40)=5;
  const auto old=a.probability_field();
  auto b=a.clone_for_pose();
  ASSERT_EQ(b->probability_field(),old);
  b->mutable_grid().at(50,40)=5;
  const auto changed=b->probability_field();
  EXPECT_NE(changed,old);EXPECT_EQ(a.probability_field(),old);
  EXPECT_NEAR(changed->sample(1.05,.05).probability,.9,1e-12);
  EXPECT_NEAR(old->sample(1.05,.05).probability,.1,1e-12);
  b->finish();
  EXPECT_NE(b->probability_field(),changed);
  EXPECT_NEAR(b->probability_field()->sample(1.05,.05).probability,.9,1e-12);
  const auto frozen=b->probability_field();
  b->release_tracking_field();
  EXPECT_NE(b->probability_field(),frozen);
  EXPECT_NEAR(frozen->sample(1.05,.05).probability,.9,1e-12);
}
TEST(ProbabilityFrontend, SameMapPFUpdateUnchangedAndFrontendFeedsGraph) {
  std::vector<double> baseline_weights;
  for(const std::string mode:{"distance","probability_ceres"}) {
    FastSLAMParams p;p.tracking_matcher=mode;p.frontend_pose_mode="frontend";
    p.max_particles=30;p.min_particles=5;p.recovery.enabled=false;
    auto slam=Slam(p);auto h=std::get<2>(*slam->particles().begin());
    auto map=std::make_shared<Submap>(0,Pose(),160,160,.05);
    const auto truth=Pose(.14,-.09,.023);
    BelugaSLAM::measurement_type scan;
    auto& grid=map->mutable_grid();
    for(int i=30;i<130;++i) {
      grid.at(125,i)=5;grid.at(i,130)=5;
      for(const auto& q:belugaslam::ScanPoints{{-4+125.5*.05,-4+(i+.5)*.05},
                                            {-4+(i+.5)*.05,-4+130.5*.05}}) {
        const auto z=truth.inverse()*Eigen::Vector2d{q.first,q.second};
        scan.emplace_back(z.x(),z.y());
      }
    }
    h->submaps.active_submaps.push_back(map);h->has_local_pose=true;
    slam->sample_motion_model({Pose(),Pose()});slam->measurement_model_map(scan);
    ASSERT_TRUE(h->tracking_usable);
    std::vector<double> weights;
    for(const auto& particle:slam->particles()) weights.push_back(std::get<3>(particle));
    if(mode=="distance") baseline_weights=weights;
    else {
      EXPECT_EQ(weights,baseline_weights); // exactly one unchanged PF likelihood update
      EXPECT_LT((h->local_pose.translation()-truth.translation()).norm(),.03);
      const auto frontend=h->local_pose;
      // post_update is the step that refreshes the published pose; without it
      // best_pose() is still the constructor's identity, not a stale estimate.
      slam->post_update(scan,slam->update_occupancy_grid(scan,1.,1000000000));
      ASSERT_FALSE(h->submaps.trajectory_nodes.empty());
      EXPECT_LT((frontend.inverse()*h->submaps.trajectory_nodes.back().global_pose).translation().norm(),1e-12);
      EXPECT_LT((frontend.inverse()*slam->best_pose()).translation().norm(),1e-12);
    }
  }
}
TEST(ProbabilityFrontend, AdaptivePriorPreservesBootstrapPFUpdateAndFeedsGraph) {
  std::vector<double> fixed_weights;
  for(const std::string mode:{"fixed","odometry"}) {
    FastSLAMParams p;p.tracking_prior_mode=mode;p.tracking_matcher="probability_ceres";
    // This independence is a bootstrap invariant. With a scan-informed q,
    // changing the frontend prior deliberately changes proposals and p/q.
    p.scan_informed_proposal=false;
    p.frontend_pose_mode="frontend";p.max_particles=30;p.min_particles=5;p.recovery.enabled=false;
    auto slam=Slam(p);auto h=std::get<2>(*slam->particles().begin());
    auto map=std::make_shared<Submap>(0,Pose(),160,160,.05);
    const auto truth=Pose(.14,-.09,.023);
    BelugaSLAM::measurement_type scan;auto& grid=map->mutable_grid();
    for(int i=30;i<130;++i) {
      grid.at(125,i)=5;grid.at(i,130)=5;
      for(const auto& q:belugaslam::ScanPoints{{-4+125.5*.05,-4+(i+.5)*.05},
                                            {-4+(i+.5)*.05,-4+130.5*.05}}) {
        const auto z=truth.inverse()*Eigen::Vector2d{q.first,q.second};scan.emplace_back(z.x(),z.y());
      }
    }
    h->submaps.active_submaps.push_back(map);h->has_local_pose=true;
    slam->sample_motion_model({Pose(.08,0,.01),Pose()});slam->measurement_model_map(scan);
    ASSERT_TRUE(h->tracking_usable);ASSERT_TRUE(h->tracking_prior_evaluated);
    std::vector<double> weights;for(const auto& particle:slam->particles())weights.push_back(std::get<3>(particle));
    if(mode=="fixed")fixed_weights=weights;
    else {
      EXPECT_EQ(weights,fixed_weights);
      EXPECT_GT(h->tracking_prior_covariance[5],0);
      EXPECT_LT((h->local_pose.translation()-truth.translation()).norm(),.03);
      const auto pose=h->local_pose;
      slam->update_occupancy_grid(scan,1.,1000000000);
      ASSERT_FALSE(h->submaps.trajectory_nodes.empty());
      EXPECT_LT((pose.inverse()*h->submaps.trajectory_nodes.back().global_pose).translation().norm(),1e-12);
    }
  }
}
