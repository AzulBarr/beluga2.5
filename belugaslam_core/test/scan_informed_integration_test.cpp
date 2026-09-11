#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {
state_type SIPose(double x=0,double y=0,double yaw=0) {
  return {Sophus::SO2d{yaw},Eigen::Vector2d{x,y}};
}
struct ScanFixture {
  FastSLAMParams params;
  beluga::DifferentialDriveModelParam noise{.1,.05,.1,.05,.01};
  std::unique_ptr<BelugaSLAM> slam;
  std::shared_ptr<Hypothesis> h;
  BelugaSLAM::measurement_type scan;
  explicit ScanFixture(bool enabled=true,bool deterministic=false,const state_type& world=SIPose()) {
    params.min_particles=5;params.max_particles=30;params.max_hypotheses=4;
    params.motion_proposal_samples=1;params.worker_threads=1;
    params.scan_informed_proposal=enabled;params.tracking_matcher="distance";
    params.enable_loop_closure=false;params.enable_pgo=false;
    params.recovery.enabled=false;params.split_persistence=100;
    if(deterministic) noise={0,0,0,0,.01};
    slam=std::make_unique<BelugaSLAM>(BelugaSLAM::MotionModel{noise},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params);
    h=std::get<2>(*slam->particles().begin());h->has_local_pose=true;h->local_pose=world;
    auto map=std::make_shared<Submap>(0,world,160,160,.05);
    std::fill(map->mutable_grid().data().begin(),map->mutable_grid().data().end(),-1.f);
    for(int y=30;y<130;++y) {
      map->mutable_grid().at(120,y)=5;
      scan.emplace_back(2.025-.24,-4+(y+.5)*.05);
    }
    h->submaps.active_submaps.push_back(map);h->submaps.next_submap_id=1;
    for(auto&& p:slam->particles()) std::get<0>(p)=world;
  }
};
}

TEST(ScanInformedIntegration, CorrectsEachParticleAndHypothesisEvidence) {
  ScanFixture f;
  const auto other=std::make_shared<Hypothesis>(*f.h);other->id=99;
  f.slam->install_population({{f.h,f.h,.7,SIPose(),true},{other,f.h,.3,SIPose(),true}},30);
  const auto masses=f.slam->hypothesis_masses();
  std::vector<state_type> ancestors;std::vector<double> priors;
  for(const auto& p:f.slam->particles()) {ancestors.push_back(std::get<0>(p));priors.push_back(std::get<3>(p));}
  const BelugaSLAM::control_type u{SIPose(.2,0,.02),SIPose()};
  const auto density=BelugaSLAM::MotionModel{f.noise}.increment_distribution(u);
  f.slam->sample_motion_model(u);f.slam->measurement_model_map(f.scan);
  const auto sparse=belugaslam::select_tracking_points(f.scan,f.params.tracking.max_points);
  std::map<std::size_t,std::vector<double>> logs;
  std::map<std::size_t,std::vector<std::size_t>> members;
  for(std::size_t i=0;i<ancestors.size();++i) {
    const auto& p=*(f.slam->particles().begin()+i);const auto& h=std::get<2>(p);
    ASSERT_EQ(h->scan_proposal_status,"mixture");
    const auto delta=ancestors[i].inverse()*std::get<0>(p);
    const auto frontend=h->local_pose; // both hypotheses started at identity
    const belugaslam::ScanInformedProposal q{density,
      {frontend.translation().x(),frontend.translation().y(),frontend.so2().log()},f.params.scan_proposal};
    const auto relative=h->submaps.matching_submap()->global_pose().inverse()*std::get<0>(p);
    const double likelihood=f.params.tracking.effective_beams*belugaslam::tracking_score(
      *h->submaps.matching_submap()->tracking_field(),sparse,
      {relative.translation().x(),relative.translation().y(),relative.so2().log()},f.params.tracking).mean_log_likelihood;
    logs[h->id].push_back(priors[i]+likelihood+q.log_importance_ratio(
      {delta.translation().x(),delta.translation().y(),delta.so2().log()}));
    members[h->id].push_back(i);
  }
  std::vector<double> graph_logs;
  for(auto& [id,values]:logs) {
    const double evidence=belugaslam::normalize_log_weights(values);
    graph_logs.push_back(std::log(masses.at(id))+evidence);
    const auto h=std::get<2>(*(f.slam->particles().begin()+members[id][0]));
    EXPECT_NEAR(h->predictive_log_evidence,evidence,1e-9);
    EXPECT_GT(h->scan_proposal_frontend_count,0U);
    EXPECT_EQ(h->submaps.active_submaps.front()->num_insertions(),0);
    for(std::size_t j=0;j<values.size();++j) {
      const auto& p=*(f.slam->particles().begin()+members[id][j]);
      EXPECT_NEAR(std::get<3>(p),values[j],1e-9);
    }
  }
  belugaslam::normalize_log_weights(graph_logs);std::size_t j=0;
  for(const auto& [id,values]:logs) EXPECT_NEAR(f.slam->hypothesis_masses().at(id),std::exp(graph_logs[j++]),1e-10);
  f.slam->resample();double joint=0;
  for(const auto& p:f.slam->particles()) joint+=joint_particle_weight(p);
  EXPECT_NEAR(joint,1.,1e-12);
}

TEST(ScanInformedIntegration, RigidGraphFrameChangeDoesNotBecomeMotion) {
  ScanFixture a,b(true,false,SIPose(8,-3,.7));
  const BelugaSLAM::control_type u{SIPose(.2,0,.02),SIPose()};
  a.slam->sample_motion_model(u);a.slam->measurement_model_map(a.scan);
  b.slam->sample_motion_model(u);b.slam->measurement_model_map(b.scan);
  ASSERT_EQ(a.h->scan_proposal_status,"mixture");ASSERT_EQ(b.h->scan_proposal_status,"mixture");
  const auto world=SIPose(8,-3,.7);
  EXPECT_NEAR((a.h->local_pose.inverse()*world.inverse()*b.h->local_pose).translation().norm(),0,1e-8);
  for(std::size_t i=0;i<a.slam->particles().size();++i) {
    const auto& p=*(a.slam->particles().begin()+i);const auto& q=*(b.slam->particles().begin()+i);
    EXPECT_NEAR((std::get<0>(p).inverse()*world.inverse()*std::get<0>(q)).translation().norm(),0,1e-8);
    EXPECT_NEAR(std::get<3>(p),std::get<3>(q),1e-8);
  }
}

TEST(ScanInformedIntegration, StationaryAndZeroNoiseExactlyRetainBootstrap) {
  for(bool deterministic:{false,true}) {
    ScanFixture on(true,deterministic),off(false,deterministic);
    const BelugaSLAM::control_type u{deterministic?SIPose(.2):SIPose(),SIPose()};
    on.slam->sample_motion_model(u);off.slam->sample_motion_model(u);
    on.slam->measurement_model_map(on.scan);off.slam->measurement_model_map(off.scan);
    EXPECT_EQ(on.h->scan_proposal_status,"singular_motion");
    EXPECT_EQ(on.h->scan_proposal_frontend_count,0U);
    for(std::size_t i=0;i<on.slam->particles().size();++i) {
      const auto& a=*(on.slam->particles().begin()+i);const auto& b=*(off.slam->particles().begin()+i);
      EXPECT_DOUBLE_EQ(std::get<3>(a),std::get<3>(b));
      EXPECT_LT((std::get<0>(a).inverse()*std::get<0>(b)).translation().norm(),1e-12);
    }
  }
}

TEST(ScanInformedIntegration, RejectedTrackingUsesOriginalProposals) {
  ScanFixture f;f.scan.assign(30,{100,100});
  f.slam->sample_motion_model({SIPose(.2),SIPose()});f.slam->measurement_model_map(f.scan);
  EXPECT_EQ(f.h->scan_proposal_status,"frontend_unavailable");
  EXPECT_EQ(f.h->scan_proposal_frontend_count,0U);
  EXPECT_DOUBLE_EQ(f.h->scan_proposal_log_ratio_min,0);
  EXPECT_DOUBLE_EQ(f.h->scan_proposal_log_ratio_max,0);
}

TEST(ScanInformedIntegration, MissingLoopCoverageHasNoArtificialEvidence) {
  ScanFixture f;
  auto history=f.h->submaps.active_submaps.front()->clone_for_pose();history->finish();
  history->set_anchor_sequence(0);f.h->submaps.history.push_back(history);
  auto child=std::make_shared<Hypothesis>(*f.h);
  BelugaSLAM::LoopVerification report;report.candidate.reference_sequence=0;report.candidate.query_sequence=1;
  report.trials.resize(1);report.trials[0].usable=true;report.trials[0].compatibility=.2;report.trials[0].hypothesis=child;
  ASSERT_TRUE(f.slam->begin_bayesian_loop_event(report,{f.h},{1.},1));
  const auto masses=f.slam->hypothesis_masses();
  const BelugaSLAM::measurement_type unknown(30,{100,100});
  f.slam->sample_motion_model({SIPose(.2),SIPose()});f.slam->measurement_model_map(unknown);
  for(const auto& [id,mass]:masses) EXPECT_DOUBLE_EQ(f.slam->hypothesis_masses().at(id),mass);
  EXPECT_EQ(f.h->scan_proposal_status,"missing_evidence");
  EXPECT_EQ(f.h->scan_proposal_frontend_count,0U);
  EXPECT_DOUBLE_EQ(f.h->predictive_log_evidence,0);
}

TEST(ScanInformedIntegration, FrozenLoopLikelihoodRetainsImportanceCorrection) {
  ScanFixture f;
  // The validation map disagrees with the live map: this catches accidental
  // reuse of the live likelihood or loss of p/q when switching sensor models.
  auto history=std::make_shared<Submap>(8,SIPose(),160,160,.05);
  std::fill(history->mutable_grid().data().begin(),history->mutable_grid().data().end(),-1.f);
  for(int y=30;y<130;++y) history->mutable_grid().at(126,y)=5;
  history->finish();history->set_anchor_sequence(0);f.h->submaps.history.push_back(history);
  auto child=std::make_shared<Hypothesis>(*f.h);
  BelugaSLAM::LoopVerification report;report.candidate.reference_sequence=0;report.candidate.query_sequence=1;
  report.trials.resize(1);report.trials[0].usable=true;report.trials[0].compatibility=.2;report.trials[0].hypothesis=child;
  ASSERT_TRUE(f.slam->begin_bayesian_loop_event(report,{f.h},{1.},1));
  std::vector<state_type> ancestors;std::vector<double> priors;
  for(const auto& p:f.slam->particles()) {ancestors.push_back(std::get<0>(p));priors.push_back(std::get<3>(p));}
  const BelugaSLAM::control_type u{SIPose(.2),SIPose()};
  const auto density=BelugaSLAM::MotionModel{f.noise}.increment_distribution(u);
  f.slam->sample_motion_model(u);f.slam->measurement_model_map(f.scan);
  const auto common=belugaslam::select_tracking_points(f.scan,f.params.tracking.max_points);
  std::map<std::size_t,std::vector<double>> logs;
  std::map<std::size_t,std::vector<std::size_t>> members;
  for(std::size_t i=0;i<ancestors.size();++i) {
    const auto& p=*(f.slam->particles().begin()+i);const auto& h=std::get<2>(p);
    ASSERT_EQ(h->scan_proposal_status,"mixture");ASSERT_EQ(h->validation_age,1U);
    const auto delta=ancestors[i].inverse()*std::get<0>(p);
    const auto frontend=h->local_pose;
    const belugaslam::ScanInformedProposal q{density,
      {frontend.translation().x(),frontend.translation().y(),frontend.so2().log()},f.params.scan_proposal};
    const auto pose=std::get<0>(p);
    const double l=h->validation_map->log_likelihood(common,
      {pose.translation().x(),pose.translation().y(),pose.so2().log()},f.params.tracking,f.params.loop_bayes.beta);
    logs[h->id].push_back(priors[i]+l+q.log_importance_ratio(
      {delta.translation().x(),delta.translation().y(),delta.so2().log()}));
    members[h->id].push_back(i);
  }
  for(auto& [id,values]:logs) {
    const double evidence=belugaslam::normalize_log_weights(values);
    const auto h=std::get<2>(*(f.slam->particles().begin()+members[id][0]));
    EXPECT_NEAR(h->predictive_log_evidence,evidence,1e-9);
    for(std::size_t j=0;j<values.size();++j)
      EXPECT_NEAR(std::get<3>(*(f.slam->particles().begin()+members[id][j])),values[j],1e-9);
  }
}
