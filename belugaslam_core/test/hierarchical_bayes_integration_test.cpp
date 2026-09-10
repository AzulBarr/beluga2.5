#include <gtest/gtest.h>
#include "belugaslam_core/fastslam_oc_grid_core.hpp"

namespace {
state_type Pose(double x=0,double y=0,double yaw=0) {
  return {Sophus::SO2d{yaw},Eigen::Vector2d{x,y}};
}
struct Fixture {
  std::unique_ptr<BelugaSLAM> slam;
  std::shared_ptr<Hypothesis> parent,child;
  BelugaSLAM::LoopVerification report;
  BelugaSLAM::measurement_type scan;
  explicit Fixture(bool loop_correct=false,std::size_t capacity=4) {
    FastSLAMParams params;
    params.min_particles=5;params.max_particles=30;params.max_hypotheses=capacity;
    params.submap_num_range_data=100;params.split_persistence=100;
    params.enable_loop_closure=false;params.enable_pgo=false;
    slam=std::make_unique<BelugaSLAM>(
      BelugaSLAM::MotionModel{beluga::DifferentialDriveModelParam{.1,.1,.1,.1}},
      BelugaSLAM::MeasurementModel{beluga::LikelihoodFieldProbModelParam{100,2,.5,.5,.2,true},GridTypeOC{}},params);
    parent=std::get<2>(*slam->particles().begin());
    const double initial_x=loop_correct ? .6 : 0;
    parent->has_local_pose=true;parent->local_pose=Pose(initial_x);
    auto reference=std::make_shared<Submap>(0,Pose(),100,80,.1);
    auto& cells=reference->mutable_grid().data();std::fill(cells.begin(),cells.end(),-1.f);
    for(int y=0;y<80;++y) cells[y*100+70]=5;
    reference->set_anchor_sequence(0);reference->finish();
    parent->submaps.history.push_back(reference);
    auto active=std::make_shared<Submap>(1,Pose(initial_x),100,80,.1);
    active->mutable_grid().data()=cells;
    parent->submaps.active_submaps.push_back(active);parent->submaps.next_submap_id=2;
    for(auto&& p : slam->particles()) std::get<0>(p)=parent->local_pose;
    for(int y=25;y<55;++y) scan.emplace_back(2.05,-4.+(y+.5)*.1);
    child=std::make_shared<Hypothesis>(*parent);
    const auto correction=Pose(loop_correct ? -.6 : .6);
    child->T_global_local=correction;child->local_pose=correction*parent->local_pose;
    child->submaps.active_submaps.front()=active->clone_for_pose();
    child->submaps.active_submaps.front()->set_global_pose(correction*active->global_pose());
    report.candidate.reference_sequence=0;report.candidate.query_sequence=1;
    report.trials.resize(1);auto& trial=report.trials.front();
    trial.usable=true;trial.compatibility=.2;trial.hypothesis=child;
  }
  bool begin() { return slam->begin_bayesian_loop_event(report,{parent},{1.},7); }
  void advance(const BelugaSLAM::measurement_type& z,int sequence) {
    slam->sample_motion_model({Pose(),Pose()});slam->measurement_model_map(z);
    slam->update_occupancy_grid(z,sequence);slam->resample();
  }
};
void CheckNormalized(const BelugaSLAM& slam) {
  double joint=0;std::map<std::size_t,double> conditional;
  for(const auto& p : slam.particles()) {
    EXPECT_DOUBLE_EQ(static_cast<double>(std::get<1>(p)),std::exp(std::get<3>(p)));
    conditional[std::get<2>(p)->id]+=std::exp(std::get<3>(p));
    joint+=joint_particle_weight(p);
  }
  EXPECT_NEAR(joint,1.,1e-12);
  for(const auto& [id,mass] : conditional) EXPECT_NEAR(mass,1.,1e-12);
}
}

TEST(HierarchicalBayesIntegration, BranchPriorsIgnoreDeformationMultiplier) {
  Fixture f;ASSERT_TRUE(f.begin());
  EXPECT_NEAR(f.slam->hypothesis_masses().at(f.parent->id),.5,1e-12);
  EXPECT_NEAR(f.slam->hypothesis_masses().at(f.child->id),.5,1e-12);
  EXPECT_EQ(f.parent->validation_age,0U);EXPECT_EQ(f.child->validation_age,0U);
  EXPECT_TRUE(f.slam->bayesian_loop_pending());CheckNormalized(*f.slam);
}

TEST(HierarchicalBayesIntegration, RejectsWrongLoopOnlyAfterTenFutureScans) {
  Fixture f;ASSERT_TRUE(f.begin());
  for(int i=0;i<10;++i) {
    f.advance(f.scan,i);CheckNormalized(*f.slam);
    if(i<9) {EXPECT_TRUE(f.slam->bayesian_loop_pending());EXPECT_EQ(f.slam->get_active_hypotheses_count(),2U);}
  }
  EXPECT_EQ(f.slam->last_bayesian_loop_status(),"rejected");
  EXPECT_LT(f.slam->last_bayesian_loop_probability(),.05);
  EXPECT_EQ(f.slam->get_active_hypotheses_count(),1U);
  EXPECT_EQ(std::get<2>(*f.slam->particles().begin()),f.parent);
  EXPECT_EQ(f.slam->particles().size(),30U);
  EXPECT_FALSE(f.parent->validation_map);
}

TEST(HierarchicalBayesIntegration, AcceptsSupportedLoopOnlyAfterTenFutureScans) {
  Fixture f(true);ASSERT_TRUE(f.begin());
  for(int i=0;i<10;++i) f.advance(f.scan,i);
  EXPECT_EQ(f.slam->last_bayesian_loop_status(),"accepted");
  EXPECT_GT(f.slam->last_bayesian_loop_probability(),.95);
  EXPECT_EQ(std::get<2>(*f.slam->particles().begin()),f.child);
  EXPECT_EQ(f.slam->particles().size(),30U);CheckNormalized(*f.slam);
}

TEST(HierarchicalBayesIntegration, ValidationOwnsItsPixelsAndWorldPose) {
  Fixture f;ASSERT_TRUE(f.begin());
  const auto frozen=f.parent->validation_map;
  belugaslam::TrackingOptions options;
  const double before=frozen->log_likelihood(f.scan,{0,0,0},options,.1);
  auto& active=f.parent->submaps.active_submaps.front();
  std::fill(active->mutable_grid().data().begin(),active->mutable_grid().data().end(),0.f);
  f.parent->submaps.history.front()->set_global_pose(Pose(100));
  EXPECT_DOUBLE_EQ(before,frozen->log_likelihood(f.scan,{0,0,0},options,.1));
}

TEST(HierarchicalBayesIntegration, MissingCoverageTimesOutWithoutManufacturingEvidence) {
  Fixture f;ASSERT_TRUE(f.begin());const auto masses=f.slam->hypothesis_masses();
  BelugaSLAM::measurement_type unknown(30,{100.,100.});
  for(int i=0;i<30;++i) f.advance(unknown,i);
  EXPECT_EQ(f.slam->last_bayesian_loop_status(),"undecided");
  EXPECT_EQ(f.parent->validation_age,0U);EXPECT_EQ(f.child->validation_age,0U);
  EXPECT_EQ(f.slam->get_active_hypotheses_count(),2U);
  for(const auto& [id,mass] : masses) EXPECT_NEAR(f.slam->hypothesis_masses().at(id),mass,1e-12);
  f.advance(f.scan,31);
  for(const auto& [id,mass] : masses) EXPECT_NEAR(f.slam->hypothesis_masses().at(id),mass,1e-12);
  EXPECT_FALSE(f.parent->validation_map);CheckNormalized(*f.slam);
}

TEST(HierarchicalBayesIntegration, DuplicateMeasurementCannotCountAsFutureEvidenceTwice) {
  Fixture f;ASSERT_TRUE(f.begin());f.slam->sample_motion_model({Pose(),Pose()});
  f.slam->measurement_model_map(f.scan);const auto masses=f.slam->hypothesis_masses();
  f.slam->measurement_model_map(f.scan);
  EXPECT_EQ(f.parent->validation_age,1U);EXPECT_EQ(f.slam->hypothesis_masses(),masses);
}

TEST(HierarchicalBayesIntegration, BudgetDeferralDoesNotDropParentOrConsumeCandidate) {
  Fixture f(false,1);EXPECT_FALSE(f.begin());
  EXPECT_EQ(f.slam->last_bayesian_loop_status(),"bayes_deferred_budget");
  EXPECT_EQ(f.slam->get_active_hypotheses_count(),1U);
  EXPECT_FALSE(f.parent->validation_map);EXPECT_FALSE(f.report.selected);
  CheckNormalized(*f.slam);
}

TEST(HierarchicalBayesIntegration, NoHistoricalReferenceDefersWithoutMutation) {
  Fixture f;f.parent->submaps.history.clear();
  EXPECT_FALSE(f.begin());EXPECT_EQ(f.slam->last_bayesian_loop_status(),"bayes_deferred_reference");
  EXPECT_FALSE(f.parent->validation_map);EXPECT_EQ(f.slam->get_active_hypotheses_count(),1U);
}

TEST(HierarchicalBayesIntegration, ResamplingPreservesSubnormalGraphLogMass) {
  Fixture f;ASSERT_TRUE(f.begin());f.parent->log_mass=0;f.child->log_mass=-2000;
  f.slam->resample();
  EXPECT_DOUBLE_EQ(f.child->log_mass,-2000.);EXPECT_EQ(f.slam->get_active_hypotheses_count(),2U);
  CheckNormalized(*f.slam);
}

TEST(HierarchicalBayesIntegration, MultipleParentMassesAreConservedAtBranchCreation) {
  Fixture f;
  auto other=std::make_shared<Hypothesis>(*f.parent);other->id=99;
  f.slam->install_population({{f.parent,f.parent,.8,Pose(),true},{other,f.parent,.2,Pose(),true}},30);
  auto other_child=std::make_shared<Hypothesis>(*f.child);
  f.report.trials.push_back(f.report.trials.front());f.report.trials.back().hypothesis=other_child;
  ASSERT_TRUE(f.slam->begin_bayesian_loop_event(f.report,{f.parent,other},{.8,.2},7));
  const auto masses=f.slam->hypothesis_masses();
  EXPECT_NEAR(masses.at(f.parent->id)+masses.at(f.child->id),.8,1e-12);
  EXPECT_NEAR(masses.at(other->id)+masses.at(other_child->id),.2,1e-12);
  EXPECT_EQ(f.slam->get_active_hypotheses_count(),4U);CheckNormalized(*f.slam);
}
