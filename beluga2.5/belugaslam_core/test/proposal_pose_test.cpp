#include "belugaslam_core/proposal_pose.hpp"
#include "belugaslam_core/particle_proposal.hpp"
#include <iomanip>
#include <iostream>
#include <random>

using namespace belugaslam;
int checks=0;
void require(bool ok,const char* why) {++checks;if(!ok)throw std::runtime_error(why);}
bool near(double a,double b,double eps=1e-10) {return std::abs(a-b)<eps;}
ProposalPoseSummary summary(const std::vector<WeightedPoseProposal>& cloud,PoseSample2 ref={}) {
  return summarize_proposal_poses(cloud,normalized_proposal_weights(cloud),ref,.5,.25);
}
PoseSample2 transform(PoseSample2 p,PoseSample2 t) {
  return {t.x+std::cos(t.yaw)*p.x-std::sin(t.yaw)*p.y,
          t.y+std::sin(t.yaw)*p.x+std::cos(t.yaw)*p.y,wrap_angle(p.yaw+t.yaw)};
}
int main() {
  // Unequal ancestor masses AND unequal proposal counts. Each raw weight
  // includes 1/K; averaging unweighted ancestor means gives the wrong result.
  std::vector<WeightedPoseProposal> cloud{{{0,0,0},std::log(.6/2*1)},
      {{2,0,0},std::log(.6/2*3)},{{10,0,0},std::log(.4/1*2)}};
  auto weights=normalized_proposal_weights(cloud);auto s=summary(cloud);
  require(s.valid && near(s.mean.x,4.9),"ancestor/proposal importance weights");
  require(near(weights[0],.15)&&near(weights[1],.45)&&near(weights[2],.4),"normalized evidence");
  // Exact expectation over categorical selections, conditional on this cloud.
  require(near(.25*4+.75*5.2,s.mean.x),"Rao-Blackwell conditional mean");
  require(near(.25*std::pow(4-s.mean.x,2)+.75*std::pow(5.2-s.mean.x,2),.27),"removed selection variance");
  const auto about=proposal_second_moment(cloud,weights,{3,0,0});
  require(near(about[0],s.covariance[0]+std::pow(3-s.mean.x,2)),"squared-risk decomposition");
  for(auto& p:cloud)p.log_weight+=1000;
  require(near(summary(cloud).mean.x,s.mean.x),"log scale invariance");
  require(!summary({}).valid,"empty cloud");
  require(!summary({{{0,0,0},-INFINITY}}).valid,"zero cloud mass");
  require(normalized_proposal_weights({{{0,0,0},NAN}}).empty(),"NaN log weight");
  require(normalized_proposal_weights({{{0,0,0},INFINITY}}).empty(),"infinite log weight");
  require(normalized_proposal_weights({{{NAN,0,0},0}}).empty(),"nonfinite supported pose");
  require(summary({{{0,0,0},0},{{NAN,0,0},-INFINITY}}).valid,"zero-mass pose ignored");
  require(near(summary({{{1,0,0},-10000},{{3,0,0},-10000}}).mean.x,2),"likelihood underflow avoided");
  const auto pi=std::acos(-1.);
  s=summary({{{0,0,pi-.02},0},{{0,0,-pi+.02},0}},{0,0,pi});
  require(s.valid && near(std::abs(s.mean.yaw),pi),"circular mean at branch cut");
  require(near(s.covariance[8],.0004),"wrapped yaw covariance");
  require(!summary({{{0,0,pi/2},0},{{0,0,-pi/2},0}}).valid,"undefined mean heading");

  cloud={{{.01,.02,.005},0},{{-.03,-.01,-.01},std::log(2.)},{{.02,-.015,.006},std::log(3.)}};
  s=summary(cloud);const PoseSample2 gauge{18,-13,.6};
  auto moved=cloud;for(auto& p:moved)p.pose=transform(p.pose,gauge);
  const auto moved_summary=summary(moved,gauge);
  const auto expected=transform(s.mean,gauge);
  require(near(moved_summary.mean.x,expected.x)&&near(moved_summary.mean.y,expected.y)&&
          near(wrap_angle(moved_summary.mean.yaw-expected.yaw),0),"SE2 gauge equivariance");
  const double c=std::cos(gauge.yaw),sn=std::sin(gauge.yaw);
  const double J[3][3]={{c,-sn,0},{sn,c,0},{0,0,1}};
  for(int a=0;a<3;++a)for(int b=0;b<3;++b) {
    double value=0;for(int i=0;i<3;++i)for(int j=0;j<3;++j)value+=J[a][i]*s.covariance[3*i+j]*J[b][j];
    require(near(value,moved_summary.covariance[3*a+b]),"covariance frame transport");
  }

  std::vector<float> cells(160*160,0);ScanPoints scan;
  for(int y=20;y<140;++y) {cells[y*160+120]=5;scan.emplace_back(2.025,-4+(y+.5)*.05);}
  TrackingField field(cells,160,160,.05,-4,-4);TrackingOptions tracking;
  const auto frontend_score=tracking_score(field,scan,{},tracking);
  cloud.clear();for(int i=0;i<8;++i)cloud.push_back({{i%2?.001:-.001,0,0},0});
  s=summary(cloud);
  require(check_proposal_pose(s,field,scan,{},frontend_score,tracking).accepted,"concentrated mean passes scan gate");
  auto bad=s;bad.ess=1;
  require(std::string(check_proposal_pose(bad,field,scan,{},frontend_score,tracking).reason)=="low_proposal_ess","ESS gate");
  bad=s;bad.local_mass=.4;
  require(!check_proposal_pose(bad,field,scan,{},frontend_score,tracking).accepted,"mass concentration gate");
  bad=s;bad.position_std=1;
  require(!check_proposal_pose(bad,field,scan,{},frontend_score,tracking).accepted,"position spread gate");
  bad=s;bad.yaw_std=1;
  require(!check_proposal_pose(bad,field,scan,{},frontend_score,tracking).accepted,"heading spread gate");
  require(std::string(check_proposal_pose(s,field,scan,{1,0,0},frontend_score,tracking).reason)=="outside_motion_gate","original prediction retained");
  bad=s;bad.mean.x=.2;
  require(std::string(check_proposal_pose(bad,field,scan,{},frontend_score,tracking).reason)=="mean_scan_fit_failed","mean scan-fit gate");
  cloud.clear();for(int i=0;i<20;++i)cloud.push_back({{i%2?1.:-1.,0,0},0});
  s=summary(cloud);
  require(!check_proposal_pose(s,field,scan,{},frontend_score,tracking).accepted,"reject bimodal mean even when midpoint matches map");

  // Controlled Monte Carlo error relative to an ANALYTIC posterior mean, not
  // robot trajectory RMSE. Prior N(0,1), observation 1 with variance .25 => .8.
  std::mt19937 draws(42),selections(912);
  std::normal_distribution<double> normal(0,1);
  constexpr int repetitions=2000,K=8;
  double error30=0,error300=0,selected_error300=0;
  for(int r=0;r<repetitions;++r) {
    cloud.clear();double selected_numerator=0,selected_denominator=0;
    for(int n=0;n<300;++n) {
      std::vector<double> logs,poses;
      for(int k=0;k<K;++k) {
        const double x=normal(draws),l=-.5*(x-1)*(x-1)/.25;
        logs.push_back(l);poses.push_back(x);cloud.push_back({{x,0,0},l-std::log(double(K))});
      }
      const auto choice=select_motion_proposal(logs,selections);
      const double w=std::exp(choice.log_evidence);
      selected_numerator+=w*poses[choice.index];selected_denominator+=w;
      if(n==29)error30+=std::pow(summary(cloud).mean.x-.8,2);
    }
    error300+=std::pow(summary(cloud).mean.x-.8,2);
    selected_error300+=std::pow(selected_numerator/selected_denominator-.8,2);
  }
  const auto rmse30=std::sqrt(error30/repetitions),rmse300=std::sqrt(error300/repetitions);
  const auto selected_rmse=std::sqrt(selected_error300/repetitions);
  require(rmse300<.65*rmse30,"more particles reduce Monte Carlo mean error in controlled model");
  require(rmse300<selected_rmse,"integrating proposals removes categorical readout noise");
  std::cout<<std::setprecision(10)<<"PASS: "<<checks<<" proposal-pose checks; "<<repetitions<<" Gaussian cloud trials\n"
           <<"Monte Carlo mean-estimation RMSE (not SLAM): N30="<<rmse30<<" N300="<<rmse300
           <<" selected-only N300="<<selected_rmse<<"\n";
}
