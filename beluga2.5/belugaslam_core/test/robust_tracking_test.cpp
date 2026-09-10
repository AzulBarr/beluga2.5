#include "belugaslam_core/robust_tracking.hpp"
#include "belugaslam_core/particle_proposal.hpp"
#include <iostream>
#include <random>
#include <stdexcept>
using namespace belugaslam;
int checks=0;
void check(bool yes,const char* text) {++checks;if(!yes) throw std::runtime_error(text);}
int main() {
  constexpr int width=240,height=240;constexpr double res=0.05,origin=-6;
  std::vector<float> cells(width*height);
  ScanPoints world;
  // Two perpendicular walls and a short asymmetric wall provide observable pose.
  for (int k=20;k<180;++k) {
    cells[k*width+180]=5;
    cells[190*width+k]=5;
    world.emplace_back(origin+(180.5)*res,origin+(k+0.5)*res);
    world.emplace_back(origin+(k+0.5)*res,origin+(190.5)*res);
  }
  for (int k=80;k<110;++k) cells[k*width+60]=5;
  TrackingField field(cells,width,height,res,origin,origin);
  TrackingOptions o;
  const PoseSample2 truth{0.21,-0.13,0.035};
  ScanPoints scan;
  const double c=std::cos(truth.yaw),s=std::sin(truth.yaw);
  for (const auto& [x,y]:world) {
    const double dx=x-truth.x,dy=y-truth.y;
    scan.emplace_back(c*dx+s*dy,-s*dx+c*dy);
  }
  auto result=match_tracking_scan(field,scan,{0,0,0},o);
  std::cout<<"Recovered pose "<<result.pose.x<<' '<<result.pose.y<<' '<<result.pose.yaw<<" overlap "<<result.score.overlap<<'\n';
  check(result.accepted,"observable match rejected");
  check(result.final_cost<result.initial_cost,"objective did not improve");
  check(std::hypot(result.pose.x-truth.x,result.pose.y-truth.y)<0.09,"translation not recovered");
  check(std::abs(result.pose.yaw-truth.yaw)<0.015,"rotation not recovered");
  // Outliers contribute bounded mixture loss, rather than dominating alignment.
  std::mt19937 rng(42);
  auto dirty=scan;
  for (int i=0;i<60;++i) dirty.emplace_back(static_cast<int>(rng()%200)/10.0-10,static_cast<int>(rng()%200)/10.0-10);
  auto noisy=match_tracking_scan(field,dirty,{0,0,0},o);
  check(noisy.accepted,"robust match rejected");
  check(std::hypot(noisy.pose.x-truth.x,noisy.pose.y-truth.y)<0.10,"outliers moved match excessively");
  auto lost=match_tracking_scan(field,{{100,100},{101,101},{102,102}},{0,0,0},o);
  check(!lost.accepted,"unsupported scan accepted");
  check(lost.pose.x==0&&lost.pose.y==0,"failure must retain odometry prediction");
  // Force rejection after the optimizer found a different pose. The returned
  // likelihood/overlap must refer to the retained prediction, not that candidate.
  auto strict=o;strict.min_points=scan.size()+1;
  const PoseSample2 rejected_prior{-.15,.10,-.02};
  const auto rejected=match_tracking_scan(field,scan,rejected_prior,strict);
  const auto prior_score=tracking_score(field,scan,rejected_prior,strict);
  check(!rejected.accepted,"strict inlier count must reject");
  check(rejected.pose.x==rejected_prior.x&&rejected.pose.y==rejected_prior.y&&rejected.pose.yaw==rejected_prior.yaw,
        "rejected result pose must be prediction");
  check(std::abs(rejected.score.mean_log_likelihood-prior_score.mean_log_likelihood)<1e-12,
        "rejected likelihood describes discarded optimizer pose");
  check(rejected.score.inliers==prior_score.inliers&&rejected.score.overlap==prior_score.overlap,
        "rejected overlap describes discarded optimizer pose");
  check(rejected.final_cost==rejected.initial_cost,"rejected cost must describe returned prediction");
  auto sample=field.sample(2.91,1.12);
  const double epsilon=1e-5;
  check(std::abs(sample.dx-(field.sample(2.91+epsilon,1.12).distance-field.sample(2.91-epsilon,1.12).distance)/(2*epsilon))<1e-6,"field x gradient");
  check(std::abs(sample.dy-(field.sample(2.91,1.12+epsilon).distance-field.sample(2.91,1.12-epsilon).distance)/(2*epsilon))<1e-6,"field y gradient");
  std::vector<float> corridor_cells(width*height);
  for (int y=0;y<height;++y) {corridor_cells[y*width+80]=5;corridor_cells[y*width+160]=5;}
  TrackingField corridor(corridor_cells,width,height,res,origin,origin);
  ScanPoints hallway;
  for (int y=60;y<180;++y) {hallway.emplace_back(-1.975,origin+(y+.5)*res);hallway.emplace_back(2.025,origin+(y+.5)*res);}
  auto weak=match_tracking_scan(corridor,hallway,{0,0.4,0},o);
  check(weak.accepted,"corridor rejected");
  check(std::abs(weak.pose.y-0.4)<1e-8,"unobservable axis drifted from prior");
  auto selected=select_tracking_points(scan,180);
  check(selected.size()==180,"point budget");
  check(select_tracking_points({},180).empty(),"empty selection");
  // Stable log evidence and K=1 bootstrap identity.
  auto proposal=select_motion_proposal({-1000,-1001},rng);
  check(std::abs(proposal.log_evidence-(-1000+std::log((1+std::exp(-1.0))/2)))<1e-10,"proposal log mean");
  check(select_motion_proposal({-3.0},rng).log_evidence==-3.0,"bootstrap identity");
  auto indices=systematic_indices({0,0.1,0.9,0},100,rng);
  check(std::count(indices.begin(),indices.end(),1)==10,"systematic quota first bin");
  check(std::count(indices.begin(),indices.end(),2)==90,"systematic quota second bin");
  // Exhaustively average a binary two-proposal update including stochastic selection.
  // E[mean likelihood * test(selected)] = E_prior[likelihood * test].
  double estimate=0;
  for (int a=0;a<2;++a) for(int b=0;b<2;++b) {
    const double la=a?0.9:0.1,lb=b?0.9:0.1;
    const double mean=(la+lb)/2;
    estimate+=0.25*mean*(la*a+lb*b)/(la+lb);
  }
  check(std::abs(estimate-0.45)<1e-12,"multi-try importance identity");
  std::cout<<"PASS: "<<checks<<" robust tracking/proposal checks\n";
}
