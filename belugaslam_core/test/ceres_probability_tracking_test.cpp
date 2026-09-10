#include "belugaslam_core/ceres_probability_tracking.hpp"
#include <iostream>
#include <stdexcept>
using namespace belugaslam;
void check(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
int main() {
  constexpr int w=160;
  constexpr double res=.05, origin=-4;
  std::vector<float> cells(w*w);
  ScanPoints scan;
  const PoseSample2 truth{.14,-.09,.023}, prior{};
  const double c=std::cos(truth.yaw),s=std::sin(truth.yaw);
  for(int i=30;i<130;++i) {
    cells[i*w+125]=5; cells[130*w+i]=5;
    for(const auto& p:ScanPoints{{origin+125.5*res,origin+(i+.5)*res},
                                {origin+(i+.5)*res,origin+130.5*res}}) {
      const double dx=p.first-truth.x,dy=p.second-truth.y;
      scan.emplace_back(c*dx+s*dy,-s*dx+c*dy);
    }
  }
  ProbabilityField probability(cells,w,w,res,origin,origin);
  TrackingField distance(cells,w,w,res,origin,origin);
  TrackingOptions tracking; ProbabilityMatchingOptions options;
  const auto result=match_probability_scan(probability,distance,scan,prior,tracking,options);
  check(result.accepted,"Ceres match rejected");
  check(result.final_cost<result.initial_cost,"Ceres objective did not improve");
  check(std::hypot(result.pose.x-truth.x,result.pose.y-truth.y)<.03,"Ceres translation error");
  check(std::abs(wrap_angle(result.pose.yaw-truth.yaw))<.008,"Ceres yaw error");
  const auto again=match_probability_scan(probability,distance,scan,prior,tracking,options);
  check(again.pose.x==result.pose.x && again.pose.y==result.pose.y && again.pose.yaw==result.pose.yaw,"non-deterministic Ceres solve");
  tracking.min_points=scan.size()+1;
  const auto rejected=match_probability_scan(probability,distance,scan,prior,tracking,options);
  check(!rejected.accepted && rejected.pose.x==prior.x && rejected.pose.y==prior.y &&
        rejected.final_cost==rejected.initial_cost,"rejection did not retain prediction");
  const auto score=tracking_score(distance,scan,prior,tracking);
  check(rejected.score.mean_log_likelihood==score.mean_log_likelihood,"rejected score describes discarded pose");
  std::cout<<"PASS: native C++ Ceres adapter matching, determinism and rejection\n";
}
