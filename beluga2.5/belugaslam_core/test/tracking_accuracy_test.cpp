#include "belugaslam_core/robust_tracking.hpp"
#include <iostream>
#include <random>
#include <stdexcept>

int main() {
  using namespace belugaslam;
  constexpr int w=240;constexpr double r=.05,origin=-6;
  std::vector<float> cells(w*w);
  ScanPoints world;
  for(int k=30;k<170;++k) {
    cells[k*w+180]=5;cells[190*w+k]=5;
    world.emplace_back(origin+180.5*r,origin+(k+.5)*r);
    world.emplace_back(origin+(k+.5)*r,origin+190.5*r);
  }
  TrackingField field(cells,w,w,r,origin,origin);
  TrackingOptions options;
  options.prior_information_scale=1/options.effective_beams;
  std::mt19937 generator(20260909);
  std::normal_distribution<double> noise(0,.005);
  double sum=0,max_error=0;
  for(int trial=0;trial<40;++trial) {
    const PoseSample2 truth{.21*std::cos(trial),.19*std::sin(trial),.04*std::cos(.7*trial)};
    ScanPoints scan;
    const double c=std::cos(truth.yaw),s=std::sin(truth.yaw);
    for(const auto& [x,y]:world) {
      const double dx=x-truth.x,dy=y-truth.y;
      scan.emplace_back(c*dx+s*dy+noise(generator),-s*dx+c*dy+noise(generator));
    }
    const auto result=match_tracking_scan(field,scan,{},options);
    if(!result.accepted) throw std::runtime_error("Observable fixture rejected");
    const double error=std::hypot(result.pose.x-truth.x,result.pose.y-truth.y);
    sum+=error*error;max_error=std::max(max_error,error);
    // A null scan direction remains regularized by the prior (tested separately
    // in robust_tracking_test); this fixture has two observable directions.
  }
  const double rmse=std::sqrt(sum/40);
  std::cout<<"Known-map registration: 40 fixtures, XY RMSE="<<rmse<<" m, max="<<max_error<<" m\n";
  if(rmse>.01 || max_error>.02) throw std::runtime_error("Observable pose is excessively biased toward odometry");
  std::cout<<"PASS: known-map accuracy regression (not end-to-end SLAM)\n";
}
