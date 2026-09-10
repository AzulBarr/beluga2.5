// Reference-free component diagnostic, NOT a complete SLAM replay or an RMSE benchmark.
// Uses the production matcher, default options, and a new endpoint field for each
// previous scan. No accumulated submaps, particles, recovery, PGO, or ground truth.
#include "belugaslam_core/robust_tracking.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

using namespace belugaslam;
struct Frame { double stamp; PoseSample2 odom; ScanPoints points, fit, held; };
PoseSample2 Compose(const PoseSample2& a, const PoseSample2& b) {
  const double c=std::cos(a.yaw),s=std::sin(a.yaw);
  return {a.x+c*b.x-s*b.y,a.y+s*b.x+c*b.y,wrap_angle(a.yaw+b.yaw)};
}
PoseSample2 Inverse(const PoseSample2& a) {
  const double c=std::cos(a.yaw),s=std::sin(a.yaw);
  return {-c*a.x-s*a.y,s*a.x-c*a.y,-a.yaw};
}
std::unique_ptr<TrackingField> Field(const Frame& frame) {
  double x0=0,x1=0,y0=0,y1=0;
  for(const auto& [x,y]:frame.points) {x0=std::min(x0,x);x1=std::max(x1,x);y0=std::min(y0,y);y1=std::max(y1,y);}
  constexpr double resolution=.05;
  x0=std::floor(x0/resolution)*resolution-1.1;y0=std::floor(y0/resolution)*resolution-1.1;
  const int width=static_cast<int>(std::ceil((x1-x0+1.1)/resolution));
  const int height=static_cast<int>(std::ceil((y1-y0+1.1)/resolution));
  std::vector<float> cells(static_cast<std::size_t>(width)*height,0);
  for(const auto& [x,y]:frame.points) {
    const int ix=static_cast<int>(std::floor((x-x0)/resolution)),iy=static_cast<int>(std::floor((y-y0)/resolution));
    cells[static_cast<std::size_t>(iy)*width+ix]=1.2F;
  }
  return std::make_unique<TrackingField>(cells,width,height,resolution,x0,y0);
}
bool Read(std::istream& input,Frame& frame) {
  std::string line;if(!std::getline(input,line))return false;
  std::istringstream row(line);std::size_t n;
  if(!(row>>frame.stamp>>frame.odom.x>>frame.odom.y>>frame.odom.yaw>>n) || n>100000)
    throw std::runtime_error("Invalid frame header");
  frame.points.clear();frame.fit.clear();frame.held.clear();
  for(std::size_t i=0;i<n;++i) {
    double x,y;if(!(row>>x>>y)||!std::isfinite(x)||!std::isfinite(y))throw std::runtime_error("Invalid endpoint");
    frame.points.emplace_back(x,y);
    (i%3==1?frame.held:frame.fit).emplace_back(x,y);
  }
  return true;
}
template<class Options>
auto set_prior_scale(Options& o,double scale,int) -> decltype(o.prior_information_scale=scale,void()) {
  o.prior_information_scale=scale;
}
template<class Options>
void set_prior_scale(Options&,double scale,long) {
  if(scale!=1.0) throw std::invalid_argument("The original matcher supports only prior scale 1");
}
int main(int argc,char** argv) {
  if(argc!=3 && argc!=4){std::cerr<<"Usage: audit_scan_pairs frames.txt results.csv [prior_information_scale]\n";return 2;}
  std::ifstream input(argv[1]);std::ofstream out(argv[2]);
  if(!input||!out)return 2;
  out<<std::setprecision(17)<<"sequence,stamp,dt,fit_points,held_points,forward_accepted,reverse_accepted,"
      "prior_held_log,matched_held_log,matched_overlap,cycle_m,cycle_rad,innovation_m,innovation_rad,match_ms\n";
  Frame previous,current;TrackingOptions options;
  if(argc==4) set_prior_scale(options,std::stod(argv[3]),0);
  if(!Read(input,previous))return 2;
  auto previous_field=Field(previous);std::size_t index=0;
  while(Read(input,current)) {
    ++index;
    if(!(current.stamp>previous.stamp))throw std::runtime_error("Nonmonotonic frame time");
    auto current_field=Field(current);
    const auto prior=Compose(Inverse(previous.odom),current.odom);
    const auto start=std::chrono::steady_clock::now();
    const auto forward=match_tracking_scan(*previous_field,current.fit,prior,options);
    const auto reverse=match_tracking_scan(*current_field,previous.fit,Inverse(prior),options);
    const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    const auto cycle=Compose(forward.pose,reverse.pose),innovation=Compose(Inverse(prior),forward.pose);
    out<<index<<','<<current.stamp<<','<<current.stamp-previous.stamp<<','<<current.fit.size()<<','<<current.held.size()<<','
       <<forward.accepted<<','<<reverse.accepted<<','
       <<tracking_score(*previous_field,current.held,prior,options).mean_log_likelihood<<','
       <<tracking_score(*previous_field,current.held,forward.pose,options).mean_log_likelihood<<','
       <<forward.score.overlap<<','<<std::hypot(cycle.x,cycle.y)<<','<<std::abs(cycle.yaw)<<','
       <<std::hypot(innovation.x,innovation.y)<<','<<std::abs(innovation.yaw)<<','<<ms<<'\n';
    std::swap(previous,current);previous_field=std::move(current_field);
    if(index%1000==0)std::cerr<<index<<" scan pairs\n";
  }
  std::cerr<<"Completed "<<index<<" scan pairs; no reference trajectory used.\n";
}
