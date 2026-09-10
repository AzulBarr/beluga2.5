#include "belugaslam_core/probability_matching.hpp"
#include <iostream>
#include <stdexcept>
using namespace belugaslam;
int checks = 0;
void check(bool value, const char* message) { ++checks; if (!value) throw std::runtime_error(message); }
int main() {
  constexpr int w=24, h=20;
  constexpr double res=.07, ox=-1.34, oy=.23;
  std::vector<float> cells(w*h);
  for (int y=0;y<h;++y) for(int x=0;x<w;++x)
    cells[y*w+x] = float(.8*std::sin(.31*x)+.6*std::cos(.29*y));
  ProbabilityField field(cells,w,h,res,ox,oy);
  const double eps=1e-6;
  for (int k=0;k<19;++k) {
    const double x=ox+(k+.63)*res, y=oy+(k*.7+.87)*res;
    const auto value=field.sample(x,y);
    check(std::abs(value.dx-(field.sample(x+eps,y).probability-field.sample(x-eps,y).probability)/(2*eps))<1e-6,
          "bicubic x derivative or grid origin/resolution wrong");
    check(std::abs(value.dy-(field.sample(x,y+eps).probability-field.sample(x,y-eps).probability)/(2*eps))<1e-6,
          "bicubic y derivative wrong");
  }
  for (int y=0;y<h;++y) for(int x=0;x<w;++x)
    check(std::abs(field.sample(ox+(x+.5)*res,oy+(y+.5)*res).probability-
          1./(1.+std::exp(-double(cells[y*w+x]))))<1e-12,"cell centre is not original occupancy probability");
  for (auto point : ScanPoints{{1e300,0},{0,-1e300},{INFINITY,0},{NAN,0}}) {
    const auto v=field.sample(point.first,point.second);
    check(v.probability==.1 && v.dx==0 && v.dy==0,"unsafe outside-field sample");
  }
  ProbabilityField unknown(std::vector<float>(w*h),w,h,res,ox,oy);
  check(unknown.sample(ox+.4,oy+.4).probability==.1,"unknown space attracts endpoints");
  auto high=cells; high[5*w+5]=5;
  auto low=cells; low[5*w+5]=.7F;
  ProbabilityField confident(high,w,h,res,ox,oy), uncertain(low,w,h,res,ox,oy);
  check(confident.sample(ox+5.5*res,oy+5.5*res).probability>
        uncertain.sample(ox+5.5*res,oy+5.5*res).probability,"occupancy confidence was thresholded away");
  TrackingOptions tracking; ProbabilityMatchingOptions options;
  const PoseSample2 prior{-.2,.1,3.13};
  const ScanPoints scan{{.1,-.3},{.4,-.5},{.7,-.6},{.8,-.4}};
  double delta[]={.013,-.021,.024};
  std::vector<double> residuals(scan.size()+3), jacobian(3*residuals.size()), plus(residuals.size()), minus(residuals.size());
  check(evaluate_probability_match(field,scan,prior,tracking,options,delta,residuals.data(),jacobian.data()),"residual evaluation failed");
  for(int a=0;a<3;++a) {
    delta[a]+=eps;
    evaluate_probability_match(field,scan,prior,tracking,options,delta,plus.data());
    delta[a]-=2*eps;
    evaluate_probability_match(field,scan,prior,tracking,options,delta,minus.data());
    delta[a]+=eps;
    for(std::size_t i=0;i<residuals.size();++i)
      check(std::abs(jacobian[3*i+a]-(plus[i]-minus[i])/(2*eps))<2e-6,"SE2 residual Jacobian, yaw branch or prior wrong");
  }
  const PoseSample2 pose{prior.x+delta[0],prior.y+delta[1],wrap_angle(prior.yaw+delta[2])};
  const double cost=probability_tracking_objective(field,scan,pose,prior,tracking,options);
  auto repeated=scan; repeated.insert(repeated.end(),scan.begin(),scan.end());
  check(std::abs(cost-probability_tracking_objective(field,repeated,pose,prior,tracking,options))<1e-12,
        "duplicating every beam changes the prior/data balance");
  const ScanPoints points{{.01,.01},{.03,.03},{-.01,-.01},{-.03,-.03},{.12,.01},{NAN,0}};
  const auto filtered=probability_tracking_points(points,.05,180);
  check(filtered.size()==3,"voxel count or negative-coordinate floor wrong");
  check(std::abs(filtered[0].first-.02)<1e-12 && std::abs(filtered[1].first+.02)<1e-12,"voxel centroid/order wrong");
  check(probability_tracking_points(points,0,180).size()==5,"disabled filter lost finite beams");
  check(probability_tracking_points(points,.05,2).size()==2,"frontend point budget exceeded");
  check(probability_tracking_points(points,.05,0).empty(),"zero point budget");
  check(probability_tracking_points({},.05,180).empty(),"empty scan filter");
  for (double invalid : {-1.,double(INFINITY),double(NAN)}) {
    bool threw=false; try { auto bad=options; bad.voxel_size=invalid; bad.validate(); } catch(const std::invalid_argument&) {threw=true;}
    check(threw,"invalid voxel setting accepted");
  }
  std::cout << "PASS: " << checks << " probability interpolation/residual/filter checks\n";
}
