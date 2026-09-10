// Optional isolated validation ABI; not part of the SLAM executable.
#include "belugaslam_core/probability_matching.hpp"
#include <memory>
namespace {
struct Fixture {
  belugaslam::ProbabilityField probability;
  belugaslam::TrackingField distance;
  belugaslam::ScanPoints scan;
  belugaslam::PoseSample2 prior;
  belugaslam::TrackingOptions tracking;
  belugaslam::ProbabilityMatchingOptions options;
  Fixture(const std::vector<float>& cells,int w,int h,double res,double ox,double oy)
      : probability(cells,w,h,res,ox,oy), distance(cells,w,h,res,ox,oy) {}
};
}
extern "C" {
void* beluga_probability_create(const float* cells,int w,int h,double res,double ox,double oy,
                                const double* scan,int n,const double* prior) {
  try {
    if (w<2 || h<2 || n<1) return nullptr;
    auto fixture=std::make_unique<Fixture>(std::vector<float>(cells,cells+std::size_t(w)*h),w,h,res,ox,oy);
    for(int i=0;i<n;++i) fixture->scan.emplace_back(scan[2*i],scan[2*i+1]);
    fixture->prior={prior[0],prior[1],prior[2]};
    return fixture.release();
  } catch(...) {return nullptr;}
}
void beluga_probability_destroy(void* pointer) {delete static_cast<Fixture*>(pointer);}
int beluga_probability_set_odometry_prior(void* pointer,const double* delta,
                                         double translation_sigma,double rotation_sigma) {
  if (!pointer) return 0;
  try {
    auto& f=*static_cast<Fixture*>(pointer);
    belugaslam::OdometryPriorOptions options;
    options.translation_sigma=translation_sigma;options.rotation_sigma=rotation_sigma;
    const auto prior=belugaslam::odometry_tracking_prior({delta[0],delta[1],delta[2]},
                                                       f.prior.yaw-delta[2],options);
    f.tracking.prior_sqrt_information=prior.sqrt_information;
    f.tracking.use_full_prior=true;
    return 1;
  } catch (...) {return 0;}
}
int beluga_probability_evaluate(void* pointer,const double* delta,double* residuals,double* jacobian) {
  if(!pointer) return 0;
  const auto& f=*static_cast<Fixture*>(pointer);
  return belugaslam::evaluate_probability_match(f.probability,f.scan,f.prior,f.tracking,f.options,delta,residuals,jacobian);
}
int beluga_probability_warm_start(void* pointer,double* pose) {
  if(!pointer) return 0;
  const auto& f=*static_cast<Fixture*>(pointer);
  const auto result=belugaslam::probability_tracking_initial_pose(f.probability,f.distance,f.scan,
                                                                f.prior,f.tracking,f.options);
  pose[0]=result.x;pose[1]=result.y;pose[2]=result.yaw;
  const auto score=belugaslam::tracking_score(f.distance,f.scan,result,f.tracking);
  return score.inliers>=f.tracking.min_points && score.overlap>=f.tracking.min_overlap;
}
}
