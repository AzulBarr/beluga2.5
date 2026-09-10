#ifndef BELUGASLAM_CORE_ODOMETRY_TRACKING_PRIOR_HPP
#define BELUGASLAM_CORE_ODOMETRY_TRACKING_PRIOR_HPP

#include "loop_belief.hpp"
#include <array>

namespace belugaslam {
using PriorMatrix3 = std::array<double,9>;  // row major, x/y in metres, yaw in radians

struct OdometryPriorOptions {
  // Uncertainty of the previous local pose, reset for each one-step prediction.
  // These are regularization floors, NOT a fitted posterior covariance.
  double translation_sigma = 0.10, rotation_sigma = 0.05;
  double alpha1 = 0.1, alpha2 = 0.05, alpha3 = 0.1, alpha4 = 0.05;
  double rotation_distance_threshold = 0.01;
  void validate() const {
    for (double v : {translation_sigma,rotation_sigma})
      if (!std::isfinite(v) || v < 1e-6 || v > 10.)
        throw std::invalid_argument("Odometry prior sigmas must be finite and in [1e-6,10]");
    for (double v : {alpha1,alpha2,alpha3,alpha4,rotation_distance_threshold})
      if (!std::isfinite(v) || v < 0 || v > 100.)
        throw std::invalid_argument("Odometry prior noise/threshold must be finite and in [0,100]");
  }
};

struct GaussianTrackingPrior {
  PriorMatrix3 covariance{};
  PriorMatrix3 sqrt_information{};  // L^{-1}, where covariance = L L^T
};

inline GaussianTrackingPrior factor_tracking_prior(const PriorMatrix3& covariance) {
  GaussianTrackingPrior result; result.covariance=covariance;
  PriorMatrix3 lower{};
  for (int i=0;i<3;++i) for(int j=0;j<=i;++j) {
    double v=covariance[3*i+j];
    if (!std::isfinite(v) || !std::isfinite(covariance[3*j+i]) ||
        std::abs(v-covariance[3*j+i])>1e-10*std::max(1.,std::abs(v)))
      throw std::invalid_argument("Non-finite or asymmetric tracking covariance");
    for(int k=0;k<j;++k) v-=lower[3*i+k]*lower[3*j+k];
    if (i==j) {
      if (!(v>0) || !std::isfinite(v)) throw std::invalid_argument("Tracking covariance is not positive definite");
      lower[3*i+j]=std::sqrt(v);
    } else lower[3*i+j]=v/lower[3*j+j];
  }
  for (int col=0;col<3;++col) for(int row=0;row<3;++row) {
    double v=row==col ? 1. : 0.;
    for(int k=0;k<row;++k) v-=lower[3*row+k]*result.sqrt_information[3*k+col];
    result.sqrt_information[3*row+col]=v/lower[3*row+row];
  }
  return result;
}

inline GaussianTrackingPrior fixed_tracking_prior(double translation_sigma,double rotation_sigma) {
  if (!std::isfinite(translation_sigma) || translation_sigma<=0 ||
      !std::isfinite(rotation_sigma) || rotation_sigma<=0)
    throw std::invalid_argument("Invalid fixed prior sigmas");
  return factor_tracking_prior({translation_sigma*translation_sigma,0,0,
                                0,translation_sigma*translation_sigma,0,0,0,rotation_sigma*rotation_sigma});
}

// A first-order, per-scan prediction in the MATCHING SUBMAP frame:
// Sigma = F D F^T + G diag(v_rot1,v_trans,v_rot2) G^T.
// D is the configured local-pose floor; this does not recursively reuse PF
// covariance or the current scan. The two rotation noises are independent.
inline GaussianTrackingPrior odometry_tracking_prior(const PoseSample2& delta,
    double previous_yaw_in_submap, const OdometryPriorOptions& options) {
  options.validate();
  for (double v : {delta.x,delta.y,delta.yaw,previous_yaw_in_submap})
    if (!std::isfinite(v)) throw std::invalid_argument("Non-finite odometry prior input");
  const double d=std::hypot(delta.x,delta.y), turn=wrap_angle(delta.yaw);
  if (!std::isfinite(d)) throw std::invalid_argument("Odometry translation overflow");
  const double bearing=d>0 ? std::atan2(delta.y,delta.x) : 0.;
  const double first=d>options.rotation_distance_threshold ? bearing : 0.;
  const double second=wrap_angle(turn-first);
  const auto rotation_noise=[](double a) {
    // Backward translation is not a noisy pi-radian turn.
    constexpr double pi=3.14159265358979323846;
    return std::min(std::abs(wrap_angle(a)),std::abs(wrap_angle(a-pi)));
  };
  const double n1=rotation_noise(first);
  const double n2=d>options.rotation_distance_threshold ? rotation_noise(second) : std::abs(turn);
  const double v1=options.alpha1*n1*n1+options.alpha2*d*d;
  const double vt=options.alpha3*d*d+options.alpha4*(n1*n1+n2*n2);
  const double v2=options.alpha1*n2*n2+options.alpha2*d*d;
  const double c=std::cos(previous_yaw_in_submap),s=std::sin(previous_yaw_in_submap);
  const double dx=c*delta.x-s*delta.y,dy=s*delta.x+c*delta.y;
  const std::array<double,3> heading{-dy,dx,1};
  const std::array<double,3> forward{std::cos(previous_yaw_in_submap+bearing),
                                      std::sin(previous_yaw_in_submap+bearing),0};
  const double t=options.translation_sigma*options.translation_sigma;
  const double r=options.rotation_sigma*options.rotation_sigma;
  PriorMatrix3 covariance{t,0,0,0,t,0,0,0,v2};
  for(int i=0;i<3;++i) for(int j=0;j<3;++j)
    covariance[3*i+j]+=(r+v1)*heading[i]*heading[j]+vt*forward[i]*forward[j];
  return factor_tracking_prior(covariance);
}

}  // namespace belugaslam
#endif
