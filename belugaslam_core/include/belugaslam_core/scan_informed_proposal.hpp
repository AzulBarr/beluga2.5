#ifndef BELUGASLAM_CORE_SCAN_INFORMED_PROPOSAL_HPP
#define BELUGASLAM_CORE_SCAN_INFORMED_PROPOSAL_HPP
#include <beluga/motion/differential_drive_distribution.hpp>
#include <string>

namespace belugaslam {
using MotionIncrement = std::array<double,3>;
inline MotionIncrement compose_increment(const MotionIncrement& a,const MotionIncrement& b) {
  const double c=std::cos(a[2]),s=std::sin(a[2]);
  return {a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],beluga::motion_detail::wrap(a[2]+b[2])};
}
inline MotionIncrement inverse_increment(const MotionIncrement& a) {
  const double c=std::cos(a[2]),s=std::sin(a[2]);
  return {-c*a[0]-s*a[1],s*a[0]-c*a[1],-a[2]};
}
struct ScanProposalOptions {
  // Retain >=5% of the ORIGINAL prior as a defensive mixture component.
  double fraction=.8;
  bool adapt_to_prior=true;
  void validate() const {
    if (!std::isfinite(fraction) || fraction<0 || fraction>.95)
      throw std::invalid_argument("scan_proposal_fraction must be in [0,0.95]");
  }
};

// O is the nominal odometry increment; F is the accepted frontend increment.
// Draw E = O^{-1} D_odom with D_odom ~ p_odom, then D_front = F E.
// Thus q_front(D) = p_odom(O F^{-1} D), with unit Haar Jacobian.
// q(D) = (1-a) p_odom(D) + a q_front(D); the target remains the ORIGINAL
// differential-drive prior, including its noise correlations and signed distance.
// Every candidate carries log(p_odom/q), without a Gaussian approximation.
class ScanInformedProposal {
 public:
  ScanInformedProposal(const beluga::DifferentialDriveDistribution2d& prior,
      const MotionIncrement& frontend_delta,const ScanProposalOptions& options)
      : prior_(prior),frontend_(frontend_delta),options_(options) {
    options_.validate();
    if (!prior_.has_density()) throw std::invalid_argument("Scan proposal needs nonsingular odometry");
    for (double v : frontend_)
      if (!std::isfinite(v)) throw std::invalid_argument("Non-finite frontend delta");
    const MotionIncrement odometry{prior_.distance_mean*std::cos(prior_.first_mean),
      prior_.distance_mean*std::sin(prior_.first_mean),
      beluga::motion_detail::wrap(prior_.first_mean+prior_.second_mean)};
    if (options_.adapt_to_prior && options_.fraction>0) {
      // A supported frontend is useful; an increment deep in the prior tails
      // wastes candidates which p/q must subsequently reject. Attenuate the
      // mixture BEFORE drawing it, using only this scan's fixed frontend and
      // motion model. The quarter log-density drop is an overlap proxy (exact
      // Gaussian equal-covariance Bhattacharyya scaling), not extra evidence.
      const double nominal_log=prior_.log_density(odometry[0],odometry[1],odometry[2]);
      const double frontend_log=prior_.log_density(frontend_[0],frontend_[1],frontend_[2]);
      options_.fraction*=std::exp(.25*std::min(0.,frontend_log-nominal_log));
    }
    shift_=compose_increment(frontend_,inverse_increment(odometry));
    inverse_shift_=inverse_increment(shift_);
  }
  [[nodiscard]] double fraction() const { return options_.fraction; }
  [[nodiscard]] double log_frontend_density(const MotionIncrement& delta) const {
    const auto original=compose_increment(inverse_shift_,delta);
    return prior_.log_density(original[0],original[1],original[2]);
  }
  [[nodiscard]] double log_importance_ratio(const MotionIncrement& delta) const {
    if (options_.fraction==0) return 0.;
    const double lp=prior_.log_density(delta[0],delta[1],delta[2]);
    const double lq=beluga::motion_detail::log_add(std::log1p(-options_.fraction)+lp,
        std::log(options_.fraction)+log_frontend_density(delta));
    return lp-lq;
  }
  struct Draw { MotionIncrement delta; double log_ratio; bool from_frontend; };
  // prior_draw is an independent draw from the SAME odometry distribution.
  template<class Generator>
  Draw draw(const MotionIncrement& prior_draw,Generator& generator) const {
    auto delta=prior_draw;
    const bool frontend=options_.fraction>0 && std::bernoulli_distribution{options_.fraction}(generator);
    if (frontend) delta=compose_increment(shift_,prior_draw);
    return {delta,log_importance_ratio(delta),frontend};
  }
 private:
  beluga::DifferentialDriveDistribution2d prior_;
  MotionIncrement frontend_,shift_{},inverse_shift_{};
  ScanProposalOptions options_;
};
}  // namespace belugaslam
#endif
