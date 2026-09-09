#ifndef BELUGASLAM_CORE_PROPOSAL_POSE_HPP
#define BELUGASLAM_CORE_PROPOSAL_POSE_HPP

#include "robust_tracking.hpp"

namespace belugaslam {

// One hypothesis only. log_weight = log(ancestor weight) + log likelihood
// - log(number of proposals for that ancestor). Never pool different graphs.
struct WeightedPoseProposal { PoseSample2 pose; double log_weight; };
struct ProposalPoseSummary {
  bool valid = false;
  PoseSample2 mean{};
  std::array<double,9> covariance{};
  double ess = 0, local_mass = 0, position_std = 0, yaw_std = 0;
};

inline std::vector<double> normalized_proposal_weights(const std::vector<WeightedPoseProposal>& cloud) {
  double maximum = -std::numeric_limits<double>::infinity();
  for (const auto& p : cloud) {
    if (std::isnan(p.log_weight) || p.log_weight == std::numeric_limits<double>::infinity()) return {};
    if (std::isfinite(p.log_weight)) {
      if (!std::isfinite(p.pose.x) || !std::isfinite(p.pose.y) || !std::isfinite(p.pose.yaw)) return {};
      maximum = std::max(maximum, p.log_weight);
    }
  }
  if (!std::isfinite(maximum)) return {};
  std::vector<double> weights; weights.reserve(cloud.size());
  double total = 0;
  for (const auto& p : cloud) {
    weights.push_back(std::exp(p.log_weight - maximum)); total += weights.back();
  }
  for (auto& w : weights) w /= total;
  return weights;
}

inline std::array<double,9> proposal_second_moment(
    const std::vector<WeightedPoseProposal>& cloud, const std::vector<double>& weights,
    const PoseSample2& center) {
  if (cloud.size() != weights.size()) throw std::invalid_argument("Mismatched proposal moments");
  std::array<double,9> result{};
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (!(weights[i] > 0)) continue;
    const auto& p = cloud[i].pose;
    const std::array<double,3> d{p.x-center.x,p.y-center.y,wrap_angle(p.yaw-center.yaw)};
    for (int a=0;a<3;++a) for (int b=0;b<3;++b) result[3*a+b] += weights[i]*d[a]*d[b];
  }
  return result;
}

inline ProposalPoseSummary summarize_proposal_poses(
    const std::vector<WeightedPoseProposal>& cloud, const std::vector<double>& weights,
    const PoseSample2& reference, double translation_window, double rotation_window) {
  ProposalPoseSummary result;
  if (weights.empty()) return result;
  if (cloud.size()!=weights.size()) throw std::invalid_argument("Mismatched proposal cloud");
  long double x=0,y=0,c=0,s=0,squares=0;
  // Reference-relative accumulation avoids cancellation far from the world origin.
  for (std::size_t i=0;i<cloud.size();++i) {
    const double w=weights[i]; if (!(w>0)) continue;
    const auto& p=cloud[i].pose;
    x+=w*(p.x-reference.x); y+=w*(p.y-reference.y);
    const auto a=wrap_angle(p.yaw-reference.yaw);
    c+=w*std::cos(a); s+=w*std::sin(a); squares+=w*w;
    if (std::hypot(p.x-reference.x,p.y-reference.y)<=translation_window && std::abs(a)<=rotation_window)
      result.local_mass+=w;
  }
  if (!(squares>0) || std::hypot(c,s)<1e-8L) return result;
  result.mean={reference.x+static_cast<double>(x),reference.y+static_cast<double>(y),
               wrap_angle(reference.yaw+std::atan2(static_cast<double>(s),static_cast<double>(c)))};
  result.ess=static_cast<double>(1/squares);
  result.covariance=proposal_second_moment(cloud,weights,result.mean);
  result.position_std=std::sqrt(std::max(0.,result.covariance[0]+result.covariance[4]));
  result.yaw_std=std::sqrt(std::max(0.,result.covariance[8]));
  result.valid=std::isfinite(result.mean.x) && std::isfinite(result.mean.y) &&
      std::all_of(result.covariance.begin(),result.covariance.end(),[](double x){return std::isfinite(x);});
  return result;
}

struct ProposalPoseDecision {
  bool accepted=false;
  const char* reason="invalid_cloud";
  TrackingScore score;
};

// The mean is used only for a concentrated, well-supported conditional belief
// that passes an independent point-pose fit check in the existing native map.
// Gates select a readout; they do not modify proposal weights or remove tails.
inline ProposalPoseDecision check_proposal_pose(
    const ProposalPoseSummary& summary, const TrackingField& field, const ScanPoints& scan,
    const PoseSample2& prediction, const TrackingScore& frontend_score,
    const TrackingOptions& tracking, double minimum_ess=5., double minimum_local_mass=.9,
    double maximum_log_drop=.02) {
  ProposalPoseDecision result;
  if (!summary.valid) return result;
  result.reason="low_proposal_ess";
  if (summary.ess<minimum_ess) return result;
  result.reason="diffuse_or_multimodal";
  if (summary.local_mass<minimum_local_mass || summary.position_std>.5*tracking.max_translation ||
      summary.yaw_std>.5*tracking.max_rotation) return result;
  result.reason="outside_motion_gate";
  if (std::hypot(summary.mean.x-prediction.x,summary.mean.y-prediction.y)>tracking.max_translation ||
      std::abs(wrap_angle(summary.mean.yaw-prediction.yaw))>tracking.max_rotation) return result;
  result.score=tracking_score(field,scan,summary.mean,tracking);
  result.reason="mean_scan_fit_failed";
  if (result.score.inliers<tracking.min_points || result.score.overlap<tracking.min_overlap ||
      result.score.mean_log_likelihood+maximum_log_drop<frontend_score.mean_log_likelihood) return result;
  result.accepted=true; result.reason="accepted";
  return result;
}
}  // namespace belugaslam
#endif
