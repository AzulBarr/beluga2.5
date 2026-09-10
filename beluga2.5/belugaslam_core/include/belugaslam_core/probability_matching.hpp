#ifndef BELUGASLAM_CORE_PROBABILITY_MATCHING_HPP
#define BELUGASLAM_CORE_PROBABILITY_MATCHING_HPP

#include "robust_tracking.hpp"
#include <map>

namespace belugaslam {

struct ProbabilityMatchingOptions {
  // Experimental frontend cost scale; this is NOT a PF likelihood temperature.
  double occupied_space_weight = 5.0;
  double voxel_size = 0.05;  // metres; zero disables spatial filtering
  void validate() const {
    if (!std::isfinite(occupied_space_weight) || occupied_space_weight <= 0 ||
        !std::isfinite(voxel_size) || voxel_size < 0 || voxel_size > 1.0)
      throw std::invalid_argument("Invalid probability matcher weight or voxel size");
  }
};

struct ProbabilitySample { double probability = 0.1, dx = 0, dy = 0; };

// Immutable frontend-only view of a submap. Unknown/outside space has maximum
// correspondence cost; it cannot attract a return more than observed free space.
// This view is never used for Bayesian hypothesis evidence.
class ProbabilityField {
 public:
  ProbabilityField(const std::vector<float>& log_odds, int width, int height,
                   double resolution, double origin_x, double origin_y)
      : width_(width), height_(height), resolution_(resolution), ox_(origin_x), oy_(origin_y) {
    if (width < 2 || height < 2 || !std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(ox_) || !std::isfinite(oy_) ||
        log_odds.size() != static_cast<std::size_t>(width) * height)
      throw std::invalid_argument("Invalid probability field extent");
    probabilities_.reserve(log_odds.size());
    for (float l : log_odds) {
      if (!std::isfinite(l)) throw std::invalid_argument("Non-finite grid log odds");
      probabilities_.push_back(l == 0 ? 0.1 : std::clamp(1.0 / (1.0 + std::exp(-double(l))), 0.1, 0.9));
    }
  }

  [[nodiscard]] ProbabilitySample sample(double x, double y) const {
    const double fx = (x - ox_) / resolution_ - 0.5;
    const double fy = (y - oy_) / resolution_ - 0.5;
    // Check BEFORE integer conversion, including finite but enormous endpoints.
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx < -2 || fy < -2 ||
        fx >= double(width_) + 1 || fy >= double(height_) + 1) return {};
    const int ix = static_cast<int>(std::floor(fx)), iy = static_cast<int>(std::floor(fy));
    std::array<double, 4> rows{}, gradients{};
    for (int j = 0; j < 4; ++j) {
      const auto row = cubic(at(ix-1, iy+j-1), at(ix, iy+j-1),
                             at(ix+1, iy+j-1), at(ix+2, iy+j-1), fx-ix);
      rows[j] = row.first; gradients[j] = row.second;
    }
    const auto value = cubic(rows[0], rows[1], rows[2], rows[3], fy-iy);
    const auto gx = cubic(gradients[0], gradients[1], gradients[2], gradients[3], fy-iy).first;
    // Cubic interpolation can overshoot. Saturate with the matching derivative,
    // so a correspondence remains bounded, including beside isolated obstacles.
    if (value.first < 0.1) return {0.1, 0, 0};
    if (value.first > 0.9) return {0.9, 0, 0};
    return {value.first, gx / resolution_, value.second / resolution_};
  }

 private:
  static std::pair<double, double> cubic(double a, double b, double c, double d, double t) {
    const double p = 0.5 * (-a + 3*b - 3*c + d);
    const double q = 0.5 * (2*a - 5*b + 4*c - d);
    const double r = 0.5 * (-a + c);
    return {((p*t + q)*t + r)*t + b, (3*p*t + 2*q)*t + r};
  }
  double at(int x, int y) const {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return 0.1;
    return probabilities_[static_cast<std::size_t>(y) * width_ + x];
  }
  int width_, height_;
  double resolution_, ox_, oy_;
  std::vector<double> probabilities_;
};

// Deterministic voxel centroids, retained in first-beam order. The old index
// selector remains the final work budget. Filtering consumes no random draws.
inline ScanPoints probability_tracking_points(const ScanPoints& scan, double voxel_size, std::size_t limit) {
  if (!std::isfinite(voxel_size) || voxel_size < 0 || voxel_size > 1)
    throw std::invalid_argument("Invalid tracking voxel size");
  struct Voxel { double x = 0, y = 0; std::size_t count = 0; };
  std::map<std::pair<double,double>, std::size_t> indices;
  std::vector<Voxel> voxels;
  ScanPoints finite;
  for (const auto& [x,y] : scan) {
    if (!std::isfinite(x) || !std::isfinite(y)) continue;
    if (voxel_size == 0) { finite.emplace_back(x,y); continue; }
    const double ix = std::floor(x / voxel_size), iy = std::floor(y / voxel_size);
    if (!std::isfinite(ix) || !std::isfinite(iy)) continue;
    const auto inserted = indices.emplace(std::make_pair(ix,iy), voxels.size());
    if (inserted.second) voxels.emplace_back();
    auto& v = voxels[inserted.first->second];
    ++v.count;
    v.x += (x - v.x) / v.count; v.y += (y - v.y) / v.count;
  }
  if (voxel_size > 0) for (const auto& v : voxels) finite.emplace_back(v.x,v.y);
  return select_tracking_points(finite, limit);
}

// Production residual/Jacobian kernel shared by the C++ Ceres adapter and the
// isolated native-Ceres validation. Parameters are [dx,dy,dyaw] from the ORIGINAL
// odometry prior. A seed never recentres the prior. Row-major Jacobian (N+3)x3.
inline bool evaluate_probability_match(const ProbabilityField& field, const ScanPoints& scan,
    const PoseSample2& prior, const TrackingOptions& tracking, const ProbabilityMatchingOptions& options,
    const double* delta, double* residuals, double* jacobian = nullptr) {
  if (!std::isfinite(prior.x) || !std::isfinite(prior.y) || !std::isfinite(prior.yaw)) return false;
  if (scan.empty() || !std::isfinite(delta[0]) || !std::isfinite(delta[1]) || !std::isfinite(delta[2])) return false;
  const double yaw = prior.yaw + delta[2], c = std::cos(yaw), s = std::sin(yaw);
  const double scale = options.occupied_space_weight / std::sqrt(double(scan.size()));
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const auto [x,y] = scan[i];
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    const double rx = c*x - s*y, ry = s*x + c*y;
    const auto v = field.sample(prior.x + delta[0] + rx, prior.y + delta[1] + ry);
    residuals[i] = scale * (1 - v.probability);
    if (jacobian) {
      jacobian[3*i] = -scale*v.dx;
      jacobian[3*i+1] = -scale*v.dy;
      jacobian[3*i+2] = scale*(v.dx*ry - v.dy*rx);
    }
  }
  if (tracking.use_full_prior) {
    const auto r=full_tracking_prior_residual(delta,tracking);
    for(int a=0;a<3;++a) {
      residuals[scan.size()+a]=r[a];
      if (jacobian) for(int b=0;b<3;++b)
        jacobian[3*(scan.size()+a)+b]=std::sqrt(tracking.prior_information_scale)*tracking.prior_sqrt_information[3*a+b];
    }
    return true;
  }
  for (int a = 0; a < 3; ++a) {
    const double weight = std::sqrt(tracking.prior_information_scale) /
        (a == 2 ? tracking.prior_rotation_sigma : tracking.prior_translation_sigma);
    residuals[scan.size()+a] = weight * delta[a];
    if (jacobian) for (int b = 0; b < 3; ++b)
      jacobian[3*(scan.size()+a)+b] = a == b ? weight : 0;
  }
  return true;
}

inline double probability_tracking_objective(const ProbabilityField& field, const ScanPoints& scan,
    const PoseSample2& pose, const PoseSample2& prior, const TrackingOptions& tracking,
    const ProbabilityMatchingOptions& options) {
  std::vector<double> residuals(scan.size()+3);
  const double delta[] = {pose.x-prior.x, pose.y-prior.y, wrap_angle(pose.yaw-prior.yaw)};
  if (!evaluate_probability_match(field,scan,prior,tracking,options,delta,residuals.data()))
    return std::numeric_limits<double>::infinity();
  double cost = 0;
  for (double r : residuals) cost += 0.5*r*r;
  return cost;
}

// A tight motion prior can shrink the distance matcher's attraction basin.
// Retain its step-1 initialization as an additional candidate in adaptive mode.
// Every candidate is scored with the SAME final occupancy objective and prior;
// the extra search neither changes the prior centre nor enters PF evidence.
inline PoseSample2 probability_tracking_initial_pose(const ProbabilityField& probability,
    const TrackingField& distance,const ScanPoints& scan,const PoseSample2& prior,
    const TrackingOptions& tracking,const ProbabilityMatchingOptions& options,
    const PoseSample2* seed=nullptr) {
  auto pose=prior;
  double cost=probability_tracking_objective(probability,scan,prior,prior,tracking,options);
  const auto consider=[&](const PoseSample2& p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
        std::hypot(p.x-prior.x,p.y-prior.y)>tracking.max_translation ||
        std::abs(wrap_angle(p.yaw-prior.yaw))>tracking.max_rotation) return;
    const auto score=tracking_score(distance,scan,p,tracking);
    if (score.inliers<tracking.min_points || score.overlap<tracking.min_overlap) return;
    const double candidate=probability_tracking_objective(probability,scan,p,prior,tracking,options);
    if (std::isfinite(candidate) && candidate<cost) {pose=p;cost=candidate;}
  };
  const auto warm=match_tracking_scan(distance,scan,prior,tracking,seed);
  if (warm.accepted) consider(warm.pose);
  if (tracking.use_full_prior) {
    auto baseline=tracking;baseline.use_full_prior=false;
    const auto previous=match_tracking_scan(distance,scan,prior,baseline,seed);
    if (previous.accepted) consider(previous.pose);
  }
  if (seed) consider(*seed);
  return pose;
}

}  // namespace belugaslam
#endif
