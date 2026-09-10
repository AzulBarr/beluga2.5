#ifndef BELUGASLAM_CORE_LOOP_BELIEF_HPP
#define BELUGASLAM_CORE_LOOP_BELIEF_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace belugaslam {

struct PoseSample2 { double x, y, yaw; };
struct TrajectoryChange {
  double translation_rmse = std::numeric_limits<double>::infinity();
  double rotation_rmse = std::numeric_limits<double>::infinity();
  bool valid = false;
};

inline double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// Align the trial trajectory rigidly to the prior. Metric LiDAR has known scale:
// SE(2), not a similarity transform that could hide a collapsed/stretched map.
inline TrajectoryChange aligned_trajectory_change(
    const std::vector<PoseSample2>& prior, const std::vector<PoseSample2>& trial) {
  if (prior.size() != trial.size() || prior.size() < 3) return {};
  double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
  for (std::size_t i = 0; i < prior.size(); ++i) {
    for (double value : {prior[i].x, prior[i].y, prior[i].yaw,
                         trial[i].x, trial[i].y, trial[i].yaw}) {
      if (!std::isfinite(value)) return {};
    }
    ax += prior[i].x; ay += prior[i].y;
    bx += trial[i].x; by += trial[i].y;
  }
  const double n = static_cast<double>(prior.size());
  ax /= n; ay /= n; bx /= n; by /= n;
  double dot = 0.0, cross = 0.0, extent = 0.0;
  for (std::size_t i = 0; i < prior.size(); ++i) {
    const double px = prior[i].x - ax, py = prior[i].y - ay;
    const double qx = trial[i].x - bx, qy = trial[i].y - by;
    dot += qx * px + qy * py;
    cross += qx * py - qy * px;
    extent += px * px + py * py;
  }
  if (extent < 1.0e-8 || std::hypot(dot, cross) < 1.0e-12) return {};
  const double angle = std::atan2(cross, dot);
  const double c = std::cos(angle), s = std::sin(angle);
  double translation_error = 0.0, rotation_error = 0.0;
  for (std::size_t i = 0; i < prior.size(); ++i) {
    const double qx = trial[i].x - bx, qy = trial[i].y - by;
    const double dx = prior[i].x - ax - (c * qx - s * qy);
    const double dy = prior[i].y - ay - (s * qx + c * qy);
    const double da = wrap_angle(prior[i].yaw - trial[i].yaw - angle);
    translation_error += dx * dx + dy * dy;
    rotation_error += da * da;
  }
  return {std::sqrt(translation_error / n), std::sqrt(rotation_error / n), true};
}

inline double trajectory_compatibility(
    const TrajectoryChange& change, double translation_scale, double rotation_scale) {
  if (!change.valid || !(translation_scale > 0.0) || !(rotation_scale > 0.0)) return 0.0;
  const double x = change.translation_rmse / translation_scale;
  const double a = change.rotation_rmse / rotation_scale;
  return std::isfinite(x) && std::isfinite(a) ? std::exp(-0.5 * (x * x + a * a)) : 0.0;
}

inline std::vector<double> normalize_masses(const std::vector<double>& masses) {
  std::vector<double> result(masses.size(), 0.0);
  double sum = 0.0;
  for (double mass : masses) {
    if (!std::isfinite(mass) || mass < 0.0) throw std::invalid_argument("Invalid hypothesis mass");
    sum += mass;
  }
  if (!std::isfinite(sum)) throw std::invalid_argument("Hypothesis mass overflow");
  if (sum > 0.0) {
    for (std::size_t i = 0; i < masses.size(); ++i) result[i] = masses[i] / sum;
  }
  return result;
}

struct BeliefEvidence { double weighted = 0.0, map = 0.0, uniform = 0.0; };

// Unsupported/missing modes have compatibility zero. Never renormalize only
// over supporting modes: that would turn tiny posterior mass into certainty.
inline BeliefEvidence marginalize_compatibilities(
    const std::vector<double>& masses, const std::vector<double>& compatibility) {
  if (masses.size() != compatibility.size()) throw std::invalid_argument("Mismatched evidence arrays");
  const auto weights = normalize_masses(masses);
  BeliefEvidence result;
  if (weights.empty()) return result;
  const auto map_index = static_cast<std::size_t>(
      std::distance(weights.begin(), std::max_element(weights.begin(), weights.end())));
  for (std::size_t i = 0; i < weights.size(); ++i) {
    const double e = std::isfinite(compatibility[i]) ? std::clamp(compatibility[i], 0.0, 1.0) : 0.0;
    result.weighted += weights[i] * e;
    result.uniform += e / weights.size();
    if (i == map_index && weights[i] > 0.0) result.map = e;
  }
  return result;
}

// A minimum particle quota is a computational allocation, not extra belief mass.
// The caller must give every particle in mode h weight W_h / quota_h.
inline std::vector<std::size_t> allocate_particle_quotas(
    const std::vector<double>& masses, std::size_t budget) {
  if (masses.empty()) return {};
  if (budget < masses.size()) throw std::invalid_argument("Particle budget smaller than hypothesis count");
  const auto weights = normalize_masses(masses);
  if (std::accumulate(weights.begin(), weights.end(), 0.0) <= 0.0) {
    throw std::invalid_argument("Empty hypothesis belief");
  }
  std::vector<std::size_t> quotas(masses.size(), 1);
  std::vector<double> remainders(masses.size());
  const auto remaining = budget - masses.size();
  std::size_t allocated = masses.size();
  for (std::size_t i = 0; i < masses.size(); ++i) {
    const double exact = weights[i] * remaining;
    const auto count = static_cast<std::size_t>(std::floor(exact));
    quotas[i] += count; allocated += count; remainders[i] = exact - count;
  }
  std::vector<std::size_t> order(masses.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) { return remainders[a] > remainders[b]; });
  for (std::size_t i = 0; allocated < budget; ++i, ++allocated) ++quotas[order[i % order.size()]];
  return quotas;
}

}  // namespace belugaslam
#endif
