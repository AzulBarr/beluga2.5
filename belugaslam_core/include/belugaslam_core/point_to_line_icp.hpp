#ifndef BELUGASLAM_CORE_POINT_TO_LINE_ICP_HPP
#define BELUGASLAM_CORE_POINT_TO_LINE_ICP_HPP
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "robust_tracking.hpp"

namespace belugaslam {

/// Geometry of the retained endpoint cloud. The voxel is deliberately finer than
/// the map cell: the chamfer TrackingField places its zero level set on cell
/// centres, so a grid-resolution floor bounds its accuracy no matter how many
/// iterations it runs. Matching raw endpoints is the only way past that floor,
/// and re-quantizing them to the grid would give the whole thing away.
struct SurfaceCloudParams {
  double voxel_size = 0.04;
  double bucket_size = 0.25;  // Kept at or above max_correspondence_distance.
  double normal_radius = 0.20;
  std::size_t min_normal_neighbors = 4;
};

struct IcpOptions {
  SurfaceCloudParams cloud;
  double max_correspondence_distance = 0.25;
  /// Below this PCA linearity the neighbourhood is a corner or clutter, not a
  /// wall. Those correspondences become point-to-point instead of being dropped:
  /// in a straight corridor every normal is parallel and point-to-line has no
  /// restoring force along the free axis, so corners carry the only data-term
  /// constraint there is. Dropping them would make degeneracy strictly worse.
  double min_linearity = 0.55;
  /// Measurement sigma of a point-to-line residual, in metres. This is NOT the
  /// chamfer kernel width: that one is sized for a 0.1 m grid, and reusing it
  /// here leaves the data term orders of magnitude weaker than the odometry
  /// regularizer, which then simply drags the pose back onto the prediction.
  /// Residuals are normalized by it so both terms carry comparable units.
  double sigma = 0.05;
  double huber_delta = 0.05;
  int max_iterations = 8;
  double max_translation_correction = 0.15;
  double max_rotation_correction = 0.052;  // ~3 degrees
  double min_inlier_ratio = 0.55;
  double max_rmse = 0.10;
  /// The 3x3 normal matrix mixes metres with radians, so a raw condition number
  /// changes meaning with the angular unit. The rotation column is scaled by the
  /// mean scan range before this is measured, making the threshold portable.
  double max_condition_number = 200.0;
  std::size_t min_correspondences = 30;
  /// Slack on the non-worsening check, in the frontend's per-beam cost units.
  ///
  /// This cannot be zero and still leave the refinement able to do anything. The
  /// frontend objective is the one the chamfer matcher already minimized, so its
  /// pose is a strict local minimum of it: every correction away from that pose
  /// scores worse there by construction, and a zero-tolerance gate rejects all of
  /// them. The tolerance is what admits a correction the chamfer field is too
  /// coarse to see.
  ///
  /// Scale, with sigma 0.15 and a 0.1 m grid: moving half a cell costs at most
  /// -log(0.05 + 0.95 exp(-0.5 (0.05/0.15)^2)) ~= 0.053 per beam, while moving
  /// the full max_translation_correction of 0.15 m costs ~0.47. Sitting at 0.03
  /// admits the sub-cell band this exists to reach and still rejects a gross move
  /// by more than an order of magnitude.
  double objective_tolerance = 0.03;
};

struct SurfacePoint { double x = 0, y = 0, nx = 0, ny = 0, linearity = 0; };

/// Voxel-deduplicated endpoint cloud with a uniform bucket index, in submap-local
/// coordinates. Append is O(1) and never rebuilds, which is what makes this usable
/// on the submap the frontend actually matches against: that submap is still
/// active and takes an insertion every accepted scan, so anything built once at
/// finish() would serve loop closure and not the tracking path at all.
class SurfaceCloud {
 public:
  SurfaceCloud() = default;
  explicit SurfaceCloud(const SurfaceCloudParams& params) : params_(params) {
    if (!(params_.voxel_size > 0) || !(params_.bucket_size > 0) || !(params_.normal_radius > 0))
      throw std::invalid_argument("Invalid surface cloud geometry");
  }

  [[nodiscard]] const SurfaceCloudParams& params() const { return params_; }
  [[nodiscard]] const std::vector<SurfacePoint>& points() const { return points_; }
  [[nodiscard]] std::size_t size() const { return points_.size(); }
  [[nodiscard]] bool empty() const { return points_.empty(); }

  /// Endpoints must arrive already expressed in the submap frame and, critically,
  /// still continuous: the grid insertion path keeps only integer cells.
  void insert(const ScanPoints& points) {
    for (const auto& [x, y] : points) {
      if (!std::isfinite(x) || !std::isfinite(y)) continue;
      const auto voxel = key(x, y, params_.voxel_size);
      if (!voxels_.emplace(voxel, static_cast<std::uint32_t>(points_.size())).second) continue;
      const auto index = static_cast<std::uint32_t>(points_.size());
      points_.push_back(SurfacePoint{x, y, 0, 0, 0});
      buckets_[key(x, y, params_.bucket_size)].push_back(index);
      dirty_.push_back(index);
      // A new point changes the neighbourhood of everything it is near, so their
      // normals are invalidated too. Only those are recomputed later.
      for_each_near(x, y, params_.normal_radius, [&](std::uint32_t other, double) {
        if (other != index) dirty_.push_back(other);
      });
    }
  }

  /// Recomputes only the normals invalidated since the last call. Run this once,
  /// serially, before a parallel read-only matching phase.
  void refresh_normals() {
    if (dirty_.empty()) return;
    std::sort(dirty_.begin(), dirty_.end());
    dirty_.erase(std::unique(dirty_.begin(), dirty_.end()), dirty_.end());
    for (const auto index : dirty_) {
      auto& point = points_[index];
      double sum_x = 0, sum_y = 0;
      std::size_t count = 0;
      for_each_near(point.x, point.y, params_.normal_radius, [&](std::uint32_t other, double) {
        sum_x += points_[other].x; sum_y += points_[other].y; ++count;
      });
      if (count < params_.min_normal_neighbors) { point.nx = point.ny = point.linearity = 0; continue; }
      const double mean_x = sum_x / static_cast<double>(count), mean_y = sum_y / static_cast<double>(count);
      double cxx = 0, cxy = 0, cyy = 0;
      for_each_near(point.x, point.y, params_.normal_radius, [&](std::uint32_t other, double) {
        const double dx = points_[other].x - mean_x, dy = points_[other].y - mean_y;
        cxx += dx * dx; cxy += dx * dy; cyy += dy * dy;
      });
      const double scale = 1.0 / static_cast<double>(count);
      cxx *= scale; cxy *= scale; cyy *= scale;
      const double trace = cxx + cyy;
      const double gap = std::sqrt(std::max(0.0, trace * trace * 0.25 - (cxx * cyy - cxy * cxy)));
      const double major = trace * 0.5 + gap, minor = trace * 0.5 - gap;
      if (!(major > 1e-12)) { point.nx = point.ny = point.linearity = 0; continue; }
      // Normal is the minor axis. Both closed forms are taken and the better
      // conditioned one kept, because either degenerates on an axis-aligned wall.
      double nx = cxy, ny = minor - cxx;
      const double alternative_x = minor - cyy, alternative_y = cxy;
      if (std::hypot(alternative_x, alternative_y) > std::hypot(nx, ny)) { nx = alternative_x; ny = alternative_y; }
      const double norm = std::hypot(nx, ny);
      if (!(norm > 1e-12)) { point.nx = point.ny = point.linearity = 0; continue; }
      point.nx = nx / norm; point.ny = ny / norm;
      point.linearity = (major - minor) / major;
    }
    dirty_.clear();
  }

  [[nodiscard]] bool has_pending_normals() const { return !dirty_.empty(); }

  static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();

  [[nodiscard]] std::size_t nearest(double x, double y, double radius) const {
    std::size_t best = npos;
    double best_squared = radius * radius;
    for_each_near(x, y, radius, [&](std::uint32_t index, double squared) {
      if (squared < best_squared) { best_squared = squared; best = index; }
    });
    return best;
  }

  [[nodiscard]] std::size_t bytes() const {
    return points_.capacity() * sizeof(SurfacePoint) + buckets_.size() * 64 + voxels_.size() * 32;
  }

 private:
  static std::int64_t key(double x, double y, double size) {
    const auto ix = static_cast<std::int32_t>(std::floor(x / size));
    const auto iy = static_cast<std::int32_t>(std::floor(y / size));
    return (static_cast<std::int64_t>(ix) << 32) |
           static_cast<std::int64_t>(static_cast<std::uint32_t>(iy));
  }

  template <class Visitor>
  void for_each_near(double x, double y, double radius, Visitor&& visit) const {
    const auto rings = static_cast<int>(std::ceil(radius / params_.bucket_size));
    const auto cx = static_cast<std::int32_t>(std::floor(x / params_.bucket_size));
    const auto cy = static_cast<std::int32_t>(std::floor(y / params_.bucket_size));
    const double limit = radius * radius;
    for (int dx = -rings; dx <= rings; ++dx) {
      for (int dy = -rings; dy <= rings; ++dy) {
        const auto bucket = buckets_.find((static_cast<std::int64_t>(cx + dx) << 32) |
            static_cast<std::int64_t>(static_cast<std::uint32_t>(cy + dy)));
        if (bucket == buckets_.end()) continue;
        for (const auto index : bucket->second) {
          const double ex = points_[index].x - x, ey = points_[index].y - y;
          const double squared = ex * ex + ey * ey;
          if (squared <= limit) visit(index, squared);
        }
      }
    }
  }

  SurfaceCloudParams params_{};
  std::vector<SurfacePoint> points_;
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> buckets_;
  std::unordered_map<std::int64_t, std::uint32_t> voxels_;
  std::vector<std::uint32_t> dirty_;
};

/// Closed-form eigenvalues of a symmetric 3x3 matrix, ordered largest first.
inline std::array<double, 3> symmetric_eigenvalues_3x3(const std::array<std::array<double, 3>, 3>& a) {
  const double p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
  const double q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
  if (p1 <= 1e-30) {
    std::array<double, 3> diagonal{a[0][0], a[1][1], a[2][2]};
    std::sort(diagonal.begin(), diagonal.end(), std::greater<double>());
    return diagonal;
  }
  const double p2 = (a[0][0] - q) * (a[0][0] - q) + (a[1][1] - q) * (a[1][1] - q) +
                    (a[2][2] - q) * (a[2][2] - q) + 2.0 * p1;
  const double p = std::sqrt(p2 / 6.0);
  if (!(p > 0) || !std::isfinite(p)) return {q, q, q};
  std::array<std::array<double, 3>, 3> b{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) b[i][j] = (a[i][j] - (i == j ? q : 0.0)) / p;
  const double determinant = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                             b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                             b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
  const double r = std::max(-1.0, std::min(1.0, determinant / 2.0));
  const double phi = std::acos(r) / 3.0;
  const double first = q + 2.0 * p * std::cos(phi);
  const double third = q + 2.0 * p * std::cos(phi + 2.0943951023931953);
  return {first, 3.0 * q - first - third, third};
}

struct IcpResult {
  PoseSample2 pose{};
  bool accepted = false;
  const char* reason = "not_run";
  double inlier_ratio = 0, rmse = 0, condition_number = 0;
  double initial_cost = 0, final_cost = 0;
  std::size_t correspondences = 0;
  int iterations = 0;
};

inline double huber_loss(double residual, double delta) {
  const double magnitude = std::abs(residual);
  return magnitude <= delta ? 0.5 * residual * residual : delta * (magnitude - 0.5 * delta);
}

struct IcpStatistics { std::size_t correspondences = 0; double squared_sum = 0; };

/// Mean robust residual plus the same odometry regularizer the chamfer matcher
/// uses. A scan point with no correspondence is charged the loss at the
/// truncation distance rather than being ignored: otherwise the cheapest move is
/// to slide every point out of correspondence range and declare victory.
inline double point_to_line_objective(const SurfaceCloud& cloud, const ScanPoints& scan,
                                      const PoseSample2& pose, const PoseSample2& prior,
                                      const TrackingOptions& tracking, const IcpOptions& o,
                                      IcpStatistics* statistics = nullptr) {
  if (scan.empty()) return std::numeric_limits<double>::infinity();
  if (!(o.sigma > 0)) return std::numeric_limits<double>::infinity();
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  const double inverse_sigma = 1.0 / o.sigma, huber = o.huber_delta / o.sigma;
  const double unmatched_penalty = huber_loss(o.max_correspondence_distance * inverse_sigma, huber);
  double total = 0;
  IcpStatistics local;
  for (const auto& [x, y] : scan) {
    const double wx = pose.x + c * x - s * y, wy = pose.y + s * x + c * y;
    const auto index = cloud.nearest(wx, wy, o.max_correspondence_distance);
    if (index == SurfaceCloud::npos) { total += unmatched_penalty; continue; }
    const auto& point = cloud.points()[index];
    const double ex = wx - point.x, ey = wy - point.y;
    const double residual = point.linearity >= o.min_linearity ? point.nx * ex + point.ny * ey
                                                               : std::hypot(ex, ey);
    total += huber_loss(residual * inverse_sigma, huber);
    ++local.correspondences;
    local.squared_sum += residual * residual;  // Reported RMSE stays in metres.
  }
  if (statistics) *statistics = local;
  double cost = total / static_cast<double>(scan.size());
  const std::array<double, 3> delta{pose.x - prior.x, pose.y - prior.y, wrap_angle(pose.yaw - prior.yaw)};
  if (tracking.use_full_prior) {
    const auto residual = full_tracking_prior_residual(delta.data(), tracking);
    cost += 0.5 * (residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2]);
  } else {
    const double dx = delta[0] / tracking.prior_translation_sigma;
    const double dy = delta[1] / tracking.prior_translation_sigma;
    const double da = delta[2] / tracking.prior_rotation_sigma;
    cost += 0.5 * (dx * dx + dy * dy + da * da) * tracking.prior_information_scale;
  }
  return cost;
}

/// Local refinement seeded by the chamfer matcher's optimum. `prior` stays the
/// odometry prediction, so the regularizer pulls toward the same point the
/// frontend was regularized to and not toward the seed.
inline IcpResult refine_point_to_line(const SurfaceCloud& cloud, const ScanPoints& scan,
                                      const PoseSample2& seed, const PoseSample2& prior,
                                      const TrackingOptions& tracking, const IcpOptions& o) {
  IcpResult result;
  result.pose = seed;
  if (!(o.sigma > 0)) throw std::invalid_argument("Invalid point-to-line measurement sigma");
  if (scan.empty() || cloud.empty()) { result.reason = "no_surface_cloud"; return result; }
  IcpStatistics statistics;
  double cost = point_to_line_objective(cloud, scan, seed, prior, tracking, o, &statistics);
  result.initial_cost = cost;
  result.final_cost = cost;
  if (statistics.correspondences < o.min_correspondences) {
    result.reason = "too_few_correspondences";
    result.correspondences = statistics.correspondences;
    result.inlier_ratio = static_cast<double>(statistics.correspondences) / static_cast<double>(scan.size());
    return result;
  }
  auto pose = seed;
  double damping = 1e-3;
  for (int iteration = 0; iteration < o.max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    std::array<std::array<double, 3>, 3> H{};
    std::array<double, 3> g{}, step{};
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const double scale = 1.0 / static_cast<double>(scan.size());
    const double inverse_sigma = 1.0 / o.sigma, huber = o.huber_delta / o.sigma;
    for (const auto& [x, y] : scan) {
      const double rx = c * x - s * y, ry = s * x + c * y;
      const double wx = pose.x + rx, wy = pose.y + ry;
      const auto index = cloud.nearest(wx, wy, o.max_correspondence_distance);
      if (index == SurfaceCloud::npos) continue;
      const auto& point = cloud.points()[index];
      const double ex = wx - point.x, ey = wy - point.y;
      // A wall constrains only its normal direction; a corner or a cluttered
      // neighbourhood constrains both, and is the only along-wall information
      // available in a corridor.
      std::array<std::array<double, 3>, 2> rows{};
      std::array<double, 2> residuals{};
      int count = 0;
      if (point.linearity >= o.min_linearity) {
        residuals[0] = (point.nx * ex + point.ny * ey) * inverse_sigma;
        rows[0] = {point.nx * inverse_sigma, point.ny * inverse_sigma,
                   (-point.nx * ry + point.ny * rx) * inverse_sigma};
        count = 1;
      } else {
        residuals[0] = ex * inverse_sigma; rows[0] = {inverse_sigma, 0.0, -ry * inverse_sigma};
        residuals[1] = ey * inverse_sigma; rows[1] = {0.0, inverse_sigma, rx * inverse_sigma};
        count = 2;
      }
      const double magnitude = count == 1 ? std::abs(residuals[0]) : std::hypot(ex, ey) * inverse_sigma;
      const double weight = scale * (magnitude <= huber ? 1.0 : huber / magnitude);
      for (int row = 0; row < count; ++row) {
        for (int a = 0; a < 3; ++a) {
          g[a] += weight * residuals[row] * rows[row][a];
          for (int b = 0; b < 3; ++b) H[a][b] += weight * rows[row][a] * rows[row][b];
        }
      }
    }
    const std::array<double, 3> delta{pose.x - prior.x, pose.y - prior.y, wrap_angle(pose.yaw - prior.yaw)};
    if (tracking.use_full_prior) {
      const auto information = full_tracking_prior_information(tracking);
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) { H[a][b] += information[3 * a + b]; g[a] += information[3 * a + b] * delta[b]; }
    } else {
      for (int a = 0; a < 3; ++a) {
        const double sigma = a == 2 ? tracking.prior_rotation_sigma : tracking.prior_translation_sigma;
        const double information = tracking.prior_information_scale / (sigma * sigma);
        H[a][a] += information; g[a] += delta[a] * information;
      }
    }
    for (int a = 0; a < 3; ++a) { H[a][a] += damping * std::max(1.0, H[a][a]); g[a] = -g[a]; }
    if (!solve_tracking_system(H, g, step)) break;
    const PoseSample2 candidate{pose.x + step[0], pose.y + step[1], wrap_angle(pose.yaw + step[2])};
    // The correction is bounded against the seed, not against the odometry
    // prediction: this is a refinement of an accepted match, so a large move is
    // a correspondence failure rather than a better optimum.
    if (std::hypot(candidate.x - seed.x, candidate.y - seed.y) > o.max_translation_correction ||
        std::abs(wrap_angle(candidate.yaw - seed.yaw)) > o.max_rotation_correction) { damping *= 10; continue; }
    const double candidate_cost = point_to_line_objective(cloud, scan, candidate, prior, tracking, o);
    if (candidate_cost < cost) {
      pose = candidate; cost = candidate_cost; damping = std::max(1e-8, damping * 0.3);
      if (std::hypot(step[0], step[1]) < 1e-5 && std::abs(step[2]) < 1e-6) break;
    } else {
      damping *= 10;
    }
  }
  result.pose = pose;
  result.final_cost = point_to_line_objective(cloud, scan, pose, prior, tracking, o, &statistics);
  result.correspondences = statistics.correspondences;
  result.inlier_ratio = static_cast<double>(statistics.correspondences) / static_cast<double>(scan.size());
  result.rmse = statistics.correspondences ? std::sqrt(statistics.squared_sum /
      static_cast<double>(statistics.correspondences)) : 0.0;

  // Conditioning is measured on the data term alone. Including the regularizer
  // would make every system look well conditioned and the gate would never fire.
  std::array<std::array<double, 3>, 3> data{};
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  double range_sum = 0;
  std::size_t matched = 0;
  for (const auto& [x, y] : scan) {
    const double rx = c * x - s * y, ry = s * x + c * y;
    if (cloud.nearest(pose.x + rx, pose.y + ry, o.max_correspondence_distance) == SurfaceCloud::npos) continue;
    range_sum += std::hypot(x, y); ++matched;
  }
  const double mean_range = matched ? std::max(1e-3, range_sum / static_cast<double>(matched)) : 1.0;
  for (const auto& [x, y] : scan) {
    const double rx = c * x - s * y, ry = s * x + c * y;
    const auto index = cloud.nearest(pose.x + rx, pose.y + ry, o.max_correspondence_distance);
    if (index == SurfaceCloud::npos) continue;
    const auto& point = cloud.points()[index];
    std::array<std::array<double, 3>, 2> rows{};
    int count = 0;
    if (point.linearity >= o.min_linearity) {
      rows[0] = {point.nx, point.ny, (-point.nx * ry + point.ny * rx) / mean_range};
      count = 1;
    } else {
      rows[0] = {1.0, 0.0, -ry / mean_range};
      rows[1] = {0.0, 1.0, rx / mean_range};
      count = 2;
    }
    for (int row = 0; row < count; ++row)
      for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) data[a][b] += rows[row][a] * rows[row][b];
  }
  const auto eigenvalues = symmetric_eigenvalues_3x3(data);
  result.condition_number = eigenvalues[2] > 1e-12 ? eigenvalues[0] / eigenvalues[2]
                                                   : std::numeric_limits<double>::infinity();

  result.reason = "low_inlier_ratio";
  if (result.inlier_ratio < o.min_inlier_ratio) return result;
  result.reason = "high_rmse";
  if (!(result.rmse <= o.max_rmse)) return result;
  result.reason = "ill_conditioned";
  if (!(result.condition_number <= o.max_condition_number)) return result;
  result.reason = "correction_out_of_bounds";
  if (std::hypot(pose.x - seed.x, pose.y - seed.y) > o.max_translation_correction ||
      std::abs(wrap_angle(pose.yaw - seed.yaw)) > o.max_rotation_correction) return result;
  result.reason = "accepted";
  result.accepted = true;
  return result;
}
}  // namespace belugaslam
#endif
