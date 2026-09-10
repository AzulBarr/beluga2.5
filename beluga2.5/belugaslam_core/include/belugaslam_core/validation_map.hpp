#ifndef BELUGASLAM_CORE_VALIDATION_MAP_HPP
#define BELUGASLAM_CORE_VALIDATION_MAP_HPP
#include <cstdint>
#include <memory>
#include "robust_tracking.hpp"

namespace belugaslam {
// Owned immutable snapshot: neither subsequent grid insertion nor PGO can
// change its occupancy, field, or world pose. Built only from a finished
// historical reference submap, never the query's active submap.
class ValidationMap {
 public:
  ValidationMap(const std::vector<float>& cells, int width, int height,
                double resolution, double ox, double oy, PoseSample2 world_pose)
      : field_(cells, width, height, resolution, ox, oy), width_(width), height_(height),
        resolution_(resolution), ox_(ox), oy_(oy), world_pose_(world_pose) {
    known_.reserve(cells.size());
    for (float v : cells) known_.push_back(std::isfinite(v) && v != 0);
  }
  [[nodiscard]] PoseSample2 local_pose(const PoseSample2& world) const {
    const double c = std::cos(world_pose_.yaw), s = std::sin(world_pose_.yaw);
    const double dx = world.x - world_pose_.x, dy = world.y - world_pose_.y;
    return {c*dx+s*dy, -s*dx+c*dy, wrap_angle(world.yaw-world_pose_.yaw)};
  }
  [[nodiscard]] bool observed_endpoint(const PoseSample2& world, const std::pair<double,double>& beam) const {
    const auto p = local_pose(world);
    const double c=std::cos(p.yaw), s=std::sin(p.yaw);
    const double x=p.x+c*beam.first-s*beam.second, y=p.y+s*beam.first+c*beam.second;
    const double gx=std::floor((x-ox_)/resolution_), gy=std::floor((y-oy_)/resolution_);
    if (!std::isfinite(gx) || !std::isfinite(gy) || gx<0 || gy<0 || gx>=width_ || gy>=height_) return false;
    return known_[static_cast<std::size_t>(gy)*width_+static_cast<std::size_t>(gx)] != 0;
  }
  [[nodiscard]] double log_likelihood(const ScanPoints& common_scan, const PoseSample2& world,
                                     const TrackingOptions& options, double beta) const {
    // Generalized Bayes: the existing likelihood-field kernel is a robust
    // sensor score, not a calibrated density over complete laser scans.
    return beta * options.effective_beams *
        tracking_score(field_, common_scan, local_pose(world), options).mean_log_likelihood;
  }
  [[nodiscard]] bool usable() const { return field_.occupied_cells() > 0; }
 private:
  TrackingField field_;
  std::vector<std::uint8_t> known_;
  int width_, height_;
  double resolution_, ox_, oy_;
  PoseSample2 world_pose_;
};
} // namespace belugaslam
#endif
