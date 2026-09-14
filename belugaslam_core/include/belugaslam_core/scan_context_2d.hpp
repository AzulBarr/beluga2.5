#ifndef BELUGASLAM_CORE_SCAN_CONTEXT_2D_HPP
#define BELUGASLAM_CORE_SCAN_CONTEXT_2D_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace belugaslam {

struct ScanContext2D {
  std::size_t rings = 20;
  std::size_t sectors = 60;
  double max_radius = 25.0;
  std::vector<float> cells;
  std::vector<float> ring_key;
  std::vector<float> sector_key;

  [[nodiscard]] bool valid() const {
    return rings > 0 && sectors > 0 && cells.size() == rings * sectors &&
           ring_key.size() == rings && sector_key.size() == sectors;
  }
};

struct ScanContext2DMatch {
  double distance = 1.0;
  int sector_shift = 0;
  double yaw = 0.0;
};

// Occupancy-only 2-D adaptation of Scan Context: every polar bin stores the
// fraction of sampled cells that are occupied.  The per-ring mean is a yaw-
// invariant retrieval key; the full context keeps azimuthal structure.
template <class Grid>
[[nodiscard]] ScanContext2D make_scan_context_2d(
    const Grid& grid, std::size_t rings = 20, std::size_t sectors = 60,
    double max_radius = 25.0, float occupied_threshold = 0.5F) {
  ScanContext2D result;
  result.rings = std::max<std::size_t>(1, rings);
  result.sectors = std::max<std::size_t>(4, sectors);
  result.max_radius = std::max(1.0e-3, max_radius);
  result.cells.assign(result.rings * result.sectors, 0.0F);
  std::vector<unsigned int> samples(result.cells.size(), 0U);
  std::vector<unsigned int> occupied(result.cells.size(), 0U);

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double resolution = grid.resolution();
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const double px = grid.origin_x() + (static_cast<double>(x) + 0.5) * resolution;
      const double py = grid.origin_y() + (static_cast<double>(y) + 0.5) * resolution;
      const double radius = std::hypot(px, py);
      if (radius >= result.max_radius) continue;
      double angle = std::atan2(py, px);
      if (angle < 0.0) angle += kTwoPi;
      const auto ring = std::min(result.rings - 1,
          static_cast<std::size_t>(radius / result.max_radius * result.rings));
      const auto sector = std::min(result.sectors - 1,
          static_cast<std::size_t>(angle / kTwoPi * result.sectors));
      const auto index = ring * result.sectors + sector;
      ++samples[index];
      if (grid.at(x, y) > occupied_threshold) ++occupied[index];
    }
  }

  for (std::size_t i = 0; i < result.cells.size(); ++i) {
    if (samples[i]) result.cells[i] = static_cast<float>(occupied[i]) / samples[i];
  }
  result.ring_key.assign(result.rings, 0.0F);
  result.sector_key.assign(result.sectors, 0.0F);
  for (std::size_t r = 0; r < result.rings; ++r) {
    for (std::size_t s = 0; s < result.sectors; ++s) {
      const float value = result.cells[r * result.sectors + s];
      result.ring_key[r] += value;
      result.sector_key[s] += value;
    }
    result.ring_key[r] /= static_cast<float>(result.sectors);
  }
  for (float& value : result.sector_key) value /= static_cast<float>(result.rings);
  return result;
}

[[nodiscard]] inline double scan_context_ring_key_distance(
    const ScanContext2D& a, const ScanContext2D& b) {
  if (!a.valid() || !b.valid() || a.rings != b.rings) return std::numeric_limits<double>::infinity();
  double squared = 0.0;
  for (std::size_t r = 0; r < a.rings; ++r) {
    const double d = static_cast<double>(a.ring_key[r]) - b.ring_key[r];
    squared += d * d;
  }
  return std::sqrt(squared / static_cast<double>(a.rings));
}

// Full rotation-invariant comparison.  To make the planar descriptor less
// brittle to modest lateral viewpoint changes, each query ring may match the
// same or an adjacent reference ring.  This is deliberately bounded (±1 ring)
// so corridors with genuinely different radial structure do not collapse to
// the same descriptor.
[[nodiscard]] inline ScanContext2DMatch match_scan_context_2d(
    const ScanContext2D& reference, const ScanContext2D& query,
    int lateral_ring_tolerance = 1) {
  ScanContext2DMatch best;
  if (!reference.valid() || !query.valid() || reference.rings != query.rings ||
      reference.sectors != query.sectors) return best;

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  best.distance = std::numeric_limits<double>::infinity();
  for (std::size_t shift = 0; shift < reference.sectors; ++shift) {
    double dot = 0.0, nr = 0.0, nq = 0.0;
    for (std::size_t qr = 0; qr < query.rings; ++qr) {
      for (std::size_t s = 0; s < query.sectors; ++s) {
        const double q = query.cells[qr * query.sectors + s];
        if (q <= 0.0) continue;
        const std::size_t rs = (s + shift) % reference.sectors;
        double rv = 0.0;
        const int lo = std::max<int>(0, static_cast<int>(qr) - lateral_ring_tolerance);
        const int hi = std::min<int>(static_cast<int>(reference.rings) - 1,
                                     static_cast<int>(qr) + lateral_ring_tolerance);
        for (int rr = lo; rr <= hi; ++rr)
          rv = std::max(rv, static_cast<double>(reference.cells[static_cast<std::size_t>(rr) * reference.sectors + rs]));
        dot += q * rv;
        nq += q * q;
        nr += rv * rv;
      }
    }
    const double similarity = (nq > 1.0e-12 && nr > 1.0e-12) ? dot / std::sqrt(nq * nr) : 0.0;
    const double distance = std::clamp(1.0 - similarity, 0.0, 1.0);
    if (distance < best.distance) {
      best.distance = distance;
      best.sector_shift = static_cast<int>(shift);
      best.yaw = kTwoPi * static_cast<double>(shift) / static_cast<double>(reference.sectors);
      if (best.yaw > 3.14159265358979323846) best.yaw -= kTwoPi;
    }
  }
  return best;
}

}  // namespace belugaslam

#endif
