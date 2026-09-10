#ifndef BELUGASLAM_CORE_GRID_UPDATE_HPP
#define BELUGASLAM_CORE_GRID_UPDATE_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace belugaslam {

// The same endpoint-excluding Bresenham traversal, without a vector per ray.
template <class Visitor>
void visit_ray_cells(int x0, int y0, int x1, int y1,
                     int width, int height, Visitor&& visit) {
  const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int error = dx - dy;
  while (x0 != x1 || y0 != y1) {
    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) visit(y0 * width + x0);
    const int twice_error = 2 * error;
    if (twice_error > -dy) { error -= dy; x0 += sx; }
    if (twice_error < dx) { error += dx; y0 += sy; }
  }
}

// Reused across sequential grid insertions. Epochs avoid clearing the whole grid
// each scan; changing the grid dimensions is safe because each call starts an epoch.
class ScanCellUpdates {
 public:
  void begin(std::size_t cells) {
    if (marks_.size() < cells) marks_.resize(cells, 0);
    if (epoch_ == std::numeric_limits<std::uint32_t>::max()) {
      std::fill(marks_.begin(), marks_.end(), 0);
      epoch_ = 0;
    }
    ++epoch_;
    endpoints.clear();
  }

  bool first_visit(std::size_t index) {
    if (marks_[index] == epoch_) return false;
    marks_[index] = epoch_;
    return true;
  }

  std::vector<std::pair<int, int>> endpoints;

 private:
  std::vector<std::uint32_t> marks_;
  std::uint32_t epoch_ = 0;
};

// All hits precede all misses, so one epoch implements both deduplication and hit
// priority. Grid growth and footprint clearing happen before calling this helper.
inline void apply_scan_cells(std::vector<float>& cells, int width, int height,
                             int origin_x, int origin_y, float hit, float miss,
                             float clamp, ScanCellUpdates& scratch,
                             std::vector<int>& hits, std::vector<int>& misses) {
  hits.clear(); misses.clear();
  for (const auto& endpoint : scratch.endpoints) {
    const auto [x, y] = endpoint;
    if (x < 0 || x >= width || y < 0 || y >= height) continue;
    const int index = y * width + x;
    if (!scratch.first_visit(static_cast<std::size_t>(index))) continue;
    cells[index] = std::min(cells[index] + hit, clamp);
    hits.push_back(index);
  }
  const int origin = origin_y * width + origin_x;
  for (const auto& endpoint : scratch.endpoints) {
    visit_ray_cells(origin_x, origin_y, endpoint.first, endpoint.second, width, height,
        [&](int index) {
          if (index == origin || !scratch.first_visit(static_cast<std::size_t>(index))) return;
          cells[index] = std::max(cells[index] + miss, -clamp);
          misses.push_back(index);
        });
  }
}

}  // namespace belugaslam
#endif
