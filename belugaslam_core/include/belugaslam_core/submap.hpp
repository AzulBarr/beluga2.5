#ifndef __BELUGASLAM_CORE_SUBMAP_HPP__
#define __BELUGASLAM_CORE_SUBMAP_HPP__

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include <sophus/se2.hpp>

#include "belugaslam_core/particle.hpp"

using SubmapId = std::uint64_t;
using ScanNodeId = std::uint64_t;

/** Log-odds increments and clamps applied by one scan insertion. */
struct ScanInsertionParams {
  float l_occ = 1.2F;
  float l_free = -0.2F;
  float clamp = 5.0F;
  double robot_radius = 0.0;
};

/** Bresenham line from (x0,y0) toward (x1,y1), excluding the endpoint, clipped to the grid. */
inline std::vector<std::pair<int, int>> bresenham_line(
    int x0, int y0, int x1, int y1, int max_x, int max_y) {
  std::vector<std::pair<int, int>> line;
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (x0 == x1 && y0 == y1) break;
    if (x0 >= 0 && x0 < max_x && y0 >= 0 && y0 < max_y) line.push_back({x0, y0});
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }
  }
  return line;
}

/**
 * \brief Inserts one scan into a submap grid with Cartographer's update semantics.
 *
 * The sequence is: grow the grid to fit the whole scan, mark the returns as hits, ray
 * cast the free space as misses, and touch every cell at most once. Nothing is written
 * until the entire scan has been collected, because applying beam by beam is wrong twice
 * over: a cell crossed by twenty beams would be counted free twenty times, and a cell
 * that is a return for one beam but merely crossed by another would have its hit eroded
 * by that beam's miss. Cartographer avoids both by marking each cell once per insertion
 * and by deferring the end of the update, so hits are already in place when misses land.
 *
 * \param grid The submap's log-odds grid, in submap-local coordinates.
 * \param T_submap_sensor Sensor pose in the submap frame.
 * \param scan Range returns in the sensor frame.
 * \param hit_scratch,miss_scratch Reused buffers, so the cost per scan is not allocation.
 */
inline void insert_scan_into_submap_grid(
    LogOddsGrid& grid, const Sophus::SE2d& T_submap_sensor,
    const std::vector<std::pair<double, double>>& scan,
    const ScanInsertionParams& params, std::vector<int>& hit_scratch,
    std::vector<int>& miss_scratch) {
  if (scan.empty()) return;

  // 1. Grow first, so that no return is silently clipped and every index below is final.
  const Eigen::Vector2d sensor_origin = T_submap_sensor.translation();
  double min_x = sensor_origin.x(), max_x = min_x;
  double min_y = sensor_origin.y(), max_y = min_y;
  for (const auto& point : scan) {
    const Eigen::Vector2d hit = T_submap_sensor * Eigen::Vector2d{point.first, point.second};
    min_x = std::min(min_x, hit.x());
    max_x = std::max(max_x, hit.x());
    min_y = std::min(min_y, hit.y());
    max_y = std::max(max_y, hit.y());
  }
  grid.grow_to_include(min_x, min_y, max_x, max_y);

  const auto to_cell = [&grid](double x, double y) {
    return std::pair<int, int>{
        static_cast<int>(std::floor((x - grid.origin_x()) / grid.resolution())),
        static_cast<int>(std::floor((y - grid.origin_y()) / grid.resolution()))};
  };
  const auto inside = [&grid](int gx, int gy) {
    return gx >= 0 && gx < grid.width() && gy >= 0 && gy < grid.height();
  };

  const auto [gx0, gy0] = to_cell(sensor_origin.x(), sensor_origin.y());

  // The robot's own footprint is forced free before the scan, never after, so a return
  // landing on it is not silently erased.
  const int radius_cells = static_cast<int>(params.robot_radius / grid.resolution());
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (inside(gx0 + dx, gy0 + dy)) grid.at(gx0 + dx, gy0 + dy) = -params.clamp;
    }
  }

  // 2. Collect the whole scan before writing anything.
  hit_scratch.clear();
  miss_scratch.clear();
  const int origin_index = gy0 * grid.width() + gx0;

  for (const auto& point : scan) {
    const Eigen::Vector2d hit = T_submap_sensor * Eigen::Vector2d{point.first, point.second};
    const auto [gx1, gy1] = to_cell(hit.x(), hit.y());
    if (inside(gx1, gy1)) hit_scratch.push_back(gy1 * grid.width() + gx1);

    // The endpoint is excluded by bresenham_line, so this is the free beam interior. A
    // return that fell outside the grid still clears everything it crossed.
    for (const auto& cell :
         bresenham_line(gx0, gy0, gx1, gy1, grid.width(), grid.height())) {
      const int index = cell.second * grid.width() + cell.first;
      if (index == origin_index) continue;
      miss_scratch.push_back(index);
    }
  }

  const auto deduplicate = [](std::vector<int>& cells) {
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
  };
  deduplicate(hit_scratch);
  deduplicate(miss_scratch);

  // 3. Hits first, once per cell.
  for (const int index : hit_scratch) {
    grid.at(index) = std::min(grid.at(index) + params.l_occ, params.clamp);
  }
  // 4. Then misses, once per cell, skipping every cell that was a return.
  for (const int index : miss_scratch) {
    if (std::binary_search(hit_scratch.begin(), hit_scratch.end(), index)) continue;
    grid.at(index) = std::max(grid.at(index) + params.l_free, -params.clamp);
  }
}

enum class SubmapRole { kAuthoritative, kRedundant, kProvisional };

/** A local probability grid with a pose in the global map frame. */
class Submap {
public:
  Submap(SubmapId id, const Sophus::SE2d& pose, int width, int height, double resolution)
      : id_(id), global_pose_(pose), num_insertions_(0), is_finished_(false),
        role_(SubmapRole::kProvisional) {
    const Sophus::SE2d grid_local_origin{
        Sophus::SO2d{0.0},
        Eigen::Vector2d{-(width * resolution) / 2.0, -(height * resolution) / 2.0}};
    grid_ = std::make_shared<LogOddsGrid>(width, height, resolution, grid_local_origin);
  }

  [[nodiscard]] SubmapId id() const { return id_; }

  LogOddsGrid& mutable_grid() {
    if (is_finished_) throw std::runtime_error("Attempted to mutate a finished submap grid");
    return *grid_;
  }

  [[nodiscard]] const LogOddsGrid& grid() const { return *grid_; }
  [[nodiscard]] const Sophus::SE2d& global_pose() const { return global_pose_; }
  void set_global_pose(const Sophus::SE2d& pose) { global_pose_ = pose; }
  [[nodiscard]] SubmapRole role() const { return role_; }
  void set_role(SubmapRole role) { role_ = role; }
  [[nodiscard]] int num_insertions() const { return num_insertions_; }
  void add_insertion() { ++num_insertions_; }
  [[nodiscard]] bool is_finished() const { return is_finished_; }

  /// Slack kept around the observed box when cropping, so that a scan matcher query
  /// just outside a wall still lands on a real cell instead of off the grid.
  static constexpr int kCropMarginCells = 5;

  /// The grid grows freely while the submap is active; it is cropped to what was
  /// actually observed on the way to being frozen, and only then are the derived
  /// structures built, so they are sized to the cropped grid.
  void finish() {
    if (is_finished_) return;
    grid_->crop_to_known_cells(kCropMarginCells);
    is_finished_ = true;
    compute_radial_signature();
    compute_distance_field();
  }

  /** Distance to the closest occupied cell in local submap coordinates. */
  [[nodiscard]] float distance_at(double local_x, double local_y) const {
    if (!distance_field_) return std::numeric_limits<float>::infinity();
    const int gx = static_cast<int>(std::floor((local_x - grid_->origin_x()) / grid_->resolution()));
    const int gy = static_cast<int>(std::floor((local_y - grid_->origin_y()) / grid_->resolution()));
    if (gx < 0 || gx >= grid_->width() || gy < 0 || gy >= grid_->height()) {
      return std::numeric_limits<float>::infinity();
    }
    return (*distance_field_)[static_cast<std::size_t>(gy * grid_->width() + gx)];
  }

  /** Frozen grid and distance field remain shared; active grids are copied. */
  [[nodiscard]] std::shared_ptr<Submap> clone() const {
    auto clone = std::make_shared<Submap>(*this);
    if (!is_finished_) clone->grid_ = std::make_shared<LogOddsGrid>(*grid_);
    return clone;
  }

  [[nodiscard]] const std::vector<double>& radial_signature() const { return radial_signature_; }

private:
  void compute_radial_signature() {
    constexpr int kNumBins = 50;
    constexpr double kBinSize = 0.5;
    radial_signature_.assign(kNumBins, 0.0);
    const double resolution = grid_->resolution();
    int occupied_cells = 0;

    // Radii are measured from the submap origin, which is local (0, 0). The grid is no
    // longer necessarily centred on it: grow_to_include() expands whichever side the
    // scan needs, so the centre of the cell array drifts away from the origin.
    for (int y = 0; y < grid_->height(); ++y) {
      for (int x = 0; x < grid_->width(); ++x) {
        if (grid_->at(x, y) <= 0.5F) continue;
        const double dx = grid_->origin_x() + (x + 0.5) * resolution;
        const double dy = grid_->origin_y() + (y + 0.5) * resolution;
        const int bin = static_cast<int>(std::hypot(dx, dy) / kBinSize);
        if (bin >= 0 && bin < kNumBins) {
          radial_signature_[bin] += 1.0;
          ++occupied_cells;
        }
      }
    }
    if (occupied_cells > 0) {
      for (double& value : radial_signature_) value /= occupied_cells;
    }
  }

  /** Linear-time chamfer distance transform used by the loop scan matcher. */
  void compute_distance_field() {
    const int width = grid_->width();
    const int height = grid_->height();
    constexpr float kInfinity = 1.0e6F;
    constexpr float kDiagonal = 1.41421356237F;
    auto field = std::make_shared<std::vector<float>>(
        static_cast<std::size_t>(width * height), kInfinity);
    auto at = [width, &field](int x, int y) -> float& {
      return (*field)[static_cast<std::size_t>(y * width + x)];
    };
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (grid_->at(x, y) > 0.5F) at(x, y) = 0.0F;
      }
    }
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        float value = at(x, y);
        if (x > 0) value = std::min(value, at(x - 1, y) + 1.0F);
        if (y > 0) value = std::min(value, at(x, y - 1) + 1.0F);
        if (x > 0 && y > 0) value = std::min(value, at(x - 1, y - 1) + kDiagonal);
        if (x + 1 < width && y > 0) value = std::min(value, at(x + 1, y - 1) + kDiagonal);
        at(x, y) = value;
      }
    }
    for (int y = height - 1; y >= 0; --y) {
      for (int x = width - 1; x >= 0; --x) {
        float value = at(x, y);
        if (x + 1 < width) value = std::min(value, at(x + 1, y) + 1.0F);
        if (y + 1 < height) value = std::min(value, at(x, y + 1) + 1.0F);
        if (x + 1 < width && y + 1 < height) value = std::min(value, at(x + 1, y + 1) + kDiagonal);
        if (x > 0 && y + 1 < height) value = std::min(value, at(x - 1, y + 1) + kDiagonal);
        at(x, y) = value;
      }
    }
    const float resolution = static_cast<float>(grid_->resolution());
    for (float& value : *field) value *= resolution;
    distance_field_ = std::move(field);
  }

  SubmapId id_;
  Sophus::SE2d global_pose_;
  std::shared_ptr<LogOddsGrid> grid_;
  int num_insertions_;
  bool is_finished_;
  SubmapRole role_;
  std::vector<double> radial_signature_;
  std::shared_ptr<const std::vector<float>> distance_field_;
};

/** Immutable keyframe range data shared between graph hypotheses. */
struct ScanNodeData {
  std::vector<std::pair<double, double>> returns;
  std::uint64_t sequence = 0;
};

struct TrajectoryNode {
  ScanNodeId id = 0;
  std::shared_ptr<const ScanNodeData> constant_data;
  Sophus::SE2d global_pose;
};

enum class ConstraintTag { kIntraSubmap, kInterSubmap };

/** Cartographer-style bipartite edge between one scan node and one submap. */
struct NodeSubmapConstraint {
  SubmapId submap_id = 0;
  ScanNodeId node_id = 0;
  Sophus::SE2d T_submap_node;
  double translation_weight = 1.0;
  double rotation_weight = 1.0;
  ConstraintTag tag = ConstraintTag::kIntraSubmap;
  double score = 1.0;
  double overlap = 1.0;
};

/** Local trajectory prior between consecutive retained scan nodes. */
struct NodeNodeConstraint {
  ScanNodeId from_node_id = 0;
  ScanNodeId to_node_id = 0;
  Sophus::SE2d T_from_to;
  double translation_weight = 1.0;
  double rotation_weight = 1.0;
};

struct FinishedSubmapEvent {
  std::size_t hypothesis_id = 0;
  SubmapId query_submap_id = 0;
};

struct SubmapList {
  std::vector<std::shared_ptr<Submap>> history;
  std::vector<std::shared_ptr<Submap>> active_submaps;
  std::vector<TrajectoryNode> trajectory_nodes;
  std::vector<NodeSubmapConstraint> node_submap_constraints;
  std::vector<NodeNodeConstraint> local_trajectory_constraints;
  SubmapId next_submap_id = 0;
  ScanNodeId next_node_id = 0;
  bool has_last_keyframe_pose = false;
  Sophus::SE2d last_keyframe_pose;

  void make_active_unique() {
    for (auto& submap : active_submaps) {
      if (submap && submap.use_count() > 1) submap = submap->clone();
    }
  }

  [[nodiscard]] std::shared_ptr<Submap> find_submap(SubmapId id) const {
    for (const auto& submap : history) if (submap->id() == id) return submap;
    for (const auto& submap : active_submaps) if (submap->id() == id) return submap;
    return nullptr;
  }

  [[nodiscard]] const TrajectoryNode* find_node(ScanNodeId id) const {
    for (const auto& node : trajectory_nodes) if (node.id == id) return &node;
    return nullptr;
  }

  [[nodiscard]] std::vector<ScanNodeId> insertion_nodes(SubmapId submap_id) const {
    std::vector<ScanNodeId> result;
    for (const auto& constraint : node_submap_constraints) {
      if (constraint.tag == ConstraintTag::kIntraSubmap && constraint.submap_id == submap_id) {
        result.push_back(constraint.node_id);
      }
    }
    return result;
  }

  std::vector<SubmapId> finish_ready_submaps(int max_insertions) {
    std::vector<SubmapId> finished_ids;
    auto it = active_submaps.begin();
    while (it != active_submaps.end()) {
      if ((*it)->num_insertions() >= max_insertions) {
        auto submap = *it;
        submap->finish();
        submap->set_role(SubmapRole::kAuthoritative);
        history.push_back(submap);
        finished_ids.push_back(submap->id());
        it = active_submaps.erase(it);
      } else {
        ++it;
      }
    }
    return finished_ids;
  }

  /** Frees a slot so that a new submap can be added without exceeding `max_active`.
   *
   * Cartographer keeps exactly two active submaps and erases the front -- already
   * finished, because its counts line up by construction -- when a third would be
   * added. Our distance trigger can start a submap before the counts line up, so the
   * front is always finished here too, since the lifecycle is driven purely by the scan
   * count. This is therefore a guard rather than a regular path: it is what makes "at
   * most two active submaps per hypothesis" an invariant rather than a consequence. A
   * submap that is no longer one of the two most recent can never receive another scan,
   * so finishing it costs nothing even if its count fell short.
   */
  std::vector<SubmapId> make_room_for_new_submap(std::size_t max_active) {
    std::vector<SubmapId> finished_ids;
    while (!active_submaps.empty() && active_submaps.size() >= max_active) {
      auto submap = active_submaps.front();
      submap->finish();
      submap->set_role(SubmapRole::kAuthoritative);
      history.push_back(submap);
      finished_ids.push_back(submap->id());
      active_submaps.erase(active_submaps.begin());
    }
    return finished_ids;
  }

  /** Global bounding box of every submap held, in metres.
   *
   * Each submap grid is a rectangle in its own frame and its submap carries a rotation,
   * so all four corners have to be transformed: taking only two would clip the box
   * whenever a submap is not axis aligned with the world.
   *
   * \return false when there is no submap yet, leaving the outputs untouched.
   */
  [[nodiscard]] bool bounding_box(
      double& min_x, double& min_y, double& max_x, double& max_y) const {
    bool any = false;
    const auto accumulate = [&](const std::shared_ptr<Submap>& submap) {
      if (!submap) return;
      const auto& grid = submap->grid();
      const double x0 = grid.origin_x();
      const double y0 = grid.origin_y();
      const double x1 = x0 + grid.width() * grid.resolution();
      const double y1 = y0 + grid.height() * grid.resolution();
      for (const auto& corner : {Eigen::Vector2d{x0, y0}, Eigen::Vector2d{x1, y0},
                                 Eigen::Vector2d{x0, y1}, Eigen::Vector2d{x1, y1}}) {
        const Eigen::Vector2d point = submap->global_pose() * corner;
        if (!any) {
          min_x = max_x = point.x();
          min_y = max_y = point.y();
          any = true;
        } else {
          min_x = std::min(min_x, point.x());
          max_x = std::max(max_x, point.x());
          min_y = std::min(min_y, point.y());
          max_y = std::max(max_y, point.y());
        }
      }
    };
    for (const auto& submap : history) accumulate(submap);
    for (const auto& submap : active_submaps) accumulate(submap);
    return any;
  }

  [[nodiscard]] std::size_t inter_constraint_count() const {
    return static_cast<std::size_t>(std::count_if(
        node_submap_constraints.begin(), node_submap_constraints.end(),
        [](const NodeSubmapConstraint& constraint) {
          return constraint.tag == ConstraintTag::kInterSubmap;
        }));
  }

  /** Release point clouds once their nodes no longer belong to an active submap. */
  void trim_scan_data_outside_active_submaps() {
    std::set<SubmapId> active_ids;
    for (const auto& submap : active_submaps) active_ids.insert(submap->id());
    std::set<ScanNodeId> retained_nodes;
    for (const auto& constraint : node_submap_constraints) {
      if (constraint.tag == ConstraintTag::kIntraSubmap &&
          active_ids.count(constraint.submap_id) != 0) {
        retained_nodes.insert(constraint.node_id);
      }
    }
    for (auto& node : trajectory_nodes) {
      if (retained_nodes.count(node.id) == 0) node.constant_data.reset();
    }
  }
};

struct Hypothesis {
  std::size_t id = 0;
  SubmapList submaps;
  std::size_t optimized_inter_constraints_count = 0;
};

#endif  // __BELUGASLAM_CORE_SUBMAP_HPP__
