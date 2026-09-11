#ifndef __BELUGASLAM_CORE_SUBMAP_HPP__
#define __BELUGASLAM_CORE_SUBMAP_HPP__
#include "validation_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sophus/se2.hpp>

#include "belugaslam_core/particle.hpp"
#include "belugaslam_core/grid_update.hpp"
#include "belugaslam_core/robust_tracking.hpp"
#include "belugaslam_core/probability_matching.hpp"
#include "belugaslam_core/point_to_line_icp.hpp"
#include "belugaslam_core/derived_cache.hpp"

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
 * cast the free space as misses, and touch every cell at most once. All hits are written
 * before ray misses, because applying complete beams one at a time is wrong twice
 * over: a cell crossed by twenty beams would be counted free twenty times, and a cell
 * that is a return for one beam but merely crossed by another would have its hit eroded
 * by that beam's miss. Cartographer avoids both by marking each cell once per insertion
 * and by deferring the end of the update, so hits are already in place when misses land.
 *
 * \param grid The submap's log-odds grid, in submap-local coordinates.
 * \param T_submap_robot Robot pose in the submap frame.
 * \param scan Range returns in the robot frame, the same frame the pose is measured in.
 * \param ray_origins Optional origins in that same frame, one per return. Empty
 * means the sensor is co-located with the robot (legacy offline callers).
 * \param hit_scratch,miss_scratch Reused buffers, so the cost per scan is not allocation.
 */
inline void insert_scan_into_submap_grid(
    LogOddsGrid& grid, const Sophus::SE2d& T_submap_robot,
    const std::vector<std::pair<double, double>>& scan,
    const ScanInsertionParams& params, std::vector<int>& hit_scratch,
    std::vector<int>& miss_scratch,
    belugaslam::ScanCellUpdates* reusable_updates = nullptr,
    const std::vector<std::pair<double, double>>& ray_origins = {}) {
  if (!ray_origins.empty() && ray_origins.size() != scan.size())
    throw std::invalid_argument("Each scan endpoint requires one ray origin");
  for (const auto& origin : ray_origins)
    if (!std::isfinite(origin.first) || !std::isfinite(origin.second))
      throw std::invalid_argument("Nonfinite ray origin");
  if (scan.empty()) return;

  // 1. Grow first, so that no return is silently clipped and every index below is final.
  const Eigen::Vector2d robot_origin = T_submap_robot.translation();
  double min_x = robot_origin.x(), max_x = min_x;
  double min_y = robot_origin.y(), max_y = min_y;
  for (const auto& origin : ray_origins) {
    const Eigen::Vector2d sensor = T_submap_robot * Eigen::Vector2d{origin.first, origin.second};
    min_x = std::min(min_x, sensor.x()); max_x = std::max(max_x, sensor.x());
    min_y = std::min(min_y, sensor.y()); max_y = std::max(max_y, sensor.y());
  }
  for (const auto& point : scan) {
    const Eigen::Vector2d hit = T_submap_robot * Eigen::Vector2d{point.first, point.second};
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

  const auto [gx0, gy0] = to_cell(robot_origin.x(), robot_origin.y());

  // The robot's own footprint is forced free before the scan, never after, so a return
  // landing on it is not silently erased.
  const int radius_cells = static_cast<int>(params.robot_radius / grid.resolution());
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (inside(gx0 + dx, gy0 + dy)) grid.at(gx0 + dx, gy0 + dy) = -params.clamp;
    }
  }

  // Grow/cell-index conversion stays identical. One reusable epoch marker per
  // cell replaces sorting all repeated free-space traversals.
  belugaslam::ScanCellUpdates temporary_updates;
  auto& updates = reusable_updates ? *reusable_updates : temporary_updates;
  updates.begin(grid.data().size());
  updates.endpoints.reserve(scan.size());
  updates.origins.reserve(ray_origins.size());
  for (const auto& origin : ray_origins) {
    const Eigen::Vector2d sensor = T_submap_robot * Eigen::Vector2d{origin.first, origin.second};
    updates.origins.push_back(to_cell(sensor.x(), sensor.y()));
  }
  for (const auto& point : scan) {
    const Eigen::Vector2d hit = T_submap_robot * Eigen::Vector2d{point.first, point.second};
    updates.endpoints.push_back(to_cell(hit.x(), hit.y()));
  }
  belugaslam::apply_scan_cells(grid.data(), grid.width(), grid.height(), gx0, gy0,
      params.l_occ, params.l_free, params.clamp, updates, hit_scratch, miss_scratch);
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
    if (grid_.use_count() != 1) grid_ = std::make_shared<LogOddsGrid>(*grid_);
    tracking_field_.reset(); probability_field_.reset();
    return *grid_;
  }

  [[nodiscard]] const LogOddsGrid& grid() const { return *grid_; }
  // Prepare serially before the parallel read-only matching phase. Invalidated
  // whenever mutable_grid() is requested or the frozen grid is cropped.
  [[nodiscard]] std::shared_ptr<const belugaslam::TrackingField> tracking_field() const {
    if (!tracking_field_) tracking_field_ = std::make_shared<belugaslam::TrackingField>(
        grid_->data(), grid_->width(), grid_->height(), grid_->resolution(), grid_->origin_x(), grid_->origin_y());
    return tracking_field_;
  }
  [[nodiscard]] std::shared_ptr<const belugaslam::ProbabilityField> probability_field() const {
    if (!probability_field_) probability_field_ = std::make_shared<belugaslam::ProbabilityField>(
        grid_->data(), grid_->width(), grid_->height(), grid_->resolution(), grid_->origin_x(), grid_->origin_y());
    return probability_field_;
  }
  /// Continuous endpoints kept for sub-resolution refinement. The grid insertion
  /// path keeps only integer cells, so these have to be captured separately or
  /// the point-to-line matcher inherits the very quantization it exists to beat.
  /// Copy-on-write, like the grid: a hypothesis that never inserts never pays.
  void insert_surface_points(const Sophus::SE2d& T_submap_robot,
                             const std::vector<std::pair<double, double>>& scan,
                             const belugaslam::SurfaceCloudParams& params) {
    if (is_finished_) throw std::runtime_error("Attempted to mutate a finished submap cloud");
    if (!surface_cloud_) surface_cloud_ = std::make_shared<belugaslam::SurfaceCloud>(params);
    else if (surface_cloud_.use_count() != 1)
      surface_cloud_ = std::make_shared<belugaslam::SurfaceCloud>(*surface_cloud_);
    std::vector<std::pair<double, double>> transformed;
    transformed.reserve(scan.size());
    for (const auto& point : scan) {
      const Eigen::Vector2d hit = T_submap_robot * Eigen::Vector2d{point.first, point.second};
      transformed.emplace_back(hit.x(), hit.y());
    }
    surface_cloud_->insert(transformed);
    // Insertion is serial; matching is parallel and read-only. Normals are brought
    // up to date here so the matching phase never mutates shared state.
    surface_cloud_->refresh_normals();
  }

  /// Null on a submap that never received an insertion, and on every frozen one:
  /// the cloud cannot be rebuilt from a cropped grid, so rather than let it
  /// accumulate over a long trajectory it is dropped at finish(). Tracking always
  /// matches an active submap, which is the only place this refinement applies.
  [[nodiscard]] std::shared_ptr<const belugaslam::SurfaceCloud> surface_cloud() const {
    return surface_cloud_;
  }

  [[nodiscard]] const Sophus::SE2d& global_pose() const { return global_pose_; }
  void set_global_pose(const Sophus::SE2d& pose) { global_pose_ = pose; }
  [[nodiscard]] const Sophus::SE2d& local_pose() const { return local_pose_; }
  void set_local_pose(const Sophus::SE2d& pose) { local_pose_ = pose; }
  [[nodiscard]] std::uint64_t anchor_sequence() const { return anchor_sequence_; }
  void set_anchor_sequence(std::uint64_t sequence) { anchor_sequence_ = sequence; }
  // Trial PGO changes poses only. Grid writes still detach in mutable_grid().
  [[nodiscard]] std::shared_ptr<Submap> clone_for_pose() const {
    return std::make_shared<Submap>(*this);
  }
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
    if (grid_.use_count() != 1) grid_ = std::make_shared<LogOddsGrid>(*grid_);
    grid_->crop_to_known_cells(kCropMarginCells);
    tracking_field_.reset(); probability_field_.reset();
    is_finished_ = true;
    surface_cloud_.reset();
    compute_radial_signature();
    loop_cache_ = std::make_shared<belugaslam::DerivedCache<LoopMatchingData>>();
  }

  struct LoopMatchingData {
    std::vector<float> distances;
    std::vector<double> scores;
    [[nodiscard]] std::size_t bytes() const {
      return distances.capacity()*sizeof(float) + scores.capacity()*sizeof(double);
    }
  };
  using LoopDataPtr = std::shared_ptr<const LoopMatchingData>;
  [[nodiscard]] LoopDataPtr loop_matching_data() const {
    if (!is_finished_ || !loop_cache_) return {};
    return loop_cache_->acquire([&] { return compute_loop_matching_data(); });
  }
  [[nodiscard]] const void* loop_cache_identity() const { return loop_cache_.get(); }
  [[nodiscard]] std::pair<std::size_t, std::uint64_t> loop_cache_statistics() const {
    return loop_cache_ ? loop_cache_->statistics() : std::pair<std::size_t,std::uint64_t>{0,0};
  }
  void release_loop_cache() const { if (loop_cache_) loop_cache_->release(); }
  void release_tracking_field() const { tracking_field_.reset(); probability_field_.reset(); }

  /** Distance to the closest occupied cell in local submap coordinates. */
  [[nodiscard]] float distance_at(double x, double y) const {
    return loop_cell_at(x,y,loop_matching_data()).first;
  }
  void prepare_loop_matching() const { (void)loop_matching_data(); }
  [[nodiscard]] std::pair<float, double> loop_cell_at(double x, double y) const {
    return loop_cell_at(x,y,loop_matching_data());
  }
  // The production matcher acquires one shared payload for the entire search.
  // No cache locks or shared-pointer churn occur inside its endpoint loop.
  [[nodiscard]] std::pair<float, double> loop_cell_at(double x, double y, const LoopDataPtr& data) const {
    if (!data || !std::isfinite(x) || !std::isfinite(y)) return {std::numeric_limits<float>::infinity(), 0.0};
    const double fx = std::floor((x-grid_->origin_x())/grid_->resolution());
    const double fy = std::floor((y-grid_->origin_y())/grid_->resolution());
    if (fx < 0 || fy < 0 || fx >= grid_->width() || fy >= grid_->height())
      return {std::numeric_limits<float>::infinity(), 0.0};
    const auto index = static_cast<std::size_t>(fy)*grid_->width()+static_cast<std::size_t>(fx);
    return {data->distances[index], data->scores[index]};
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
  [[nodiscard]] LoopMatchingData compute_loop_matching_data() const {
    const int width = grid_->width();
    const int height = grid_->height();
    constexpr float kInfinity = 1.0e6F;
    constexpr float kDiagonal = 1.41421356237F;
    LoopMatchingData data;
    auto& field = data.distances;
    field.assign(static_cast<std::size_t>(width) * height, kInfinity);
    auto at = [width, &field](int x, int y) -> float& {
      return field[static_cast<std::size_t>(y * width + x)];
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
    for (float& value : field) value *= resolution;
    data.scores.reserve(field.size());
    for (float distance : field) {
      const double normalized = static_cast<double>(distance)/0.20;
      data.scores.push_back(std::exp(-0.5*normalized*normalized));
    }
    return data;
  }

  SubmapId id_;
  Sophus::SE2d global_pose_;
  Sophus::SE2d local_pose_ = global_pose_;
  std::uint64_t anchor_sequence_ = 0;
  std::shared_ptr<LogOddsGrid> grid_;
  mutable std::shared_ptr<const belugaslam::TrackingField> tracking_field_;
  mutable std::shared_ptr<const belugaslam::ProbabilityField> probability_field_;
  std::shared_ptr<belugaslam::SurfaceCloud> surface_cloud_;
  int num_insertions_;
  bool is_finished_;
  SubmapRole role_;
  std::vector<double> radial_signature_;
  std::shared_ptr<belugaslam::DerivedCache<LoopMatchingData>> loop_cache_;
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
  Sophus::SE2d local_pose;  // immutable frontend pose, never rewritten by PGO
  std::uint64_t sequence = 0;  // survives point-cloud trimming; shared scan identity
};

struct TrajectorySample {
  std::uint64_t sequence = 0;
  SubmapId submap_id = 0;
  Sophus::SE2d T_submap_robot;
  // The pose the frontend published for this scan, in the global frame of that moment.
  // PGO never rewrites it, so it keeps the uncorrected trajectory next to the corrected
  // one that T_submap_robot yields through its submap.
  Sophus::SE2d frontend_pose;
  // Stamp of the scan this sample came from, so a trajectory read back from the graph
  // can be dated without asking the node for its sequence-to-stamp record.
  std::int64_t stamp_ns = 0;
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
  std::uint64_t reference_sequence = 0;
  std::uint64_t query_sequence = 0;
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
  std::vector<TrajectorySample> trajectory_samples;
  std::vector<NodeSubmapConstraint> node_submap_constraints;
  std::vector<NodeNodeConstraint> local_trajectory_constraints;
  SubmapId next_submap_id = 0;
  ScanNodeId next_node_id = 0;
  bool has_last_keyframe_pose = false;
  Sophus::SE2d last_keyframe_pose;
  double last_keyframe_time = 0.0;

  // The oldest active submap is the normal reference. Handover occurs when its
  // predecessor finishes, so rejected scans cannot pin tracking to a retired map.
  // Confirmed recovery can explicitly select another mature active reference.
  bool has_matching_submap = false;
  SubmapId matching_submap_id = 0;

  [[nodiscard]] std::shared_ptr<const Submap> matching_submap() const {
    if (has_matching_submap) return find_submap(matching_submap_id);
    return active_submaps.empty() ? nullptr : active_submaps.front();
  }

  void make_active_unique() {
    for (auto& submap : active_submaps) {
      if (submap && submap.use_count() > 1) submap = submap->clone();
    }
  }

  [[nodiscard]] std::shared_ptr<Submap> find_submap(SubmapId id) const {
    // Tracking usually references one of at most two active submaps.
    for (const auto& submap : active_submaps) if (submap->id() == id) return submap;
    // The count-driven lifecycle appends frozen submaps in increasing ID order.
    const auto it = std::lower_bound(history.begin(), history.end(), id,
        [](const auto& submap, SubmapId value) { return submap->id() < value; });
    return it != history.end() && (*it)->id() == id ? *it : nullptr;
  }

  [[nodiscard]] const TrajectoryNode* find_node(ScanNodeId id) const {
    for (const auto& node : trajectory_nodes) if (node.id == id) return &node;
    return nullptr;
  }

  [[nodiscard]] const TrajectoryNode* find_node_by_sequence(std::uint64_t sequence) const {
    const auto it = std::lower_bound(trajectory_nodes.begin(), trajectory_nodes.end(), sequence,
        [](const auto& node, auto value) { return node.sequence < value; });
    return it != trajectory_nodes.end() && it->sequence == sequence ? &*it : nullptr;
  }

  [[nodiscard]] const TrajectorySample* find_sample(std::uint64_t sequence) const {
    const auto it = std::lower_bound(trajectory_samples.begin(), trajectory_samples.end(), sequence,
        [](const auto& sample, auto value) { return sample.sequence < value; });
    return it != trajectory_samples.end() && it->sequence == sequence ? &*it : nullptr;
  }

  [[nodiscard]] bool pose_at_sequence(std::uint64_t sequence, Sophus::SE2d& pose) const {
    if (const auto* node = find_node_by_sequence(sequence)) { pose = node->global_pose; return true; }
    const auto* sample = find_sample(sequence);
    if (!sample) return false;
    const auto submap = find_submap(sample->submap_id);
    if (!submap) return false;
    const auto local = submap->local_pose() * sample->T_submap_robot;
    // Apply a continuous correction field to the immutable local trajectory.
    // A rigid submap readout between individually optimized nodes gives a
    // different answer immediately next to a keyframe and at each handover.
    const auto next = std::lower_bound(trajectory_nodes.begin(), trajectory_nodes.end(), sequence,
        [](const auto& node, auto seq) { return node.sequence < seq; });
    if (trajectory_nodes.empty()) {
      pose = submap->global_pose() * sample->T_submap_robot;
    } else if (next == trajectory_nodes.begin()) {
      pose = next->global_pose * next->local_pose.inverse() * local;
    } else if (next == trajectory_nodes.end()) {
      const auto& last = trajectory_nodes.back();
      pose = last.global_pose * last.local_pose.inverse() * local;
    } else {
      const auto& previous = *std::prev(next);
      double alpha = static_cast<double>(sequence-previous.sequence)/(next->sequence-previous.sequence);
      const auto* left_sample = find_sample(previous.sequence);
      const auto* right_sample = find_sample(next->sequence);
      if (left_sample && right_sample && right_sample->stamp_ns > left_sample->stamp_ns) {
        const long double left = left_sample->stamp_ns, right = right_sample->stamp_ns;
        alpha = static_cast<double>((static_cast<long double>(sample->stamp_ns)-left)/(right-left));
      }
      alpha = std::clamp(alpha, 0.0, 1.0);
      const auto left_correction = previous.global_pose * previous.local_pose.inverse();
      const auto right_correction = next->global_pose * next->local_pose.inverse();
      pose = left_correction * Sophus::SE2d::exp(alpha * (left_correction.inverse()*right_correction).log()) * local;
    }
    return true;
  }

  // Submap frames are born at the robot pose of their anchor scan. A later PGO
  // can move that frame differently from the preceding (overlapping) submap.
  [[nodiscard]] std::shared_ptr<Submap> find_submap_by_anchor(std::uint64_t sequence) const {
    for (const auto& sm : history) if (sm->anchor_sequence() == sequence) return sm;
    for (const auto& sm : active_submaps) if (sm->anchor_sequence() == sequence) return sm;
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

/** Weighted mean of SE(2) poses.
 *
 * Translation is a plain weighted average; rotation is a circular mean, because angles
 * wrap and averaging them arithmetically puts the mean of 179 and -179 degrees at zero
 * instead of 180. Falls back to an unweighted mean when the weights sum to nothing, and
 * to the identity when there is nothing to average.
 */
inline Sophus::SE2d weighted_mean_pose(
    const std::vector<Sophus::SE2d>& poses, const std::vector<double>& weights) {
  if (poses.empty()) return Sophus::SE2d{};
  double total = 0.0;
  for (std::size_t i = 0; i < poses.size() && i < weights.size(); ++i) total += weights[i];
  const bool uniform = !(total > 0.0);

  Eigen::Vector2d translation = Eigen::Vector2d::Zero();
  double cos_sum = 0.0;
  double sin_sum = 0.0;
  double applied = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const double weight = uniform ? 1.0 : (i < weights.size() ? weights[i] : 0.0);
    translation += weight * poses[i].translation();
    const double angle = poses[i].so2().log();
    cos_sum += weight * std::cos(angle);
    sin_sum += weight * std::sin(angle);
    applied += weight;
  }
  if (!(applied > 0.0)) return poses.front();
  translation /= applied;
  // All poses exactly opposed: the circular mean is undefined, keep the first angle.
  const double angle = (cos_sum == 0.0 && sin_sum == 0.0)
      ? poses.front().so2().log() : std::atan2(sin_sum, cos_sum);
  return Sophus::SE2d{Sophus::SO2d{angle}, translation};
}

struct Hypothesis {
  // Explicit normalized graph mass. Particle weights are conditional on this graph.
  double log_mass = 0.0;
  std::shared_ptr<const belugaslam::ValidationMap> validation_map;
  bool validation_loop = false;
  std::uint64_t validation_event = 0;
  std::size_t validation_age = 0;
  std::string validation_status = "none";
  double predictive_log_evidence = 0.0;

  std::size_t id = 0;
  SubmapList submaps;
  std::size_t optimized_inter_constraints_count = 0;
  std::size_t optimized_node_count = 0;
  std::size_t last_pgo_attempt_node_count = 0;
  Sophus::SE2d T_global_local;
  std::uint64_t last_loop_sequence = 0;
  bool has_loop = false;
  bool pgo_usable = true;

  /** Continuous local-SLAM pose of this hypothesis.
   *
   * The particles represent the posterior; this represents the trajectory the submaps
   * are built along. They are deliberately different things. Taking the highest weight
   * particle each scan means the trajectory can hop between particles that are each
   * smooth but offset from one another, and that hop is indistinguishable from real
   * motion at insertion time: the same wall lands in slightly different places on
   * consecutive scans, and the resulting thick or doubled walls are baked into the
   * occupancy grid where no later optimisation can undo them -- a pose graph can move a
   * submap, never repair its interior.
   *
   * This pose is seeded from the cluster mean. Each scan uses odometry prediction
   * and native-grid matching. The opt-in proposal_mean readout then uses the entire
   * scored proposal cloud if concentration and scan-fit gates pass. The final pose
   * is shared by insertion, graph node construction, publication and the next prior.
   */
  Sophus::SE2d local_pose;
  bool has_local_pose = false;
  Eigen::Matrix3d pose_covariance = Eigen::Matrix3d::Zero();
  bool has_pose_covariance = false;
  std::string pose_source = "frontend", proposal_pose_decision = "not_evaluated";
  double proposal_ess=0, proposal_local_mass=0, proposal_position_std=0, proposal_yaw_std=0, proposal_mean_offset=0;
  bool tracking_evaluated = false;
  bool tracking_usable = true;
  double tracking_overlap = 0.0;
  std::size_t tracking_failures = 0;
  std::string tracking_status = "bootstrap";
  double tracking_log_likelihood = 0.0, tracking_correction = 0.0;
  std::string icp_decision = "disabled";
  double icp_inlier_ratio = 0.0, icp_rmse = 0.0, icp_condition_number = 0.0, icp_correction = 0.0;
  belugaslam::PriorMatrix3 tracking_prior_covariance{};
  bool tracking_prior_evaluated = false;
  bool has_pending_recovery = false;
  Sophus::SE2d recovery_pose;
  SubmapId recovery_reference = 0, tracking_reference = 0;
  std::size_t recovery_confirmations = 0;
  std::uint64_t recovery_last_sequence = 0;
  std::uint64_t last_recovery_attempt = std::numeric_limits<std::uint64_t>::max();
  struct PendingSplit { Sophus::SE2d pose; std::size_t count = 0; std::uint64_t sequence = 0; };
  std::vector<PendingSplit> pending_splits;
};

#endif  // __BELUGASLAM_CORE_SUBMAP_HPP__
