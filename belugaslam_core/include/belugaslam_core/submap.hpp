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

  void finish() {
    if (is_finished_) return;
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
    const double center_x = grid_->width() * resolution / 2.0;
    const double center_y = grid_->height() * resolution / 2.0;
    int occupied_cells = 0;

    for (int y = 0; y < grid_->height(); ++y) {
      for (int x = 0; x < grid_->width(); ++x) {
        if (grid_->at(x, y) <= 0.5F) continue;
        const double dx = x * resolution - center_x;
        const double dy = y * resolution - center_y;
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

  /** Freeze active submaps the robot has driven out of.
   *
   * Fixed-size grids cannot grow, so once the robot is farther than `max_distance`
   * from a submap origin every raycast falls outside that grid and the submap only
   * occupies an active slot. Submaps are ordered oldest first and their origins follow
   * the trajectory, so only the front needs checking. The newest active submap is never
   * abandoned: something has to receive the current scan.
   */
  std::vector<SubmapId> finish_abandoned_submaps(
      const Eigen::Vector2d& robot_position, double max_distance) {
    std::vector<SubmapId> finished_ids;
    while (active_submaps.size() > 1) {
      const auto& oldest = active_submaps.front();
      const double distance =
          (robot_position - oldest->global_pose().translation()).norm();
      if (distance < max_distance) break;
      oldest->finish();
      oldest->set_role(SubmapRole::kAuthoritative);
      history.push_back(oldest);
      finished_ids.push_back(oldest->id());
      active_submaps.erase(active_submaps.begin());
    }
    return finished_ids;
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
