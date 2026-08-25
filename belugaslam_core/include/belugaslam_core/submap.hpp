#ifndef __BELUGASLAM_CORE_SUBMAP_HPP__
#define __BELUGASLAM_CORE_SUBMAP_HPP__

#include <memory>
#include <vector>
#include <sophus/se2.hpp>
#include "belugaslam_core/particle.hpp"

/**
 * \brief Represents a local map patch (Submap).
 * 
 * Instead of maintaining a single monolithic grid, the SLAM system
 * maintains a sequence of submaps. Each submap has its own local
 * LogOddsGrid and a global pose that anchors it to the world frame.
 */
class Submap {
public:
  /**
   * \brief Construct a new Submap at a given global pose.
   * \param global_pose The origin of this submap in the global frame.
   * \param width Width of the submap grid in cells.
   * \param height Height of the submap grid in cells.
   * \param resolution Resolution of the grid in meters/cell.
   */
  Submap(const Sophus::SE2d& global_pose, int width, int height, double resolution)
      : global_pose_(global_pose),
        num_insertions_(0),
        is_finished_(false) {
    
    // We center the submap origin so the robot starts in the middle of it.
    Sophus::SE2d grid_origin_offset{Sophus::SO2d{0.0}, Eigen::Vector2d{- (width * resolution) / 2.0, - (height * resolution) / 2.0}};
    Sophus::SE2d grid_global_origin = global_pose * grid_origin_offset;

    grid_ = std::make_shared<LogOddsGrid>(width, height, resolution, grid_global_origin);
  }

  /// Get the local LogOddsGrid.
  std::shared_ptr<LogOddsGrid> grid() { return grid_; }
  std::shared_ptr<const LogOddsGrid> grid() const { return grid_; }

  /// Get the global pose of this submap.
  const Sophus::SE2d& global_pose() const { return global_pose_; }

  /// Number of scans inserted into this submap.
  int num_insertions() const { return num_insertions_; }

  /// Increment the insertion count.
  void add_insertion() { num_insertions_++; }

  /// Check if the submap is finished (frozen).
  bool is_finished() const { return is_finished_; }

  /// Mark the submap as finished.
  void finish() { is_finished_ = true; }

  /// Deep copy the submap (used for copy-on-write in particle filter)
  std::shared_ptr<Submap> clone() const {
    auto new_submap = std::make_shared<Submap>(*this);
    new_submap->grid_ = std::make_shared<LogOddsGrid>(*this->grid_);
    return new_submap;
  }

private:
  Sophus::SE2d global_pose_;
  std::shared_ptr<LogOddsGrid> grid_;
  int num_insertions_;
  bool is_finished_;
};

/**
 * \brief A list of submaps maintained by each particle.
 */
struct SubmapList {
  std::vector<std::shared_ptr<Submap>> history;
  std::shared_ptr<Submap> active_submap;

  // Make sure active_submap is independent if shared
  void make_active_unique() {
    if (active_submap && active_submap.use_count() > 1) {
      active_submap = active_submap->clone();
    }
  }
};

#endif // __BELUGASLAM_CORE_SUBMAP_HPP__
