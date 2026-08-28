#ifndef __BELUGASLAM_CORE_SUBMAP_HPP__
#define __BELUGASLAM_CORE_SUBMAP_HPP__

#include <memory>
#include <vector>
#include <set>
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
    
    // The grid's origin is now strictly in the local frame of the submap.
    // We center it so the submap's origin (0,0) is precisely in the middle of the grid matrix.
    Sophus::SE2d grid_local_origin{Sophus::SO2d{0.0}, Eigen::Vector2d{- (width * resolution) / 2.0, - (height * resolution) / 2.0}};

    grid_ = std::make_shared<LogOddsGrid>(width, height, resolution, grid_local_origin);
  }

  /// Get the local LogOddsGrid.
  std::shared_ptr<LogOddsGrid> grid() { return grid_; }
  std::shared_ptr<const LogOddsGrid> grid() const { return grid_; }

  /// Get the global pose of this submap.
  const Sophus::SE2d& global_pose() const { return global_pose_; }

  /// Set the global pose of this submap (used during Pose Graph Optimization).
  void set_global_pose(const Sophus::SE2d& pose) { global_pose_ = pose; }

  /// Number of scans inserted into this submap.
  int num_insertions() const { return num_insertions_; }

  /// Increment the insertion count.
  void add_insertion() { num_insertions_++; }

  /// Check if the submap is finished (frozen).
  bool is_finished() const { return is_finished_; }

  /// Mark the submap as finished and compute its radial signature.
  void finish() { 
    is_finished_ = true; 
    compute_radial_signature();
  }

  /// Deep copy the submap (used for copy-on-write in particle filter)
  std::shared_ptr<Submap> clone() const {
    auto new_submap = std::make_shared<Submap>(*this);
    new_submap->grid_ = std::make_shared<LogOddsGrid>(*this->grid_);
    return new_submap;
  }

  /// Get the computed radial signature (histogram of occupied cell distances)
  const std::vector<double>& radial_signature() const { return radial_signature_; }

private:
  void compute_radial_signature() {
    // 50 bins of 0.5 meters = up to 25 meters radius
    const int NUM_BINS = 50;
    const double BIN_SIZE = 0.5;
    radial_signature_.assign(NUM_BINS, 0.0);

    double res = grid_->resolution();
    double cx = grid_->width() * res / 2.0;
    double cy = grid_->height() * res / 2.0;

    int valid_cells = 0;

    for (int y = 0; y < grid_->height(); ++y) {
      for (int x = 0; x < grid_->width(); ++x) {
        // Only consider highly occupied cells
        if (grid_->at(x, y) > 0.5) {
          // Distance from the submap's center (0,0) in local coordinates
          double dx = (x * res) - cx;
          double dy = (y * res) - cy;
          double dist = std::sqrt(dx*dx + dy*dy);

          int bin = static_cast<int>(dist / BIN_SIZE);
          if (bin >= 0 && bin < NUM_BINS) {
            radial_signature_[bin] += 1.0;
            valid_cells++;
          }
        }
      }
    }

    // Normalize the histogram to make it robust to different densities
    if (valid_cells > 0) {
      for (int i = 0; i < NUM_BINS; ++i) {
        radial_signature_[i] /= valid_cells;
      }
    }
  }

  Sophus::SE2d global_pose_;
  std::shared_ptr<LogOddsGrid> grid_;
  int num_insertions_;
  bool is_finished_;
  std::vector<double> radial_signature_;
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

/**
 * \brief Represents a global hypothesis (cluster) in the multi-hypothesis SLAM system.
 * 
 * Each cluster owns a shared submap history that all its member particles reference.
 * Particles within a cluster handle local pose refinement; the cluster handles
 * the global map, loop closure detection, and pose graph optimization.
 */
struct Cluster {
  size_t id = 0;
  SubmapList submaps;                                    // Shared submap history
  int loop_closure_cooldown = 0;                         // Per-cluster cooldown
  bool has_loop_closure = false;                         // Was this cluster born from a loop closure?
  Sophus::SE2d loop_drift_error;                         // Drift error at the moment of loop closure
  std::set<std::pair<size_t, size_t>> optimized_loops;   // Already-optimized loop pairs
};

#endif // __BELUGASLAM_CORE_SUBMAP_HPP__
