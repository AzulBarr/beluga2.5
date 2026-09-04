#ifndef __PARTICLE_H__
#define __PARTICLE_H__
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <cstdint>
#include <vector>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <beluga/sensor/likelihood_field_prob_model.hpp>
#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga/primitives.hpp>

#include "belugaslam_core/grid_config.hpp"

static constexpr std::size_t GRID_COLS = kGridCols;
static constexpr std::size_t GRID_ROWS = kGridRows;

// Submap size (decoupled from global map size).
// Dynamically calculated to always be exactly 12m x 12m regardless of the resolution chosen in grid_config.py
static constexpr std::size_t SUBMAP_COLS = static_cast<std::size_t>(12.0 / kGridResolution);
static constexpr std::size_t SUBMAP_ROWS = static_cast<std::size_t>(12.0 / kGridResolution);
static constexpr double GRID_RESOLUTION = kGridResolution;
static constexpr double ORIGIN_X = kOriginX;
static constexpr double ORIGIN_Y = kOriginY;


/*
* \file
* \brief Implementation of a log-odds occupancy grid and StaticOccupancyGrid.
*/

/**
 * \brief Probabilistic occupancy grid using log-odds representation.
 *
 * This class maintains a fixed-size, square grid where each cell stores a floating-point
 * value representing the log-odds of occupancy. This representation allows for
 * efficient Bayesian updates through simple addition, avoiding numerical instability.
 * 
 * \note A log-odds value of 0.0 corresponds to "unknown".
 */
class LogOddsGrid {
public:
  /// Default constructor using global constants for initialization.
  LogOddsGrid() : LogOddsGrid(
    GRID_COLS, 
    GRID_ROWS,
    GRID_RESOLUTION,
    Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{ORIGIN_X, ORIGIN_Y}}
    ) {}
  
  /**
   * \brief Constructs a LogOddsGrid with specified dimensions and origin.
   * \param w Grid width in cells.
   * \param h Grid height in cells.
   * \param res Grid resolution in meters per cell.
   * \param origin The pose of the grid's bottom-left corner in the map frame.
   */
  LogOddsGrid(int w, int h, double res, const Sophus::SE2d& origin = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{ORIGIN_X, ORIGIN_Y}}):
    width_(w),
    height_(h),
    resolution_(res),
    origin_pose_(origin),
    data_(w * h, 0.0f){} 

   /**
   * \brief Accesses a cell value by grid coordinates (x, y).
   * \param x Column index.
   * \param y Row index.
   * \return Reference to the log-odds value of the cell.
   */
  [[nodiscard]] float& at(int x, int y){ return data_[y * width_ + x]; }

  /**
   * \brief Accesses a cell value by grid coordinates (x, y) (read-only).
   * \param x Column index.
   * \param y Row index.
   * \return Constant reference to the log-odds value of the cell.
   */
  [[nodiscard]] const float& at(int x, int y) const { return data_[y * width_ + x]; }

  /**
   * \brief Accesses a cell value by linear index.
   * \param index The flat index in row-major order.
   * \return Reference to the log-odds value of the cell.
   */
  float& at(int index){ return data_[index]; }

  /**
   * \brief Accesses a cell value by linear index (read-only).
   * \param index The flat index in row-major order.
   * \return Constant reference to the log-odds value.
   */
  [[nodiscard]] const float& at(int index) const { return data_[index]; }  

  /// Get the log-odds grid origin in the map frame.
  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_pose_; }

  /// Get the x-coordinate of the grid origin.
  [[nodiscard]] const double origin_x() const { return origin_pose_.translation().x(); }
  
  /// Get the y-coordinate of the grid origin.
  [[nodiscard]] const double origin_y() const { return origin_pose_.translation().y(); }

  /// Get the width of the log-odds grid.
  [[nodiscard]] int width() const { return width_; }

  /// Get the height of the log-odds grid.
  [[nodiscard]] int height() const { return height_; }

  /// Get the resolution of the log-odds grid discretization, in meters.
  [[nodiscard]] double resolution() const { return resolution_; }

  /// Get a reference to the underlying data storage (read-only)
  [[nodiscard]] const std::vector<float>& data() const { return data_; }

  /// Get a reference to the underlying data storage
  [[nodiscard]] std::vector<float>& data() { return data_; }

  /**
   * \brief Re-sizes the grid to a new extent and clears it.
   *
   * Used to rebuild a derived view whose bounds are recomputed from scratch each time.
   * When the extent is unchanged this is a plain clear, with no reallocation, which is
   * the common case: the bounds of a map grow slowly.
   */
  void reset(int w, int h, const Sophus::SE2d& origin) {
    const std::size_t cells = static_cast<std::size_t>(w) * h;
    if (w != width_ || h != height_) {
      data_.assign(cells, 0.0f);
      width_ = w;
      height_ = h;
    } else {
      std::fill(data_.begin(), data_.end(), 0.0f);
    }
    origin_pose_ = origin;
  }

  /**
   * \brief Shrinks the grid to the cells that were actually observed.
   *
   * The counterpart of grow_to_include(): while a submap is active its grid expands to
   * whatever the scans reach, which over a whole submap is far more than the sensor ever
   * observed. Cartographer's Submap2D::Finish() calls ComputeCroppedGrid() for exactly
   * this reason, right before marking the submap finished.
   *
   * Like growth, cropping only moves the origin: every retained cell keeps its
   * coordinates in the grid frame, so poses and constraints measured against the owner
   * of the grid stay valid. A cell counts as observed when its log-odds differ from 0,
   * the same convention the rest of the code uses for "unknown"; `margin_cells` of slack
   * is kept around the observed box so that queries just outside a wall still land on a
   * real cell instead of falling off the grid.
   *
   * \param margin_cells Cells of unknown space to keep around the observed box.
   * \return true if the grid was actually resized.
   */
  bool crop_to_known_cells(int margin_cells) {
    int min_x = width_;
    int max_x = -1;
    int min_y = height_;
    int max_y = -1;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (data_[static_cast<std::size_t>(y) * width_ + x] == 0.0f) continue;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }
    }
    // Nothing was ever observed: leave the grid alone rather than produce an empty one.
    if (max_x < 0) return false;

    min_x = std::max(0, min_x - margin_cells);
    min_y = std::max(0, min_y - margin_cells);
    max_x = std::min(width_ - 1, max_x + margin_cells);
    max_y = std::min(height_ - 1, max_y + margin_cells);
    if (min_x == 0 && min_y == 0 && max_x == width_ - 1 && max_y == height_ - 1) return false;

    const int new_width = max_x - min_x + 1;
    const int new_height = max_y - min_y + 1;
    std::vector<float> cropped(static_cast<std::size_t>(new_width) * new_height, 0.0f);
    for (int y = 0; y < new_height; ++y) {
      const auto row_begin =
          data_.begin() + static_cast<std::size_t>(y + min_y) * width_ + min_x;
      std::copy(row_begin, row_begin + new_width,
                cropped.begin() + static_cast<std::size_t>(y) * new_width);
    }

    data_ = std::move(cropped);
    width_ = new_width;
    height_ = new_height;
    /// Dropping cells from the low side moves the origin up, the mirror of growth.
    origin_pose_.translation() +=
        Eigen::Vector2d{min_x * resolution_, min_y * resolution_};
    return true;
  }

  /**
   * \brief Enlarges the grid so that an axis-aligned box fits inside it.
   *
   * Cartographer resizes a submap grid to contain the sensor origin and every scan
   * endpoint before ray casting; this is the equivalent. Growth only ever prepends or
   * appends cells, so every existing cell keeps its coordinates in the grid frame. What
   * moves is the origin, by exactly the number of cells prepended. The pose of whatever
   * owns the grid -- a submap's global_pose, for instance -- is not affected.
   *
   * \param min_x,min_y,max_x,max_y The box to include, in grid-frame coordinates.
   * \return true if the grid was actually resized.
   */
  bool grow_to_include(double min_x, double min_y, double max_x, double max_y) {
    /// Grow in blocks so that a slowly expanding scan does not reallocate every frame.
    constexpr int kGrowthChunk = 32;
    /// A diverged pose must not be able to request an unbounded allocation.
    constexpr int kMaxCellsPerSide = 4000;

    const auto cell = [this](double value, double origin) {
      return static_cast<int>(std::floor((value - origin) / resolution_));
    };
    const int gx_min = cell(min_x, origin_x());
    const int gx_max = cell(max_x, origin_x());
    const int gy_min = cell(min_y, origin_y());
    const int gy_max = cell(max_y, origin_y());

    int pad_left = gx_min < 0 ? -gx_min : 0;
    int pad_right = gx_max >= width_ ? gx_max - width_ + 1 : 0;
    int pad_bottom = gy_min < 0 ? -gy_min : 0;
    int pad_top = gy_max >= height_ ? gy_max - height_ + 1 : 0;
    if (pad_left == 0 && pad_right == 0 && pad_bottom == 0 && pad_top == 0) return false;

    const auto round_up = [](int pad) {
      return pad > 0 ? ((pad + kGrowthChunk - 1) / kGrowthChunk) * kGrowthChunk : 0;
    };
    pad_left = round_up(pad_left);
    pad_right = round_up(pad_right);
    pad_bottom = round_up(pad_bottom);
    pad_top = round_up(pad_top);

    const int new_width = width_ + pad_left + pad_right;
    const int new_height = height_ + pad_bottom + pad_top;
    if (new_width > kMaxCellsPerSide || new_height > kMaxCellsPerSide) return false;

    /// 0.0 is the log-odds of an unknown cell, so the new border starts unobserved.
    std::vector<float> grown(static_cast<std::size_t>(new_width) * new_height, 0.0f);
    for (int y = 0; y < height_; ++y) {
      const auto row_begin = data_.begin() + static_cast<std::size_t>(y) * width_;
      const auto destination = grown.begin() +
          static_cast<std::size_t>(y + pad_bottom) * new_width + pad_left;
      std::copy(row_begin, row_begin + width_, destination);
    }

    data_ = std::move(grown);
    width_ = new_width;
    height_ = new_height;
    /// The grid frame is axis aligned in every use, so prepending cells is a pure shift.
    origin_pose_.translation() -=
        Eigen::Vector2d{pad_left * resolution_, pad_bottom * resolution_};
    return true;
  }

private:
  int width_;
  int height_;
  double resolution_;
  Sophus::SE2d origin_pose_;
  std::vector<float> data_;
};

/**
 * \brief Occupancy grid whose extent is decided at run time.
 *
 * StaticOccupancyGrid fixes its size at compile time, which suits a map of known bounds
 * but not one derived from submaps that grow and move. This holds the same information
 * with a run-time extent and exposes exactly the accessors a publisher needs.
 */
class DynamicOccupancyGrid {
public:
  DynamicOccupancyGrid() = default;

  /// Re-sizes to match a reference extent and clears to `fill`.
  void reset(int w, int h, double resolution, const Sophus::SE2d& origin, std::int8_t fill) {
    data_.assign(static_cast<std::size_t>(w) * h, fill);
    width_ = w;
    height_ = h;
    resolution_ = resolution;
    origin_pose_ = origin;
  }

  [[nodiscard]] const std::vector<std::int8_t>& data() const { return data_; }
  [[nodiscard]] std::vector<std::int8_t>& data() { return data_; }
  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] double resolution() const { return resolution_; }
  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_pose_; }

private:
  std::vector<std::int8_t> data_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = GRID_RESOLUTION;
  Sophus::SE2d origin_pose_{Sophus::SO2d{0.0}, Eigen::Vector2d{ORIGIN_X, ORIGIN_Y}};
};

/**
 * \brief Traits struct to interpret grid cell values for different underlying types.
 *
 * This template allows the sensor models to generically determine the semantic 
 * state of a cell (free, occupied, or unknown) regardless of the storage type.
 * 
 * \tparam T The data type stored in each grid cell.
 */
template <class T>
struct ValueTraits;

/**
 * \brief Specialization of ValueTraits for boolean storage.
 */
template <>
struct ValueTraits<bool> {
  /// Returns true if the value represents free space.
  [[nodiscard]] static bool is_free(bool value) { return !value; }
  /// Returns true if the value represents unknown space.
  [[nodiscard]] static bool is_unknown(bool) { return false; }
  /// Returns true if the value represents an occupied cell.
  [[nodiscard]] static bool is_occupied(bool value) { return value; }
};

/**
 * \brief Specialization of ValueTraits for standard ROS-style int8 storage.
 */
template <>
struct ValueTraits<std::int8_t> {
  static constexpr std::int8_t kFreeValue = 0;
  static constexpr std::int8_t kUnknownValue = -1;
  static constexpr std::int8_t kOccupiedValue = 100;

  [[nodiscard]] static bool is_free(std::int8_t value) { return value == kFreeValue; }
  [[nodiscard]] static bool is_unknown(std::int8_t value) { return value == kUnknownValue; }
  [[nodiscard]] static bool is_occupied(std::int8_t value) { return value == kOccupiedValue; }
};

/**
 * \brief Implementation of a fixed-size occupancy grid stored in a stack-allocated array.
 *
 * This class satisfies the \ref OccupancyGrid2Page requirements and can be used 
 * with Beluga sensor models. It uses a row-major `std::array` for internal storage.
 * 
 * \tparam Rows Number of rows in the grid.
 * \tparam Cols Number of columns in the grid.
 * \tparam T Data type for each cell. Defaults to bool.
 */
template <std::size_t Rows, std::size_t Cols, class T = bool>
class StaticOccupancyGrid : public beluga::BaseOccupancyGrid2<StaticOccupancyGrid<Rows, Cols, T>> {
 public:
  /// Default constructor using global configuration constants.
  StaticOccupancyGrid() : StaticOccupancyGrid(
    std::array<T, GRID_ROWS * GRID_COLS>{},
    GRID_RESOLUTION,
    Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{ORIGIN_X, ORIGIN_Y}}
    ) {}

  /**
   * \brief Constructs a grid from an existing array.
   * \param array Initial data for the grid cells.
   * \param resolution Grid resolution in meters per cell.
   * \param origin Pose of the grid origin (bottom-left corner) in the map frame.
   */
  explicit StaticOccupancyGrid(
      std::array<T, Rows * Cols> array,
      double resolution = GRID_RESOLUTION,
      const Sophus::SE2d& origin = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{ORIGIN_X, ORIGIN_Y}}
    )
      : grid_{array}, origin_(origin), resolution_{resolution} {}

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] auto& data() { return grid_; }
  [[nodiscard]] const auto& data() const { return grid_; }
  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return ValueTraits<T>{}; }

 private:
  std::array<T, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  double resolution_;
};

using GridTypeLO = LogOddsGrid;
using GridTypeOC = StaticOccupancyGrid<GRID_ROWS, GRID_COLS, int8_t>;

#endif
