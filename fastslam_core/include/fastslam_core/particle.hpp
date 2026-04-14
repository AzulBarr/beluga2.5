#ifndef __PARTICLE_H__
#define __PARTICLE_H__
#include <array>
#include <memory>
#include <cstdint>
#include <vector>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <beluga/sensor/likelihood_field_prob_model.hpp>
#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga/primitives.hpp>

#include "fastslam_core/grid_config.hpp"

static constexpr std::size_t GRID_ROWS = kGridRows;
static constexpr std::size_t GRID_COLS = kGridCols;
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
   * \brief Accesses a cell value by linear index.
   * \param index The flat index in row-major order.
   * \return Reference to the log-odds value of the cell.
   */
  [[nodiscard]] float& at(int index){ return data_[index]; }

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
  [[nodiscard]] const int width() const { return width_; }

  /// Get the height of the log-odds grid.
  [[nodiscard]] const int height() const { return height_; }

  /// Get the resolution of the log-odds grid discretization, in meters.
  [[nodiscard]] const double resolution() const { return resolution_; }

  /// Get a reference to the underlying data storage
  [[nodiscard]] const std::vector<float>& data() const { return data_; }

private:
  int width_;
  int height_;
  double resolution_;
  Sophus::SE2d origin_pose_;
  std::vector<float> data_;
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
