#ifndef __FASTSLAM_NODE_H__
#define __FASTSLAM_NODE_H__

#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <tuple>
#include <execution>
#include <iomanip>
#include <chrono>

#include <range/v3/view/take.hpp>
#include <range/v3/range/conversion.hpp> 

#include "particle.hpp"

/// Beluga Core & Models
#include <beluga/algorithm/estimation.hpp>
#include <beluga/containers/tuple_vector.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/sensor/likelihood_field_prob_model.hpp>
#include <beluga/algorithm/spatial_hash.hpp>

/// Beluga Views & Actions
#include <beluga/views/sample.hpp>
#include <beluga/actions/assign.hpp>
#include <beluga/primitives.hpp>
#include <beluga/views/elements.hpp>

#include "fastslam_core/grid_config.hpp"

const int OCCUPPIED = kOccupiedValue;
const int FREE = kFreeValue;
const int UNKNOWN = kUnknownValue;
const double ROBOT_RADIUS = kRobotRadius;

#define TIMER_START(name) auto name##_start = std::chrono::high_resolution_clock::now();
#define TIMER_END(name) auto name##_end = std::chrono::high_resolution_clock::now(); \
    auto name##_ms = std::chrono::duration_cast<std::chrono::milliseconds>(name##_end - name##_start).count();

/**
 * \file
 * \brief FastSLAM implementation using occupancy grid and lidar 2D.
 * \details The algorithm is based on \cite thrun2005probabilistic, Chapter 13.10.
 */

/// 2D pose for particle's state and motion model state.
using state_type = Sophus::SE2d;
/// Particle type, containing pose, weight, log-odds grid and occupancy grid.
using FastSLAMParticle = std::tuple<
    state_type,
    beluga::Weight,
    GridTypeLO,
    GridTypeOC
>;

/// Parameters to construct a FastSLAM instance.
struct FastSLAMParams {
  /// Number of particles in the filter.
  std::size_t num_particles = 500UL;
};

/**
 * \page FastSLAMPage FastSLAM Algorithm overview
 *
 * \section FastSLAMDescription Description
 * FastSLAM is a particle filter-based algorithm for simultaneous localization and mapping (SLAM).
 * Uses a likelihood field measurement model and a differential drive motion model.
 * 
 * \section FastSLAMComponents Components
 * - MotionModel: \ref beluga::DifferentialDriveModel using state_type.
 * - MeasurementModel: \ref beluga::LikelihoodFieldProbModel using \ref GridTypeOC.
 * - FastSLAMParams: configuration parameters for the FastSLAM algorithm, such as the number of particles.
 */
class FastSLAM {
public:
    /// Motion model type: sampled odometry model for a differential drive.
    using MotionModel = beluga::DifferentialDriveModel<state_type>;
    /// Measurement model type: Likelihood field prob sensor model for range finders.
    using MeasurementModel = beluga::LikelihoodFieldProbModel<GridTypeOC>;
    /// Measurement type of the sensor: a point cloud for the range finder.
    using measurement_type = std::vector<std::pair<double, double>>;
    /// Current and previous odometry estimates as motion model control action.
    using control_type = std::tuple<state_type, state_type>;

    /// Construct a FastSLAM instance.
    /**
     * \param motion_model Motion model instance.
     * \param measurement_model Measurement model Instance.
     * \param params Parameters for FastSLAM implementation.
     */
    FastSLAM(
        MotionModel motion_model,
        MeasurementModel measurement_model,
        const FastSLAMParams& params = FastSLAMParams{})
        : motion_model_(std::move(motion_model)),
          measurement_model_(std::move(measurement_model)),
          params_(params) {
      particles_.resize(params_.num_particles);
      for (auto&& p : particles_) {
        std::get<0>(p) = state_type{};
        std::get<1>(p) = beluga::Weight(1.0);
      }
    }

    /// Returns a reference to the current set of particles.
    [[nodiscard]] const auto& particles() const { return particles_; }
    [[nodiscard]] auto& particles() { return particles_; }

    /// Samples from the motion distribution to propagate particle states.
    /**
     * This function computes a motion sampler based on the provided control action 
     * (the delta between current and previous odometry) and updates each particle's 
     * pose by sampling from the resulting distribution.
     *
     * \param control_action Control action.
     */
    void sample_motion_model(const control_type& u) {
        auto sampler = motion_model_(u);
        static thread_local std::mt19937 gen{std::random_device{}()};

        for (auto&& p : particles_) {
            auto& pose = std::get<0>(p);
            pose = sampler(pose, gen);
        }
    }

    /// Updates particle weights based on the measurement model and the received measurement.
    /**
     * Correction step of the particle filter.
     * - (optional) Subsamples the input measurements to reduce computational overhead.
     * - Updates the measurement model with a new occupancy grid map, from the best particle.
     * - Calculates the likelihood of each particle's state given a state weighting function conditioned on 2D lidar hits (measurement model).
     * 
     * \param measurement Measurement data.
     * \param best_idx Index of the particle whose map is currently considered the most accurate.
     */
    void measurement_model_map(const measurement_type& z, size_t best_idx) {
        /// Subsample the scan to improve performance
        measurement_type z_sparse;
        constexpr size_t kStep = 1;
        z_sparse.reserve(z.size() / kStep + 1);
        for (size_t i = 0; i < z.size(); i += kStep) {
            z_sparse.push_back(z[i]);
        }
        
        /// Synchronize the sensor model with the reference map from the best particle
        /// to ensure likelihood calculations are based on the latest environment estimate.
        auto lo_grids = particles_ | beluga::views::elements<2>;
        const auto& best_log_odds = lo_grids[best_idx];
        auto best_oc_grid = sync_log_odds_to_occupancy(best_log_odds);
        measurement_model_.update_map(best_oc_grid);

        /// Update individual particle weights by evaluating the measurement model likelihood function.
        for (auto&& p : particles_) {
            const auto& pose = std::get<0>(p);
            auto& weight = std::get<1>(p);
            auto weight_fn = measurement_model_(measurement_type(z_sparse));
            double p_z = weight_fn(pose);
            weight *= p_z;
        }

        double sum_w = 0.0;
        for (auto&& p : particles_) {
            sum_w += static_cast<double>(std::get<1>(p));
        }

        for (auto&& p : particles_) {
            auto& weight = std::get<1>(p);
           
            weight = weight/sum_w;
        }
    }

    /// Update the occupancy grid map of each particle based on the transformed measurement.
    /**
     * - Converts the particle's pose to grid coordinates to establish the ray's origin.
     * - (optional) Clears the robot's footprint area to mitigate self-mapping noise.
     * - Projects each local measurement point into the world frame based on the particle's pose.
     * - Updates the Log-Odds representation along the beam (free space) and at the hit point (occupied).
     * - Synchronizes the internal StaticOccupancyGrid used by the measurement model.
     * 
     * \param measurement Measurement data.
     */
    void update_occupancy_grid(const measurement_type& z) {        
        for (auto&& p : particles_) {
            auto& pose = std::get<0>(p);
            auto& lo_grid = std::get<2>(p);
            auto& oc_grid = std::get<3>(p);

            /// Determine the ray origin in grid coordinates from the current particle pose.
            int gx0, gy0, dummy_idx;
            if (!world_to_index(pose.translation().x(), pose.translation().y(), gx0, gy0, dummy_idx, lo_grid)) continue; // Skip particles currently outside the map bounds.
            
            /// Clear the area occupied by the robot to remove potential sensor artifacts.
            clear_robot_footprint(ROBOT_RADIUS, gx0, gy0, lo_grid);

            for (const auto& local_point : z) {
                /// Project the local sensor hit into world coordinates using the particle's pose hypothesis.
                auto world_point = pose * Eigen::Vector2d(local_point.first, local_point.second);

                int gx1, gy1, hit_idx;
                bool impact_in_map = world_to_index(world_point.x(), world_point.y(), gx1, gy1, hit_idx, lo_grid);

                /// Update cells along the beam path as free space using Bresenham's algorithm.
                auto points_in_line = bresenham(gx0, gy0, gx1, gy1);
                for (const auto& cell : points_in_line) {
                    /// Avoid clearing the cell where the robot is currently located.
                    if (cell.first == gx0 && cell.second == gy0) continue;
                    
                    const int idx = cell.second * lo_grid.width() + cell.first;
                    lo_grid.at(idx) = std::max(lo_grid.at(idx) + l_free_, -5.0f);
                }
                /// Integrate the obstacle detection at the end of the beam.
                if (impact_in_map) {
                    lo_grid.at(hit_idx) = std::min(lo_grid.at(hit_idx) + l_occ_, 5.0f);
                }
            }

            /// Synchronize the occupancy grid representation.
            oc_grid = sync_log_odds_to_occupancy(lo_grid);
        }
    }

    /// Multinomial resampling based on the current importance weights.
    /**
     * - Creates a discrete distribution based on the importance weights.
     * - Samples with replacement to favor particles that better represent the posterior.
     * - Clones the selected particles (including their maps).
     * - Resets all weights to a uniform value (1.0).
     */
    void resample() {
        const std::size_t n_particles = particles_.size();
        
        double n_eff = 0.0;
        double sum_sq = 0.0;

        for (auto&& p : particles_) {
            auto& weight = std::get<1>(p);
            sum_sq += weight * weight;
        }

        n_eff = 1.0 / sum_sq;
        
        if (n_particles == 0 || n_eff > (n_particles / 2.0)) {
            return;
            // for (auto&& w : beluga::views::weights(particles_)) {
            //     w = beluga::Weight(1.0);
            // }
            std::cout << "Effective Sample Size: " << n_eff << " / " << n_particles << std::endl;
        }

        /// Internal weight type to double for compatibility with std::discrete_distribution.
        std::vector<double> weights;
        weights.reserve(n_particles);

        for (auto&& w : beluga::views::weights(particles_)) {
            weights.push_back(static_cast<double>(w));
        }

        /// Discrete distribution to sample particle indices proportional to their weights.
        std::discrete_distribution<size_t> weight_distribution(weights.begin(), weights.end());

        /// Generate the resampled set of particles.
        std::vector<decltype(particles_)::value_type> resampled_particles;
        resampled_particles.reserve(n_particles);

        for (size_t i = 0; i < n_particles; ++i) {
            auto it = particles_.begin();
            std::advance(it, weight_distribution(rng_));
            const auto& selected = *it;
            resampled_particles.push_back(selected);
        }

        particles_.assign(resampled_particles.begin(), resampled_particles.end());

        /// Reset importance weights to a uniform distribution.
        for (auto&& w : beluga::views::weights(particles_)) {
            w = beluga::Weight(1.0);
        }
    }

    /// Bresenham's 2D line drawing algorithm.
    std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) {
        std::vector<std::pair<int, int>> line;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x0 == x1 && y0 == y1) break;
            if (x0 >= 0 && x0 < GRID_COLS && y0 >= 0 && y0 < GRID_ROWS) {
                line.push_back({x0, y0});
            }
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
        return line;
    }

    /// Converts world coordinates to grid indices and linear index for map access.
    inline bool world_to_index(double wx, double wy, int &gx, int &gy, int &index, const GridTypeLO& grid) const {
        gx = static_cast<int>(std::floor((wx - grid.origin_x()) / grid.resolution()));
        gy = static_cast<int>(std::floor((wy - grid.origin_y()) / grid.resolution()));

        if (gx < 0 || gx >= grid.width() || gy < 0 || gy >= grid.height()) return false;

        index = gy * grid.width() + gx;
        return true;
    }

    /// Synchronizes the occupancy grid representation with the log-odds grid by applying a thresholding function.
    GridTypeOC sync_log_odds_to_occupancy(const GridTypeLO& log_odds_grid) {
        GridTypeOC oc_grid;
        auto& oc_data = oc_grid.data();
        const auto& lo_data = log_odds_grid.data();
        constexpr float OCCUPIED_THRESH = 0.65f;
        constexpr float FREE_THRESH     = 0.35f; //0.196

        for (size_t i = 0; i < lo_data.size(); ++i) {
            if (std::abs(lo_data[i]) < 0.01f) {
                oc_data[i] = UNKNOWN; 
            } else {
                float p = 1.0f / (1.0f + std::exp(-lo_data[i]));
                oc_data[i] = (p > OCCUPIED_THRESH) ? OCCUPPIED : (p < FREE_THRESH ? FREE : UNKNOWN);
            }
        }
        return oc_grid;
    }

    /// Clears the robot's footprint area in the log-odds grid to mitigate self-mapping noise.
    void clear_robot_footprint(double radius, int rx, int ry, GridTypeLO& log_odds_grid){
        int r_cells = static_cast<int>(radius / GRID_RESOLUTION);

        for (int dx = -r_cells; dx <= r_cells; ++dx)
        {
            for (int dy = -r_cells; dy <= r_cells; ++dy)
            {
                int x = rx + dx;
                int y = ry + dy;

                if (x < 0 || x >= (int)GRID_ROWS ||
                    y < 0 || y >= (int)GRID_COLS)
                    continue;

                int idx = y * GRID_ROWS + x;
                log_odds_grid.at(idx) = -5.0f;
            }
        }
    }

private:
    beluga::TupleVector<FastSLAMParticle> particles_;

    MotionModel motion_model_;
    MeasurementModel measurement_model_;
    FastSLAMParams params_;

    /// Log-Odds constants for occupancy grid updates.
    const float l_occ_ = 1.2f;
    const float l_free_ = -0.2f;

    std::mt19937 rng_;
};

#endif