#ifndef __BELUGASLAM_NODE_H__
#define __BELUGASLAM_NODE_H__

#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <iostream>
#include <tuple>
#include <execution>
#include <iomanip>
#include <chrono>
#include <numeric>
#include <ceres/ceres.h>

#include <range/v3/view/take.hpp>
#include <range/v3/range/conversion.hpp> 
#include <range/v3/view/take_exactly.hpp>

#include "particle.hpp"
#include "submap.hpp"

/// Beluga Core & Models
#include <beluga/algorithm/estimation.hpp>
#include <beluga/containers/tuple_vector.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/sensor/likelihood_field_prob_model.hpp>
#include <beluga/algorithm/spatial_hash.hpp>

/// Beluga Views & Actions
#include <beluga/actions/assign.hpp>
#include <beluga/primitives.hpp>
#include <beluga/views.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/algorithm/cluster_based_estimation.hpp>
#include <beluga/algorithm/estimation.hpp>

#include "belugaslam_core/grid_config.hpp"

const int OCCUPPIED = kOccupiedValue;
const int FREE = kFreeValue;
const int UNKNOWN = kUnknownValue;
const double ROBOT_RADIUS = kRobotRadius;

/**
 * \file
 * \brief BelugaSLAM implementation using occupancy grid and lidar 2D.
 * \details The algorithm is based on \cite thrun2005probabilistic, Chapter 13.10.
 */

/// 2D pose for particle's state and motion model state.
using state_type = Sophus::SE2d;
/// Particle type, containing pose, weight, and a reference to its hypothesis.
using FastSLAMParticle = std::tuple<
    state_type,
    beluga::Weight,
    std::shared_ptr<Hypothesis>
>;

struct PoseGraphEdgeError {
    PoseGraphEdgeError(double dx, double dy, double dtheta, double weight_translation = 1.0, double weight_rotation = 1.0)
        : dx_(dx), dy_(dy), dtheta_(dtheta), 
          weight_translation_(weight_translation), weight_rotation_(weight_rotation) {}

    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j, T* residuals) const {
        T xi = pose_i[0];
        T yi = pose_i[1];
        T theta_i = pose_i[2];

        T xj = pose_j[0];
        T yj = pose_j[1];
        T theta_j = pose_j[2];

        T cos_theta_i = ceres::cos(theta_i);
        T sin_theta_i = ceres::sin(theta_i);

        // Relative position of j in i's frame
        T dx_ij = xj - xi;
        T dy_ij = yj - yi;

        T local_x = cos_theta_i * dx_ij + sin_theta_i * dy_ij;
        T local_y = -sin_theta_i * dx_ij + cos_theta_i * dy_ij;

        // Residuals scaled by weights
        residuals[0] = (local_x - T(dx_)) * T(weight_translation_);
        residuals[1] = (local_y - T(dy_)) * T(weight_translation_);
        
        T diff_theta = (theta_j - theta_i) - T(dtheta_);
        residuals[2] = ceres::atan2(ceres::sin(diff_theta), ceres::cos(diff_theta)) * T(weight_rotation_);

        return true;
    }

    static ceres::CostFunction* Create(double dx, double dy, double dtheta, double weight_translation = 1.0, double weight_rotation = 1.0) {
        return new ceres::AutoDiffCostFunction<PoseGraphEdgeError, 3, 3, 3>(
            new PoseGraphEdgeError(dx, dy, dtheta, weight_translation, weight_rotation));
    }

    double dx_, dy_, dtheta_;
    double weight_translation_, weight_rotation_;
};

/// Parameters to construct a BelugaSLAM instance.
struct FastSLAMParams {
    /// Minimum number of particles for adaptive resampling.
    std::size_t min_particles = 10UL;

    /// Maximum number of particles for adaptive resampling.
    std::size_t max_particles = 50UL;

    /// \brief Maximum particle filter population error between the true distribution and the
    /// estimated distribution. It is used in KLD resampling \cite fox2001adaptivekldsampling
    /// to limit the allowed number of particles to the minimum necessary.
    double kld_epsilon = 0.5;

    /// \brief Upper standard normal quantile for \f$P\f$, where \f$P\f$ is the probability that the error in
    /// the estimated distribution will be less than `kld_epsilon` in KLD resampling \cite fox2001adaptivekldsampling .
    double kld_z = 3.0;

    /// \brief Spatial resolution along the x-axis to create buckets for KLD resampling.
    double spatial_resolution_x = 0.5;

    /// \brief Spatial resolution along the y-axis to create buckets for KLD resampling.
    double spatial_resolution_y = 0.5;

    /// \brief Spatial resolution around the z-axis to create buckets for KLD resampling.
    double spatial_resolution_theta = 10 * Sophus::Constants<double>::pi() / 180;

    /// \brief Scaling factor for the endpoint score to tune particle filter confidence.
    double likelihood_scaling_factor = 0.05;
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
class BelugaSLAM {
public:
    /// Motion model type: sampled odometry model for a differential drive.
    using MotionModel = beluga::DifferentialDriveModel<state_type>;
    /// Measurement model type: Likelihood field prob sensor model for range finders.
    using MeasurementModel = beluga::LikelihoodFieldProbModel<GridTypeOC>;
    /// Measurement type of the sensor: a point cloud for the range finder.
    using measurement_type = std::vector<std::pair<double, double>>;
    /// Current and previous odometry estimates as motion model control action.
    using control_type = std::tuple<state_type, state_type>;

    /// Construct a BelugaSLAM instance.
    /**
     * \param motion_model Motion model instance.
     * \param measurement_model Measurement model Instance.
     * \param params Parameters for FastSLAM implementation.
     */
    BelugaSLAM(
        MotionModel motion_model,
        MeasurementModel measurement_model,
        const FastSLAMParams& params = FastSLAMParams{})
        : motion_model_(std::move(motion_model)),
          measurement_model_(std::move(measurement_model)),
          spatial_hasher_{params.spatial_resolution_x, 
                      params.spatial_resolution_y, 
                      params.spatial_resolution_theta},
          params_(params){
      
      // Create the initial hypothesis (single hypothesis: "exploring")
      auto initial_hypothesis = std::make_shared<Hypothesis>();
      initial_hypothesis->id = next_hypothesis_id_++;
      hypotheses_.push_back(initial_hypothesis);

      particles_.resize(params_.min_particles);
      for (auto&& p : particles_) {
        std::get<0>(p) = state_type{};
        std::get<1>(p) = beluga::Weight(1.0);
        std::get<2>(p) = initial_hypothesis;  // All particles share the same hypothesis
      }
      best_oc_grid_ = GridTypeOC();
      best_lo_grid_ = GridTypeLO();
      local_oc_grid_ = GridTypeOC();
      local_lo_grid_ = GridTypeLO();
      best_pose_ = state_type{};

    }

    /// Returns a reference to the current set of particles.
    [[nodiscard]] const auto& particles() const { return particles_; }
    [[nodiscard]] auto& particles() { return particles_; }

    [[nodiscard]] size_t get_active_hypotheses_count() const { return hypotheses_.size(); }
    
    [[nodiscard]] size_t get_submaps_count() const { 
        return hypotheses_.empty() ? 0 : hypotheses_.front()->submaps.history.size(); 
    }

    /// Samples from the motion distribution to propagate particle states.
    /**
     * This function computes a motion sampler based on the provided control action 
     * (the delta between current and previous odometry) and updates each particle's 
     * pose by sampling from the resulting distribution.
     *
     * \param control_action Control action.
     * \param measurement Measurement data.
     */
    void sample_motion_model(const control_type& u) {
        auto sampler = motion_model_(u);

        double max_weight =  0.0;
        for (auto&& p : particles_) {
            auto& pose = std::get<0>(p);
            pose = sampler(pose, rng_);
            max_weight = std::max(max_weight, static_cast<double>(std::get<1>(p)));
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
     */
    void measurement_model_map(const measurement_type& z) {
        /// Subsample the scan to improve performance         
        measurement_type z_sparse;
        constexpr size_t kStep = 2;
        z_sparse.reserve(z.size() / kStep + 1);
        for (size_t i = 0; i < z.size(); i += kStep) {
            z_sparse.push_back(z[i]);
        }
        
        /// Scan matching search grids
        auto dxys1 = {-0.1, 0.0, 0.1};
        auto dthetas1 = {-5 * Sophus::Constants<double>::pi() / 180, 0.0, 5 * Sophus::Constants<double>::pi() / 180};

        auto dxys2 = {-0.05, 0.0, 0.05};
        auto dthetas2 = {-2.5 * Sophus::Constants<double>::pi() / 180, 0.0, 2.5 * Sophus::Constants<double>::pi() / 180};

        // 1. Composite local map ONCE PER HYPOTHESIS and cache it
        std::map<size_t, GridTypeLO> hypothesis_lo_cache;
        for (auto& hypothesis : hypotheses_) {
            composite_submaps(hypothesis->submaps, local_lo_grid_, true);
            hypothesis_lo_cache[hypothesis->id] = local_lo_grid_;
        }

        std::vector<double> log_scores;
        log_scores.reserve(particles_.size());

        // 2. Score each particle against ITS hypothesis's cached map
        for (auto&& p : particles_) {
            auto& pose_pred = std::get<0>(p);
            auto& weight = std::get<1>(p);
            auto& hypothesis = std::get<2>(p);

            auto& cached_grid = hypothesis_lo_cache[hypothesis->id];

            // Fast Endpoint-Score Model (Cartographer style)
            auto score_fn = [&](const state_type& candidate_pose) {
                double log_prob_sum = 0.0;
                for (const auto& local_point : z_sparse) {
                    auto hit = candidate_pose * Eigen::Vector2d(local_point.first, local_point.second);
                    int gx, gy, hit_idx;
                    if (world_to_index(hit.x(), hit.y(), gx, gy, hit_idx, cached_grid)) {
                        log_prob_sum += cached_grid.at(hit_idx);
                    }
                }
                return log_prob_sum;
            };

            auto best_pose = pose_pred;
            double best_log_score = score_fn(pose_pred);

            for (double dx : dxys1) {
                for (double dy : dxys1) {
                    for (double dtheta : dthetas1) {
                        auto candidate_pose = state_type{
                            Sophus::SO2d{pose_pred.so2().log() + dtheta}, 
                            Eigen::Vector2d{pose_pred.translation().x() + dx, pose_pred.translation().y() + dy}
                        };
                        double score = score_fn(candidate_pose);
                        if (score > best_log_score) {
                            best_log_score = score;
                            best_pose = candidate_pose;
                        }
                    }
                }
            }

            for (double dx : dxys2) {
                for (double dy : dxys2) {
                    for (double dtheta : dthetas2) {
                        auto candidate_pose = state_type{
                            Sophus::SO2d{best_pose.so2().log() + dtheta}, 
                            Eigen::Vector2d{best_pose.translation().x() + dx, best_pose.translation().y() + dy}
                        };
                        double score = score_fn(candidate_pose);
                        if (score > best_log_score) {
                            best_log_score = score;
                            best_pose = candidate_pose;
                        }
                    }
                }
            }

            pose_pred = best_pose;
            log_scores.push_back(best_log_score);
        }

        // --- Safely Convert Log-Scores to Weights and Accumulate ---
        double max_log_score = -std::numeric_limits<double>::infinity();
        for (double s : log_scores) {
            if (s > max_log_score) max_log_score = s;
        }

        double sum_w = 0.0;
        size_t idx = 0;
        for (auto&& p : particles_) {
            auto& weight = std::get<1>(p);
            
            double likelihood = std::exp((log_scores[idx] - max_log_score) * params_.likelihood_scaling_factor);
            
            double new_weight = static_cast<double>(weight) * likelihood;
            weight = beluga::Weight(new_weight);
            
            sum_w += new_weight;
            idx++;
        }

        if (sum_w > 1e-9) {
            for (auto&& p : particles_) {
                auto& weight = std::get<1>(p);
                weight = beluga::Weight(static_cast<double>(weight) / sum_w);
            }
        }
        else {
            for (auto&& p : particles_) {
                auto& weight = std::get<1>(p);
                weight = beluga::Weight(1.0 / particles_.size());
            }
        }
        
    }

    /// Update the occupancy grid map of each hypothesis based on the transformed measurement.
    void update_occupancy_grid(const measurement_type& z) {
        for (auto& hypothesis : hypotheses_) {
            auto& submaps = hypothesis->submaps;

            // 1. Find the best particle in THIS hypothesis
            double best_w = -1.0;
            state_type best_pose;
            for (const auto& p : particles_) {
                if (std::get<2>(p) != hypothesis) continue;
                double w = static_cast<double>(std::get<1>(p));
                if (w > best_w) {
                    best_w = w;
                    best_pose = std::get<0>(p);
                }
            }
            if (best_w < 0) continue;  // Dead hypothesis (no particles)

            // 2. Ensure we have an active submap, and enforce Copy-On-Write if shared
            if (!submaps.active_submap) {
                submaps.active_submap = std::make_shared<Submap>(best_pose, SUBMAP_COLS, SUBMAP_ROWS, GRID_RESOLUTION);
            } else {
                submaps.make_active_unique();
            }

            auto& lo_grid = *submaps.active_submap->grid();

            // 3. Compute the robot's pose relative to the active submap (Local SLAM)
            auto T_w_s = submaps.active_submap->global_pose();
            auto T_s_r = T_w_s.inverse() * best_pose;

            int gx0, gy0, dummy_idx;
            if (!world_to_index(T_s_r.translation().x(), T_s_r.translation().y(), gx0, gy0, dummy_idx, lo_grid)) {
                // The robot drove outside the physical bounds of the submap before reaching 50 insertions!
                // Force finish the submap early so a new one is created at the new position on the next frame.
                submaps.finish_active_submap();
                continue;
            }
            
            clear_robot_footprint(ROBOT_RADIUS, gx0, gy0, lo_grid);

            for (const auto& local_point : z) {
                auto hit_in_submap = T_s_r * Eigen::Vector2d(local_point.first, local_point.second);

                int gx1, gy1, hit_idx;
                bool impact_in_map = world_to_index(hit_in_submap.x(), hit_in_submap.y(), gx1, gy1, hit_idx, lo_grid);

                auto points_in_line = bresenham(gx0, gy0, gx1, gy1, lo_grid.width(), lo_grid.height());
                for (const auto& cell : points_in_line) {
                    if (cell.first == gx0 && cell.second == gy0) continue;
                    
                    const int idx = cell.second * lo_grid.width() + cell.first;
                    lo_grid.at(idx) = std::max(lo_grid.at(idx) + l_free_, -5.0f);
                }
                if (impact_in_map) {
                    lo_grid.at(hit_idx) = std::min(lo_grid.at(hit_idx) + l_occ_, 5.0f);
                }
            }

            submaps.active_submap->add_insertion();
            // Freeze submap after a certain number of insertions
            if (submaps.active_submap->num_insertions() >= 50) {
                submaps.finish_active_submap();
            }
        }
    }

    /// Step 5: Post-update processing (Loop closure, PGO, Best map composite)
    void post_update(const measurement_type& z) {
        measurement_type z_sparse;
        constexpr size_t kStep = 2;
        z_sparse.reserve(z.size() / kStep + 1);
        for (size_t i = 0; i < z.size(); i += kStep) {
            z_sparse.push_back(z[i]);
        }

        // --- Loop Closure Detection (per hypothesis) ---
        detect_loop_closure(z_sparse);

        // --- Determine Best Hypothesis & Best Pose ---
        std::map<size_t, double> hypothesis_weights;
        for (const auto& p : particles_) {
            hypothesis_weights[std::get<2>(p)->id] += static_cast<double>(std::get<1>(p));
        }

        size_t best_hypothesis_id = hypotheses_.front()->id;
        double max_hypothesis_weight = -1.0;
        for (const auto& [hid, hw] : hypothesis_weights) {
            if (hw > max_hypothesis_weight) {
                max_hypothesis_weight = hw;
                best_hypothesis_id = hid;
            }
        }

        std::shared_ptr<Hypothesis> best_hypothesis = nullptr;
        for (auto& h : hypotheses_) {
            if (h->id == best_hypothesis_id) {
                best_hypothesis = h;
                break;
            }
        }

        double best_particle_weight = -1.0;
        for (const auto& p : particles_) {
            if (std::get<2>(p)->id == best_hypothesis_id) {
                double w = static_cast<double>(std::get<1>(p));
                if (w > best_particle_weight) {
                    best_particle_weight = w;
                    best_pose_ = std::get<0>(p);
                }
            }
        }

        // --- PGO Trigger (for the winning hypothesis) ---
        if (best_hypothesis->submaps.loop_constraints.size() > best_hypothesis->optimized_loops_count) {
            optimize_pose_graph(best_hypothesis);
            best_hypothesis->optimized_loops_count = best_hypothesis->submaps.loop_constraints.size();
        }

        // Composite full global map from the best particle's hypothesis for RViz
        composite_submaps(best_hypothesis->submaps, best_lo_grid_, false);
        sync_log_odds_to_occupancy(best_lo_grid_, best_oc_grid_);

        // RESET WEIGHTS: After all mapping and decision logic is done, reset weights 
        // to uniform (1/N) so the filter is ready for the next motion update.
        double uniform_weight = 1.0 / particles_.size();
        for (auto&& w : beluga::views::weights(particles_)) {
            w = beluga::Weight(uniform_weight);
        }
    }

    /// Multinomial resampling based on the current importance weights.
    /**
     * - Creates a discrete distribution based on the importance weights.
     * - Samples with replacement to favor particles that better represent the posterior.
     * - Clones the selected particles (including their maps).
     * - Resets all weights to a uniform value (1.0).
     */
    /// Multi-hypothesis resampling based on explicit cluster membership.
    void resample() {
        auto weights_view = particles_ | beluga::views::elements<1> | 
                       ranges::views::transform([](auto w) { return static_cast<double>(w); }) |
                       ranges::to<std::vector<double>>();

        // 1. Always detect and split spatial modes BEFORE deciding whether to resample
        detect_and_split_modes(weights_view);

        // 2. Kill hypotheses with negligible total weight (< 5%)
        std::map<size_t, double> hypothesis_total_weight;
        double global_total = 0.0;
        for (size_t i = 0; i < particles_.size(); ++i) {
            auto& hypothesis = std::get<2>(*(particles_.begin() + i));
            double w = weights_view[i];
            hypothesis_total_weight[hypothesis->id] += w;
            global_total += w;
        }

        std::set<size_t> dead_hypotheses;
        for (auto& h : hypotheses_) {
            if (global_total > 0 && (hypothesis_total_weight[h->id] / global_total) < 0.05) {
                dead_hypotheses.insert(h->id);
            }
        }

        // 3. Calculate Global ESS
        double sum_sq = 0.0;
        for (double w : weights_view) {
            sum_sq += w * w;
        }
        double n_eff = (sum_sq > 0.0) ? (1.0 / sum_sq) : 0.0;

        // We MUST resample if ESS is low, OR if there are dead hypotheses that need to be purged,
        // OR if particle counts drastically mismatch weight quotas (we assume ESS catches this).
        if (n_eff >= particles_.size() / 2.0 && dead_hypotheses.empty()) {
            // We can safely skip resampling. 
            // The spatial modes are already split and tracked as independent hypotheses.
            return;
        }

        // --- ACTUAL RESAMPLING ---
        std::map<size_t, std::vector<size_t>> hypothesis_particle_indices;
        for (size_t i = 0; i < particles_.size(); ++i) {
            hypothesis_particle_indices[std::get<2>(*(particles_.begin() + i))->id].push_back(i);
        }

        // Gather surviving hypotheses and normalize their weights
        std::vector<std::pair<size_t, double>> surviving_hypotheses;
        double surviving_weight_sum = 0.0;
        for (auto& h : hypotheses_) {
            if (!dead_hypotheses.count(h->id)) {
                double w = hypothesis_total_weight[h->id];
                surviving_hypotheses.push_back({h->id, w});
                surviving_weight_sum += w;
            }
        }

        // Sort descending by weight
        std::sort(surviving_hypotheses.begin(), surviving_hypotheses.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        // Enforce maximum of 4 hypotheses to protect the budget
        size_t max_hypotheses = 4;
        while (surviving_hypotheses.size() > max_hypotheses) {
            dead_hypotheses.insert(surviving_hypotheses.back().first);
            surviving_weight_sum -= surviving_hypotheses.back().second;
            surviving_hypotheses.pop_back();
        }

        size_t K = surviving_hypotheses.size();
        if (K == 0) return; // Should never happen unless weights are all exactly zero

        size_t N_budget = (K == 1) ? params_.min_particles : params_.max_particles;
        size_t N_min = 12;
        
        // Safety check if parameters are strange
        if (K * N_min > N_budget) {
            N_min = N_budget / K;
        }

        size_t remaining_budget = N_budget - (K * N_min);

        // Pre-calculate exact allocations
        std::map<size_t, size_t> h_quotas;
        size_t allocated = 0;
        for (size_t i = 0; i < K; ++i) {
            size_t hid = surviving_hypotheses[i].first;
            double w_norm = surviving_weight_sum > 0 ? (surviving_hypotheses[i].second / surviving_weight_sum) : (1.0 / K);
            
            size_t quota = N_min + std::round(remaining_budget * w_norm);
            
            // Adjust last one to exactly match budget (absorb rounding errors)
            if (i == K - 1) {
                quota = N_budget - allocated;
            }
            h_quotas[hid] = quota;
            allocated += quota;
        }

        std::vector<FastSLAMParticle> buffer;
        buffer.reserve(N_budget);

        for (auto& hypothesis : hypotheses_) {
            if (dead_hypotheses.count(hypothesis->id)) continue;

            size_t h_quota = h_quotas[hypothesis->id];

            std::vector<FastSLAMParticle> c_particles;
            std::vector<double> c_weights;
            
            for (size_t idx : hypothesis_particle_indices[hypothesis->id]) {
                c_particles.push_back(*(particles_.begin() + idx));
                c_weights.push_back(weights_view[idx]);
            }

            if (c_particles.empty()) continue;

            auto resampling_view = beluga::views::sample(c_particles, c_weights) | 
                                   ranges::views::take(h_quota);
            
            for (auto it = resampling_view.begin(); it != resampling_view.end(); ++it) {
                buffer.push_back(*it); 
            }
        }

        particles_.assign(buffer.begin(), buffer.end());

        // Garbage collect: remove hypotheses with no surviving particles
        hypotheses_.erase(
            std::remove_if(hypotheses_.begin(), hypotheses_.end(),
                [this](const auto& hypothesis) {
                    for (const auto& p : particles_) {
                        if (std::get<2>(p) == hypothesis) return false;
                    }
                    return true;
                }),
            hypotheses_.end());
    }

    void detect_and_split_modes(const std::vector<double>& weights_view) {
        auto hypotheses_snapshot = hypotheses_;
        
        std::map<size_t, std::vector<size_t>> hypothesis_particle_indices;
        for (size_t i = 0; i < particles_.size(); ++i) {
            hypothesis_particle_indices[std::get<2>(*(particles_.begin() + i))->id].push_back(i);
        }

        for (auto& hypothesis : hypotheses_snapshot) {
            const auto& indices = hypothesis_particle_indices[hypothesis->id];
            if (indices.empty()) continue;

            std::vector<state_type> c_states;
            std::vector<double> c_weights;
            c_states.reserve(indices.size());
            c_weights.reserve(indices.size());
            for (size_t idx : indices) {
                c_states.push_back(std::get<0>(*(particles_.begin() + idx)));
                c_weights.push_back(weights_view[idx]);
            }

            // --- SPATIAL CLUSTERING ---
            beluga::ParticleClusterizerParam cluster_params;
            cluster_params.linear_hash_resolution = 1.0; 
            cluster_params.angular_hash_resolution = 0.5;
            beluga::ParticleClusterizer clusterizer(cluster_params);
            
            auto spatial_cluster_ids = clusterizer(c_states, c_weights);

            std::map<size_t, std::vector<size_t>> s_cluster_to_indices;
            std::map<size_t, double> s_cluster_weights;
            for (size_t i = 0; i < spatial_cluster_ids.size(); ++i) {
                s_cluster_to_indices[spatial_cluster_ids[i]].push_back(i);
                s_cluster_weights[spatial_cluster_ids[i]] += c_weights[i];
            }

            double s_total_weight = 0;
            for (auto& pair : s_cluster_weights) s_total_weight += pair.second;

            // Sort spatial clusters by weight descending
            std::vector<std::pair<size_t, double>> sorted_s_clusters(s_cluster_weights.begin(), s_cluster_weights.end());
            std::sort(sorted_s_clusters.begin(), sorted_s_clusters.end(), [](const auto& a, const auto& b){ return a.second > b.second; });

            bool is_first_spatial_cluster = true;
            for (const auto& [scid, scweight] : sorted_s_clusters) {
                // Only preserve valid spatial sub-hypotheses (e.g. > 5% weight of the local hypothesis)
                if (scweight / s_total_weight < 0.05) continue;

                std::shared_ptr<Hypothesis> target_hypothesis = hypothesis;

                if (!is_first_spatial_cluster) {
                    // SPATIAL DIVERGENCE FORK!
                    target_hypothesis = std::make_shared<Hypothesis>(*hypothesis); // Copy history & properties
                    target_hypothesis->id = next_hypothesis_id_++;
                    
                    // The new hypothesis explicitly SHARES the exact same active submap.
                    // Copy-on-Write (COW) is enforced right before inserting the scan in update_occupancy_grid.
                    target_hypothesis->submaps.active_submap = hypothesis->submaps.active_submap;
                    hypotheses_.push_back(target_hypothesis);
                    
                    std::cout << "\n[SPATIAL DIVERGENCE] Hipotesis " << hypothesis->id 
                              << " se bifurco en la hipotesis " << target_hypothesis->id << std::endl;
                }
                is_first_spatial_cluster = false;

                // Re-assign the particles to the correct hypothesis
                for (size_t local_idx : s_cluster_to_indices[scid]) {
                    size_t global_idx = indices[local_idx];
                    std::get<2>(*(particles_.begin() + global_idx)) = target_hypothesis;
                }
            }
        }
    }

    /// Bresenham's 2D line drawing algorithm.
    std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1, int max_x, int max_y) const {
        std::vector<std::pair<int, int>> line;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x0 == x1 && y0 == y1) break;
            if (x0 >= 0 && x0 < max_x && y0 >= 0 && y0 < max_y) {
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
    void sync_log_odds_to_occupancy(const GridTypeLO& log_odds_grid, GridTypeOC& out_oc) {
        auto& oc_data = out_oc.data();
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

                if (x < 0 || x >= (int)log_odds_grid.width() ||
                    y < 0 || y >= (int)log_odds_grid.height())
                    continue;

                int idx = y * log_odds_grid.width() + x;
                log_odds_grid.at(idx) = -5.0f;
            }
        }
    }

    GridTypeOC best_occupancy_grid() const {
        return best_oc_grid_;
    }

    state_type best_pose() const {
        return best_pose_;
    }

    GridTypeLO best_log_odds_grid() const {
        return best_lo_grid_;
    }

    /// Composites the history of submaps and active submap into a global LogOddsGrid.
    /// \param only_local If true, only draws the active submap and the most recent frozen submap.
    void composite_submaps(const SubmapList& submaps, GridTypeLO& global_lo, bool only_local = false) {
        std::fill(global_lo.data().begin(), global_lo.data().end(), 0.0f);

        auto draw_submap = [&](const std::shared_ptr<Submap>& sm) {
            if (!sm) return;
            const auto& local_lo = *sm->grid();
            for (int ly = 0; ly < local_lo.height(); ++ly) {
                for (int lx = 0; lx < local_lo.width(); ++lx) {
                    float val = local_lo.at(lx, ly);
                    if (val == 0.0f) continue;

                    // Convert local cell index to local coordinates
                    double local_x = local_lo.origin_x() + (lx + 0.5) * local_lo.resolution();
                    double local_y = local_lo.origin_y() + (ly + 0.5) * local_lo.resolution();

                    // Convert local coordinates to global coordinates using the submap's global pose
                    auto global_pt = sm->global_pose() * Eigen::Vector2d(local_x, local_y);

                    int gx, gy, g_idx;
                    if (world_to_index(global_pt.x(), global_pt.y(), gx, gy, g_idx, global_lo)) {
                        global_lo.at(g_idx) += val;
                    }
                }
            }
        };

        if (only_local) {
            // Draw only the most recent frozen submap and the active one
            if (!submaps.history.empty()) {
                draw_submap(submaps.history.back());
            }
        } else {
            // Draw full history
            for (const auto& sm : submaps.history) {
                draw_submap(sm);
            }
        }
        
        draw_submap(submaps.active_submap);

        for (auto& val : global_lo.data()) {
            val = std::clamp(val, -5.0f, 5.0f);
        }
    }

    /// Step 3: Detect and Inject Loop Closure candidates (per hypothesis)
    void detect_loop_closure(const std::vector<std::pair<double, double>>& z_sparse) {
        if (z_sparse.empty()) return;

        // We iterate over a copy of hypotheses_ because we may add new hypotheses during iteration
        auto hypotheses_snapshot = hypotheses_;

        for (auto& hypothesis : hypotheses_snapshot) {
            // Per-hypothesis cooldown ticks down every scan
            if (hypothesis->loop_closure_cooldown > 0) {
                hypothesis->loop_closure_cooldown--;
            }

            // OPTIMIZATION: Only search for loop closures precisely when a submap finishes.
            // This prevents redundant intra-submap checks and ensures the constraint links full submaps.
            if (hypothesis->submaps.active_submap != nullptr) {
                continue;
            }

            // If cooldown is still active, wait until it finishes
            if (hypothesis->loop_closure_cooldown > 0) {
                continue;
            }

            // 1. Find the best particle in THIS hypothesis for its representative pose
            double best_w = -1.0;
            state_type representative_pose;
            for (const auto& p : particles_) {
                if (std::get<2>(p) != hypothesis) continue;
                double w = static_cast<double>(std::get<1>(p));
                if (w > best_w) {
                    best_w = w;
                    representative_pose = std::get<0>(p);
                }
            }
            if (best_w < 0) continue;  // Dead hypothesis

            // 2. Search THIS hypothesis's history for proximity matches
            const auto& history = hypothesis->submaps.history;
            for (size_t i = 0; i < history.size(); ++i) {
                if (i + 5 >= history.size()) continue;

                auto old_submap = history[i];
                
                double dx = representative_pose.translation().x() - old_submap->global_pose().translation().x();
                double dy = representative_pose.translation().y() - old_submap->global_pose().translation().y();
                double distance = std::sqrt(dx*dx + dy*dy);

                if (distance < 5.0) {
                    std::cout << "\n[LOOP CLOSURE] Hipotesis " << hypothesis->id 
                              << ": Candidato por PROXIMIDAD! Submapa " << i 
                              << " detectado a " << distance << "m. Iniciando escaneo correlativo..." << std::endl;

                    // 3. Fast Correlative Scan Matching (FCSM)
                    double best_score = -std::numeric_limits<double>::infinity();
                    state_type best_match = representative_pose;

                    for (double sx = -3.0; sx <= 3.0; sx += 0.25) {
                        for (double sy = -3.0; sy <= 3.0; sy += 0.25) {
                            for (double stheta = -0.5; stheta <= 0.5; stheta += 0.1) {
                                
                                state_type candidate{
                                    Sophus::SO2d{representative_pose.so2().log() + stheta},
                                    Eigen::Vector2d{representative_pose.translation().x() + sx, representative_pose.translation().y() + sy}
                                };

                                Sophus::SE2d T_submap_robot = old_submap->global_pose().inverse() * candidate;
                                double score = 0.0;
                                for (const auto& pt : z_sparse) {
                                    auto hit = T_submap_robot * Eigen::Vector2d(pt.first, pt.second);
                                    int gx, gy, hit_idx;
                                    if (world_to_index(hit.x(), hit.y(), gx, gy, hit_idx, *(old_submap->grid()))) {
                                        score += old_submap->grid()->at(hit_idx);
                                    }
                                }

                                if (score > best_score) {
                                    best_score = score;
                                    best_match = candidate;
                                }
                            }
                        }
                    }

                    double avg_score = best_score / z_sparse.size();
                    if (avg_score > 0.5) {
                        std::cout << "[LOOP CLOSURE] Hipotesis " << hypothesis->id 
                                  << ": ALINEACION EXITOSA! Score: " << avg_score 
                                  << ". Creando nueva hipotesis bifurcada..." << std::endl;

                        // 3. Move worst 20% of particles from this hypothesis
                        std::vector<size_t> h_indices;
                        for (size_t pi = 0; pi < particles_.size(); ++pi) {
                            if (std::get<2>(*(particles_.begin() + pi)) == hypothesis) {
                                h_indices.push_back(pi);
                            }
                        }

                        if (h_indices.size() > 5) {
                            std::sort(h_indices.begin(), h_indices.end(), [&](size_t a, size_t b) {
                                return std::get<1>(*(particles_.begin() + a)) < std::get<1>(*(particles_.begin() + b));
                            });
                            
                            size_t to_move = h_indices.size() * 0.2;
                            if (to_move > 0) {
                                // 4. Create a NEW hypothesis (fork)
                                auto new_hypothesis = std::make_shared<Hypothesis>();
                                new_hypothesis->id = next_hypothesis_id_++;
                                new_hypothesis->submaps = hypothesis->submaps;  // Copy the full history
                                
                                // Insert explicit loop constraint
                                LoopConstraint constraint;
                                constraint.id = new_hypothesis->submaps.loop_constraints.size();
                                constraint.query_idx = new_hypothesis->submaps.history.size() - 1; // The latest finished submap
                                constraint.reference_idx = i;
                                // Calculate the rigid correction delta that snaps the drifted robot to the true aligned pose
                                auto correction_delta = best_match * representative_pose.inverse();
                                
                                // The query submap suffered the same drift as the robot. Apply the delta to find its true pose.
                                auto query_submap = new_hypothesis->submaps.history.back();
                                auto true_query_pose = correction_delta * query_submap->global_pose();

                                // The relative transform constraint is from the reference submap to the true query submap
                                constraint.T_reference_query = old_submap->global_pose().inverse() * true_query_pose;
                                constraint.information = Eigen::Matrix3d::Identity();
                                new_hypothesis->submaps.loop_constraints.push_back(constraint);

                                // Throw away the drifted active submap because it overlaps with the old visited map!
                                // The system will automatically create a fresh one at the corrected pose on the next scan.
                                new_hypothesis->submaps.active_submap = nullptr;
                                
                                hypotheses_.push_back(new_hypothesis);

                                // Calculate average weight of the top 80% to give to the moved ones
                                double sum_w_top = 0.0;
                                for (size_t k = to_move; k < h_indices.size(); ++k) {
                                    sum_w_top += static_cast<double>(std::get<1>(*(particles_.begin() + h_indices[k])));
                                }
                                double avg_w = sum_w_top / (h_indices.size() - to_move);

                                for (size_t k = 0; k < to_move; ++k) {
                                    auto it = particles_.begin() + h_indices[k];
                                    // Teleport particle
                                    std::get<0>(*it) = best_match * representative_pose.inverse() * std::get<0>(*it);
                                    // Assign to new hypothesis
                                    std::get<2>(*it) = new_hypothesis;
                                    // Boost weight
                                    std::get<1>(*it) = beluga::Weight(avg_w);
                                }

                                // Set cooldown for BOTH to avoid rapid re-triggering while ambiguity resolves
                                hypothesis->loop_closure_cooldown = 200;
                                new_hypothesis->loop_closure_cooldown = 200;

                                // Renormalize all weights across all particles
                                double global_sum = 0.0;
                                for (const auto& p : particles_) global_sum += static_cast<double>(std::get<1>(p));
                                if (global_sum > 0.0) {
                                    for (auto&& p : particles_) {
                                        std::get<1>(p) = beluga::Weight(static_cast<double>(std::get<1>(p)) / global_sum);
                                    }
                                }
                                break; // Only process one loop closure per hypothesis per step
                            }
                        }
                    } else {
                        std::cout << "[LOOP CLOSURE] Hipotesis " << hypothesis->id 
                                  << ": Falsa alarma (Score bajo: " << avg_score << ")" << std::endl;
                    }
                }
            }
        }
    }

    /// Step 4: Pose Graph Optimization (PGO) - Ceres Solver
    void optimize_pose_graph(std::shared_ptr<Hypothesis> target_hypothesis) {
        auto& submap_list = target_hypothesis->submaps;
        auto& hist = submap_list.history;
        size_t num_poses = hist.size();
        if (num_poses < 2) return;

        std::cout << "\n[PGO] Iniciando Ceres Pose Graph Optimization..." << std::endl;
        std::cout << "[PGO] Optimizando grafo con " << num_poses << " nodos y " 
                  << submap_list.loop_constraints.size() << " loop closures." << std::endl;

        std::vector<std::array<double, 3>> poses(num_poses);

        // 1. Initialize parameter blocks with current poses
        for (size_t i = 0; i < num_poses; ++i) {
            auto current_pose = hist[i]->global_pose();
            poses[i][0] = current_pose.translation().x();
            poses[i][1] = current_pose.translation().y();
            poses[i][2] = current_pose.so2().log();
        }

        ceres::Problem problem;

        // 2. Add Odometry Edges (Sequential constraints)
        for (const auto& odom : submap_list.odometry_constraints) {
            if (odom.from_idx >= num_poses || odom.to_idx >= num_poses) continue;
            
            ceres::CostFunction* cost_function = PoseGraphEdgeError::Create(
                odom.relative_pose.translation().x(),
                odom.relative_pose.translation().y(),
                odom.relative_pose.so2().log()
            );

            problem.AddResidualBlock(cost_function, nullptr, poses[odom.from_idx].data(), poses[odom.to_idx].data());
        }

        // 3. Add Loop Closure Edges
        for (const auto& constraint : submap_list.loop_constraints) {
            if (constraint.query_idx >= num_poses || constraint.reference_idx >= num_poses) continue;

            ceres::CostFunction* loop_cost = PoseGraphEdgeError::Create(
                constraint.T_reference_query.translation().x(),
                constraint.T_reference_query.translation().y(),
                constraint.T_reference_query.so2().log(),
                10.0, // translation weight (10x stronger means 100x squared cost)
                10.0  // rotation weight
            );

            // We can add a robust loss function (e.g., HuberLoss) for loop closures to reject outliers
            ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
            problem.AddResidualBlock(loop_cost, loss_function, poses[constraint.reference_idx].data(), poses[constraint.query_idx].data());
        }

        // 4. Anchor the first pose
        problem.SetParameterBlockConstant(poses[0].data());

        // 5. Solve
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << std::endl;

        // 6. Write back optimized poses and compute correction delta
        auto old_last_pose = hist.back()->global_pose();
        
        for (size_t i = 1; i < num_poses; ++i) {
            Sophus::SE2d corrected_pose{
                Sophus::SO2d{poses[i][2]}, 
                Eigen::Vector2d{poses[i][0], poses[i][1]}
            };

            // Copy-on-Write: If this submap is shared with another hypothesis, clone it
            if (hist[i].use_count() > 1) {
                hist[i] = hist[i]->clone();
            }

            hist[i]->set_global_pose(corrected_pose);
        }

        auto new_last_pose = hist.back()->global_pose();
        auto delta_tf = new_last_pose * old_last_pose.inverse();

        // 7. Apply correction delta to the active submap to preserve relative anchoring
        if (submap_list.active_submap) {
            submap_list.make_active_unique();
            submap_list.active_submap->set_global_pose(delta_tf * submap_list.active_submap->global_pose());
        }

        // 8. Apply correction delta to all particles in THIS hypothesis
        for (auto&& p : particles_) {
            if (std::get<2>(p) == target_hypothesis) {
                std::get<0>(p) = delta_tf * std::get<0>(p);
            }
        }

        // Correct the global best pose if it was moved
        best_pose_ = delta_tf * best_pose_;

        std::cout << "[PGO] Ceres Optimization Finalizada. Particulas y submapas corregidos." << std::endl;
    }

private:
    std::vector<std::shared_ptr<Hypothesis>> hypotheses_;
    size_t next_hypothesis_id_ = 0;
    beluga::TupleVector<FastSLAMParticle> particles_;

    MotionModel motion_model_;
    MeasurementModel measurement_model_;
    FastSLAMParams params_;

    /// Log-Odds constants for occupancy grid updates.
    const float l_occ_ = 1.2f;
    const float l_free_ = -0.2f;

    beluga::spatial_hash<state_type> spatial_hasher_;

    GridTypeOC best_oc_grid_;
    GridTypeLO best_lo_grid_;
    GridTypeOC local_oc_grid_;
    GridTypeLO local_lo_grid_;
    state_type best_pose_;

    std::mt19937 rng_ = std::mt19937(std::random_device{}());
};  

#endif