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
#include <array>
#include <limits>
#include <set>
#include <unordered_map>
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

    /// \brief Cartographer's overlapping submap lifecycle, driven by one number.
    /// A submap is created, and once it has received `submap_num_range_data` scans the
    /// next one is created; from then on both receive every scan. The older one is
    /// finished when it reaches twice that count, at which point the newer one has
    /// reached the count itself and starts the next submap. Every finished submap has
    /// therefore seen 2 * submap_num_range_data scans, and consecutive submaps overlap
    /// by exactly half.
    int submap_num_range_data = 15;

    /// Motion filter for retaining graph scan nodes (not every processed scan).
    double keyframe_min_translation = 0.15;
    double keyframe_min_rotation = 5.0 * Sophus::Constants<double>::pi() / 180.0;
    std::size_t max_points_per_scan_node = 180;

    /// Bounded loop-constraint search and branching.
    std::size_t loop_recent_submaps = 5;
    std::size_t loop_max_candidates = 6;
    std::size_t loop_max_branches = 2;
    std::size_t max_hypotheses = 4;
    double loop_candidate_distance = 10.0;
    double loop_search_translation = 3.0;
    double loop_search_rotation = 0.7;
    double loop_min_score = 0.55;
    double loop_min_overlap = 0.35;
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

      params_.submap_num_range_data = std::max(1, params_.submap_num_range_data);
      params_.max_points_per_scan_node =
          std::max<std::size_t>(1, params_.max_points_per_scan_node);
      params_.loop_max_candidates =
          std::max<std::size_t>(1, params_.loop_max_candidates);
      params_.loop_max_branches =
          std::max<std::size_t>(1, params_.loop_max_branches);
      params_.max_hypotheses = std::max<std::size_t>(1, params_.max_hypotheses);
      
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
      // Start at the configured default extent, so the first publications before any
      // submap exists look exactly as they did before the views became dynamic.
      best_lo_grid_ = GridTypeLO();
      local_lo_grid_ = GridTypeLO();
      sync_log_odds_to_occupancy(best_lo_grid_, best_oc_grid_);
      best_pose_ = state_type{};

    }

    /// Returns a reference to the current set of particles.
    [[nodiscard]] const auto& particles() const { return particles_; }
    [[nodiscard]] auto& particles() { return particles_; }

    [[nodiscard]] size_t get_active_hypotheses_count() const { return hypotheses_.size(); }
    
    [[nodiscard]] size_t get_submaps_count() const { 
        return hypotheses_.empty() ? 0 : hypotheses_.front()->submaps.history.size(); 
    }

    /// Returns all detected loop closure poses (persistent, for RViz visualization)
    [[nodiscard]] const std::vector<Sophus::SE2d>& loop_closure_poses() const { return loop_closure_poses_; }

    /// Returns all spatial cluster split poses (persistent, for RViz visualization)
    [[nodiscard]] const std::vector<Sophus::SE2d>& spatial_split_poses() const { return spatial_split_poses_; }

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
        constexpr size_t kStep = 4;
        z_sparse.reserve(z.size() / kStep + 1);
        for (size_t i = 0; i < z.size(); i += kStep) {
            z_sparse.push_back(z[i]);
        }
        
        /// Scan matching search grids
        auto dxys1 = {-0.1, 0.0, 0.1};
        auto dthetas1 = {-5 * Sophus::Constants<double>::pi() / 180, 0.0, 5 * Sophus::Constants<double>::pi() / 180};

        auto dxys2 = {-0.05, 0.0, 0.05};
        auto dthetas2 = {-2.5 * Sophus::Constants<double>::pi() / 180, 0.0, 2.5 * Sophus::Constants<double>::pi() / 180};

        auto dxys3 = {-0.02, 0.0, 0.02};
        auto dthetas3 = {-1.0 * Sophus::Constants<double>::pi() / 180, 0.0, 1.0 * Sophus::Constants<double>::pi() / 180};

        // 1. Composite local map ONCE PER HYPOTHESIS and cache it
        std::map<size_t, GridTypeLO> hypothesis_lo_cache;
        for (auto& hypothesis : hypotheses_) {
            // Find a representative pose for this hypothesis to select nearby submaps
            state_type representative_pose;
            double max_w = -1.0;
            for (const auto& p : particles_) {
                if (std::get<2>(p)->id == hypothesis->id) {
                    double w = static_cast<double>(std::get<1>(p));
                    if (w > max_w) {
                        max_w = w;
                        representative_pose = std::get<0>(p);
                    }
                }
            }

            compose_tracking_view(hypothesis, representative_pose, local_lo_grid_);
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

            // Fast Endpoint-Score Model with Out-of-Bounds Penalty
            auto score_fn = [&](const state_type& candidate_pose) {
                double log_prob_sum = 0.0;
                for (const auto& local_point : z_sparse) {
                    auto hit = candidate_pose * Eigen::Vector2d(local_point.first, local_point.second);
                    int gx, gy, hit_idx;
                    if (world_to_index(hit.x(), hit.y(), gx, gy, hit_idx, cached_grid)) {
                        log_prob_sum += cached_grid.at(hit_idx);
                    } else {
                        // Heavily penalize out-of-bounds endpoints (equivalent to hitting solidly confirmed free space)
                        // This prevents the filter from trying to eject the scan into unknown space
                        // to avoid slight negative scores from noise in known space.
                        log_prob_sum -= 5.0; 
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
            
            auto best_pose_st2 = best_pose;

            for (double dx : dxys3) {
                for (double dy : dxys3) {
                    for (double dtheta : dthetas3) {
                        auto candidate_pose = state_type{
                            Sophus::SO2d{best_pose_st2.so2().log() + dtheta}, 
                            Eigen::Vector2d{best_pose_st2.translation().x() + dx, best_pose_st2.translation().y() + dy}
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

    struct ScanMatchResult {
        Sophus::SE2d T_submap_node;
        double score = 0.0;
        double overlap = 0.0;
        bool valid = false;
    };

    struct SearchState {
        Sophus::SE2d pose;
        double score = 0.0;
        double overlap = 0.0;
    };

    [[nodiscard]] std::shared_ptr<const ScanNodeData> make_scan_node_data(
        const measurement_type& z) {
        auto data = std::make_shared<ScanNodeData>();
        data->sequence = next_scan_sequence_++;
        if (z.empty()) return data;
        const std::size_t point_limit =
            std::max<std::size_t>(1, params_.max_points_per_scan_node);
        const std::size_t stride = std::max<std::size_t>(
            1, (z.size() + point_limit - 1) / point_limit);
        data->returns.reserve(std::min(z.size(), point_limit));
        for (std::size_t i = 0; i < z.size() && data->returns.size() < point_limit;
             i += stride) {
            data->returns.push_back(z[i]);
        }
        return data;
    }

    [[nodiscard]] bool should_create_scan_node(
        const SubmapList& submaps, const state_type& pose) const {
        if (!submaps.has_last_keyframe_pose) return true;
        const auto delta = submaps.last_keyframe_pose.inverse() * pose;
        return delta.translation().norm() >= params_.keyframe_min_translation ||
               std::abs(delta.so2().log()) >= params_.keyframe_min_rotation;
    }

    [[nodiscard]] SearchState score_scan_in_submap(
        const ScanNodeData& data, const Submap& submap,
        const Sophus::SE2d& T_submap_node) const {
        SearchState result{T_submap_node, 0.0, 0.0};
        if (data.returns.empty()) return result;
        constexpr double kSigma = 0.20;
        std::size_t inside = 0;
        double likelihood_sum = 0.0;
        for (const auto& point : data.returns) {
            const Eigen::Vector2d hit =
                T_submap_node * Eigen::Vector2d{point.first, point.second};
            const float distance = submap.distance_at(hit.x(), hit.y());
            if (!std::isfinite(distance)) continue;
            ++inside;
            const double normalized = static_cast<double>(distance) / kSigma;
            likelihood_sum += std::exp(-0.5 * normalized * normalized);
        }
        result.overlap = static_cast<double>(inside) / data.returns.size();
        if (inside > 0) result.score = likelihood_sum / inside;
        return result;
    }

    /**
     * Bounded correlative scan matching: a coarse lattice followed by beam refinements.
     * This replaces the previous exhaustive four-level submap-to-submap search.
     */
    [[nodiscard]] ScanMatchResult match_scan_to_submap(
        const ScanNodeData& data, const Submap& submap,
        const Sophus::SE2d& initial_pose) const {
        constexpr std::size_t kBeamWidth = 8;
        std::vector<SearchState> beam;
        for (double dx = -params_.loop_search_translation;
             dx <= params_.loop_search_translation + 1.0e-9; dx += 0.50) {
            for (double dy = -params_.loop_search_translation;
                 dy <= params_.loop_search_translation + 1.0e-9; dy += 0.50) {
                for (double angle = -params_.loop_search_rotation;
                     angle <= params_.loop_search_rotation + 1.0e-9; angle += 0.14) {
                    const auto candidate = initial_pose * Sophus::SE2d{
                        Sophus::SO2d{angle}, Eigen::Vector2d{dx, dy}};
                    auto state = score_scan_in_submap(data, submap, candidate);
                    if (state.overlap >= params_.loop_min_overlap * 0.75) beam.push_back(state);
                }
            }
        }
        if (beam.empty()) return {};
        auto better = [](const SearchState& a, const SearchState& b) {
            return a.score * std::min(1.0, a.overlap / 0.5) >
                   b.score * std::min(1.0, b.overlap / 0.5);
        };
        if (beam.size() > kBeamWidth) {
            std::partial_sort(beam.begin(), beam.begin() + kBeamWidth, beam.end(), better);
            beam.resize(kBeamWidth);
        } else {
            std::sort(beam.begin(), beam.end(), better);
        }

        for (const auto& resolution : std::array<std::pair<double, double>, 3>{
                 std::pair<double, double>{0.15, 0.04}, {0.05, 0.015}, {0.015, 0.005}}) {
            std::vector<SearchState> refined;
            refined.reserve(beam.size() * 27);
            for (const auto& seed : beam) {
                for (int ix = -1; ix <= 1; ++ix) {
                    for (int iy = -1; iy <= 1; ++iy) {
                        for (int ia = -1; ia <= 1; ++ia) {
                            const auto candidate = seed.pose * Sophus::SE2d{
                                Sophus::SO2d{ia * resolution.second},
                                Eigen::Vector2d{ix * resolution.first, iy * resolution.first}};
                            auto state = score_scan_in_submap(data, submap, candidate);
                            if (state.overlap >= params_.loop_min_overlap * 0.75) refined.push_back(state);
                        }
                    }
                }
            }
            if (refined.empty()) break;
            if (refined.size() > kBeamWidth) {
                std::partial_sort(refined.begin(), refined.begin() + kBeamWidth, refined.end(), better);
                refined.resize(kBeamWidth);
            } else {
                std::sort(refined.begin(), refined.end(), better);
            }
            beam = std::move(refined);
        }

        const auto& best = beam.front();
        return {best.pose, best.score, best.overlap,
                best.score >= params_.loop_min_score &&
                    best.overlap >= params_.loop_min_overlap};
    }

    /// Update the occupancy grid map of each hypothesis based on the transformed measurement.
    std::vector<FinishedSubmapEvent> update_occupancy_grid(const measurement_type& z) {
        std::vector<FinishedSubmapEvent> finished_events;
        std::shared_ptr<const ScanNodeData> shared_scan_data;

        if (z.empty()) return finished_events;

        for (auto& hypothesis : hypotheses_) {
            auto& submaps = hypothesis->submaps;

            // The graph receives one representative local-SLAM pose per hypothesis.
            double best_w = -1.0;
            state_type best_pose;
            for (const auto& p : particles_) {
                if (std::get<2>(p)->id == hypothesis->id) {
                    double w = static_cast<double>(std::get<1>(p));
                    if (w > best_w) {
                        best_w = w;
                        best_pose = std::get<0>(p);
                    }
                }
            }
            if (best_w < 0) continue;

            // Cartographer-style motion filter: it bounds the size of the pose graph
            // only. Every scan is still inserted into the active submaps, so grid
            // quality does not depend on the keyframe rate.
            const bool create_graph_node = should_create_scan_node(submaps, best_pose);
            if (create_graph_node && !shared_scan_data) {
                shared_scan_data = make_scan_node_data(z);
            }
            const bool retain_node =
                create_graph_node && shared_scan_data && !shared_scan_data->returns.empty();

            submaps.make_active_unique();

            // Cartographer's lifecycle, driven only by the scan count: the next submap
            // starts once the newest one has received submap_num_range_data scans, and
            // both then receive every scan until the older reaches twice the count and
            // is finished below. Grids grow to fit the scan, so nothing about the
            // lifecycle depends on how far the robot travelled.
            const bool needs_new_submap =
                submaps.active_submaps.empty() ||
                submaps.active_submaps.back()->num_insertions() >=
                    params_.submap_num_range_data;
            if (needs_new_submap) {
                // Guard for the at-most-two-active invariant. The count-driven path
                // never trips it: the front reaches twice the count and is finished on
                // the scan before the newest reaches the count and starts the next one.
                for (SubmapId id : submaps.make_room_for_new_submap(kMaxActiveSubmaps)) {
                    finished_events.push_back({hypothesis->id, id});
                }
                submaps.active_submaps.push_back(std::make_shared<Submap>(
                    submaps.next_submap_id++, best_pose, SUBMAP_COLS, SUBMAP_ROWS,
                    GRID_RESOLUTION));
            }

            TrajectoryNode node;
            if (retain_node) {
                node.id = submaps.next_node_id++;
                node.constant_data = shared_scan_data;
                node.global_pose = best_pose;

                if (!submaps.trajectory_nodes.empty()) {
                    const auto& previous = submaps.trajectory_nodes.back();
                    submaps.local_trajectory_constraints.push_back({
                        previous.id, node.id,
                        previous.global_pose.inverse() * node.global_pose,
                        3.0, 5.0});
                }
            }

            // Every scan is inserted into every overlapping active submap. Intra-submap
            // constraints are measured before insertion, but only for retained nodes.
            for (auto& active_submap : submaps.active_submaps) {
                const auto T_s_r = active_submap->global_pose().inverse() * best_pose;
                if (retain_node) {
                    submaps.node_submap_constraints.push_back({
                        active_submap->id(), node.id, T_s_r,
                        5.0, 8.0, ConstraintTag::kIntraSubmap, 1.0, 1.0});
                }

                auto& lo_grid = active_submap->mutable_grid();

                // Grow, hits, misses, one touch per cell, hits win. See the helper.
                ScanInsertionParams insertion_params;
                insertion_params.l_occ = l_occ_;
                insertion_params.l_free = l_free_;
                insertion_params.clamp = 5.0f;
                insertion_params.robot_radius = ROBOT_RADIUS;
                insert_scan_into_submap_grid(
                    lo_grid, T_s_r, z, insertion_params, scan_hit_cells_, scan_miss_cells_);

                active_submap->add_insertion();
            }

            if (retain_node) {
                submaps.trajectory_nodes.push_back(std::move(node));
                submaps.last_keyframe_pose = best_pose;
                submaps.has_last_keyframe_pose = true;
            }

            // A submap is finished after twice the count, which is exactly when the
            // submap that started at its halfway point has itself reached the count.
            for (SubmapId id :
                 submaps.finish_ready_submaps(2 * params_.submap_num_range_data)) {
                finished_events.push_back({hypothesis->id, id});
            }
        }
        return finished_events;
    }

    /// Step 5: Post-update processing (Loop closure, PGO, Best map composite)
    void post_update(const measurement_type& z, const std::vector<FinishedSubmapEvent>& finished_events) {
        // --- Loop Closure Detection (driven by FinishedSubmapEvents) ---
        (void)z;
        detect_loop_closure(finished_events);
        if (!finished_events.empty()) {
            for (auto& hypothesis : hypotheses_) {
                hypothesis->submaps.trim_scan_data_outside_active_submaps();
            }
        }

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
        if (best_hypothesis->submaps.inter_constraint_count() >
            best_hypothesis->optimized_inter_constraints_count) {
            optimize_pose_graph(best_hypothesis);
            best_hypothesis->optimized_inter_constraints_count =
                best_hypothesis->submaps.inter_constraint_count();
            best_particle_weight = -1.0;
            for (const auto& p : particles_) {
                if (std::get<2>(p) != best_hypothesis) continue;
                const double weight = static_cast<double>(std::get<1>(p));
                if (weight > best_particle_weight) {
                    best_particle_weight = weight;
                    best_pose_ = std::get<0>(p);
                }
            }
        }

        // Composite full global map from the best particle's hypothesis for RViz
        compose_publication_view(best_hypothesis, best_lo_grid_);
        sync_log_odds_to_occupancy(best_lo_grid_, best_oc_grid_);
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

        // Gather surviving hypotheses
        std::vector<std::pair<size_t, double>> surviving_hypotheses;
        for (auto& h : hypotheses_) {
            if (!dead_hypotheses.count(h->id)) {
                surviving_hypotheses.push_back({h->id, hypothesis_total_weight[h->id]});
            }
        }

        // Sort descending by weight
        std::sort(surviving_hypotheses.begin(), surviving_hypotheses.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        // Enforce maximum of 4 hypotheses to protect the budget
        size_t max_hypotheses = 4;
        while (surviving_hypotheses.size() > max_hypotheses) {
            dead_hypotheses.insert(surviving_hypotheses.back().first);
            surviving_hypotheses.pop_back();
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

        double surviving_weight_sum = 0.0;
        for (const auto& sh : surviving_hypotheses) {
            surviving_weight_sum += sh.second;
        }

        size_t K = surviving_hypotheses.size();
        if (K == 0) {
            // If all hypotheses are dead, fallback to keeping the best one
            K = 1;
            surviving_hypotheses.push_back({hypotheses_.front()->id, 1.0});
            surviving_weight_sum = 1.0;
            dead_hypotheses.erase(hypotheses_.front()->id);
        }

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

        // RESET WEIGHTS: After resampling, reset weights to a uniform distribution 
        // so they are fresh for the next sample/measurement step.
        double uniform_weight = 1.0 / particles_.size();
        for (auto&& w : beluga::views::weights(particles_)) {
            w = beluga::Weight(uniform_weight);
        }
    }

    void detect_and_split_modes(std::vector<double>& weights_view) {
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
                if (scweight / s_total_weight < 0.05) {
                    // Kill particles in small modes to avoid polluting the main hypothesis
                    for (size_t local_idx : s_cluster_to_indices[scid]) {
                        size_t global_idx = indices[local_idx];
                        std::get<1>(*(particles_.begin() + global_idx)) = beluga::Weight(0.0);
                        weights_view[global_idx] = 0.0;
                    }
                    continue;
                }

                std::shared_ptr<Hypothesis> target_hypothesis = hypothesis;

                if (!is_first_spatial_cluster) {
                    if (hypotheses_.size() >= params_.max_hypotheses) {
                        // Keep this geometric mode in its parent when the bounded
                        // hypothesis budget is exhausted.
                        for (size_t local_idx : s_cluster_to_indices[scid]) {
                            const size_t global_idx = indices[local_idx];
                            std::get<2>(*(particles_.begin() + global_idx)) = hypothesis;
                        }
                        continue;
                    }
                    // SPATIAL DIVERGENCE FORK!
                    target_hypothesis = std::make_shared<Hypothesis>(*hypothesis); // Copy history & properties
                    target_hypothesis->id = next_hypothesis_id_++;
                    
                    // The new hypothesis explicitly SHARES the exact same active submaps.
                    // Copy-on-Write (COW) is enforced right before inserting the scan in update_occupancy_grid.
                    // ScanNodeData and submap grids remain shared. Only the small vectors
                    // of poses/constraints are copied for the diverging graph state.
                    target_hypothesis->submaps.active_submaps = hypothesis->submaps.active_submaps;
                    target_hypothesis->optimized_inter_constraints_count =
                        hypothesis->optimized_inter_constraints_count;
                    hypotheses_.push_back(target_hypothesis);

                    // Find representative pose of the diverged cluster
                    double best_c_weight = -1.0;
                    state_type split_pose = c_states[s_cluster_to_indices[scid].front()];
                    for (size_t local_idx : s_cluster_to_indices[scid]) {
                        if (c_weights[local_idx] > best_c_weight) {
                            best_c_weight = c_weights[local_idx];
                            split_pose = c_states[local_idx];
                        }
                    }
                    spatial_split_poses_.push_back(split_pose);
                    
                    std::cout << "\n\033[1;31m[SPATIAL DIVERGENCE] Hipotesis " << hypothesis->id 
                              << " se bifurco en la hipotesis " << target_hypothesis->id << "\033[0m" << std::endl;
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

    /// Converts world coordinates to grid indices and linear index for map access.
    inline bool world_to_index(double wx, double wy, int &gx, int &gy, int &index, const GridTypeLO& grid) const {
        gx = static_cast<int>(std::floor((wx - grid.origin_x()) / grid.resolution()));
        gy = static_cast<int>(std::floor((wy - grid.origin_y()) / grid.resolution()));

        if (gx < 0 || gx >= grid.width() || gy < 0 || gy >= grid.height()) return false;

        index = gy * grid.width() + gx;
        return true;
    }

    /// Synchronizes the occupancy grid representation with the log-odds grid by applying a thresholding function.
    void sync_log_odds_to_occupancy(const GridTypeLO& log_odds_grid,
                                    DynamicOccupancyGrid& out_oc) {
        const auto& lo_data = log_odds_grid.data();
        constexpr float OCCUPIED_THRESH = 0.65f;
        constexpr float FREE_THRESH     = 0.35f; //0.196

        // The occupancy view always adopts the extent of the log-odds view it mirrors, so
        // the two can never disagree about which rectangle of the world they describe.
        out_oc.reset(log_odds_grid.width(), log_odds_grid.height(),
                     log_odds_grid.resolution(), log_odds_grid.origin(),
                     static_cast<std::int8_t>(UNKNOWN));
        auto& oc_data = out_oc.data();

        for (size_t i = 0; i < lo_data.size(); ++i) {
            if (std::abs(lo_data[i]) < 0.01f) {
                oc_data[i] = UNKNOWN; 
            } else {
                float p = 1.0f / (1.0f + std::exp(-lo_data[i]));
                oc_data[i] = (p > OCCUPIED_THRESH) ? OCCUPPIED : (p < FREE_THRESH ? FREE : UNKNOWN);
            }
        }
    }

    DynamicOccupancyGrid best_occupancy_grid() const {
        return best_oc_grid_;
    }

    state_type best_pose() const {
        return best_pose_;
    }

    GridTypeLO best_log_odds_grid() const {
        return best_lo_grid_;
    }

    void draw_submap_into_grid(const std::shared_ptr<Submap>& sm, GridTypeLO& target_lo) const {
        if (!sm) return;
        const auto& local_lo = sm->grid();
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
                if (world_to_index(global_pt.x(), global_pt.y(), gx, gy, g_idx, target_lo)) {
                    float current = target_lo.at(g_idx);
                    if (val > 0.0f) {
                        // Maximize occupied space confidence
                        target_lo.at(g_idx) = std::max(current, val);
                    } else if (val < 0.0f && current <= 0.0f) {
                        // Maximize free space confidence (negative direction), but don't overwrite walls
                        target_lo.at(g_idx) = std::min(current, val);
                    }
                }
            }
        }
    }

    /// Composites the full history of submaps for RViz publication.
    /**
     * The view is sized to the hypothesis it is drawn from rather than to a fixed
     * rectangle, so a trajectory that leaves the configured bounds is published whole
     * instead of clipped. This is a derived view only: nothing here feeds back into the
     * SLAM state, which lives entirely in the submaps and the pose graph.
     */
    void compose_publication_view(const std::shared_ptr<Hypothesis>& hypothesis, GridTypeLO& global_lo) const {
        /// Unknown border kept around the map so it does not end flush against a wall.
        constexpr double kPublicationMargin = 1.0;
        /// A diverged pose must not be able to request an unbounded allocation.
        constexpr int kMaxCellsPerSide = 8000;

        double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
        if (hypothesis->submaps.bounding_box(min_x, min_y, max_x, max_y)) {
            const double resolution = global_lo.resolution();
            min_x -= kPublicationMargin;
            min_y -= kPublicationMargin;
            max_x += kPublicationMargin;
            max_y += kPublicationMargin;
            const int width = std::min(kMaxCellsPerSide,
                std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution))));
            const int height = std::min(kMaxCellsPerSide,
                std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution))));
            global_lo.reset(width, height,
                            Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{min_x, min_y}});
        } else {
            // No submap yet: keep the current extent and just clear it.
            global_lo.reset(global_lo.width(), global_lo.height(), global_lo.origin());
        }

        for (const auto& sm : hypothesis->submaps.history) {
            draw_submap_into_grid(sm, global_lo);
        }
        for (const auto& active_submap : hypothesis->submaps.active_submaps) {
            draw_submap_into_grid(active_submap, global_lo);
        }

        for (auto& val : global_lo.data()) {
            val = std::clamp(val, -5.0f, 5.0f);
        }
    }

    /// Local tracking view: match against every active submap. Cartographer can match
    /// against a single one because its grids grow on demand; our submaps are a fixed
    /// 12 x 12 m while the LiDAR reaches much further, so restricting the view to the
    /// oldest active submap starves the scan matcher of endpoints as the robot leaves
    /// it. Historical submaps still participate only through graph constraints, not by
    /// being mixed into the local likelihood field.
    void compose_tracking_view(const std::shared_ptr<Hypothesis>& hypothesis, const state_type& representative_pose, GridTypeLO& tracking_lo) const {
        (void)representative_pose;
        std::fill(tracking_lo.data().begin(), tracking_lo.data().end(), 0.0f);

        if (!hypothesis->submaps.active_submaps.empty()) {
            for (const auto& active_submap : hypothesis->submaps.active_submaps) {
                draw_submap_into_grid(active_submap, tracking_lo);
            }
        } else if (!hypothesis->submaps.history.empty()) {
            draw_submap_into_grid(hypothesis->submaps.history.back(), tracking_lo);
        }

        for (auto& val : tracking_lo.data()) {
            val = std::clamp(val, -5.0f, 5.0f);
        }
    }


    struct LoopCandidate {
        SubmapId reference_submap_id = 0;
        ScanNodeId query_node_id = 0;
        double score = 0.0;
        double overlap = 0.0;
        std::size_t reference_history_index = 0;
        Sophus::SE2d T_reference_node;
        bool valid = false;
    };

    /**
     * Detects loop constraints between selected scan nodes and old frozen submaps.
     * Candidate retrieval is bounded; geometric verification uses a distance field
     * and beam refinement rather than exhaustive submap-to-submap raster matching.
     */
    void detect_loop_closure(const std::vector<FinishedSubmapEvent>& finished_events) {
        for (const auto& event : finished_events) {
            auto hypothesis_it = std::find_if(
                hypotheses_.begin(), hypotheses_.end(), [&](const auto& hypothesis) {
                    return hypothesis->id == event.hypothesis_id;
                });
            if (hypothesis_it == hypotheses_.end()) continue;
            const auto hypothesis = *hypothesis_it;
            auto& graph = hypothesis->submaps;
            const auto query_submap = graph.find_submap(event.query_submap_id);
            if (!query_submap || graph.history.size() <= params_.loop_recent_submaps) continue;

            auto query_node_ids = graph.insertion_nodes(event.query_submap_id);
            if (query_node_ids.empty()) continue;

            // Three scans distributed through the finished submap provide robustness
            // without matching all of its keyframes against every candidate.
            std::vector<ScanNodeId> representative_nodes;
            const std::size_t samples = std::min<std::size_t>(3, query_node_ids.size());
            for (std::size_t i = 0; i < samples; ++i) {
                const std::size_t index = samples == 1
                    ? query_node_ids.size() - 1
                    : i * (query_node_ids.size() - 1) / (samples - 1);
                representative_nodes.push_back(query_node_ids[index]);
            }

            const auto query_history_it = std::find_if(
                graph.history.begin(), graph.history.end(), [&](const auto& submap) {
                    return submap->id() == event.query_submap_id;
                });
            if (query_history_it == graph.history.end()) continue;
            const std::size_t query_index =
                static_cast<std::size_t>(std::distance(graph.history.begin(), query_history_it));

            struct RetrievalCandidate {
                std::size_t history_index;
                double distance;
                double signature_distance;
            };
            std::vector<RetrievalCandidate> retrieval;
            for (std::size_t index = 0; index < graph.history.size(); ++index) {
                if (index + params_.loop_recent_submaps >= query_index) continue;
                const auto& reference = graph.history[index];
                const double distance =
                    (reference->global_pose().translation() -
                     query_submap->global_pose().translation()).norm();
                if (distance > params_.loop_candidate_distance) continue;

                double signature_distance = 0.0;
                const auto& a = reference->radial_signature();
                const auto& b = query_submap->radial_signature();
                const std::size_t count = std::min(a.size(), b.size());
                for (std::size_t i = 0; i < count; ++i) {
                    const double delta = a[i] - b[i];
                    signature_distance += delta * delta;
                }
                retrieval.push_back({index, distance, signature_distance});
            }
            std::sort(retrieval.begin(), retrieval.end(), [](const auto& a, const auto& b) {
                // Geometry is primary; the signature cheaply breaks nearby ties.
                return a.distance + 4.0 * a.signature_distance <
                       b.distance + 4.0 * b.signature_distance;
            });
            if (retrieval.size() > params_.loop_max_candidates) {
                retrieval.resize(params_.loop_max_candidates);
            }

            std::vector<LoopCandidate> candidates;
            for (const auto& retrieved : retrieval) {
                const auto& reference = graph.history[retrieved.history_index];
                LoopCandidate best;
                best.reference_submap_id = reference->id();
                best.reference_history_index = retrieved.history_index;
                for (ScanNodeId node_id : representative_nodes) {
                    const auto* node = graph.find_node(node_id);
                    if (!node || !node->constant_data) continue;
                    const auto initial =
                        reference->global_pose().inverse() * node->global_pose;
                    const auto match = match_scan_to_submap(
                        *node->constant_data, *reference, initial);
                    if (match.valid && match.score > best.score) {
                        best.query_node_id = node_id;
                        best.score = match.score;
                        best.overlap = match.overlap;
                        best.T_reference_node = match.T_submap_node;
                        best.valid = true;
                    }
                }
                if (best.valid) candidates.push_back(best);
            }
            std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
                return a.score * a.overlap > b.score * b.overlap;
            });

            // NMS avoids branching on adjacent old submaps that represent one place.
            std::vector<LoopCandidate> distinct;
            for (const auto& candidate : candidates) {
                bool separated = true;
                for (const auto& accepted : distinct) {
                    const auto delta = static_cast<long long>(candidate.reference_history_index) -
                                       static_cast<long long>(accepted.reference_history_index);
                    if (std::abs(delta) < static_cast<long long>(params_.loop_recent_submaps)) {
                        separated = false;
                        break;
                    }
                }
                if (separated) distinct.push_back(candidate);
            }

            std::vector<std::size_t> particle_indices;
            for (std::size_t i = 0; i < particles_.size(); ++i) {
                if (std::get<2>(*(particles_.begin() + i)) == hypothesis) {
                    particle_indices.push_back(i);
                }
            }
            if (particle_indices.size() <= 5 || distinct.empty()) continue;
            std::sort(particle_indices.begin(), particle_indices.end(), [&](std::size_t a, std::size_t b) {
                return std::get<1>(*(particles_.begin() + a)) <
                       std::get<1>(*(particles_.begin() + b));
            });

            const std::size_t available_hypotheses =
                params_.max_hypotheses > hypotheses_.size()
                    ? params_.max_hypotheses - hypotheses_.size() : 0;
            const std::size_t forks = std::min({
                params_.loop_max_branches, distinct.size(), available_hypotheses});
            if (forks == 0) continue;
            const std::size_t particles_per_fork = std::max<std::size_t>(
                1, std::min(particle_indices.size() / 5,
                            particle_indices.size() / (forks + 1)));

            for (std::size_t fork = 0; fork < forks; ++fork) {
                const auto& candidate = distinct[fork];
                auto child = std::make_shared<Hypothesis>(*hypothesis);
                child->id = next_hypothesis_id_++;
                child->submaps.node_submap_constraints.push_back({
                    candidate.reference_submap_id,
                    candidate.query_node_id,
                    candidate.T_reference_node,
                    10.0,
                    12.0,
                    ConstraintTag::kInterSubmap,
                    candidate.score,
                    candidate.overlap});
                hypotheses_.push_back(child);

                const std::size_t begin = fork * particles_per_fork;
                const std::size_t end = std::min(begin + particles_per_fork, particle_indices.size());
                for (std::size_t i = begin; i < end; ++i) {
                    std::get<2>(*(particles_.begin() + particle_indices[i])) = child;
                }

                const auto* matched_node = child->submaps.find_node(candidate.query_node_id);
                if (matched_node) loop_closure_poses_.push_back(matched_node->global_pose);
                optimize_pose_graph(child);
                child->optimized_inter_constraints_count =
                    child->submaps.inter_constraint_count();
                std::cout << "[LOOP CLOSURE] H" << hypothesis->id << " -> H"
                          << child->id << ", scan " << candidate.query_node_id
                          << " x submap " << candidate.reference_submap_id
                          << ", score=" << candidate.score
                          << ", overlap=" << candidate.overlap << std::endl;
            }

            double total_weight = 0.0;
            for (const auto& particle : particles_) {
                total_weight += static_cast<double>(std::get<1>(particle));
            }
            if (total_weight > 0.0) {
                for (auto&& particle : particles_) {
                    std::get<1>(particle) = beluga::Weight(
                        static_cast<double>(std::get<1>(particle)) / total_weight);
                }
            }
        }
    }

    /// Step 4: joint scan-node/submap Pose Graph Optimization.

    void optimize_pose_graph(const std::shared_ptr<Hypothesis>& hypothesis) {
        auto& graph = hypothesis->submaps;
        if (graph.trajectory_nodes.empty() || graph.node_submap_constraints.empty()) return;

        std::map<SubmapId, std::array<double, 3>> submap_poses;
        for (const auto& submap : graph.history) {
            const auto& pose = submap->global_pose();
            submap_poses[submap->id()] = {
                pose.translation().x(), pose.translation().y(), pose.so2().log()};
        }
        for (const auto& submap : graph.active_submaps) {
            const auto& pose = submap->global_pose();
            submap_poses[submap->id()] = {
                pose.translation().x(), pose.translation().y(), pose.so2().log()};
        }

        std::map<ScanNodeId, std::array<double, 3>> node_poses;
        for (const auto& node : graph.trajectory_nodes) {
            node_poses[node.id] = {
                node.global_pose.translation().x(), node.global_pose.translation().y(),
                node.global_pose.so2().log()};
        }
        if (submap_poses.empty() || node_poses.empty()) return;

        const auto old_latest_node_pose = graph.trajectory_nodes.back().global_pose;
        ceres::Problem problem;
        std::size_t residual_count = 0;

        for (const auto& constraint : graph.node_submap_constraints) {
            auto submap_it = submap_poses.find(constraint.submap_id);
            auto node_it = node_poses.find(constraint.node_id);
            if (submap_it == submap_poses.end() || node_it == node_poses.end()) continue;
            auto* cost = PoseGraphEdgeError::Create(
                constraint.T_submap_node.translation().x(),
                constraint.T_submap_node.translation().y(),
                constraint.T_submap_node.so2().log(),
                constraint.translation_weight,
                constraint.rotation_weight);
            ceres::LossFunction* loss = constraint.tag == ConstraintTag::kInterSubmap
                ? static_cast<ceres::LossFunction*>(new ceres::HuberLoss(1.0)) : nullptr;
            problem.AddResidualBlock(
                cost, loss, submap_it->second.data(), node_it->second.data());
            ++residual_count;
        }

        for (const auto& constraint : graph.local_trajectory_constraints) {
            auto from_it = node_poses.find(constraint.from_node_id);
            auto to_it = node_poses.find(constraint.to_node_id);
            if (from_it == node_poses.end() || to_it == node_poses.end()) continue;
            auto* cost = PoseGraphEdgeError::Create(
                constraint.T_from_to.translation().x(),
                constraint.T_from_to.translation().y(),
                constraint.T_from_to.so2().log(),
                constraint.translation_weight,
                constraint.rotation_weight);
            problem.AddResidualBlock(cost, nullptr, from_it->second.data(), to_it->second.data());
            ++residual_count;
        }
        if (residual_count == 0) return;

        // Gauge freedom is removed by anchoring the first submap only.
        problem.SetParameterBlockConstant(submap_poses.begin()->second.data());
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 50;
        options.num_threads = 1;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        auto write_submap_pose = [&](std::shared_ptr<Submap>& submap) {
            const auto pose_it = submap_poses.find(submap->id());
            if (pose_it == submap_poses.end()) return;
            if (submap.use_count() > 1) submap = submap->clone();
            submap->set_global_pose(Sophus::SE2d{
                Sophus::SO2d{pose_it->second[2]},
                Eigen::Vector2d{pose_it->second[0], pose_it->second[1]}});
        };
        for (auto& submap : graph.history) write_submap_pose(submap);
        for (auto& submap : graph.active_submaps) write_submap_pose(submap);

        for (auto& node : graph.trajectory_nodes) {
            const auto pose_it = node_poses.find(node.id);
            if (pose_it == node_poses.end()) continue;
            node.global_pose = Sophus::SE2d{
                Sophus::SO2d{pose_it->second[2]},
                Eigen::Vector2d{pose_it->second[0], pose_it->second[1]}};
        }

        // Particles are current online states, not graph variables. Transport them
        // by the correction of the latest retained local-SLAM node.
        const auto correction =
            graph.trajectory_nodes.back().global_pose * old_latest_node_pose.inverse();
        for (auto&& particle : particles_) {
            if (std::get<2>(particle) == hypothesis) {
                std::get<0>(particle) = correction * std::get<0>(particle);
            }
        }
        graph.last_keyframe_pose = correction * graph.last_keyframe_pose;

        std::cout << "[PGO] H" << hypothesis->id << ": "
                  << submap_poses.size() << " submaps, " << node_poses.size()
                  << " scan nodes, " << residual_count << " constraints. "
                  << summary.BriefReport() << std::endl;
    }

private:
    std::vector<std::shared_ptr<Hypothesis>> hypotheses_;
    size_t next_hypothesis_id_ = 0;
    std::uint64_t next_scan_sequence_ = 0;
    beluga::TupleVector<FastSLAMParticle> particles_;

    MotionModel motion_model_;
    MeasurementModel measurement_model_;
    FastSLAMParams params_;

    /// Cartographer's structural invariant: one submap being filled and one being
    /// started, both receiving every accepted scan. Not a tuning knob.
    static constexpr std::size_t kMaxActiveSubmaps = 2;

    /// Log-Odds constants for occupancy grid updates.
    const float l_occ_ = 1.2f;
    const float l_free_ = -0.2f;

    /// Scratch for one scan insertion, reused so the per-scan cost is not allocation.
    std::vector<int> scan_hit_cells_;
    std::vector<int> scan_miss_cells_;

    beluga::spatial_hash<state_type> spatial_hasher_;

    /// Derived publication views. Both are rebuilt from the best hypothesis every scan
    /// and sized to it; neither is part of the SLAM representation.
    DynamicOccupancyGrid best_oc_grid_;
    GridTypeLO best_lo_grid_;
    GridTypeLO local_lo_grid_;
    state_type best_pose_;

    /// Persistent record of all loop closure detection poses for RViz visualization
    std::vector<Sophus::SE2d> loop_closure_poses_;

    /// Persistent record of all spatial divergence split poses for RViz visualization
    std::vector<Sophus::SE2d> spatial_split_poses_;

    std::mt19937 rng_ = std::mt19937(std::random_device{}());
};  

#endif
