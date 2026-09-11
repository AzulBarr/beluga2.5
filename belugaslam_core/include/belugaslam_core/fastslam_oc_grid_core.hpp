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
#include <fstream>
#include <string>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include "loop_belief.hpp"
#include "hierarchical_bayes.hpp"
#include "particle_proposal.hpp"
#include "scan_informed_proposal.hpp"
#include "proposal_pose.hpp"
#include "output_selection.hpp"
#include <ceres/ceres.h>
#include "pose_graph_cost.hpp"

#include <range/v3/view/take.hpp>
#include <range/v3/range/conversion.hpp> 
#include <range/v3/view/take_exactly.hpp>

#include "particle.hpp"
#include "submap.hpp"
#include "ceres_probability_tracking.hpp"
#include "point_to_line_icp.hpp"
#include "motion_filter.hpp"

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
    beluga::Weight,                 // derived LINEAR conditional weight, for Beluga views
    std::shared_ptr<Hypothesis>,
    double                         // authoritative normalized LOG conditional weight
>;

// Keep the legacy tuple interface for Beluga; inference owns the log weight.
template<class Particle>
inline void set_conditional_log_weight(Particle&& p, double log_weight) {
    std::get<3>(p) = log_weight;
    std::get<1>(p) = beluga::Weight(std::exp(log_weight));
}
template<class Particle>
inline double joint_particle_weight(const Particle& p) {
    return std::exp(std::get<2>(p)->log_mass + std::get<3>(p));
}

// Compile-time override remains available, but loop closure is enabled by default.
#ifndef BELUGASLAM_ENABLE_LOOP_CLOSURE
#define BELUGASLAM_ENABLE_LOOP_CLOSURE 1
#endif

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

    /// \brief Legacy setting; production scan evidence uses tracking.effective_beams.
    double likelihood_scaling_factor = 0.05;

    /// \brief Cartographer's overlapping submap lifecycle, driven by one number.
    /// The counted unit is a KEYFRAME, not a raw scan: insertion sits behind the
    /// motion filter, so a scan that fails keyframe_min_translation/rotation/max_time
    /// never reaches this count. A submap is created, and once it has received
    /// `submap_num_range_data` keyframes the next one is created; from then on both
    /// receive every keyframe. The older one is finished when it reaches twice that
    /// count, at which point the newer one has reached the count itself and starts
    /// the next submap. Every finished submap has therefore seen
    /// 2 * submap_num_range_data keyframes, and consecutive submaps overlap by
    /// exactly half. At the default 30 that is 60 keyframes per finished submap.
    int submap_num_range_data = 30;

    /// Motion filter for BOTH grid insertion and graph nodes, after scan matching.
    double keyframe_min_translation = 0.15;
    double keyframe_min_rotation = 5.0 * Sophus::Constants<double>::pi() / 180.0;
    double keyframe_max_time = 5.0;  // seconds in the scan timestamp clock
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
    bool enable_loop_closure = true;
    bool enable_pgo = true;
    std::string loop_verifier_mode = "belief";  // belief, map, uniform, geometry
    // Output decision only. pose_risk minimizes retained frontend position loss.
    std::string output_selection_mode = "map";  // map, pose_risk
    std::string frontend_pose_mode = "frontend";  // frontend, proposal_mean, proposal_seed
    double proposal_pose_min_ess = 5.0;
    double proposal_pose_min_local_mass = 0.90;
    double proposal_pose_max_log_drop = 0.02;
    /// Scans an undecided loop event may stay unresolved before the MAP branch is
    /// forced. The window does NOT gather evidence: graph masses are deliberately
    /// frozen while unresolved, because two branches holding different maps have
    /// no comparable tracking likelihood. What it buys is the chance for another
    /// loop event to supersede the ambiguity with real evidence before anything
    /// is committed. Zero commits immediately.
    std::size_t loop_undecided_max_scans = 20;
    double loop_belief_threshold = 0.25;
    double loop_translation_scale = 0.30;  // aligned trajectory RMSE, meters
    double loop_rotation_scale = 0.10;     // aligned trajectory RMSE, radians
    double loop_max_fit_translation = 0.30;
    double loop_max_fit_rotation = 0.12;
    double loop_branch_prior = 0.5;
    double loop_null_compatibility = 0.2; // legacy heuristic mode only
    std::string loop_update_mode = "bayes"; // bayes or heuristic (controlled ablation)
    belugaslam::SequentialLoopOptions loop_bayes;
    double loop_geometry_min_compatibility = 0.01; // gate, never a probability
    double hypothesis_prune_mass = 1.e-6;
    std::string loop_bayes_diagnostics_path;
    std::size_t loop_max_verifications = 6;
    std::size_t loop_trajectory_samples = 200;
    std::size_t loop_min_points = 30;
    std::size_t pgo_every_n_nodes = 20;
    int pgo_max_iterations = 50;
    bool pgo_analytic_jacobians = true;
    bool loop_robust_polish = true;

    /// Continuous refinement of each loop measurement, as Cartographer refines its
    /// correlative optimum with Ceres before building a constraint. Measured on a
    /// synthetic L-wall fixture, this recovers between 0 and 6 mm: the beam search
    /// already samples at 0.015 m, finer than the chamfer field's own accuracy, so
    /// the residual is the field's discretization bias and not a search artifact.
    /// Halving map_resolution halves that residual; refining it does not. Off by
    /// default for that reason, and kept as an explicit ablation. The guard below
    /// makes it non-worsening under the loop metric, never a silent regression.
    bool loop_refine = false;
    double loop_refine_translation = 0.15;  // window around the lattice optimum, meters
    double loop_refine_rotation = 0.05;     // window around the lattice optimum, radians
    /// Pose-graph constraint weights, applied as 1/sigma on the SE(2) residual.
    /// They are the ONLY thing that tells the optimizer how much each measurement
    /// class is trusted relative to the others; equal weights make PGO plain
    /// unweighted least squares. Cartographer's 2D ratios for reference:
    /// intra-submap 5e2/1.6e3, loop 1.1e4/1e5, i.e. loops outrank intra-submap
    /// constraints by ~22x in translation and ~62x in rotation. These defaults
    /// keep the historical 3/5, 5/8, 10/12 so nothing changes until swept.
    double pgo_odometry_translation_weight = 3.0;
    double pgo_odometry_rotation_weight = 5.0;
    double pgo_intra_translation_weight = 5.0;
    double pgo_intra_rotation_weight = 8.0;
    double pgo_loop_translation_weight = 10.0;
    double pgo_loop_rotation_weight = 12.0;
    /// Huber scale on the WHITENED inter-submap residual, so the raw translation
    /// it starts to down-weight is pgo_huber_scale / pgo_loop_translation_weight.
    /// It must be retuned whenever the loop weights move.
    double pgo_huber_scale = 1.0;

    /// Log-odds increments of one scan insertion. A single hit at 1.2 already
    /// reads 0.77 and two saturate the matcher's 0.9 clamp, so the probability
    /// grid the Ceres matcher interpolates is nearly binary. Cartographer's
    /// 0.55/0.49 are +0.2/-0.04, an order of magnitude softer.
    float insertion_l_occ = 1.2F;
    float insertion_l_free = -0.2F;

    std::uint32_t random_seed = 42;
    std::string loop_diagnostics_path;
    int worker_threads = 2;  // bounded total concurrency, including the caller
    bool verbose_backend = false;
    belugaslam::TrackingOptions tracking;
    std::string tracking_matcher = "distance";  // distance or probability_ceres (experimental)
    belugaslam::ProbabilityMatchingOptions probability_matching;
    std::string tracking_prior_mode = "fixed";  // fixed (step 1) or odometry (steps 1+2)
    belugaslam::OdometryPriorOptions odometry_prior;
    belugaslam::RecoveryOptions recovery;
    /// Optional point-to-line refinement of the accepted frontend match, against
    /// the retained endpoint cloud rather than the chamfer field. Off by default:
    /// it costs one endpoint cloud per active submap and only ever narrows the
    /// result through a non-worsening gate.
    bool icp_refine = false;
    belugaslam::IcpOptions icp;
    std::size_t motion_proposal_samples = 8;
    bool scan_informed_proposal = true;
    belugaslam::ScanProposalOptions scan_proposal;

    double map_resolution = GRID_RESOLUTION;
    double split_min_mass = 0.02;
    std::size_t split_min_particles = 2, split_persistence = 3;
    std::size_t loop_validation_scans = 3;
    std::string tracking_diagnostics_path;
    std::size_t loop_cache_budget_mb = 64;

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
          params_(params),
          worker_arena_(std::max(1, params.worker_threads)),
          spatial_hasher_{params.spatial_resolution_x, 
                          params.spatial_resolution_y,
                          params.spatial_resolution_theta} {

      params_.loop_bayes.validate();
      params_.probability_matching.validate();
      params_.odometry_prior.validate();
      if (params_.tracking_prior_mode != "fixed" && params_.tracking_prior_mode != "odometry")
          throw std::invalid_argument("tracking_prior_mode must be fixed or odometry");
      if (params_.tracking_matcher != "distance" && params_.tracking_matcher != "probability_ceres")
          throw std::invalid_argument("tracking_matcher must be distance or probability_ceres");
      if (params_.loop_update_mode != "bayes" && params_.loop_update_mode != "heuristic")
          throw std::invalid_argument("loop_update_mode must be bayes or heuristic");
      if (params_.loop_update_mode=="bayes" && params_.loop_verifier_mode!="belief")
          throw std::invalid_argument("Bayesian mode requires loop_verifier_mode=belief; use loop_update_mode=heuristic for the legacy MAP/uniform/geometry ablations");
      if (!(params_.loop_geometry_min_compatibility >= 0 && params_.loop_geometry_min_compatibility <= 1) ||
          !(params_.hypothesis_prune_mass >= 0 && params_.hypothesis_prune_mass < .05))
          throw std::invalid_argument("Invalid geometry gate or hypothesis pruning threshold");
      if (!params_.loop_bayes_diagnostics_path.empty()) {
          bayes_diagnostics_.open(params_.loop_bayes_diagnostics_path);
          if (!bayes_diagnostics_) throw std::runtime_error("Cannot open loop_bayes_diagnostics_path");
          bayes_diagnostics_ << std::setprecision(17)
              << "sequence,event_id,hypothesis,association,prior_mass,posterior_mass,log_mass,log_evidence,evidence_scans,attempted_scans,common_beams,loop_probability,status\n";
      }
      for (const double weight : {params_.pgo_odometry_translation_weight, params_.pgo_odometry_rotation_weight,
                                  params_.pgo_intra_translation_weight, params_.pgo_intra_rotation_weight,
                                  params_.pgo_loop_translation_weight, params_.pgo_loop_rotation_weight,
                                  params_.pgo_huber_scale})
          if (!std::isfinite(weight) || weight <= 0)
              throw std::invalid_argument("PGO constraint weights and Huber scale must be finite and positive");
      if (!std::isfinite(params_.insertion_l_occ) || params_.insertion_l_occ <= 0 ||
          !std::isfinite(params_.insertion_l_free) || params_.insertion_l_free >= 0)
          throw std::invalid_argument("insertion_l_occ must be positive and insertion_l_free negative");
      if (params_.worker_threads < 1) throw std::invalid_argument("worker_threads must be positive");
      if (params_.output_selection_mode != "map" && params_.output_selection_mode != "pose_risk")
          throw std::invalid_argument("output_selection_mode must be map or pose_risk");
      params_.scan_proposal.validate();
      if (params_.frontend_pose_mode != "frontend" && params_.frontend_pose_mode != "proposal_mean" &&
          params_.frontend_pose_mode != "proposal_seed")
          throw std::invalid_argument("frontend_pose_mode must be frontend, proposal_mean or proposal_seed");
      if (!std::isfinite(params_.proposal_pose_min_ess) || params_.proposal_pose_min_ess<1 ||
          !(params_.proposal_pose_min_local_mass>0.5 && params_.proposal_pose_min_local_mass<=1) ||
          !std::isfinite(params_.proposal_pose_max_log_drop) || params_.proposal_pose_max_log_drop<0)
          throw std::invalid_argument("Invalid proposal pose gates");
      for (double x : {params_.tracking.sigma, params_.tracking.prior_translation_sigma,
                       params_.tracking.prior_rotation_sigma, params_.tracking.max_translation,
                       params_.tracking.max_rotation, params_.tracking.inlier_distance,
                       params_.tracking.effective_beams, params_.tracking.prior_information_scale, params_.map_resolution})
          if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("Tracking scales must be finite and positive");
      if (!(params_.tracking.outlier_probability > 0 && params_.tracking.outlier_probability < 1) ||
          !(params_.tracking.min_overlap >= 0 && params_.tracking.min_overlap <= 1) ||
          !(params_.split_min_mass >= 0 && params_.split_min_mass <= 1) ||
          params_.tracking.max_points < params_.tracking.min_points || params_.tracking.min_points == 0 ||
          params_.tracking.max_iterations < 1 || params_.motion_proposal_samples < 1 ||
          params_.split_persistence < 1 || params_.split_min_particles < 1 || params_.loop_validation_scans < 1 ||
          params_.map_resolution < 0.01 || params_.map_resolution > 1.0)
          throw std::invalid_argument("Invalid tracking, proposal, map or split parameters");
      if (params_.icp_refine) {
          for (double x : {params_.icp.cloud.voxel_size, params_.icp.cloud.bucket_size,
                           params_.icp.cloud.normal_radius, params_.icp.max_correspondence_distance,
                           params_.icp.sigma, params_.icp.huber_delta, params_.icp.max_translation_correction,
                           params_.icp.max_rotation_correction, params_.icp.max_rmse, params_.icp.max_condition_number})
              if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("ICP scales must be finite and positive");
          // A bucket narrower than the correspondence radius still works, but the
          // query then sweeps rings of buckets for no benefit.
          if (params_.icp.cloud.bucket_size < params_.icp.max_correspondence_distance ||
              params_.icp.cloud.normal_radius > params_.icp.cloud.bucket_size ||
              params_.icp.cloud.voxel_size >= params_.icp.cloud.normal_radius ||
              params_.icp.cloud.min_normal_neighbors < 3 || params_.icp.max_iterations < 1 ||
              params_.icp.min_correspondences == 0 || params_.icp.objective_tolerance < 0 ||
              !(params_.icp.min_linearity >= 0 && params_.icp.min_linearity <= 1) ||
              !(params_.icp.min_inlier_ratio >= 0 && params_.icp.min_inlier_ratio <= 1))
              throw std::invalid_argument("Invalid ICP refinement geometry");
      }
      if (!std::isfinite(params_.recovery.translation_window) || params_.recovery.translation_window<=0 || params_.recovery.translation_window>3 ||
          !std::isfinite(params_.recovery.rotation_window) || params_.recovery.rotation_window<=0 || params_.recovery.rotation_window>1 ||
          !(params_.recovery.min_overlap>=0 && params_.recovery.min_overlap<=1) ||
          !std::isfinite(params_.recovery.ambiguity_margin) || params_.recovery.ambiguity_margin<0 ||
          params_.recovery.after_failures<1 || params_.recovery.interval<1 || params_.recovery.confirmations<2 ||
          params_.loop_cache_budget_mb>16384)
          throw std::invalid_argument("Invalid bounded recovery/cache parameters");
      if (!params_.tracking_diagnostics_path.empty()) {
          tracking_diagnostics_.open(params_.tracking_diagnostics_path);
          if (!tracking_diagnostics_) throw std::runtime_error("Cannot open tracking_diagnostics_path");
          tracking_diagnostics_ << std::setprecision(17)
              << "sequence,hypothesis,usable,overlap,mean_log_likelihood,x,y,yaw,mass,particles,status,consecutive_weak_scans,reference_submap,correction_m,pf_frontend_distance_m,nodes,submaps,pose_source,proposal_pose_decision,proposal_ess,proposal_local_mass,proposal_position_std_m,proposal_yaw_std_rad,proposal_mean_offset_m,tracking_matcher,tracking_points,tracking_prior_mode,prior_evaluated,prior_cov_xx,prior_cov_xy,prior_cov_xyaw,prior_cov_yy,prior_cov_yyaw,prior_cov_yawyaw,icp_decision,icp_inlier_ratio,icp_rmse,icp_condition_number,icp_correction_m,scan_proposal_status,scan_proposals,frontend_proposals,scan_proposal_fraction,log_p_over_q_min,log_p_over_q_max\n";
      }
      params_.submap_num_range_data = std::max(1, params_.submap_num_range_data);
      params_.max_points_per_scan_node =
          std::max<std::size_t>(1, params_.max_points_per_scan_node);
      params_.loop_max_candidates =
          std::max<std::size_t>(1, params_.loop_max_candidates);
      params_.loop_max_branches =
          std::max<std::size_t>(1, params_.loop_max_branches);
      params_.max_hypotheses = std::max<std::size_t>(1, params_.max_hypotheses);
      for (double threshold : {params_.keyframe_min_translation,
                               params_.keyframe_min_rotation, params_.keyframe_max_time}) {
          if (!std::isfinite(threshold) || threshold < 0.0) {
              throw std::invalid_argument("Motion filter thresholds must be finite and nonnegative");
          }
      }
      
      if (!std::isfinite(params_.likelihood_scaling_factor) || params_.likelihood_scaling_factor < 0.0)
          throw std::invalid_argument("likelihood_scaling_factor must be finite and nonnegative");
      if (params_.min_particles == 0 || params_.max_particles < params_.min_particles)
          throw std::invalid_argument("Require 1 <= min_particles <= max_particles");
      params_.max_hypotheses = std::min(params_.max_hypotheses, params_.max_particles);
      if (params_.loop_undecided_max_scans > 100000)
          throw std::invalid_argument("loop_undecided_max_scans is implausibly large");
      if (params_.loop_verifier_mode != "belief" && params_.loop_verifier_mode != "map" &&
          params_.loop_verifier_mode != "uniform" && params_.loop_verifier_mode != "geometry")
          throw std::invalid_argument("loop_verifier_mode must be belief, map, uniform or geometry");
      for (double scale : {params_.loop_translation_scale, params_.loop_rotation_scale,
                           params_.loop_max_fit_translation, params_.loop_max_fit_rotation,
                           params_.loop_null_compatibility}) {
          if (!std::isfinite(scale) || scale <= 0.0) throw std::invalid_argument("Loop scales must be finite and positive");
      }
      if (!(params_.loop_branch_prior > 0.0 && params_.loop_branch_prior < 1.0) ||
          !(params_.loop_belief_threshold >= 0.0 && params_.loop_belief_threshold <= 1.0))
          throw std::invalid_argument("Invalid loop branch prior or belief threshold");
      for (double value : {params_.loop_candidate_distance, params_.loop_search_translation, params_.loop_search_rotation}) {
          if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Loop search bounds must be finite and nonnegative");
      }
      for (double value : {params_.loop_min_score, params_.loop_min_overlap}) {
          if (!(value >= 0.0 && value <= 1.0)) throw std::invalid_argument("Loop geometric thresholds must be in [0,1]");
      }
      params_.loop_max_verifications = std::max<std::size_t>(1, params_.loop_max_verifications);
      params_.loop_trajectory_samples = std::max<std::size_t>(3, params_.loop_trajectory_samples);
      params_.loop_min_points = std::max<std::size_t>(3, params_.loop_min_points);
      params_.pgo_every_n_nodes = std::max<std::size_t>(1, params_.pgo_every_n_nodes);
      params_.pgo_max_iterations = std::max(1, params_.pgo_max_iterations);
      for (double value : {params_.loop_refine_translation, params_.loop_refine_rotation}) {
          if (!std::isfinite(value) || !(value > 0.0))
              throw std::invalid_argument("Loop refinement windows must be finite and positive");
      }
      rng_.seed(params_.random_seed == 0 ? std::random_device{}() : params_.random_seed);
      if (!params_.loop_diagnostics_path.empty()) {
          loop_diagnostics_.open(params_.loop_diagnostics_path);
          if (!loop_diagnostics_) throw std::runtime_error("Cannot open loop_diagnostics_path");
          loop_diagnostics_ << std::setprecision(17);
          loop_diagnostics_ << "candidate_id,query_sequence,reference_sequence,source_hypothesis,"
              "candidate_dx,candidate_dy,candidate_dtheta,verifier_mode,hypothesis,prior_weight,"
              "trial_usable,fit_translation,fit_rotation,translation_rmse,rotation_rmse,compatibility,"
              "belief_score,map_score,uniform_score,geometry_score,eligible,selected,forced_fit_translation,forced_fit_rotation,forced_compatibility,settled_compatibility,polish_attempted,polish_ms,verification_status,trial_installed,event_id,query_consumed,retained_branch_mass\n";
      }

      // Create the initial hypothesis (single hypothesis: "exploring")
      auto initial_hypothesis = std::make_shared<Hypothesis>();
      initial_hypothesis->id = next_hypothesis_id_++;
      hypotheses_.push_back(initial_hypothesis);

      // Start with the configured particle budget. Previously a single hypothesis
      // could remain at min_particles forever (five in the Intel launch).
      particles_.resize(params_.max_particles);
      for (auto&& p : particles_) {
        std::get<0>(p) = state_type{};
        set_conditional_log_weight(p, -std::log(static_cast<double>(particles_.size())));
        std::get<2>(p) = initial_hypothesis;  // All particles share the same hypothesis
      }
      // Start at the configured default extent, so the first publications before any
      // submap exists look exactly as they did before the views became dynamic.
      best_lo_grid_ = GridTypeLO(GRID_COLS, GRID_ROWS, params_.map_resolution,
          state_type{Sophus::SO2d{}, Eigen::Vector2d{ORIGIN_X, ORIGIN_Y}});
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
        last_odom_delta_ = std::get<1>(u).inverse() * std::get<0>(u);
        motion_proposals_.clear(); motion_proposals_.resize(particles_.size());
        motion_ancestors_.clear(); motion_ancestors_.reserve(particles_.size());
        motion_distribution_ = motion_model_.increment_distribution(u);
        const bool stationary = last_odom_delta_.translation().squaredNorm() < 1e-24 &&
            std::abs(last_odom_delta_.so2().log()) < 1e-12;
        auto sampler = motion_model_(u);
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            auto&& p = *(particles_.begin() + i);
            const auto previous = std::get<0>(p);
            motion_ancestors_.push_back(previous);
            auto& proposals = motion_proposals_[i];
            const auto count = stationary ? std::size_t{1} : params_.motion_proposal_samples;
            proposals.reserve(count);
            for (std::size_t k = 0; k < count; ++k) proposals.push_back(stationary ? previous : sampler(previous, rng_));
            std::get<0>(p) = proposals.front();
        }
    }

    /// Endpoint score of a pose against a log-odds grid.
    /**
     * Sums the log-odds under every scan endpoint. Endpoints that fall outside the grid
     * are penalised heavily, so the filter cannot improve its score by ejecting the scan
     * into unobserved space.
     */
    [[nodiscard]] double endpoint_log_score(
        const state_type& pose, const measurement_type& z_sparse,
        const GridTypeLO& grid) const {
        double log_prob_sum = 0.0;
        for (const auto& local_point : z_sparse) {
            const auto hit = pose * Eigen::Vector2d(local_point.first, local_point.second);
            int gx, gy, hit_idx;
            if (world_to_index(hit.x(), hit.y(), gx, gy, hit_idx, grid)) {
                log_prob_sum += grid.at(hit_idx);
            } else {
                log_prob_sum -= 5.0;
            }
        }
        return log_prob_sum;
    }

    /// Three-level discrete scan matching around an initial pose.
    /**
     * Coarse to fine: +-0.1 m / +-5 deg, then +-0.05 m / +-2.5 deg around the winner,
     * then +-0.02 m / +-1 deg. Retained as a reference utility for tests and API
     * compatibility. Production tracking uses robust_tracking.hpp; particles use
     * stochastic motion proposals and their mean likelihood importance factor.
     */
    [[nodiscard]] state_type refine_pose_on_grid(
        const state_type& initial_pose, const measurement_type& z_sparse,
        const GridTypeLO& grid, double& best_log_score) const {
        static const auto dxys1 = {-0.1, 0.0, 0.1};
        static const auto dthetas1 = {-5 * Sophus::Constants<double>::pi() / 180, 0.0,
                                      5 * Sophus::Constants<double>::pi() / 180};
        static const auto dxys2 = {-0.05, 0.0, 0.05};
        static const auto dthetas2 = {-2.5 * Sophus::Constants<double>::pi() / 180, 0.0,
                                      2.5 * Sophus::Constants<double>::pi() / 180};
        static const auto dxys3 = {-0.02, 0.0, 0.02};
        static const auto dthetas3 = {-1.0 * Sophus::Constants<double>::pi() / 180, 0.0,
                                      1.0 * Sophus::Constants<double>::pi() / 180};

        auto best_pose = initial_pose;
        best_log_score = endpoint_log_score(initial_pose, z_sparse, grid);

        const auto sweep = [&](const state_type& around, const auto& dxys, const auto& dthetas) {
            for (double dx : dxys) {
                for (double dy : dxys) {
                    for (double dtheta : dthetas) {
                        const auto candidate = state_type{
                            Sophus::SO2d{around.so2().log() + dtheta},
                            Eigen::Vector2d{around.translation().x() + dx,
                                            around.translation().y() + dy}};
                        const double score = endpoint_log_score(candidate, z_sparse, grid);
                        if (score > best_log_score) {
                            best_log_score = score;
                            best_pose = candidate;
                        }
                    }
                }
            }
        };

        sweep(initial_pose, dxys1, dthetas1);
        const auto after_level1 = best_pose;
        sweep(after_level1, dxys2, dthetas2);
        const auto after_level2 = best_pose;
        sweep(after_level2, dxys3, dthetas3);
        return best_pose;
    }

    /// Weighted mean of the particles currently assigned to a hypothesis.
    [[nodiscard]] state_type hypothesis_mean_pose(
        const std::shared_ptr<Hypothesis>& hypothesis) const {
        std::vector<state_type> poses;
        std::vector<double> weights;
        for (const auto& p : particles_) {
            if (std::get<2>(p)->id != hypothesis->id) continue;
            poses.push_back(std::get<0>(p));
            weights.push_back(static_cast<double>(std::get<1>(p)));
        }
        return weighted_mean_pose(poses, weights);
    }

    /// Match directly in the reference submap's native grid. No world raster is
    /// created, so grid growth and arbitrary submap poses cannot clip tracking.
    [[nodiscard]] state_type refine_pose_on_submap(
        const state_type& initial_pose, const measurement_type& z_sparse,
        const Submap* submap, double& best_log_score) const {
        if (!submap || z_sparse.empty()) {
            best_log_score = 0.0;
            return initial_pose;
        }
        const auto T_world_submap = submap->global_pose();
        const auto local_initial_pose = T_world_submap.inverse() * initial_pose;
        const auto local_pose = refine_pose_on_grid(
            local_initial_pose, z_sparse, submap->grid(), best_log_score);
        return T_world_submap * local_pose;
    }

    /// Updates particle weights based on the measurement model and the received measurement.
    /**
     * Register each hypothesis's frontend pose in its native reference submap.
     * Select one of each ancestor's motion proposals by normalized scan likelihood,
     * and update its weight with their mean likelihood before inserting this scan.
     * 
     * \param measurement Measurement data. 
     */
    belugaslam::GaussianTrackingPrior frontend_motion_prior(const belugaslam::PoseSample2& prior) const {
        if (params_.tracking_prior_mode == "fixed")
            return belugaslam::fixed_tracking_prior(params_.tracking.prior_translation_sigma,
                                                    params_.tracking.prior_rotation_sigma);
        const double turn=last_odom_delta_.so2().log();
        return belugaslam::odometry_tracking_prior(
            {last_odom_delta_.translation().x(),last_odom_delta_.translation().y(),turn},
            belugaslam::wrap_angle(prior.yaw-turn),params_.odometry_prior);
    }

    belugaslam::TrackingOptions frontend_tracking_options(const belugaslam::PoseSample2& prior,
        belugaslam::TrackingOptions options) const {
        if (params_.tracking_prior_mode == "odometry") {
            options.prior_sqrt_information=frontend_motion_prior(prior).sqrt_information;
            options.use_full_prior=true;
        }
        return options;
    }

    void record_tracking_prior(const std::shared_ptr<Hypothesis>& h,
        const belugaslam::PoseSample2& prior) const {
        h->tracking_prior_covariance=frontend_motion_prior(prior).covariance;
        // Diagnostics report the effective covariance of the regularizer.
        for(auto& v:h->tracking_prior_covariance) v/=params_.tracking.prior_information_scale;
        h->tracking_prior_evaluated=true;
    }

    belugaslam::TrackingResult match_frontend_scan(const Submap& submap, const measurement_type& scan,
        const belugaslam::PoseSample2& prior, const belugaslam::TrackingOptions& options,
        const belugaslam::PoseSample2* seed = nullptr) const {
        const auto field = submap.tracking_field();
        const auto configured=frontend_tracking_options(prior,options);
        if (params_.tracking_matcher == "probability_ceres")
            return belugaslam::match_probability_scan(*submap.probability_field(), *field, scan, prior,
                                                      configured, params_.probability_matching, seed);
        return belugaslam::match_tracking_scan(*field,scan,prior,configured,seed);
    }

    double frontend_objective(const Submap& submap, const measurement_type& scan,
        const belugaslam::PoseSample2& pose, const belugaslam::PoseSample2& prior,
        const belugaslam::TrackingOptions& options) const {
        const auto configured=frontend_tracking_options(prior,options);
        if (params_.tracking_matcher == "probability_ceres")
            return belugaslam::probability_tracking_objective(*submap.probability_field(),scan,pose,prior,
                                                              configured,params_.probability_matching);
        return belugaslam::tracking_objective(*submap.tracking_field(),scan,pose,prior,configured);
    }

    /// Point-to-line refinement of an accepted match, seeded by that match and
    /// regularized by the same odometry prior. It can only ever narrow the result:
    /// the correction is bounded against the seed and is kept only when it does
    /// not worsen the frontend's own objective.
    ///
    /// The non-worsening gate is a safety net, not an improvement detector, and it
    /// has to be read that way to be worth anything. frontend_objective is what the
    /// chamfer matcher already minimized, so normal.pose is a strict local minimum
    /// of it and any correction scores worse there no matter how much better it
    /// actually is; the chamfer field cannot resolve below half a map cell, which
    /// is the whole band this refinement works in. icp.objective_tolerance sets how
    /// much of that blind spot is allowed, and at zero the refinement is a no-op.
    belugaslam::PoseSample2 refine_with_icp(const std::shared_ptr<Hypothesis>& h, const Submap& submap,
        const measurement_type& scan, const belugaslam::TrackingResult& normal,
        const belugaslam::PoseSample2& prior) const {
        const auto cloud = submap.surface_cloud();
        h->icp_decision="no_surface_cloud";
        if (!cloud || cloud->empty()) return normal.pose;
        auto options=params_.tracking;
        options.min_overlap=std::max(options.min_overlap,params_.recovery.min_overlap);
        const auto configured=frontend_tracking_options(prior,options);
        const auto result=belugaslam::refine_point_to_line(*cloud,scan,normal.pose,prior,configured,params_.icp);
        h->icp_decision=result.reason;
        h->icp_inlier_ratio=result.inlier_ratio; h->icp_rmse=result.rmse;
        h->icp_condition_number=result.condition_number;
        if (!result.accepted) return normal.pose;
        // Compared on the frontend's objective, never on the ICP's own cost: the
        // two measure different things and only this one is commensurable with
        // the pose the frontend already accepted.
        const double refined_cost=frontend_objective(submap,scan,result.pose,prior,options);
        const double original_cost=frontend_objective(submap,scan,normal.pose,prior,options);
        if (!(refined_cost<=original_cost+params_.icp.objective_tolerance)) {
            h->icp_decision="frontend_cost_retained";
            return normal.pose;
        }
        h->icp_decision="icp_accepted";
        h->icp_correction=std::hypot(result.pose.x-normal.pose.x,result.pose.y-normal.pose.y);
        return result.pose;
    }

    struct TrackingReference {
        std::shared_ptr<const Submap> submap;
        std::shared_ptr<const belugaslam::TrackingField> field;
        state_type prediction{};
        state_type frontend_delta{};
        bool has_frontend_delta=false;
    };

    TrackingReference track_hypothesis(const std::shared_ptr<Hypothesis>& h, const measurement_type& scan) {
        const auto predicted = h->has_local_pose ? h->local_pose * last_odom_delta_ : hypothesis_mean_pose(h);
        h->has_local_pose = true; h->tracking_evaluated = true;
        const auto submap = h->submaps.matching_submap();
        TrackingReference reference{submap, submap ? submap->tracking_field() : nullptr, predicted};
        const auto pose_sample = [](const state_type& p) {
            return belugaslam::PoseSample2{p.translation().x(),p.translation().y(),p.so2().log()};
        };
        const auto from_sample = [](const belugaslam::PoseSample2& p) {
            return state_type{Sophus::SO2d{p.yaw},Eigen::Vector2d{p.x,p.y}};
        };
        h->local_pose = predicted; h->tracking_overlap = 0; h->tracking_log_likelihood = 0;
        h->tracking_correction = 0;
        h->tracking_prior_evaluated=false; h->tracking_prior_covariance={};
        h->tracking_usable = !reference.field || reference.field->occupied_cells()==0;
        if (h->tracking_usable) {
            h->tracking_status="bootstrap"; h->tracking_failures=0; h->has_pending_recovery=false;
            return reference;
        }
        h->tracking_reference=submap->id();
        const auto initial = submap->global_pose().inverse()*predicted;
        record_tracking_prior(h,pose_sample(initial));
        const auto normal = match_frontend_scan(*submap,scan,pose_sample(initial),params_.tracking);
        h->tracking_usable=normal.accepted; h->tracking_overlap=normal.score.overlap;
        h->tracking_log_likelihood=normal.score.mean_log_likelihood;
        auto accepted_pose = normal.pose;
        h->icp_decision="disabled"; h->icp_inlier_ratio=0; h->icp_rmse=0;
        h->icp_condition_number=0; h->icp_correction=0;
        if (params_.icp_refine && normal.accepted &&
            normal.score.overlap>=params_.recovery.min_overlap)
            accepted_pose = refine_with_icp(h,*submap,scan,normal,pose_sample(initial));
        h->local_pose=submap->global_pose()*from_sample(accepted_pose);
        const bool weak=!normal.accepted || normal.score.overlap<params_.recovery.min_overlap;
        h->tracking_status=normal.accepted ? (weak ? "weak" : "tracked") : "rejected";
        h->tracking_correction=(predicted.inverse()*h->local_pose).translation().norm();
        if (!weak) {h->tracking_failures=0;h->has_pending_recovery=false;return reference;}
        ++h->tracking_failures;
        if (!params_.recovery.enabled) return reference;

        // A recovery proposal must survive a later scan with no map
        // insertion in between. Odometry transports the pending pose between scans.
        if (h->has_pending_recovery && h->recovery_last_sequence+1 != next_scan_sequence_) h->has_pending_recovery=false;
        if (h->has_pending_recovery) {
            const auto recovery_map=h->submaps.find_submap(h->recovery_reference);
            if (recovery_map) {
                const auto expected=h->recovery_pose*last_odom_delta_;
                auto options=params_.tracking;
                options.min_overlap=std::max(options.min_overlap,params_.recovery.min_overlap);
                const auto field=recovery_map->tracking_field();
                const auto confirmation=match_frontend_scan(*recovery_map,scan,
                    pose_sample(recovery_map->global_pose().inverse()*expected),options);
                const auto pose=recovery_map->global_pose()*from_sample(confirmation.pose);
                const auto difference=expected.inverse()*pose;
                if (confirmation.accepted && difference.translation().norm()<0.25 &&
                    std::abs(difference.so2().log())<0.10) {
                    h->recovery_pose=pose; ++h->recovery_confirmations; h->recovery_last_sequence=next_scan_sequence_;
                    h->tracking_status="recovery_pending"; h->tracking_usable=false;
                    if (h->recovery_confirmations>=params_.recovery.confirmations) {
                        h->local_pose=pose; h->tracking_usable=true; h->tracking_status="recovered";
                        record_tracking_prior(h,pose_sample(recovery_map->global_pose().inverse()*expected));
                        h->tracking_overlap=confirmation.score.overlap;
                        h->tracking_log_likelihood=confirmation.score.mean_log_likelihood;
                        h->tracking_failures=0;h->has_pending_recovery=false;
                        h->submaps.has_matching_submap=true;h->submaps.matching_submap_id=recovery_map->id();
                        h->tracking_reference=recovery_map->id();
                        h->tracking_correction=(predicted.inverse()*pose).translation().norm();
                        reference={recovery_map,field,predicted};
                    }
                    return reference;
                }
            }
            h->has_pending_recovery=false;
        }
        const bool due=h->last_recovery_attempt==std::numeric_limits<std::uint64_t>::max() ||
            next_scan_sequence_-h->last_recovery_attempt>=params_.recovery.interval;
        if (!due || h->tracking_failures<params_.recovery.after_failures) return reference;
        h->last_recovery_attempt=next_scan_sequence_;
        std::vector<std::shared_ptr<const Submap>> choices{submap};
        for (auto it=h->submaps.active_submaps.rbegin();it!=h->submaps.active_submaps.rend();++it) {
            if ((*it)->id()!=submap->id() && (*it)->num_insertions()>=std::min(5,params_.submap_num_range_data)) {
                choices.push_back(*it);break;
            }
        }
        bool found=false,ambiguous=false;
        double best_score=-std::numeric_limits<double>::infinity();
        state_type recovered_pose;
        SubmapId recovered_reference=0;
        for (const auto& choice:choices) {
            const auto field=choice->tracking_field();
            const auto prior=pose_sample(choice->global_pose().inverse()*predicted);
            belugaslam::TrackingResult attempt;
            if (choice->id()!=submap->id()) {
                auto options=params_.tracking;
                options.min_overlap=std::max(options.min_overlap,params_.recovery.min_overlap);
                attempt=match_frontend_scan(*choice,scan,prior,options);
            }
            if (!attempt.accepted) attempt=belugaslam::recover_tracking_scan(*field,scan,prior,params_.tracking,params_.recovery);
            if (!attempt.accepted || attempt.score.mean_log_likelihood<h->tracking_log_likelihood+0.05) continue;
            const auto pose=choice->global_pose()*from_sample(attempt.pose);
            if (found) {
                const auto delta=recovered_pose.inverse()*pose;
                if ((delta.translation().norm()>0.45 || std::abs(delta.so2().log())>0.18) &&
                    std::abs(attempt.score.mean_log_likelihood-best_score)<params_.recovery.ambiguity_margin) ambiguous=true;
            }
            if (attempt.score.mean_log_likelihood>best_score) {
                found=true;best_score=attempt.score.mean_log_likelihood;
                recovered_pose=pose;recovered_reference=choice->id();
            }
        }
        if (found && !ambiguous) {
            h->has_pending_recovery=true;h->recovery_pose=recovered_pose;
            h->recovery_reference=recovered_reference;h->recovery_confirmations=1;h->recovery_last_sequence=next_scan_sequence_;
            h->tracking_status="recovery_pending";h->tracking_usable=false;
        }
        return reference;
    }

    // Integrate every scored proposal BEFORE selecting one representative per
    // ancestor. This readout consumes no random draws and does not change PF weights.
    void estimate_proposal_poses(const measurement_type& scan,
        const std::map<std::size_t, TrackingReference>& references,
        const std::vector<std::vector<double>>& proposal_logs) {
        std::map<std::size_t,std::vector<belugaslam::WeightedPoseProposal>> clouds;
        const auto sample=[](const state_type& p) {
            return belugaslam::PoseSample2{p.translation().x(),p.translation().y(),p.so2().log()};
        };
        for (std::size_t i=0;i<particles_.size();++i) {
            const auto& p=*(particles_.begin()+i);
            const double weight=static_cast<double>(std::get<1>(p));
            if (!(weight>0)) continue;
            const auto id=std::get<2>(p)->id;
            const auto& ref=references.at(id);
            const auto inverse=ref.submap ? ref.submap->global_pose().inverse() : state_type{};
            const auto& proposals=motion_proposals_[i];
            const double log_prior=std::log(weight)-std::log(static_cast<double>(proposals.size()));
            auto& cloud=clouds[id];
            for (std::size_t k=0;k<proposals.size();++k)
                cloud.push_back({sample(inverse*proposals[k]),log_prior+proposal_logs[i][k]});
        }
        for (const auto& h:hypotheses_) {
            const auto& ref=references.at(h->id);
            const auto transform=ref.submap ? ref.submap->global_pose() : state_type{};
            const auto inverse=transform.inverse();
            const auto frontend=sample(inverse*h->local_pose);
            const auto& cloud=clouds[h->id];
            const auto weights=belugaslam::normalized_proposal_weights(cloud);
            const auto summary=belugaslam::summarize_proposal_poses(cloud,weights,frontend,
                params_.tracking.max_translation,params_.tracking.max_rotation);
            h->pose_source="frontend";
            h->proposal_pose_decision="estimator_disabled";
            h->proposal_ess=summary.ess;h->proposal_local_mass=summary.local_mass;
            h->proposal_position_std=summary.position_std;h->proposal_yaw_std=summary.yaw_std;
            h->proposal_mean_offset=summary.valid ? std::hypot(summary.mean.x-frontend.x,summary.mean.y-frontend.y) : 0.;
            if (params_.frontend_pose_mode=="proposal_mean") {
                h->proposal_pose_decision="frontend_not_tracked";
                if (h->tracking_status=="tracked" && ref.field) {
                    auto gate=params_.tracking;
                    gate.min_overlap=std::max(gate.min_overlap,params_.recovery.min_overlap);
                    const auto frontend_score=belugaslam::tracking_score(*ref.field,scan,frontend,gate);
                    const auto decision=belugaslam::check_proposal_pose(summary,*ref.field,scan,
                        sample(inverse*ref.prediction),frontend_score,gate,params_.proposal_pose_min_ess,
                        params_.proposal_pose_min_local_mass,params_.proposal_pose_max_log_drop);
                    h->proposal_pose_decision=decision.reason;
                    if (decision.accepted) {
                        h->local_pose=transform*state_type{Sophus::SO2d{summary.mean.yaw},
                            Eigen::Vector2d{summary.mean.x,summary.mean.y}};
                        h->pose_source="proposal_mean";
                        h->tracking_overlap=decision.score.overlap;
                        h->tracking_log_likelihood=decision.score.mean_log_likelihood;
                        h->tracking_correction=(ref.prediction.inverse()*h->local_pose).translation().norm();
                    }
                }
            }
            if (params_.frontend_pose_mode=="proposal_seed") {
                // Use the integrated particle cloud as an additional optimizer
                // seed, retaining the same odometry prior and original optimum.
                // A noisy Monte Carlo mean must not replace a better scan fit.
                h->proposal_pose_decision="seed_not_eligible";
                if (h->tracking_status=="tracked" && ref.field && summary.valid &&
                    summary.ess>=params_.proposal_pose_min_ess &&
                    summary.local_mass>=params_.proposal_pose_min_local_mass) {
                    auto options=params_.tracking;
                    options.min_overlap=std::max(options.min_overlap,params_.recovery.min_overlap);
                    const auto prior=sample(inverse*ref.prediction);
                    const auto refined=match_frontend_scan(*ref.submap,scan,prior,options,&summary.mean);
                    const double original_cost=frontend_objective(*ref.submap,scan,frontend,prior,options);
                    h->proposal_pose_decision="frontend_cost_retained";
                    if (refined.accepted && refined.final_cost+1e-10<original_cost) {
                        h->local_pose=transform*state_type{Sophus::SO2d{refined.pose.yaw},
                            Eigen::Vector2d{refined.pose.x,refined.pose.y}};
                        h->pose_source="proposal_seed";
                        h->proposal_pose_decision="seed_refinement_accepted";
                        h->tracking_overlap=refined.score.overlap;
                        h->tracking_log_likelihood=refined.score.mean_log_likelihood;
                        h->tracking_correction=(ref.prediction.inverse()*h->local_pose).translation().norm();
                    }
                }
            }
            h->has_pose_covariance=false;
            if (!weights.empty()) {
                const auto moment=belugaslam::proposal_second_moment(cloud,weights,sample(inverse*h->local_pose));
                Eigen::Matrix3d local;
                for (int a=0;a<3;++a) for (int b=0;b<3;++b) local(a,b)=moment[3*a+b];
                Eigen::Matrix3d J=Eigen::Matrix3d::Identity();
                J.topLeftCorner<2,2>()=transform.so2().matrix();
                h->pose_covariance=J*local*J.transpose();
                h->has_pose_covariance=h->pose_covariance.allFinite();
            }
        }
    }

    std::vector<std::vector<double>> prepare_scan_informed_proposals(
        const std::map<std::size_t,TrackingReference>& references,bool evidence_available) {
        std::map<std::size_t,belugaslam::ScanInformedProposal> samplers;
        for (const auto& h:hypotheses_) {
            h->scan_proposal_count=0;h->scan_proposal_frontend_count=0;h->scan_proposal_fraction=0;
            h->scan_proposal_log_ratio_min=0;h->scan_proposal_log_ratio_max=0;
            const auto& ref=references.at(h->id);
            h->scan_proposal_status="disabled";
            if (!params_.scan_informed_proposal || params_.scan_proposal.fraction==0) continue;
            h->scan_proposal_status="missing_evidence";
            if (!evidence_available) continue;
            h->scan_proposal_status="frontend_unavailable";
            if (!ref.has_frontend_delta) continue;
            h->scan_proposal_status="singular_motion";
            if (!motion_distribution_.has_density()) continue;
            h->scan_proposal_status="no_motion_update";
            if (motion_ancestors_.size()!=particles_.size()) continue;
            const auto& d=ref.frontend_delta;
            samplers.emplace(h->id,belugaslam::ScanInformedProposal{motion_distribution_,
                {d.translation().x(),d.translation().y(),d.so2().log()},params_.scan_proposal});
            h->scan_proposal_fraction=samplers.at(h->id).fraction();
            h->scan_proposal_status=h->scan_proposal_fraction>0 ? "mixture" : "frontend_outside_prior";
        }
        std::vector<std::vector<double>> corrections(particles_.size());
        for (std::size_t i=0;i<particles_.size();++i) {
            auto&& particle=*(particles_.begin()+i);
            const auto& h=std::get<2>(particle);
            auto& proposals=motion_proposals_[i];
            if (proposals.empty()) proposals.push_back(std::get<0>(particle));
            corrections[i].resize(proposals.size(),0.);
            const auto sampler=samplers.find(h->id);
            for (std::size_t k=0;k<proposals.size();++k) {
                if (sampler!=samplers.end()) {
                    const auto prior=motion_ancestors_[i].inverse()*proposals[k];
                    const auto draw=sampler->second.draw(
                        {prior.translation().x(),prior.translation().y(),prior.so2().log()},rng_);
                    if (draw.from_frontend) {
                        proposals[k]=motion_ancestors_[i]*state_type{Sophus::SO2d{draw.delta[2]},
                            Eigen::Vector2d{draw.delta[0],draw.delta[1]}};
                        ++h->scan_proposal_frontend_count;
                    }
                    corrections[i][k]=draw.log_ratio;
                }
                const double ratio=corrections[i][k];
                if (h->scan_proposal_count==0) {
                    h->scan_proposal_log_ratio_min=ratio;h->scan_proposal_log_ratio_max=ratio;
                } else {
                    h->scan_proposal_log_ratio_min=std::min(h->scan_proposal_log_ratio_min,ratio);
                    h->scan_proposal_log_ratio_max=std::max(h->scan_proposal_log_ratio_max,ratio);
                }
                ++h->scan_proposal_count;
            }
        }
        return corrections;
    }

    void measurement_model_map(const measurement_type& z) {
        if (z.empty()) return;
        if (bayes_event_.active && bayes_event_.last_sequence==next_scan_sequence_) return;
        measurement_type finite_scan;
        for (const auto& beam : z) if (std::isfinite(beam.first) && std::isfinite(beam.second)) finite_scan.push_back(beam);
        const auto sparse = belugaslam::select_tracking_points(finite_scan, params_.tracking.max_points);
        // Spatial filtering is frontend-only. PF predictive evidence retains its
        // original beam subset and likelihood model in both A/B modes.
        const auto frontend_scan = params_.tracking_matcher == "probability_ceres" ?
            belugaslam::probability_tracking_points(finite_scan,params_.probability_matching.voxel_size,
                                                    params_.tracking.max_points) : sparse;
        std::map<std::size_t, TrackingReference> references;
        for (auto& h : hypotheses_) {
            // Both poses are in the same current graph frame: no PGO correction
            // or switch of selected global hypothesis enters this body increment.
            const auto previous=h->local_pose;
            const bool had_pose=h->has_local_pose;
            auto reference=track_hypothesis(h,frontend_scan);
            reference.frontend_delta=previous.inverse()*h->local_pose;
            reference.has_frontend_delta=had_pose && h->tracking_status=="tracked";
            references.emplace(h->id,std::move(reference));
        }
        if (motion_proposals_.size() != particles_.size()) motion_proposals_.resize(particles_.size());
        const bool future_scan = bayes_event_.active && next_scan_sequence_ >= bayes_event_.first_sequence &&
            bayes_event_.last_sequence != next_scan_sequence_;
        measurement_type common_scan;
        if (future_scan) {
            // Use one common beam subset. Unknown space in one branch must not
            // be treated as a measured obstacle mismatch in another branch.
            for (const auto& beam : sparse) {
                bool known=true;
                for (const auto& h : hypotheses_) {
                    const auto& predicted=references.at(h->id).prediction;
                    known = known && h->validation_map && h->validation_map->observed_endpoint(
                        {predicted.translation().x(),predicted.translation().y(),predicted.so2().log()},beam);
                }
                if (known) common_scan.push_back(beam);
            }
        }
        const bool validation_usable = future_scan && common_scan.size() >= params_.tracking.min_points &&
            common_scan.size() >= params_.loop_bayes.min_known_fraction * sparse.size();
        const auto proposal_corrections=prepare_scan_informed_proposals(
            references,!bayes_event_.active || validation_usable);
        std::vector<std::vector<double>> proposal_logs(particles_.size());
        parallel_indices(particles_.size(), [&](std::size_t i) {
            const auto&& p = *(particles_.begin() + i);
            auto& proposals = motion_proposals_[i];
            if (proposals.empty()) proposals.push_back(std::get<0>(p));
            const auto& ref = std::as_const(references).at(std::get<2>(p)->id);
            auto& logs = proposal_logs[i]; logs=proposal_corrections[i];
            if (!ref.field || ref.field->occupied_cells() == 0) return;
            const auto inverse = ref.submap->global_pose().inverse();
            for (std::size_t k = 0; k < proposals.size(); ++k) {
                const auto pose = inverse * proposals[k];
                logs[k] += params_.tracking.effective_beams * belugaslam::tracking_score(*ref.field, sparse,
                    {pose.translation().x(), pose.translation().y(), pose.so2().log()}, params_.tracking).mean_log_likelihood;
            }
        });
        // The frontend uses its live tracking map. Inference during a validation
        // event uses ONLY the frozen historical map and the SAME corrected
        // proposal weights. The frontend optimum is never a likelihood factor.
        estimate_proposal_poses(frontend_scan,references,proposal_logs);
        if (bayes_event_.active && !validation_usable) {
            // An uninformative scan is missing evidence at BOTH hierarchy levels.
            // The independent frontend can still track using its live map.
            proposal_logs=proposal_corrections; // q=p here: all corrections are exactly zero.
        }
        if (validation_usable) {
            parallel_indices(particles_.size(), [&](std::size_t i) {
                const auto& h=std::get<2>(*(particles_.begin()+i));
                for (std::size_t k=0; k<motion_proposals_[i].size(); ++k) {
                    const auto& pose=motion_proposals_[i][k];
                    proposal_logs[i][k]=proposal_corrections[i][k]+h->validation_map->log_likelihood(common_scan,
                        {pose.translation().x(),pose.translation().y(),pose.so2().log()},params_.tracking,params_.loop_bayes.beta);
                }
            });
        }
        const auto prior_masses=hypothesis_masses();
        std::map<std::size_t,std::vector<std::size_t>> indices;
        std::vector<double> increments(particles_.size());
        // Fixed serial RNG order. Keep MEAN(likelihood * p/q),
        // then the conditional-particle normalizer, then normalize graph masses.
        for (std::size_t i=0; i<particles_.size(); ++i) {
            auto&& particle=*(particles_.begin()+i);
            const auto choice=belugaslam::select_motion_proposal(proposal_logs[i],rng_);
            std::get<0>(particle)=motion_proposals_[i][choice.index];
            increments[i]=choice.log_evidence;
            indices[std::get<2>(particle)->id].push_back(i);
        }
        for (const auto& h : hypotheses_) {
            const auto& members=indices.at(h->id);
            std::vector<double> prior, likelihood;
            for (auto i : members) { prior.push_back(std::get<3>(*(particles_.begin()+i))); likelihood.push_back(increments[i]); }
            const auto update=belugaslam::update_conditional(prior,likelihood);
            for (std::size_t j=0; j<members.size(); ++j)
                set_conditional_log_weight(*(particles_.begin()+members[j]), update.log_weights[j]);
            h->predictive_log_evidence=0;
            // Missing common coverage gives no graph evidence. Outside a loop
            // window the pre-insertion tracking evidence updates ordinary modes.
            // An undecided window keeps its graph masses until another event.
            if (validation_usable || (!bayes_event_.active && !bayes_unresolved_)) {
                h->log_mass += update.log_evidence;
                h->predictive_log_evidence=update.log_evidence;
            }
            // During validation the PF used a different sensor model from the
            // frontend readout. Report its actual conditional particle moment.
            if (bayes_event_.active) cache_particle_pose_covariance(h);
        }
        normalize_hypothesis_masses();
        motion_proposals_.clear(); motion_ancestors_.clear();
        if (future_scan) finish_bayesian_scan(validation_usable,common_scan.size(),prior_masses);
        if (tracking_diagnostics_.is_open()) {
            const auto masses = hypothesis_masses();
            for (const auto& h : hypotheses_) {
                const auto count = std::count_if(particles_.begin(), particles_.end(), [&](const auto& p) { return std::get<2>(p) == h; });
                tracking_diagnostics_ << next_scan_sequence_ << ',' << h->id << ',' << h->tracking_usable << ','
                    << h->tracking_overlap << ',' << h->tracking_log_likelihood << ',' << h->local_pose.translation().x() << ','
                    << h->local_pose.translation().y() << ',' << h->local_pose.so2().log() << ',' << masses.at(h->id) << ',' << count << ','
                    << h->tracking_status << ',' << h->tracking_failures << ',' << h->tracking_reference << ',' << h->tracking_correction << ','
                    << (hypothesis_mean_pose(h).translation()-h->local_pose.translation()).norm() << ','
                    << h->submaps.trajectory_nodes.size() << ',' << h->submaps.history.size()+h->submaps.active_submaps.size() << ','
                    << h->pose_source << ',' << h->proposal_pose_decision << ',' << h->proposal_ess << ',' << h->proposal_local_mass << ','
                    << h->proposal_position_std << ',' << h->proposal_yaw_std << ',' << h->proposal_mean_offset << ','
                    << params_.tracking_matcher << ',' << frontend_scan.size() << ',' << params_.tracking_prior_mode << ','
                    << h->tracking_prior_evaluated << ',' << h->tracking_prior_covariance[0] << ','
                    << h->tracking_prior_covariance[1] << ',' << h->tracking_prior_covariance[2] << ','
                    << h->tracking_prior_covariance[4] << ',' << h->tracking_prior_covariance[5] << ','
                    << h->tracking_prior_covariance[8] << ',' << h->icp_decision << ','
                    << h->icp_inlier_ratio << ',' << h->icp_rmse << ',' << h->icp_condition_number << ','
                    << h->icp_correction << ',' << h->scan_proposal_status << ','
                    << h->scan_proposal_count << ',' << h->scan_proposal_frontend_count << ',' << h->scan_proposal_fraction << ','
                    << h->scan_proposal_log_ratio_min << ',' << h->scan_proposal_log_ratio_max << '\n';
            }
            if (next_scan_sequence_ % 100 == 0) tracking_diagnostics_.flush();
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
        const measurement_type& z, std::uint64_t sequence) {
        auto data = std::make_shared<ScanNodeData>();
        data->sequence = sequence;
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

    [[nodiscard]] bool should_insert_scan(
        const SubmapList& submaps, const state_type& pose, double time_seconds) const {
        if (!submaps.has_last_keyframe_pose) return true;
        const auto delta = submaps.last_keyframe_pose.inverse() * pose;
        return motion_filter_accepts(
            time_seconds - submaps.last_keyframe_time,
            delta.translation().norm(), std::abs(delta.so2().log()),
            params_.keyframe_max_time, params_.keyframe_min_translation,
            params_.keyframe_min_rotation);
    }

    [[nodiscard]] SearchState score_scan_in_submap(
        const ScanNodeData& data, const Submap& submap,
        const Sophus::SE2d& T_submap_node, Submap::LoopDataPtr cached = {}) const {
        SearchState result{T_submap_node, 0.0, 0.0};
        if (data.returns.empty()) return result;
        if (!cached) cached = submap.loop_matching_data();
        std::size_t inside = 0;
        double likelihood_sum = 0.0;
        for (const auto& point : data.returns) {
            const Eigen::Vector2d hit =
                T_submap_node * Eigen::Vector2d{point.first, point.second};
            const auto [distance, score] = submap.loop_cell_at(hit.x(), hit.y(), cached);
            if (!std::isfinite(distance)) continue;
            if (distance <= 0.30F) ++inside;
            likelihood_sum += score;
        }
        result.overlap = static_cast<double>(inside) / data.returns.size();
        result.score = likelihood_sum / data.returns.size();
        return result;
    }

    /**
     * Bounded correlative scan matching: a coarse lattice followed by beam refinements.
     * This replaces the previous exhaustive four-level submap-to-submap search.
     */
    [[nodiscard]] ScanMatchResult match_scan_to_submap(
        const ScanNodeData& data, const Submap& submap,
        const Sophus::SE2d& initial_pose) const {
        const auto cached = submap.loop_matching_data();
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
                    auto state = score_scan_in_submap(data, submap, candidate, cached);
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
                            auto state = score_scan_in_submap(data, submap, candidate, cached);
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
        auto pose = best.pose;
        double score = best.score, overlap = best.overlap;

        // Continuous polish of the lattice optimum. The tracking field is the same
        // chamfer transform as the loop field, but carries analytic derivatives, so
        // the optimizer can leave the search lattice. The matcher's own accept
        // thresholds are cleared here because the LOOP field's score and overlap
        // decide below, on their own terms: the two fields differ in occupancy
        // threshold and clamping, so a lower cost there is not by itself a better
        // constraint. Expect small gains; see loop_refine for what actually binds.
        if (params_.loop_refine) {
            const auto field = submap.tracking_field();
            if (field && field->occupied_cells() > 0) {
                auto options = params_.tracking;
                options.max_translation = params_.loop_refine_translation;
                options.max_rotation = params_.loop_refine_rotation;
                options.prior_translation_sigma = params_.loop_refine_translation;
                options.prior_rotation_sigma = params_.loop_refine_rotation;
                options.min_overlap = 0.0;
                options.min_points = 0;
                const belugaslam::PoseSample2 prior{best.pose.translation().x(),
                    best.pose.translation().y(), best.pose.so2().log()};
                const auto polished = belugaslam::match_tracking_scan(*field, data.returns, prior, options);
                const auto candidate = Sophus::SE2d{Sophus::SO2d{polished.pose.yaw},
                    Eigen::Vector2d{polished.pose.x, polished.pose.y}};
                const auto moved = best.pose.inverse() * candidate;
                if (moved.translation().norm() > 1.0e-9 || std::abs(moved.so2().log()) > 1.0e-9) {
                    // Never accept a refinement the loop metric does not endorse.
                    // The two fields differ in occupancy threshold and clamping, so
                    // a lower cost there is not automatically a better constraint.
                    const auto rescored = score_scan_in_submap(data, submap, candidate, cached);
                    if (rescored.score >= score && rescored.overlap >= overlap) {
                        pose = candidate; score = rescored.score; overlap = rescored.overlap;
                    }
                }
            }
        }

        return {pose, score, overlap,
                data.returns.size() >= params_.loop_min_points &&
                    score >= params_.loop_min_score &&
                    overlap >= params_.loop_min_overlap};
    }

    /// Update the occupancy grid map of each hypothesis based on the transformed measurement.
    std::vector<FinishedSubmapEvent> update_occupancy_grid(
        const measurement_type& z, double time_seconds,
        std::int64_t stamp_ns=std::numeric_limits<std::int64_t>::min(),
        const measurement_type& ray_origins = {}) {
        std::vector<FinishedSubmapEvent> finished_events;
        std::shared_ptr<const ScanNodeData> shared_scan_data;

        if (!ray_origins.empty() && ray_origins.size() != z.size())
            throw std::invalid_argument("Each scan endpoint requires one ray origin");
        for (const auto& origin : ray_origins)
            if (!std::isfinite(origin.first) || !std::isfinite(origin.second))
                throw std::invalid_argument("Nonfinite ray origin");
        if (z.empty() || !std::isfinite(time_seconds)) return finished_events;
        if (stamp_ns==std::numeric_limits<std::int64_t>::min()) {
            const long double ns=std::round(static_cast<long double>(time_seconds)*1.e9L);
            if (ns<std::numeric_limits<std::int64_t>::min() || ns>std::numeric_limits<std::int64_t>::max())
                throw std::invalid_argument("Trajectory timestamp outside int64 range");
            stamp_ns=static_cast<std::int64_t>(ns);
        }
        const std::uint64_t sequence = next_scan_sequence_++;

        for (auto& hypothesis : hypotheses_) {
            auto& submaps = hypothesis->submaps;

            // The continuous local-SLAM pose of this hypothesis. Reading the highest
            // weight particle here instead would let the trajectory hop between offset
            // particles, and that hop would be inserted into the grid as if it were
            // motion. Everything below -- keyframe decision, submap lifecycle, node and
            // constraint poses, ray casting -- uses this one pose.
            if (!hypothesis->has_local_pose) continue;
            const state_type tracking_pose = hypothesis->local_pose;

            // Scan matching has already run. A rejected scan changes neither the
            // grids, lifecycle counts, graph nor the last accepted pose/time.
            if ((hypothesis->tracking_evaluated && !hypothesis->tracking_usable) ||
                !should_insert_scan(submaps, tracking_pose, time_seconds)) {
                record_trajectory_sample(submaps, tracking_pose, sequence, stamp_ns);
                continue;
            }
            if (!shared_scan_data) {
                shared_scan_data = make_scan_node_data(z, sequence);
            }

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
                    submaps.next_submap_id++, tracking_pose,
                    std::max(2, static_cast<int>(std::ceil(12.0 / params_.map_resolution))),
                    std::max(2, static_cast<int>(std::ceil(12.0 / params_.map_resolution))), params_.map_resolution));
                submaps.active_submaps.back()->set_local_pose(
                    hypothesis->T_global_local.inverse() * tracking_pose);
                submaps.active_submaps.back()->set_anchor_sequence(sequence);
            }

            const auto current_reference=submaps.matching_submap();
            if (!current_reference || current_reference->is_finished())
                submaps.matching_submap_id = submaps.active_submaps.front()->id();
            submaps.has_matching_submap = true;

            TrajectoryNode node;
            node.id = submaps.next_node_id++;
            node.constant_data = shared_scan_data;
            node.global_pose = tracking_pose;
            node.local_pose = hypothesis->T_global_local.inverse() * tracking_pose;
            node.sequence = sequence;

            if (!submaps.trajectory_nodes.empty()) {
                const auto& previous = submaps.trajectory_nodes.back();
                submaps.local_trajectory_constraints.push_back({
                    previous.id, node.id,
                    previous.local_pose.inverse() * node.local_pose,
                    params_.pgo_odometry_translation_weight, params_.pgo_odometry_rotation_weight});
            }

            // Every accepted scan creates a node and is inserted into both overlapping
            // submaps. Measure constraints in each native frame before writing cells.
            for (auto& active_submap : submaps.active_submaps) {
                const auto T_s_r = active_submap->global_pose().inverse() * tracking_pose;
                submaps.node_submap_constraints.push_back({
                    active_submap->id(), node.id, T_s_r,
                    params_.pgo_intra_translation_weight, params_.pgo_intra_rotation_weight,
                    ConstraintTag::kIntraSubmap, 1.0, 1.0});

                auto& lo_grid = active_submap->mutable_grid();

                // Grow, hits, misses, one touch per cell, hits win. See the helper.
                ScanInsertionParams insertion_params;
                insertion_params.l_occ = params_.insertion_l_occ;
                insertion_params.l_free = params_.insertion_l_free;
                insertion_params.clamp = 5.0f;
                insertion_params.robot_radius = ROBOT_RADIUS;
                insert_scan_into_submap_grid(
                    lo_grid, T_s_r, z, insertion_params, scan_hit_cells_, scan_miss_cells_, &scan_updates_, ray_origins);
                // The same endpoints, kept continuous. insert_scan_into_submap_grid
                // rounds them to cells and that is all the grid ever sees.
                if (params_.icp_refine) active_submap->insert_surface_points(T_s_r, z, params_.icp.cloud);

                active_submap->add_insertion();
            }

            submaps.trajectory_nodes.push_back(std::move(node));
            submaps.last_keyframe_pose = tracking_pose;
            submaps.has_last_keyframe_pose = true;
            submaps.last_keyframe_time = time_seconds;
            record_trajectory_sample(submaps, tracking_pose, sequence, stamp_ns);

            // A submap is finished after twice the count, which is exactly when the
            // submap that started at its halfway point has itself reached the count.
            for (SubmapId id :
                 submaps.finish_ready_submaps(2 * params_.submap_num_range_data)) {
                finished_events.push_back({hypothesis->id, id});
            }
            if (const auto reference=submaps.matching_submap(); reference && reference->is_finished() && !submaps.active_submaps.empty())
                submaps.matching_submap_id=submaps.active_submaps.front()->id();
        }
        return finished_events;
    }

    void record_trajectory_sample(SubmapList& graph, const state_type& pose, std::uint64_t sequence, std::int64_t stamp_ns=0) {
        const auto reference = graph.matching_submap();
        if (!reference) return;
        graph.trajectory_samples.push_back({sequence, reference->id(), reference->global_pose().inverse() * pose, pose, stamp_ns});
    }

    struct BackendTiming {
        double baseline_pgo_ms = 0.0, retrieval_ms = 0.0, verification_ms = 0.0;
        std::size_t baseline_solves = 0, candidates = 0, trials = 0;
        std::size_t local_only_skips = 0, loop_cache_bytes = 0;
        std::size_t polish_solves = 0;
        double polish_work_ms = 0.0;  // sum of per-trial elapsed times; workers may overlap
    };
    [[nodiscard]] const BackendTiming& backend_timing() const { return backend_timing_; }

    /// Step 5: Loop closure, PGO and best pose selection. Publication is on demand.
    void post_update(const measurement_type& z, const std::vector<FinishedSubmapEvent>& finished_events) {
        // --- Loop Closure Detection (driven by FinishedSubmapEvents) ---
        (void)z;
        backend_timing_ = {};
        // An undecided event holds the filter's masses frozen. Give it a deadline.
        if (bayes_unresolved_ && !bayes_event_.active &&
            next_scan_sequence_ - unresolved_since_sequence_ >= params_.loop_undecided_max_scans)
            resolve_undecided_event("undecided_expired");
#if BELUGASLAM_ENABLE_LOOP_CLOSURE
        // Retrieval fixes geometric measurements in native submap coordinates.
        // A baseline solve is needed before verification, not before a search that
        // may produce no candidate. All hypothesis priors are still frozen AFTER it.
        const auto retrieval_start = std::chrono::steady_clock::now();
        const auto candidates = !bayes_event_.active && params_.enable_loop_closure && params_.enable_pgo
            ? retrieve_loop_candidates(finished_events) : std::vector<LoopCandidate>{};
        backend_timing_.retrieval_ms = elapsed_ms(retrieval_start);
        backend_timing_.candidates = candidates.size();
        const auto pgo_start = std::chrono::steady_clock::now();
        if (params_.enable_pgo && !bayes_event_.active) {
            for (auto& hypothesis : hypotheses_) {
                const auto node_count = hypothesis->submaps.trajectory_nodes.size();
                const auto inter_count = hypothesis->submaps.inter_constraint_count();
                if (inter_count == 0) {
                    // The live frontend constructs all local edges from the same
                    // immutable local poses. With no loop edges this graph already
                    // has a zero-residual solution: rebuilding Ceres adds no evidence.
                    hypothesis->optimized_node_count = node_count;
                    hypothesis->last_pgo_attempt_node_count = node_count;
                    hypothesis->optimized_inter_constraints_count = 0;
                    hypothesis->pgo_usable = true;
                    ++backend_timing_.local_only_skips;
                    continue;
                }
                const bool unchanged = hypothesis->pgo_usable &&
                    node_count == hypothesis->optimized_node_count &&
                    inter_count == hypothesis->optimized_inter_constraints_count;
                if (!unchanged && (!candidates.empty() ||
                    node_count >= hypothesis->last_pgo_attempt_node_count + params_.pgo_every_n_nodes)) {
                    ++backend_timing_.baseline_solves;
                    hypothesis->pgo_usable = optimize_pose_graph(hypothesis);
                }
            }
        }
        backend_timing_.baseline_pgo_ms = elapsed_ms(pgo_start);
        if (!candidates.empty()) verify_loop_candidates(candidates);
#endif
        if (!finished_events.empty()) {
            for (auto& hypothesis : hypotheses_) {
                hypothesis->submaps.trim_scan_data_outside_active_submaps();
            }
        }

        refresh_output_selection();
        trim_derived_caches();
    }

    // Population branching can change a mode's mass without moving any particle.
    // Keep the selected pose/map tied to the population that will be published.
    void refresh_output_selection() {
        struct OutputCandidate {
            std::shared_ptr<Hypothesis> hypothesis;
            double mass = 0.0;
            state_type fallback_pose{};
        };
        // Stable IDs preserve MAP tie-breaking. Integer particle quotas do not vote.
        std::map<std::size_t, OutputCandidate> candidates;
        const auto masses = hypothesis_masses();
        for (const auto& h : hypotheses_) candidates[h->id] = {h, masses.at(h->id), state_type{}};
        for (auto& [id,candidate]:candidates) {
            if (!candidate.hypothesis->has_local_pose)
                candidate.fallback_pose=hypothesis_mean_pose(candidate.hypothesis);
        }
        std::vector<belugaslam::OutputPoseHypothesis> belief;
        belief.reserve(candidates.size());
        for (const auto& [id, candidate] : candidates) {
            const auto& h = candidate.hypothesis;
            const auto& pose = h->has_local_pose ? h->local_pose : candidate.fallback_pose;
            belief.push_back({id, candidate.mass, pose.translation().x(), pose.translation().y()});
        }
        const auto decision = belugaslam::select_output_pose(belief);
        const bool use_risk = params_.output_selection_mode == "pose_risk";
        const auto selected_index = use_risk ? decision.risk_index : decision.map_index;
        const auto& selected = candidates.at(belief[selected_index].id);
        map_hypothesis_id_ = belief[decision.map_index].id;
        map_position_risk_m2_ = decision.map_position_risk;
        selected_position_risk_m2_ = use_risk ? decision.minimum_position_risk : decision.map_position_risk;
        // Always publish one complete hypothesis's frontend pose and map together.
        // No particle, graph, weight, or random-generator state changes here.
        best_hypothesis_ = selected.hypothesis;
        best_pose_ = best_hypothesis_->has_local_pose ? best_hypothesis_->local_pose : selected.fallback_pose;
        publication_dirty_ = true;
    }

    void trim_derived_caches() {
        struct Cached { std::shared_ptr<Submap> submap; std::size_t bytes; std::uint64_t use; };
        std::set<const void*> seen;
        std::vector<Cached> caches;
        std::size_t bytes = 0;
        for (const auto& h : hypotheses_) for (const auto& sm : h->submaps.history) {
            // Retired matching fields are rebuildable and otherwise accumulate
            // one full float array per frozen submap over a long trajectory.
            if (!h->submaps.has_matching_submap || sm->id() != h->submaps.matching_submap_id)
                sm->release_tracking_field();
            if (!seen.insert(sm->loop_cache_identity()).second) continue;
            const auto [size,last_use] = sm->loop_cache_statistics();
            if (size) { bytes += size; caches.push_back({sm,size,last_use}); }
        }
        const auto budget = params_.loop_cache_budget_mb * std::size_t{1024*1024};
        std::sort(caches.begin(),caches.end(),[](const auto& a,const auto& b) {return a.use < b.use;});
        for (const auto& cache : caches) {
            if (bytes <= budget) break;
            cache.submap->release_loop_cache(); bytes -= cache.bytes;
        }
        backend_timing_.loop_cache_bytes = bytes;
    }

    struct PopulationBranch {
        std::shared_ptr<Hypothesis> hypothesis;
        std::shared_ptr<Hypothesis> source;
        double mass = 0.0;
        Sophus::SE2d correction;
        bool no_loop = true;
        std::size_t candidate_index = std::numeric_limits<std::size_t>::max();
        double log_mass = std::numeric_limits<double>::quiet_NaN(); // optional exact mass for resampling
    };

    [[nodiscard]] std::map<std::size_t, double> hypothesis_masses() const {
        std::vector<double> logs;
        for (const auto& h : hypotheses_) logs.push_back(h->log_mass);
        const double normalizer = belugaslam::log_sum_exp(logs);
        if (!std::isfinite(normalizer)) throw std::logic_error("No hypothesis mass");
        std::map<std::size_t,double> masses;
        for (const auto& h : hypotheses_) masses[h->id] = std::exp(h->log_mass-normalizer);
        return masses;
    }

    void normalize_hypothesis_masses() {
        std::vector<double> logs;
        for (const auto& h : hypotheses_) logs.push_back(h->log_mass);
        belugaslam::normalize_log_weights(logs);
        for (std::size_t i=0; i<hypotheses_.size(); ++i) hypotheses_[i]->log_mass=logs[i];
    }

    // Conditional resampling preserves the continuous mass W_h even when each
    // hypothesis receives a minimum integer quota. It never favors low-weight
    // source particles when creating a loop branch.
    void install_population(const std::vector<PopulationBranch>& branches, std::size_t budget) {
        if (branches.empty()) return;
        if (budget<branches.size()) throw std::invalid_argument("Particle budget cannot represent every hypothesis");
        std::set<std::size_t> ids;
        std::vector<double> log_masses;
        for (const auto& branch : branches) {
            if (!branch.hypothesis || !branch.source || !ids.insert(branch.hypothesis->id).second ||
                !std::isfinite(branch.mass) || branch.mass<0)
                throw std::invalid_argument("Invalid population branch");
            log_masses.push_back(std::isnan(branch.log_mass) ? std::log(branch.mass) : branch.log_mass);
        }
        belugaslam::normalize_log_weights(log_masses);
        std::vector<double> masses;
        for (double l : log_masses) masses.push_back(std::exp(l));
        const auto quotas = belugaslam::allocate_particle_quotas(masses, budget);
        std::vector<FastSLAMParticle> buffer;
        buffer.reserve(budget);
        for (std::size_t h = 0; h < branches.size(); ++h) {
            const auto& branch = branches[h];
            std::vector<state_type> poses;
            std::vector<double> weights;
            for (const auto& p : particles_) {
                if (std::get<2>(p) == branch.source) {
                    poses.push_back(std::get<0>(p));
                    weights.push_back(static_cast<double>(std::get<1>(p)));
                }
            }
            if (poses.empty()) throw std::logic_error("Branch has no source particles");
            const auto selected = belugaslam::systematic_indices(weights, quotas[h], rng_);
            for (std::size_t n = 0; n < quotas[h]; ++n) {
                buffer.emplace_back(branch.correction * poses[selected[n]],
                    beluga::Weight(1.0 / quotas[h]), branch.hypothesis, -std::log(static_cast<double>(quotas[h])));
            }
        }
        particles_.assign(buffer.begin(), buffer.end());
        hypotheses_.clear();
        for (std::size_t i=0; i<branches.size(); ++i) {
            const auto& branch=branches[i];
            branch.hypothesis->log_mass=log_masses[i];
            hypotheses_.push_back(branch.hypothesis);
            next_hypothesis_id_ = std::max(next_hypothesis_id_, branch.hypothesis->id + 1);
        }
    }

    void cache_particle_pose_covariance(const std::shared_ptr<Hypothesis>& h) {
        h->pose_covariance.setZero(); double mass=0;
        const auto center=h->has_local_pose ? h->local_pose : hypothesis_mean_pose(h);
        for (const auto& p:particles_) {
            if (std::get<2>(p)!=h) continue;
            const double w=static_cast<double>(std::get<1>(p));
            const auto& pose=std::get<0>(p);
            const Eigen::Vector3d d{pose.translation().x()-center.translation().x(),
                pose.translation().y()-center.translation().y(),belugaslam::wrap_angle(pose.so2().log()-center.so2().log())};
            h->pose_covariance+=w*d*d.transpose();mass+=w;
        }
        h->has_pose_covariance=mass>0;
        if (h->has_pose_covariance) h->pose_covariance/=mass;
    }

    void resample() {
        std::vector<double> weights;
        for (const auto& p : particles_) weights.push_back(static_cast<double>(std::get<1>(p)));
        if (!bayes_event_.active && !bayes_unresolved_) detect_and_split_modes(weights);
        // Split populations need conditional moments of their new memberships.
        // Cache before resampling, which otherwise adds avoidable reporting noise.
        for (const auto& h:hypotheses_) if (!h->has_pose_covariance) cache_particle_pose_covariance(h);
        const auto masses = hypothesis_masses();
        std::vector<PopulationBranch> branches;
        for (const auto& h : hypotheses_) {
            const auto it = masses.find(h->id);
            if (it != masses.end() && (bayes_event_.active || bayes_unresolved_ || it->second > params_.hypothesis_prune_mass)) {
                branches.push_back({h, h, it->second, state_type{}, true});
                branches.back().log_mass=h->log_mass;
            }
        }
        std::stable_sort(branches.begin(), branches.end(), [](const auto& a, const auto& b) { return a.mass > b.mass; });
        if (branches.size() > params_.max_hypotheses) branches.resize(params_.max_hypotheses);
        if (branches.empty()) {
            const auto h=*std::max_element(hypotheses_.begin(),hypotheses_.end(),[](const auto& a,const auto& b) {return a->log_mass<b->log_mass;});
            branches.push_back({h,h,1.0,state_type{},true});
        }
        double sum = 0.0, squares = 0.0;
        for (const auto& p : particles_) {
            const double w = joint_particle_weight(p);
            sum += w; squares += w * w;
        }
        const double ess = squares > 0.0 ? sum * sum / squares : 0.0;
        bool conditional_depletion = false;
        for (const auto& branch : branches) {
            double mass = 0, squares_in_mode = 0; std::size_t count = 0;
            for (const auto& p : particles_) if (std::get<2>(p) == branch.hypothesis) {
                const double w = static_cast<double>(std::get<1>(p));
                mass += w; squares_in_mode += w*w; ++count;
            }
            if (squares_in_mode > 0 && mass*mass/squares_in_mode < 0.5*count) conditional_depletion = true;
        }
        if (!conditional_depletion && ess >= particles_.size() / 2.0 && branches.size() == hypotheses_.size()) {
            // detect_and_split_modes can change the MAP mode even when ESS is high.
            refresh_output_selection();
            return;
        }
        const auto budget = branches.size() > 1 ? params_.max_particles :
            std::min(params_.max_particles, std::max(params_.min_particles, particles_.size()));
        install_population(branches, std::max(budget, branches.size()));
        refresh_output_selection();
    }

    void detect_and_split_modes(std::vector<double>& weights_view) {
        auto hypotheses_snapshot = hypotheses_;
        
        std::map<size_t, std::vector<size_t>> hypothesis_particle_indices;
        for (size_t i = 0; i < particles_.size(); ++i) {
            hypothesis_particle_indices[std::get<2>(*(particles_.begin() + i))->id].push_back(i);
        }

        for (auto& hypothesis : hypotheses_snapshot) {
            const double original_log_mass = hypothesis->log_mass;
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

            std::vector<Hypothesis::PendingSplit> next_pending;
            bool is_first_spatial_cluster = true;
            for (const auto& [scid, scweight] : sorted_s_clusters) {
                // Leave negligible support in the parent. Particle count must
                // never silently remove or create probability at a spatial fork.
                if (!(s_total_weight > 0.0) || scweight <= 1.0e-12 * s_total_weight) continue;

                std::shared_ptr<Hypothesis> target_hypothesis = hypothesis;

                if (!is_first_spatial_cluster) {
                    // A sparse noisy tail must persist before consuming a graph slot.
                    // Unconfirmed particles stay in the parent with unchanged weights.
                    if (scweight / s_total_weight < params_.split_min_mass ||
                        s_cluster_to_indices[scid].size() < params_.split_min_particles) continue;
                    std::vector<state_type> mode_poses;
                    std::vector<double> mode_weights;
                    for (auto idx : s_cluster_to_indices[scid]) {
                        mode_poses.push_back(c_states[idx]); mode_weights.push_back(c_weights[idx]);
                    }
                    const auto center = weighted_mean_pose(mode_poses, mode_weights);
                    std::size_t persistence = 1;
                    for (const auto& pending : hypothesis->pending_splits) {
                        const auto delta = (pending.pose * last_odom_delta_).inverse() * center;
                        if (delta.translation().norm() < 0.4 && std::abs(delta.so2().log()) < 0.25) {
                            if (pending.sequence + 1 == next_scan_sequence_) persistence = std::max(persistence, pending.count + 1);
                            else if (pending.sequence == next_scan_sequence_) persistence = std::max(persistence, pending.count);
                        }
                    }
                    if (persistence < params_.split_persistence) {
                        next_pending.push_back({center, persistence, next_scan_sequence_}); continue;
                    }
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
                    // Initialize this scan's child pose from its weighted cluster
                    // BEFORE resampling. Publication must not use the first/highest
                    // particle for one frame while waiting for the next scan.
                    target_hypothesis->local_pose = center;
                    target_hypothesis->has_local_pose = true;
                    target_hypothesis->pose_source = "spatial_mean";
                    target_hypothesis->tracking_status = "spatial_seed";
                    target_hypothesis->proposal_pose_decision = "awaiting_next_scan";
                    target_hypothesis->has_pose_covariance = false;
                    hypothesis->has_pose_covariance = false;
                    target_hypothesis->tracking_evaluated = false;
                    target_hypothesis->has_pending_recovery = false;
                    target_hypothesis->tracking_failures = 0;
                    target_hypothesis->pending_splits.clear();
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
                    
                    if (params_.verbose_backend) std::cout << "\n\033[1;31m[SPATIAL DIVERGENCE] Hipotesis " << hypothesis->id
                              << " se bifurco en la hipotesis " << target_hypothesis->id << "\033[0m" << std::endl;
                }
                is_first_spatial_cluster = false;

                // Re-assign the particles to the correct hypothesis
                for (size_t local_idx : s_cluster_to_indices[scid]) {
                    size_t global_idx = indices[local_idx];
                    std::get<2>(*(particles_.begin() + global_idx)) = target_hypothesis;
                }
            }
            hypothesis->pending_splits = std::move(next_pending);
            // Partition this parent's probability, then renormalize each child locally.
            std::map<std::size_t,std::vector<std::size_t>> memberships;
            for (auto i : indices) memberships[std::get<2>(*(particles_.begin()+i))->id].push_back(i);
            for (const auto& [id, members] : memberships) {
                std::vector<double> logs;
                for (auto i : members) logs.push_back(std::get<3>(*(particles_.begin()+i)));
                const double fraction = belugaslam::normalize_log_weights(logs);
                std::get<2>(*(particles_.begin()+members.front()))->log_mass = original_log_mass + fraction;
                for (std::size_t j=0; j<members.size(); ++j) {
                    auto&& particle=*(particles_.begin()+members[j]);
                    set_conditional_log_weight(particle, logs[j]);
                    weights_view[members[j]]=std::exp(logs[j]);
                }
            }
        }
        normalize_hypothesis_masses();
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
                                    DynamicOccupancyGrid& out_oc) const {
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

    // Lazily rasterize once per requested publication, with no full-grid return
    // copies. References remain valid until the next publication refresh.
    void refresh_publication() const {
        if (!publication_dirty_ || !best_hypothesis_) return;
        compose_publication_view(best_hypothesis_, best_lo_grid_);
        sync_log_odds_to_occupancy(best_lo_grid_, best_oc_grid_);
        publication_dirty_ = false;
        ++publication_rebuilds_;
    }
    [[nodiscard]] const DynamicOccupancyGrid& best_occupancy_grid() const {
        refresh_publication(); return best_oc_grid_;
    }
    [[nodiscard]] state_type best_pose() const { return best_pose_; }
    // Drain the backend at the end of a finite recording before either export.
    // Periodic PGO otherwise leaves the last < pgo_every_n_nodes unoptimized.
    bool finalize_trajectory() {
        final_pgo_attempted_ = params_.enable_pgo;
        final_pgo_succeeded_ = true;
        if (params_.enable_pgo) {
            for (const auto& h : hypotheses_) {
                if (h->submaps.inter_constraint_count() == 0) continue;
                if (!optimize_pose_graph(h, true, std::numeric_limits<std::size_t>::max(), true))
                    final_pgo_succeeded_ = false;
            }
        }
        refresh_output_selection();
        return final_pgo_succeeded_;
    }
    // A retrospective trajectory from ONE currently selected graph, distinct from
    // the sequence of online /best_pose messages, which may have switched graphs.
    std::size_t write_optimized_trajectory(std::ostream& out) const {
        if (!best_hypothesis_) return 0;
        const auto& graph=best_hypothesis_->submaps;
        out<<"# retrospective_graph selected_hypothesis="<<best_hypothesis_->id<<'\n';
        out<<"# nodes="<<graph.trajectory_nodes.size()<<" nodes_at_last_pgo="<<best_hypothesis_->optimized_node_count
           <<" extra_shutdown_optimization="<<(final_pgo_attempted_ ? "true" : "false")
           <<" final_pgo_success="<<(final_pgo_succeeded_ ? "true" : "false")
           <<" interpolation=se2_node_corrections\n";
        out<<"# final_hypotheses=";
        for (std::size_t i=0;i<hypotheses_.size();++i) out<<(i ? "," : "")<<hypotheses_[i]->id;
        out<<'\n';
        out<<std::setprecision(17);
        std::size_t count=0;
        std::int64_t previous=0;
        for (const auto& sample:graph.trajectory_samples) {
            state_type pose;
            if (!graph.pose_at_sequence(sample.sequence,pose)) throw std::runtime_error("Missing retrospective graph pose");
            if (count && sample.stamp_ns<=previous) throw std::runtime_error("Non-increasing retrospective timestamps");
            previous=sample.stamp_ns;
            const auto magnitude=sample.stamp_ns<0 ? std::uint64_t(-(sample.stamp_ns+1))+1 : std::uint64_t(sample.stamp_ns);
            if (sample.stamp_ns<0) out<<'-';
            out<<magnitude/1000000000ULL<<'.'<<std::setw(9)<<std::setfill('0')<<magnitude%1000000000ULL<<std::setfill(' ')
               <<' '<<pose.translation().x()<<' '<<pose.translation().y()<<" 0 0 0 "
               <<std::sin(.5*pose.so2().log())<<' '<<std::cos(.5*pose.so2().log())<<'\n';
            ++count;
        }
        if (!out) throw std::runtime_error("Failed to write retrospective trajectory");
        return count;
    }
    [[nodiscard]] Eigen::Matrix3d best_pose_covariance() const {
        const auto& h=best_hypothesis_ ? best_hypothesis_ : hypotheses_.front();
        if (h->has_pose_covariance) return h->pose_covariance;
        return 1e3*Eigen::Matrix3d::Identity();
    }
    [[nodiscard]] const std::string& best_tracking_status() const {
        return (best_hypothesis_ ? best_hypothesis_ : hypotheses_.front())->tracking_status;
    }
    [[nodiscard]] std::size_t best_tracking_failures() const {
        return (best_hypothesis_ ? best_hypothesis_ : hypotheses_.front())->tracking_failures;
    }
    [[nodiscard]] std::size_t best_hypothesis_id() const {
        return best_hypothesis_ ? best_hypothesis_->id : hypotheses_.front()->id;
    }

    /// One processed scan of the selected hypothesis, before and after optimisation.
    struct FinalTrajectoryPoint {
        std::uint64_t sequence = 0;
        SubmapId submap_id = 0;
        state_type online;     ///< what the frontend published at that scan
        state_type optimized;  ///< the same scan carried through every later PGO
    };

    /** Trajectory of the selected hypothesis over the whole run.
     *
     * The online poses are a replay of what was published; the optimized ones are read
     * back through the pose graph, so every loop closure that has landed since is
     * already in them. Keyframes take their own optimized node pose, and the scans
     * between keyframes ride on the submap they were recorded against. Meant to be
     * called once at the end of a run, not per scan.
     */
    [[nodiscard]] std::vector<FinalTrajectoryPoint> final_trajectory() const {
        std::vector<FinalTrajectoryPoint> trajectory;
        const auto hypothesis = best_hypothesis_ ? best_hypothesis_
            : (hypotheses_.empty() ? nullptr : hypotheses_.front());
        if (!hypothesis) return trajectory;
        const auto& graph = hypothesis->submaps;
        trajectory.reserve(graph.trajectory_samples.size());
        for (const auto& sample : graph.trajectory_samples) {
            state_type optimized;
            // Samples recorded against a submap that no longer exists have no
            // optimized pose to report; there is nothing to interpolate them from.
            if (!graph.pose_at_sequence(sample.sequence, optimized)) continue;
            trajectory.push_back({sample.sequence, sample.submap_id, sample.frontend_pose, optimized});
        }
        return trajectory;
    }

    /// Number of scans the core has processed; the next one takes this sequence.
    [[nodiscard]] std::uint64_t scan_sequence_count() const { return next_scan_sequence_; }
    [[nodiscard]] const std::string& output_selection_mode() const { return params_.output_selection_mode; }
    [[nodiscard]] std::size_t map_hypothesis_id() const { return map_hypothesis_id_; }
    // Expected squared position losses under frontend point poses, not ground-truth errors.
    [[nodiscard]] double map_position_risk_m2() const { return map_position_risk_m2_; }
    [[nodiscard]] double selected_position_risk_m2() const { return selected_position_risk_m2_; }
    [[nodiscard]] const GridTypeLO& best_log_odds_grid() const {
        refresh_publication(); return best_lo_grid_;
    }
    [[nodiscard]] std::size_t publication_rebuilds() const { return publication_rebuilds_; }

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

        const auto& history = hypothesis->submaps.history;
        bool reuse = frozen_publication_ &&
            frozen_publication_->width() == global_lo.width() &&
            frozen_publication_->height() == global_lo.height() &&
            frozen_publication_->origin_x() == global_lo.origin_x() &&
            frozen_publication_->origin_y() == global_lo.origin_y() &&
            frozen_publication_->resolution() == global_lo.resolution() &&
            frozen_publication_keys_.size() <= history.size();
        if (reuse) {
            for (std::size_t i = 0; i < frozen_publication_keys_.size(); ++i) {
                const auto& key = frozen_publication_keys_[i];
                const auto& sm = history[i];
                if (key.submap.lock() != sm || key.pose != pose_key(sm->global_pose())) {
                    reuse = false; break;
                }
            }
        }
        if (!reuse) {
            frozen_publication_ = std::make_unique<GridTypeLO>(global_lo);
            frozen_publication_keys_.clear();
        }
        for (std::size_t i = frozen_publication_keys_.size(); i < history.size(); ++i) {
            draw_submap_into_grid(history[i], *frozen_publication_);
            frozen_publication_keys_.push_back({history[i], pose_key(history[i]->global_pose())});
        }
        global_lo.data() = frozen_publication_->data();
        for (const auto& active_submap : hypothesis->submaps.active_submaps) {
            draw_submap_into_grid(active_submap, global_lo);
        }

        for (auto& val : global_lo.data()) {
            val = std::clamp(val, -5.0f, 5.0f);
        }
    }

    struct LoopCandidate {
        std::uint64_t reference_sequence = 0, query_sequence = 0;
        std::size_t source_hypothesis = 0;
        double score = 0.0, overlap = 0.0;
        Sophus::SE2d T_reference_query;
        std::uint64_t candidate_id = 0;
    };

    struct LoopTrial {
        std::shared_ptr<Hypothesis> hypothesis;
        bool usable = false;
        double compatibility = 0.0;
        double fit_translation = std::numeric_limits<double>::infinity();
        double fit_rotation = std::numeric_limits<double>::infinity();
        belugaslam::TrajectoryChange change;
        double forced_compatibility = 0.0, settled_compatibility = 0.0;
        double forced_fit_translation = std::numeric_limits<double>::infinity();
        double forced_fit_rotation = std::numeric_limits<double>::infinity();
        bool polish_attempted = false;
        bool installed = false;
        double polish_ms = 0.0;
        std::string status = "unavailable";
    };

    struct LoopVerification {
        LoopCandidate candidate;
        std::vector<LoopTrial> trials;
        belugaslam::BeliefEvidence evidence;
        double decision_score = 0.0;
        bool eligible = false, selected = false;
    };

    [[nodiscard]] bool bayesian_loop_pending() const { return bayes_event_.active; }
    [[nodiscard]] const std::string& last_bayesian_loop_status() const { return last_bayes_status_; }
    [[nodiscard]] double last_bayesian_loop_probability() const { return last_bayes_probability_; }

    // Transactional: build all snapshots and verify the full population fits
    // before mutating any live parent. No parent mode is dropped to make room.
    bool begin_bayesian_loop_event(LoopVerification& report,
        const std::vector<std::shared_ptr<Hypothesis>>& parents,
        const std::vector<double>& masses, std::uint64_t event_id) {
        if (parents.size()!=masses.size() || parents.size()!=report.trials.size())
            throw std::invalid_argument("Mismatched loop event population");
        if (bayes_event_.active) return false;
        std::vector<bool> eligible;
        std::size_t children=0;
        for (const auto& trial : report.trials) {
            eligible.push_back(trial.usable && trial.hypothesis &&
                trial.compatibility >= params_.loop_geometry_min_compatibility);
            children += eligible.back();
        }
        auto deferred = [&](const char* reason) {
            last_bayes_status_=reason;
            for (auto& trial : report.trials) if (trial.usable) trial.status=reason;
            return false;
        };
        if (!children) return deferred("bayes_deferred_geometry");
        if (parents.size()+children > params_.max_hypotheses || parents.size()+children > params_.max_particles)
            return deferred("bayes_deferred_budget");
        auto snapshot = [&](const std::shared_ptr<Hypothesis>& h) -> std::shared_ptr<const belugaslam::ValidationMap> {
            const auto& graph=h->submaps;
            auto reference=graph.find_submap_by_anchor(report.candidate.reference_sequence);
            if (!reference) {
                const auto* sample=graph.find_sample(report.candidate.reference_sequence);
                if (sample) reference=graph.find_submap(sample->submap_id);
            }
            if (!reference || !reference->is_finished()) return {};
            // Even in synthetic/direct callers, forbid a reference that contains
            // the candidate's query scan (or later data).
            for (const auto node_id : graph.insertion_nodes(reference->id())) {
                const auto* node=graph.find_node(node_id);
                if (node && node->sequence>=report.candidate.query_sequence) return {};
            }
            const auto& grid=reference->grid(); const auto& pose=reference->global_pose();
            auto result=std::make_shared<const belugaslam::ValidationMap>(grid.data(),grid.width(),grid.height(),
                grid.resolution(),grid.origin_x(),grid.origin_y(),
                belugaslam::PoseSample2{pose.translation().x(),pose.translation().y(),pose.so2().log()});
            return result->usable() ? result : nullptr;
        };
        std::vector<std::shared_ptr<const belugaslam::ValidationMap>> null_maps,loop_maps;
        for (std::size_t i=0; i<parents.size(); ++i) {
            null_maps.push_back(snapshot(parents[i]));
            loop_maps.push_back(eligible[i] ? snapshot(report.trials[i].hypothesis) : nullptr);
            if (!null_maps.back() || (eligible[i] && !loop_maps.back())) return deferred("bayes_deferred_reference");
        }
        std::vector<PopulationBranch> pool;
        const auto initialize = [&](const auto& h,const auto& map,bool loop) {
            h->validation_map=map;h->validation_loop=loop;h->validation_event=event_id;
            h->validation_age=0;h->validation_status="pending";h->predictive_log_evidence=0;
            h->pending_splits.clear();
        };
        for (std::size_t i=0; i<parents.size(); ++i) {
            const auto& parent=parents[i];
            initialize(parent,null_maps[i],false);
            // A hard-impossible association is absent from this parent's model.
            // Otherwise the categorical prior sums to ONE within every parent.
            const double loop_prior=eligible[i] ? params_.loop_branch_prior : 0;
            pool.push_back({parent,parent,masses[i]*(1-loop_prior),state_type{},true});
            pool.back().log_mass=parent->log_mass+std::log(1-loop_prior);
            if (!eligible[i]) continue;
            auto& trial=report.trials[i];const auto& child=trial.hypothesis;
            child->id=next_hypothesis_id_++;child->has_loop=true;
            child->last_loop_sequence=report.candidate.query_sequence;
            initialize(child,loop_maps[i],true);
            pool.push_back({child,parent,masses[i]*loop_prior,
                child->T_global_local*parent->T_global_local.inverse(),false});
            pool.back().log_mass=parent->log_mass+std::log(loop_prior);
            trial.installed=true;trial.status="bayes_pending";
        }
        install_population(pool,params_.max_particles);
        bayes_event_={};bayes_event_.active=true;bayes_event_.id=event_id;
        // Branch creation runs after insertion. next_scan_sequence_ is the first
        // FUTURE scan, regardless of the older query sequence used by retrieval.
        bayes_event_.first_sequence=next_scan_sequence_;
        bayes_unresolved_=false;last_bayes_status_="pending";
        last_bayes_probability_=0;
        for (const auto& h : hypotheses_) if (h->validation_loop) last_bayes_probability_+=std::exp(h->log_mass);
        if (bayes_diagnostics_) {
            const auto initial_masses=hypothesis_masses();
            for (const auto& h : hypotheses_) bayes_diagnostics_
                << (next_scan_sequence_ ? next_scan_sequence_-1 : 0) << ',' << event_id << ',' << h->id << ','
                << (h->validation_loop ? "loop" : "no_loop") << ',' << initial_masses.at(h->id) << ','
                << initial_masses.at(h->id) << ',' << h->log_mass << ",0,0,0,0," << last_bayes_probability_ << ",pending\n";
            bayes_diagnostics_.flush();
        }
        report.selected=true; // installed as tentative; NOT a posterior acceptance
        consumed_loop_queries_.insert(report.candidate.query_sequence);
        return true;
    }

    /// Keeps one side of a validation event and discards the other. Returns false
    /// without touching the population when the requested side is empty, which
    /// resampling can produce by evicting every low-mass branch on one side.
    bool collapse_validation_branches(bool keep_loop) {
        const bool populated=std::any_of(hypotheses_.begin(),hypotheses_.end(),
            [&](const auto& h){return h->validation_loop==keep_loop;});
        if (!populated) return false;
        std::vector<FastSLAMParticle> retained;
        for (const auto& particle : particles_)
            if (std::get<2>(particle)->validation_loop==keep_loop) retained.push_back(particle);
        if (retained.empty()) return false;
        particles_.assign(retained.begin(),retained.end());
        hypotheses_.erase(std::remove_if(hypotheses_.begin(),hypotheses_.end(),
            [&](const auto& h) {return h->validation_loop!=keep_loop;}),hypotheses_.end());
        normalize_hypothesis_masses();
        std::vector<PopulationBranch> survivors;
        for (const auto& h : hypotheses_) {
            survivors.push_back({h,h,std::exp(h->log_mass),state_type{},true});
            survivors.back().log_mass=h->log_mass;
        }
        install_population(survivors,params_.max_particles);
        if (keep_loop) for (const auto& h : hypotheses_) loop_closure_poses_.push_back(h->local_pose);
        return true;
    }

    /// Commits an undecided event to its MAP branch.
    ///
    /// Carrying the ambiguity forward indefinitely is not neutral. While
    /// bayes_unresolved_ holds, graph masses stop responding to scan evidence,
    /// mode splitting is suspended and mass pruning is bypassed, so nothing can
    /// ever separate the two branches again; and because the population stays at
    /// its cap, every later event is refused for budget and the flag never clears.
    /// Committing is what breaks that, and a tie commits to no_loop: an
    /// unverified loop must not be installed by default.
    void resolve_undecided_event(const char* reason) {
        if (!bayes_unresolved_ || bayes_event_.active) return;
        if (hypotheses_.empty()) { bayes_unresolved_=false; return; }
        double loop_mass=0, null_mass=0;
        const auto masses=hypothesis_masses();
        for (const auto& h : hypotheses_) {
            const auto it=masses.find(h->id);
            if (it==masses.end()) continue;
            (h->validation_loop ? loop_mass : null_mass)+=it->second;
        }
        const bool keep_loop=loop_mass>null_mass;
        if (!collapse_validation_branches(keep_loop) && !collapse_validation_branches(!keep_loop)) {
            // Neither side is representable. Leave the population alone and clear
            // the flag anyway, so the filter resumes instead of freezing forever.
            bayes_unresolved_=false;
            last_bayes_status_=reason;
            return;
        }
        bayes_unresolved_=false;
        last_bayes_probability_=loop_mass;
        last_bayes_status_=reason;
        for (const auto& h : hypotheses_) {
            h->validation_map.reset();
            h->validation_loop=false;
            h->validation_status=reason;
        }
        refresh_output_selection();
    }

    void finish_bayesian_scan(bool usable, std::size_t beams, const std::map<std::size_t,double>& priors) {
        if (!bayes_event_.active || bayes_event_.last_sequence==next_scan_sequence_) return;
        bayes_event_.last_sequence=next_scan_sequence_;
        ++bayes_event_.attempted_scans;
        if (usable) ++bayes_event_.evidence_scans;
        const auto masses=hypothesis_masses();
        double probability=0;
        for (const auto& h : hypotheses_) if (h->validation_loop) probability+=masses.at(h->id);
        probability=std::clamp(probability,0.0,1.0);
        const auto decision=belugaslam::decide_loop(probability,bayes_event_.evidence_scans,
            bayes_event_.attempted_scans,params_.loop_bayes);
        last_bayes_probability_=probability;last_bayes_status_=belugaslam::decision_name(decision);
        for (const auto& h : hypotheses_) {
            h->validation_age=bayes_event_.evidence_scans;
            h->validation_status=last_bayes_status_;
            if (bayes_diagnostics_) bayes_diagnostics_ << next_scan_sequence_ << ',' << bayes_event_.id << ',' << h->id << ','
                << (h->validation_loop ? "loop" : "no_loop") << ',' << priors.at(h->id) << ',' << masses.at(h->id) << ','
                << h->log_mass << ',' << h->predictive_log_evidence << ',' << bayes_event_.evidence_scans << ','
                << bayes_event_.attempted_scans << ',' << beams << ',' << probability << ',' << last_bayes_status_ << '\n';
        }
        if (decision==belugaslam::LoopDecision::pending) return;
        bayes_event_.active=false;
        bayes_unresolved_=decision==belugaslam::LoopDecision::undecided;
        if (bayes_unresolved_) unresolved_since_sequence_=next_scan_sequence_;
        if (!bayes_unresolved_) collapse_validation_branches(decision==belugaslam::LoopDecision::accepted);
        // Snapshots are bounded to one event window. Undecided alternatives and
        // their masses survive; new events can be attempted if the budget allows.
        for (const auto& h : hypotheses_) h->validation_map.reset();
        if (bayes_diagnostics_) bayes_diagnostics_.flush();
        refresh_output_selection();
    }

    [[nodiscard]] std::vector<LoopCandidate> retrieve_loop_candidates(
        const std::vector<FinishedSubmapEvent>& events) const {
        std::vector<LoopCandidate> candidates;
        for (const auto& event : events) {
            const auto h_it = std::find_if(hypotheses_.begin(), hypotheses_.end(),
                [&](const auto& h) { return h->id == event.hypothesis_id; });
            if (h_it == hypotheses_.end()) continue;
            const auto& graph = (*h_it)->submaps;
            const auto query_map = graph.find_submap(event.query_submap_id);
            if (!query_map) continue;
            const auto query_it = std::find_if(graph.history.begin(), graph.history.end(),
                [&](const auto& sm) { return sm->id() == event.query_submap_id; });
            if (query_it == graph.history.end()) continue;
            const auto query_index = static_cast<std::size_t>(std::distance(graph.history.begin(), query_it));
            const auto inserted = graph.insertion_nodes(event.query_submap_id);
            if (inserted.empty()) continue;
            // One fixed query scan per event. Different hypotheses can have different
            // keyframe IDs; raw sequence IDs are the shared association identity.
            const auto* query = graph.find_node(inserted.back());
            if (!query || !query->constant_data || query->constant_data->returns.size() < params_.loop_min_points) continue;
            struct Retrieved { std::shared_ptr<Submap> map; double rank; };
            std::vector<Retrieved> retrieved;
            for (std::size_t i = 0; i < query_index; ++i) {
                if (query_index - i <= params_.loop_recent_submaps) continue;
                const auto& reference = graph.history[i];
                const double distance = (reference->global_pose().translation() - query->global_pose.translation()).norm();
                if (distance > params_.loop_candidate_distance) continue;
                double signature_distance = 0.0;
                const auto& a = reference->radial_signature();
                const auto& b = query_map->radial_signature();
                for (std::size_t j = 0; j < std::min(a.size(), b.size()); ++j) {
                    signature_distance += (a[j] - b[j]) * (a[j] - b[j]);
                }
                retrieved.push_back({reference, distance + 4.0 * signature_distance});
            }
            std::stable_sort(retrieved.begin(), retrieved.end(), [](const auto& a, const auto& b) { return a.rank < b.rank; });
            if (retrieved.size() > params_.loop_max_candidates) retrieved.resize(params_.loop_max_candidates);
            // tracking_field() builds a mutable member lazily, so it must be warmed
            // serially before the read-only parallel phase, exactly as the loop cache
            // is warmed by prepare_loop_matching().
            if (params_.loop_refine)
                for (const auto& entry : retrieved) (void)entry.map->tracking_field();
            std::vector<ScanMatchResult> matches(retrieved.size());
            parallel_indices(retrieved.size(), [&](std::size_t i) {
                const auto& entry = retrieved[i];
                matches[i] = match_scan_to_submap(*query->constant_data, *entry.map,
                    entry.map->global_pose().inverse() * query->global_pose);
                if (!matches[i].valid || params_.loop_validation_scans == 1) return;
                std::size_t checked = 1;
                double score_sum = matches[i].score;
                for (auto node_it = inserted.rbegin(); node_it != inserted.rend() && checked < params_.loop_validation_scans; ++node_it) {
                    if (*node_it == query->id) continue;
                    const auto* other = graph.find_node(*node_it);
                    if (!other || !other->constant_data || other->constant_data->returns.size() < params_.loop_min_points) continue;
                    // Corroborate the same loop transform on neighboring scans;
                    // do not independently realign every scan to manufacture agreement.
                    const auto relative = query->local_pose.inverse() * other->local_pose;
                    const auto support = score_scan_in_submap(*other->constant_data, *entry.map, matches[i].T_submap_node * relative);
                    if (support.score < params_.loop_min_score || support.overlap < params_.loop_min_overlap) {
                        matches[i].valid = false; break;
                    }
                    score_sum += support.score;
                    matches[i].overlap = std::min(matches[i].overlap, support.overlap);
                    ++checked;
                }
                if (checked < std::min(params_.loop_validation_scans, inserted.size())) matches[i].valid = false;
                matches[i].score = score_sum / checked;
            });
            for (std::size_t i = 0; i < retrieved.size(); ++i) {
                const auto& match = matches[i];
                if (!match.valid) continue;
                candidates.push_back({retrieved[i].map->anchor_sequence(), query->sequence, (*h_it)->id,
                    match.score, match.overlap, match.T_submap_node});
            }
            // A retired field is rebuildable and otherwise accumulates one float
            // array per frozen submap, which is precisely what the cache pruning
            // releases. A loop event must not defeat that policy.
            if (params_.loop_refine) {
                for (const auto& entry : retrieved) {
                    if (!graph.has_matching_submap || entry.map->id() != graph.matching_submap_id)
                        entry.map->release_tracking_field();
                }
            }
        }
        std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
            return a.score * a.overlap > b.score * b.overlap;
        });
        std::vector<LoopCandidate> distinct;
        for (const auto& candidate : candidates) {
            bool duplicate = false;
            for (const auto& kept : distinct) {
                const auto delta = kept.T_reference_query.inverse() * candidate.T_reference_query;
                if (kept.reference_sequence == candidate.reference_sequence && kept.query_sequence == candidate.query_sequence &&
                    delta.translation().norm() < 0.25 && std::abs(delta.so2().log()) < 0.10) duplicate = true;
            }
            if (!duplicate) distinct.push_back(candidate);
            if (distinct.size() >= params_.loop_max_verifications) break;
        }
        return distinct;
    }

    // Promote a filtered scan only if an accepted loop needs it as a graph node.
    // In a trial this affects only the temporary graph. The grid is not reinserted.
    [[nodiscard]] ScanNodeId ensure_query_node(SubmapList& graph, std::uint64_t sequence) const {
        if (const auto* existing = graph.find_node_by_sequence(sequence)) return existing->id;
        const auto* sample = graph.find_sample(sequence);
        if (!sample) throw std::logic_error("Loop query sample is missing");
        const auto submap = graph.find_submap(sample->submap_id);
        if (!submap) throw std::logic_error("Loop query submap is missing");
        TrajectoryNode node;
        node.id = graph.next_node_id++;
        node.sequence = sequence;
        node.local_pose = submap->local_pose() * sample->T_submap_robot;
        node.global_pose = submap->global_pose() * sample->T_submap_robot;
        graph.node_submap_constraints.push_back({submap->id(), node.id, sample->T_submap_robot,
            params_.pgo_intra_translation_weight, params_.pgo_intra_rotation_weight,
            ConstraintTag::kIntraSubmap, 1.0, 1.0});
        const auto id = node.id;
        const auto position = std::lower_bound(graph.trajectory_nodes.begin(), graph.trajectory_nodes.end(), sequence,
            [](const auto& n, auto value) { return n.sequence < value; });
        graph.trajectory_nodes.insert(position, std::move(node));
        return id;
    }

    [[nodiscard]] LoopTrial evaluate_loop_candidate(
        const std::shared_ptr<Hypothesis>& prior, const LoopCandidate& candidate) {
        LoopTrial result;
        if (!prior->pgo_usable) return result;
        const auto& before = prior->submaps;
        const auto* reference_sample = before.find_sample(candidate.reference_sequence);
        const auto* query_sample = before.find_sample(candidate.query_sequence);
        if (!reference_sample || !query_sample || candidate.reference_sequence >= candidate.query_sequence) return result;
        const auto anchored_reference = before.find_submap_by_anchor(candidate.reference_sequence);
        const auto reference_map = anchored_reference ? anchored_reference : before.find_submap(reference_sample->submap_id);
        if (!reference_map) return result;
        for (const auto& edge : before.node_submap_constraints) {
            if (edge.tag == ConstraintTag::kInterSubmap && edge.reference_sequence == candidate.reference_sequence &&
                edge.query_sequence == candidate.query_sequence) return result;
        }
        // Fixed sequence interval and sampling positions, independent of each mode's
        // insertion filter. No test candidate is added to the prior trajectory.
        const auto begin = std::lower_bound(before.trajectory_samples.begin(), before.trajectory_samples.end(), candidate.reference_sequence,
            [](const auto& sample, auto seq) { return sample.sequence < seq; });
        const auto end = std::upper_bound(before.trajectory_samples.begin(), before.trajectory_samples.end(), candidate.query_sequence,
            [](auto seq, const auto& sample) { return seq < sample.sequence; });
        const std::size_t available = static_cast<std::size_t>(std::distance(begin, end));
        const std::size_t count = std::min(params_.loop_trajectory_samples, available);
        if (count < 3) return result;
        std::vector<std::uint64_t> sequences;
        std::vector<belugaslam::PoseSample2> original;
        for (std::size_t i = 0; i < count; ++i) {
            const auto sequence = (begin + i * (available - 1) / (count - 1))->sequence;
            state_type pose;
            if (!before.pose_at_sequence(sequence, pose)) return result;
            sequences.push_back(sequence);
            original.push_back({pose.translation().x(), pose.translation().y(), pose.so2().log()});
        }
        auto trial = std::make_shared<Hypothesis>(*prior);
        auto& graph = trial->submaps;
        const auto query_id = ensure_query_node(graph, candidate.query_sequence);
        // Transport the SAME r->q measurement into this hypothesis's reference
        // submap using its recorded r pose. Never copy another mode's numeric IDs.
        // Prefer the actual native frame used by retrieval. The anchor scan's
        // recorded matching submap is often its PREDECESSOR. After PGO those two
        // frozen frames are no longer rigidly tied, so routing the loop through
        // the predecessor installs a different geometric constraint.
        // If this hypothesis has no submap born at r, use its recorded local r.
        const auto measurement = anchored_reference ? candidate.T_reference_query :
            reference_sample->T_submap_robot * candidate.T_reference_query;
        const auto loop_index = graph.node_submap_constraints.size();
        graph.node_submap_constraints.push_back({reference_map->id(), query_id, measurement,
            params_.pgo_loop_translation_weight, params_.pgo_loop_rotation_weight,
            ConstraintTag::kInterSubmap, candidate.score, candidate.overlap,
            candidate.reference_sequence, candidate.query_sequence});
        result.status = "forced_solve_failed";
        if (!optimize_pose_graph(trial, false, loop_index)) return result;
        const auto fits = [&]() {
            const auto reference_after = graph.find_submap(reference_map->id());
            const auto* query_after = graph.find_node(query_id);
            if (!reference_after || !query_after) return false;
            const auto error = measurement.inverse() * reference_after->global_pose().inverse() * query_after->global_pose;
            result.fit_translation = error.translation().norm();
            result.fit_rotation = std::abs(error.so2().log());
            return std::isfinite(result.fit_translation) && std::isfinite(result.fit_rotation) &&
                result.fit_translation <= params_.loop_max_fit_translation && result.fit_rotation <= params_.loop_max_fit_rotation;
        };
        const auto change = [&]() -> belugaslam::TrajectoryChange {
            std::vector<belugaslam::PoseSample2> updated;
            updated.reserve(sequences.size());
            for (auto sequence : sequences) {
                state_type pose;
                if (!graph.pose_at_sequence(sequence, pose)) return {};
                updated.push_back({pose.translation().x(), pose.translation().y(), pose.so2().log()});
            }
            return belugaslam::aligned_trajectory_change(original, updated);
        };
        result.status = "forced_fit_failed";
        const bool forced_fits = fits();
        result.forced_fit_translation = result.fit_translation;
        result.forced_fit_rotation = result.fit_rotation;
        if (!forced_fits) return result;
        result.change = change();
        result.status = "invalid_trajectory";
        if (!result.change.valid) return result;
        result.forced_compatibility = belugaslam::trajectory_compatibility(result.change,
            params_.loop_translation_scale, params_.loop_rotation_scale);
        result.settled_compatibility = result.forced_compatibility;

        // The trial forces the candidate, but ordinary PGO uses Huber(1) for it.
        // If its whitened residual lies outside Huber's quadratic region, settle
        // the trial under the SAME objective used after installation. A candidate
        // that is then ignored must fail the fit gate, not acquire extra evidence.
        const auto& edge = graph.node_submap_constraints[loop_index];
        if (params_.loop_robust_polish && belugaslam::weighted_loop_residual_squared(
                result.fit_translation, result.fit_rotation, edge.translation_weight, edge.rotation_weight) >
            params_.pgo_huber_scale * params_.pgo_huber_scale) {
            result.polish_attempted = true;
            const auto polish_start = std::chrono::steady_clock::now();
            const bool settled = optimize_pose_graph(trial, false, std::numeric_limits<std::size_t>::max(), true);
            result.polish_ms = elapsed_ms(polish_start);
            result.status = "polish_solve_failed";
            if (!settled) return result;
            result.status = "settled_fit_failed";
            if (!fits()) return result;
            result.change = change();
            result.status = "invalid_settled_trajectory";
            if (!result.change.valid) return result;
            result.settled_compatibility = belugaslam::trajectory_compatibility(result.change,
                params_.loop_translation_scale, params_.loop_rotation_scale);
        }
        // Same fixed candidate and original trajectory throughout. This guard
        // cannot raise any mode's compatibility by weakening the candidate loss.
        result.compatibility = std::min(result.forced_compatibility, result.settled_compatibility);
        result.usable = true;
        result.status = result.polish_attempted ? "verified_polished" : "verified";
        result.hypothesis = std::move(trial);
        return result;
    }

    void detect_loop_closure(const std::vector<FinishedSubmapEvent>& events) {
        if (events.empty()) return;
        const auto retrieval_start = std::chrono::steady_clock::now();
        const auto candidates = retrieve_loop_candidates(events);
        backend_timing_.retrieval_ms = elapsed_ms(retrieval_start);
        backend_timing_.candidates = candidates.size();
        if (candidates.empty()) return;
        verify_loop_candidates(candidates);
    }

    void verify_loop_candidates(const std::vector<LoopCandidate>& proposed_candidates) {
        if (bayes_event_.active) return;
        std::vector<LoopCandidate> candidates;
        for (const auto& candidate : proposed_candidates)
            if (!consumed_loop_queries_.count(candidate.query_sequence)) candidates.push_back(candidate);
        if (candidates.empty()) return;
        // Commit the previous ambiguity BEFORE snapshotting the population. An
        // unresolved event keeps the hypothesis budget full, and deferring on that
        // budget is what used to leave bayes_unresolved_ set forever, since the
        // only place it cleared was a successful event start.
        if (bayes_unresolved_) resolve_undecided_event("undecided_superseded");
        const auto event_id = next_loop_event_id_++;
        double retained_branch_mass = 1.0;
        const auto prior_hypotheses = hypotheses_;
        const auto mass_map = hypothesis_masses();
        std::vector<double> masses;
        for (const auto& h : prior_hypotheses) {
            const auto it = mass_map.find(h->id);
            masses.push_back(it == mass_map.end() ? 0.0 : it->second);
        }
        const auto verification_start = std::chrono::steady_clock::now();
        std::vector<LoopVerification> reports(candidates.size());
        for (std::size_t c = 0; c < candidates.size(); ++c) {
            reports[c].candidate = candidates[c];
            reports[c].candidate.candidate_id = next_loop_candidate_id_++;
            reports[c].trials.resize(prior_hypotheses.size());
        }
        backend_timing_.trials = candidates.size() * prior_hypotheses.size();
        parallel_indices(backend_timing_.trials, [&](std::size_t index) {
            const auto c = index / prior_hypotheses.size();
            const auto h = index % prior_hypotheses.size();
            reports[c].trials[h] = evaluate_loop_candidate(prior_hypotheses[h], candidates[c]);
        });
        for (auto& report : reports) {
            std::vector<double> compatibilities;
            compatibilities.reserve(prior_hypotheses.size());
            for (const auto& trial : report.trials) {
                compatibilities.push_back(trial.compatibility);
                backend_timing_.polish_solves += trial.polish_attempted;
                backend_timing_.polish_work_ms += trial.polish_ms;
            }
            report.evidence = belugaslam::marginalize_compatibilities(masses, compatibilities);
            report.decision_score = params_.loop_verifier_mode == "map" ? report.evidence.map :
                params_.loop_verifier_mode == "uniform" ? report.evidence.uniform :
                params_.loop_verifier_mode == "geometry" ? 1.0 : report.evidence.weighted;
            report.eligible = params_.loop_update_mode == "bayes"
                ? std::any_of(report.trials.begin(), report.trials.end(), [&](const auto& trial) {
                    return trial.usable && trial.compatibility >= params_.loop_geometry_min_compatibility; })
                : report.decision_score >= params_.loop_belief_threshold &&
                    std::any_of(report.trials.begin(), report.trials.end(), [](const auto& trial) { return trial.usable; });
        }
        backend_timing_.verification_ms = elapsed_ms(verification_start);
        std::vector<std::size_t> order(reports.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
            const auto& x = reports[a]; const auto& y = reports[b];
            const double sx=params_.loop_update_mode=="bayes" ? 1.0 : x.decision_score;
            const double sy=params_.loop_update_mode=="bayes" ? 1.0 : y.decision_score;
            return sx * x.candidate.score * x.candidate.overlap > sy * y.candidate.score * y.candidate.overlap;
        });
        std::vector<std::size_t> accepted;
        for (auto index : order) {
            if (!reports[index].eligible) continue;
            // Competing candidates are categorical alternatives for one query scan,
            // not independent evidence factors to multiply into the same graph.
            if (!accepted.empty() && reports[index].candidate.query_sequence != reports[accepted.front()].candidate.query_sequence) continue;
            accepted.push_back(index);
            if (params_.loop_update_mode == "bayes" || accepted.size() >= params_.loop_max_branches) break;
        }
        if (!accepted.empty() && params_.loop_update_mode == "bayes") {
            begin_bayesian_loop_event(reports[accepted.front()], prior_hypotheses, masses, event_id);
        }
        if (!accepted.empty() && params_.loop_update_mode == "heuristic") {
            std::vector<PopulationBranch> pool;
            for (std::size_t h = 0; h < prior_hypotheses.size(); ++h) {
                if (masses[h] <= 0.0) continue;
                pool.push_back({prior_hypotheses[h], prior_hypotheses[h],
                    masses[h] * (1.0 - params_.loop_branch_prior) * params_.loop_null_compatibility, state_type{}, true});
                for (auto index : accepted) {
                    auto& trial = reports[index].trials[h];
                    const double compatibility = params_.loop_verifier_mode == "geometry" ? 1.0 : trial.compatibility;
                    if (!trial.usable || compatibility <= 0.0) continue;
                    auto child = trial.hypothesis;
                    child->id = next_hypothesis_id_++;
                    child->has_loop = true;
                    child->last_loop_sequence = reports[index].candidate.query_sequence;
                    pool.push_back({child, prior_hypotheses[h],
                        masses[h] * params_.loop_branch_prior * compatibility / accepted.size(),
                        child->T_global_local * prior_hypotheses[h]->T_global_local.inverse(), false, index});
                }
            }
            std::stable_sort(pool.begin(), pool.end(), [](const auto& a, const auto& b) { return a.mass > b.mass; });
            double unpruned_mass = 0.0;
            for (const auto& branch : pool) unpruned_mass += branch.mass;
            // Reserve a no-loop alternative if the configured hypothesis budget
            // permits ambiguity. Its quota does not inflate its probability mass.
            const auto null_it = std::find_if(pool.begin(), pool.end(), [](const auto& b) { return b.no_loop; });
            const auto best_null = null_it == pool.end() ? PopulationBranch{} : *null_it;
            if (pool.size() > params_.max_hypotheses) pool.resize(params_.max_hypotheses);
            if (pool.size() >= 2 && best_null.hypothesis &&
                std::none_of(pool.begin(), pool.end(), [](const auto& b) { return b.no_loop; })) pool.back() = best_null;
            double kept_mass = 0.0;
            for (const auto& branch : pool) kept_mass += branch.mass;
            retained_branch_mass = unpruned_mass > 0 ? kept_mass / unpruned_mass : 1.0;
            for (const auto& branch : pool) {
                if (!branch.no_loop) {
                    auto& report = reports.at(branch.candidate_index);
                    report.selected = true;
                    for (auto& trial : report.trials) {
                        if (trial.hypothesis == branch.hypothesis) trial.installed = true;
                    }
                }
            }
            install_population(pool, params_.max_particles);
            consumed_loop_queries_.insert(reports[accepted.front()].candidate.query_sequence);
            for (auto index : accepted) {
                if (!reports[index].selected) continue;
                // Marker is diagnostic only; the actual constraints carry stable scan IDs.
                for (const auto& branch : pool) {
                    if (!branch.no_loop) { loop_closure_poses_.push_back(branch.hypothesis->local_pose); break; }
                }
            }
        }
        for (const auto& report : reports) {
            if (params_.verbose_backend) std::cout << "[LOOP VERIFY] r=" << report.candidate.reference_sequence << " q=" << report.candidate.query_sequence
                << " belief=" << report.evidence.weighted << " MAP=" << report.evidence.map
                << " uniform=" << report.evidence.uniform << " selected=" << report.selected << std::endl;
            if (!loop_diagnostics_.is_open() || !loop_diagnostics_) continue;
            for (std::size_t h = 0; h < prior_hypotheses.size(); ++h) {
                const auto& trial = report.trials[h];
                loop_diagnostics_ << report.candidate.candidate_id << ',' << report.candidate.query_sequence << ',' << report.candidate.reference_sequence << ','
                    << report.candidate.source_hypothesis << ',' << report.candidate.T_reference_query.translation().x() << ','
                    << report.candidate.T_reference_query.translation().y() << ',' << report.candidate.T_reference_query.so2().log() << ','
                    << params_.loop_verifier_mode << ',' << prior_hypotheses[h]->id << ',' << masses[h] << ','
                    << trial.usable << ',' << trial.fit_translation << ',' << trial.fit_rotation << ','
                    << trial.change.translation_rmse << ',' << trial.change.rotation_rmse << ',' << trial.compatibility << ','
                    << report.evidence.weighted << ',' << report.evidence.map << ',' << report.evidence.uniform << ','
                    << report.candidate.score * report.candidate.overlap << ',' << report.eligible << ',' << report.selected << ','
                    << trial.forced_fit_translation << ',' << trial.forced_fit_rotation << ','
                    << trial.forced_compatibility << ',' << trial.settled_compatibility << ','
                    << trial.polish_attempted << ',' << trial.polish_ms << ',' << trial.status << ',' << trial.installed << ','
                    << event_id << ',' << consumed_loop_queries_.count(report.candidate.query_sequence) << ','
                    << retained_branch_mass << '\n';
            }
        }
        if (loop_diagnostics_.is_open()) loop_diagnostics_.flush();
    }

    // Transactional PGO: optimize temporary parameter arrays and publish them only
    // after solver/finite checks. Trial hypotheses never transport live particles.
    bool optimize_pose_graph(const std::shared_ptr<Hypothesis>& hypothesis,
                             bool transport_particles = true,
                             std::size_t forced_loop_index = std::numeric_limits<std::size_t>::max(),
                             bool require_convergence = false) {
        auto& graph = hypothesis->submaps;
        hypothesis->last_pgo_attempt_node_count = graph.trajectory_nodes.size();
        if (graph.trajectory_nodes.empty() || graph.node_submap_constraints.empty()) return false;
        std::map<SubmapId, std::shared_ptr<Submap>> submaps;
        for (const auto& sm : graph.history) submaps.emplace(sm->id(), sm);
        for (const auto& sm : graph.active_submaps) submaps.emplace(sm->id(), sm);
        if (submaps.empty()) return false;

        // Online grids are immutable internally under PGO. Tie the overlapping live
        // grids and the current tracking reference to one exact SE(2) variable.
        const auto reference = graph.matching_submap();
        if (!reference && !graph.active_submaps.empty()) return false;
        std::set<SubmapId> rigid_ids;
        if (reference) rigid_ids.insert(reference->id());
        for (const auto& sm : graph.active_submaps) rigid_ids.insert(sm->id());
        const SubmapId rigid_anchor = reference ? reference->id() : submaps.begin()->first;
        std::map<SubmapId, SubmapId> variable_id;
        std::map<SubmapId, Sophus::SE2d> offsets;
        std::map<SubmapId, std::array<double, 3>> variables;
        for (const auto& [id, sm] : submaps) {
            const bool rigid = reference && rigid_ids.count(id) != 0;
            variable_id[id] = rigid ? rigid_anchor : id;
            offsets[id] = rigid ? reference->local_pose().inverse() * sm->local_pose() : state_type{};
            const auto pose = rigid ? reference->global_pose() : sm->global_pose();
            variables[variable_id[id]] = {pose.translation().x(), pose.translation().y(), pose.so2().log()};
        }
        std::map<ScanNodeId, std::array<double, 3>> nodes;
        for (const auto& node : graph.trajectory_nodes) {
            nodes[node.id] = {node.global_pose.translation().x(), node.global_pose.translation().y(), node.global_pose.so2().log()};
        }
        ceres::Problem problem;
        for (std::size_t i = 0; i < graph.node_submap_constraints.size(); ++i) {
            const auto& edge = graph.node_submap_constraints[i];
            if (!variable_id.count(edge.submap_id) || !nodes.count(edge.node_id)) return false;
            const auto& offset = offsets.at(edge.submap_id);
            auto* cost = PoseGraphEdgeError::Create(
                edge.T_submap_node.translation().x(), edge.T_submap_node.translation().y(), edge.T_submap_node.so2().log(),
                edge.translation_weight, edge.rotation_weight,
                offset.translation().x(), offset.translation().y(), offset.so2().log(), params_.pgo_analytic_jacobians);
            // The candidate being verified MUST exert its full constraint. A robust
            // loss that silently ignores it could otherwise yield a misleadingly
            // unchanged trajectory and make an impossible loop pass verification.
            ceres::LossFunction* loss = edge.tag == ConstraintTag::kInterSubmap && i != forced_loop_index
                ? static_cast<ceres::LossFunction*>(new ceres::HuberLoss(params_.pgo_huber_scale)) : nullptr;
            problem.AddResidualBlock(cost, loss, variables.at(variable_id.at(edge.submap_id)).data(), nodes.at(edge.node_id).data());
        }
        for (const auto& edge : graph.local_trajectory_constraints) {
            if (!nodes.count(edge.from_node_id) || !nodes.count(edge.to_node_id)) return false;
            problem.AddResidualBlock(PoseGraphEdgeError::Create(
                edge.T_from_to.translation().x(), edge.T_from_to.translation().y(), edge.T_from_to.so2().log(),
                edge.translation_weight, edge.rotation_weight, 0.0, 0.0, 0.0, params_.pgo_analytic_jacobians), nullptr,
                nodes.at(edge.from_node_id).data(), nodes.at(edge.to_node_id).data());
        }
        auto* gauge = variables.at(variable_id.at(submaps.begin()->first)).data();
        if (!problem.HasParameterBlock(gauge)) return false;
        problem.SetParameterBlockConstant(gauge);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = params_.pgo_max_iterations;
        options.num_threads = 1;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // A verifier must not interpret an unfinished trial deformation as settled evidence.
        if ((require_convergence || forced_loop_index != std::numeric_limits<std::size_t>::max()) &&
            summary.termination_type != ceres::CONVERGENCE) return false;
        if (!summary.IsSolutionUsable() || !std::isfinite(summary.initial_cost) || !std::isfinite(summary.final_cost) ||
            summary.final_cost > summary.initial_cost + 1.0e-6 * std::max(1.0, summary.initial_cost)) return false;
        for (const auto& [id, pose] : variables) for (double v : pose) if (!std::isfinite(v)) return false;
        for (const auto& [id, pose] : nodes) for (double v : pose) if (!std::isfinite(v)) return false;

        const auto old_global_local = hypothesis->T_global_local;
        const auto write_pose = [&](std::shared_ptr<Submap>& sm) {
            const auto& value = variables.at(variable_id.at(sm->id()));
            const auto pose = state_type{Sophus::SO2d{value[2]}, Eigen::Vector2d{value[0], value[1]}} * offsets.at(sm->id());
            sm = sm->clone_for_pose();
            sm->set_global_pose(pose);
        };
        for (auto& sm : graph.history) write_pose(sm);
        for (auto& sm : graph.active_submaps) write_pose(sm);
        for (auto& node : graph.trajectory_nodes) {
            const auto& value = nodes.at(node.id);
            node.global_pose = state_type{Sophus::SO2d{value[2]}, Eigen::Vector2d{value[0], value[1]}};
        }
        if (reference) {
            const auto updated_reference = graph.find_submap(reference->id());
            hypothesis->T_global_local = updated_reference->global_pose() * updated_reference->local_pose().inverse();
        }
        const auto correction = hypothesis->T_global_local * old_global_local.inverse();
        if (hypothesis->has_pose_covariance) {
            Eigen::Matrix3d J=Eigen::Matrix3d::Identity();
            J.topLeftCorner<2,2>()=correction.so2().matrix();
            hypothesis->pose_covariance=(J*hypothesis->pose_covariance*J.transpose()).eval();
        }
        if (transport_particles) {
            for (auto&& particle : particles_) {
                if (std::get<2>(particle) == hypothesis) std::get<0>(particle) = correction * std::get<0>(particle);
            }
        }
        hypothesis->local_pose = correction * hypothesis->local_pose;
        if (hypothesis->has_pending_recovery) hypothesis->recovery_pose = correction * hypothesis->recovery_pose;
        for (auto& pending : hypothesis->pending_splits) pending.pose = correction * pending.pose;
        if (graph.has_last_keyframe_pose) graph.last_keyframe_pose = correction * graph.last_keyframe_pose;
        hypothesis->optimized_node_count = graph.trajectory_nodes.size();
        hypothesis->optimized_inter_constraints_count = graph.inter_constraint_count();
        hypothesis->pgo_usable = true;
        if (transport_particles && params_.verbose_backend) std::cout << "[PGO] H" << hypothesis->id << " " << summary.BriefReport() << std::endl;
        return true;
    }

private:
    struct FrozenPublicationKey {
        std::weak_ptr<const Submap> submap;
        std::array<double, 4> pose;
    };
    static std::array<double, 4> pose_key(const state_type& pose) {
        return {pose.translation().x(), pose.translation().y(),
                pose.so2().unit_complex().x(), pose.so2().unit_complex().y()};
    }
    mutable std::unique_ptr<GridTypeLO> frozen_publication_;
    mutable std::vector<FrozenPublicationKey> frozen_publication_keys_;

    static double elapsed_ms(std::chrono::steady_clock::time_point start) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    }
    template <class Function>
    void parallel_indices(std::size_t count, const Function& function) const {
        if (params_.worker_threads == 1 || count < 2) {
            for (std::size_t i = 0; i < count; ++i) function(i);
        } else {
            worker_arena_.execute([&] { tbb::parallel_for(std::size_t{0}, count, function); });
        }
    }
    std::vector<std::shared_ptr<Hypothesis>> hypotheses_;
    size_t next_hypothesis_id_ = 0;
    std::uint64_t next_scan_sequence_ = 0;
    std::uint64_t next_loop_candidate_id_ = 0;
    std::uint64_t next_loop_event_id_ = 0;
    std::set<std::uint64_t> consumed_loop_queries_;
    std::ofstream loop_diagnostics_;
    std::ofstream tracking_diagnostics_;
    std::ofstream bayes_diagnostics_;
    struct BayesEvent {
        bool active=false;
        std::uint64_t id=0, first_sequence=0;
        std::uint64_t last_sequence=std::numeric_limits<std::uint64_t>::max();
        std::size_t evidence_scans=0, attempted_scans=0;
    } bayes_event_;
    bool bayes_unresolved_=false;
    std::uint64_t unresolved_since_sequence_=0;
    std::string last_bayes_status_="none";
    double last_bayes_probability_=0;
    std::vector<std::vector<state_type>> motion_proposals_;
    std::vector<state_type> motion_ancestors_;
    beluga::DifferentialDriveDistribution2d motion_distribution_;
    beluga::TupleVector<FastSLAMParticle> particles_;

    MotionModel motion_model_;
    MeasurementModel measurement_model_;
    FastSLAMParams params_;
    mutable tbb::task_arena worker_arena_;
    BackendTiming backend_timing_;

    /// Odometry increment of the current scan, in the robot frame. The per-hypothesis
    /// local pose predicts with this before scan matching.
    state_type last_odom_delta_{};

    /// Cartographer's structural invariant: one submap being filled and one being
    /// started, both receiving every accepted scan. Not a tuning knob.
    static constexpr std::size_t kMaxActiveSubmaps = 2;

    /// Scratch for one scan insertion, reused so the per-scan cost is not allocation.
    std::vector<int> scan_hit_cells_;
    std::vector<int> scan_miss_cells_;
    belugaslam::ScanCellUpdates scan_updates_;

    beluga::spatial_hash<state_type> spatial_hasher_;

    /// Derived publication views are refreshed only when a consumer asks for them.
    std::shared_ptr<Hypothesis> best_hypothesis_;
    std::size_t map_hypothesis_id_ = 0;
    double map_position_risk_m2_ = 0.0, selected_position_risk_m2_ = 0.0;
    mutable bool publication_dirty_ = true;
    bool final_pgo_attempted_ = false, final_pgo_succeeded_ = false;
    mutable std::size_t publication_rebuilds_ = 0;
    mutable DynamicOccupancyGrid best_oc_grid_;
    mutable GridTypeLO best_lo_grid_;
    state_type best_pose_;

    /// Persistent record of all loop closure detection poses for RViz visualization
    std::vector<Sophus::SE2d> loop_closure_poses_;

    /// Persistent record of all spatial divergence split poses for RViz visualization
    std::vector<Sophus::SE2d> spatial_split_poses_;

    std::mt19937 rng_ = std::mt19937(std::random_device{}());
};  

#endif
