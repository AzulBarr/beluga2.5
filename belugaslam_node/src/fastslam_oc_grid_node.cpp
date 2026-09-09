#include "belugaslam_node/fastslam_oc_grid_node.hpp"
#include <string_view>
using namespace rclcpp;

BelugaSLAMNode::BelugaSLAMNode() : Node("belugaslam_node") {
    this->declare_parameter("min_particles", 10);
    this->declare_parameter("max_particles", 50);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("publish_trajectory", false);
    this->declare_parameter("save_map", true);
    this->declare_parameter("range_max", 25.0);
    this->declare_parameter("kld_epsilon", 0.5);
    this->declare_parameter("kld_z", 3.0);
    this->declare_parameter("spatial_resolution_x", 0.05);
    this->declare_parameter("spatial_resolution_y", 0.05);
    this->declare_parameter("spatial_resolution_theta", 10 * Sophus::Constants<double>::pi() / 180);
    // Accepted for existing launch files; insertion is now filtered after matching.
    this->declare_parameter("min_update_distance", 0.0);
    this->declare_parameter("min_update_angle", 0.0);
    this->declare_parameter("uncertainty_map_publish_interval", 10);
    this->declare_parameter("alpha1", 0.1);
    this->declare_parameter("alpha2", 0.05);
    this->declare_parameter("alpha3", 0.1);
    this->declare_parameter("alpha4", 0.05);
    this->declare_parameter("alpha5", 0.1);
    this->declare_parameter("likelihood_scaling_factor", 0.05);
    this->declare_parameter("submap_num_range_data", 15);
    this->declare_parameter("keyframe_min_translation", 0.15);
    this->declare_parameter("keyframe_min_rotation", 5.0 * Sophus::Constants<double>::pi() / 180.0);
    this->declare_parameter("keyframe_max_time", 5.0);
    this->declare_parameter("max_points_per_scan_node", 180);
    this->declare_parameter("loop_recent_submaps", 5);
    this->declare_parameter("loop_max_candidates", 6);
    this->declare_parameter("loop_max_branches", 2);
    this->declare_parameter("max_hypotheses", 4);
    this->declare_parameter("loop_candidate_distance", 10.0);
    this->declare_parameter("loop_search_translation", 3.0);
    this->declare_parameter("loop_search_rotation", 0.7);
    this->declare_parameter("loop_min_score", 0.55);
    this->declare_parameter("loop_min_overlap", 0.35);

    this->declare_parameter("enable_loop_closure", true);
    this->declare_parameter("enable_pgo", true);
    this->declare_parameter("loop_verifier_mode", "belief");
    this->declare_parameter("output_selection_mode", "map");
    this->declare_parameter("loop_belief_threshold", 0.25);
    this->declare_parameter("loop_translation_scale", 0.30);
    this->declare_parameter("loop_rotation_scale", 0.10);
    this->declare_parameter("loop_max_fit_translation", 0.30);
    this->declare_parameter("loop_max_fit_rotation", 0.12);
    this->declare_parameter("loop_branch_prior", 0.5);
    this->declare_parameter("loop_null_compatibility", 0.2);
    this->declare_parameter("loop_max_verifications", 6);
    this->declare_parameter("loop_trajectory_samples", 200);
    this->declare_parameter("loop_min_points", 30);
    this->declare_parameter("pgo_every_n_nodes", 20);
    this->declare_parameter("pgo_max_iterations", 50);
    this->declare_parameter("pgo_analytic_jacobians", true);
    this->declare_parameter("loop_robust_polish", true);
    this->declare_parameter("random_seed", 42);
    this->declare_parameter("loop_diagnostics_path", "");

    this->declare_parameter("worker_threads", 2);
    this->declare_parameter("verbose_backend", false);
    this->declare_parameter("map_publish_period", 1.0);
    this->declare_parameter("visualization_publish_period", 0.2);
    this->declare_parameter("trajectory_max_poses", 5000);
    this->declare_parameter("scan_queue_depth", 50);
    this->declare_parameter("scan_reliable", false);
    this->declare_parameter("performance_diagnostics_path", "");
    this->declare_parameter("final_trajectory_path", "");

    declare_parameter("tracking_sigma", 0.15);
    declare_parameter("tracking_outlier_probability", 0.05);
    declare_parameter("tracking_translation_prior_sigma", 0.50);
    declare_parameter("tracking_rotation_prior_sigma", 0.20);
    declare_parameter("tracking_max_translation", 0.50);
    declare_parameter("tracking_max_rotation", 0.25);
    declare_parameter("tracking_min_overlap", 0.35);
    declare_parameter("tracking_inlier_distance", 0.20);
    declare_parameter("tracking_effective_beams", 20.0);
    declare_parameter("tracking_min_points", 12);
    declare_parameter("tracking_max_points", 180);
    declare_parameter("tracking_max_iterations", 20);
    declare_parameter("motion_proposal_samples", 8);
    declare_parameter("map_resolution", 0.05);
    declare_parameter("split_min_mass", 0.02);
    declare_parameter("split_min_particles", 2);
    declare_parameter("split_persistence", 3);
    declare_parameter("loop_validation_scans", 3);
    declare_parameter("tracking_diagnostics_path", "");
    declare_parameter("motion_distance_threshold", 0.01);
    declare_parameter("deskew_scan", true);
    this->declare_parameter("tracking_recovery", true);
    this->declare_parameter("recovery_translation_window", 1.0);
    this->declare_parameter("recovery_rotation_window", 0.35);
    this->declare_parameter("recovery_min_overlap", 0.55);
    this->declare_parameter("recovery_ambiguity_margin", 0.05);
    this->declare_parameter("recovery_after_failures", 1);
    this->declare_parameter("recovery_interval", 3);
    this->declare_parameter("recovery_confirmations", 2);
    this->declare_parameter("loop_cache_budget_mb", 64);


    setup_slam();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    const auto depth = get_parameter("scan_queue_depth").as_int();
    if (depth < 1) throw std::invalid_argument("scan_queue_depth must be positive");
    auto scan_qos = rclcpp::SensorDataQoS();
    scan_qos.keep_last(static_cast<std::size_t>(depth));
    if (get_parameter("scan_reliable").as_bool()) scan_qos.reliable();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", scan_qos, std::bind(&BelugaSLAMNode::laser_callback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
    particle_cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 10);
    entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("/localization_entropy", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/best_pose", 10);
    uncertainty_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_uncertainty", 1);
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    loop_closure_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_markers", rclcpp::QoS(1).transient_local());
    spatial_split_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/spatial_split_markers", rclcpp::QoS(1).transient_local());
    trajectory_msg_.header.frame_id = "map";
    visualization_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(visualization_publish_period_)),
        std::bind(&BelugaSLAMNode::publish_visualization, this));
    
    std::cout << "\033[1;32m[BelugaSLAM] Node initialized and waiting for data...\033[0m" << std::endl;
}

void BelugaSLAMNode::setup_slam() {
    double a1 = get_parameter("alpha1").as_double();
    double a2 = get_parameter("alpha2").as_double();
    double a3 = get_parameter("alpha3").as_double();
    double a4 = get_parameter("alpha4").as_double();
    const double motion_threshold = get_parameter("motion_distance_threshold").as_double();
    for (double value : {a1,a2,a3,a4,motion_threshold})
        if (!std::isfinite(value) || value < 0) throw std::invalid_argument("Motion noise/threshold must be finite and nonnegative");
    if (get_parameter("alpha5").as_double() != 0.1)
        RCLCPP_WARN(get_logger(), "alpha5 is deprecated and ignored: this differential-drive model has four noise coefficients. Use motion_distance_threshold.");
    beluga::DifferentialDriveModelParam motion_params{a1, a2, a3, a4, motion_threshold};
    beluga::DifferentialDriveModel<state_type> motion_model{motion_params};

    beluga::LikelihoodFieldProbModelParam sensor_params{100.0, 2.0, 0.5, 0.5, 0.2, true};
    beluga::LikelihoodFieldProbModel<GridTypeOC> measurement_model(sensor_params, GridTypeOC());

    auto params = FastSLAMParams{};
    if (get_parameter("min_particles").as_int() < 1 ||
        get_parameter("max_particles").as_int() < get_parameter("min_particles").as_int())
        throw std::invalid_argument("Require 1 <= min_particles <= max_particles");
    params.min_particles = static_cast<std::size_t>(get_parameter("min_particles").as_int());
    params.max_particles = static_cast<std::size_t>(get_parameter("max_particles").as_int());
    publish_trajectory = this->get_parameter("publish_trajectory").as_bool();
    save_grid = this->get_parameter("save_map").as_bool();
    odom_f = this->get_parameter("odom_frame").as_string();
    base_f = this->get_parameter("base_frame").as_string();
    range_max = this->get_parameter("range_max").as_double();
    params.kld_epsilon = get_parameter("kld_epsilon").as_double();
    params.kld_z = get_parameter("kld_z").as_double();
    params.spatial_resolution_x = get_parameter("spatial_resolution_x").as_double();
    params.spatial_resolution_y = get_parameter("spatial_resolution_y").as_double();
    params.spatial_resolution_theta = get_parameter("spatial_resolution_theta").as_double();
    params.likelihood_scaling_factor = get_parameter("likelihood_scaling_factor").as_double();
    if (params.likelihood_scaling_factor != 0.05)
        RCLCPP_WARN(get_logger(), "likelihood_scaling_factor is legacy; use tracking_effective_beams for normalized robust scan evidence");
    params.submap_num_range_data = get_parameter("submap_num_range_data").as_int();
    params.keyframe_min_translation = get_parameter("keyframe_min_translation").as_double();
    params.keyframe_min_rotation = get_parameter("keyframe_min_rotation").as_double();
    params.keyframe_max_time = get_parameter("keyframe_max_time").as_double();
    params.max_points_per_scan_node = static_cast<std::size_t>(get_parameter("max_points_per_scan_node").as_int());
    params.loop_recent_submaps = static_cast<std::size_t>(get_parameter("loop_recent_submaps").as_int());
    params.loop_max_candidates = static_cast<std::size_t>(get_parameter("loop_max_candidates").as_int());
    params.loop_max_branches = static_cast<std::size_t>(get_parameter("loop_max_branches").as_int());
    params.max_hypotheses = static_cast<std::size_t>(get_parameter("max_hypotheses").as_int());
    params.loop_candidate_distance = get_parameter("loop_candidate_distance").as_double();
    params.loop_search_translation = get_parameter("loop_search_translation").as_double();
    params.loop_search_rotation = get_parameter("loop_search_rotation").as_double();
    params.loop_min_score = get_parameter("loop_min_score").as_double();
    params.loop_min_overlap = get_parameter("loop_min_overlap").as_double();
    if (get_parameter("min_update_angle").as_double() != 0.0 ||
        get_parameter("min_update_distance").as_double() != 0.0) {
        RCLCPP_WARN(get_logger(),
            "min_update_angle/min_update_distance are deprecated and ignored. "
            "Use keyframe_min_rotation/keyframe_min_translation/keyframe_max_time "
            "to filter insertion after scan matching.");
    }
    uncertainty_map_publish_interval = get_parameter("uncertainty_map_publish_interval").as_int();

    params.enable_loop_closure = get_parameter("enable_loop_closure").as_bool();
    params.enable_pgo = get_parameter("enable_pgo").as_bool();
    params.loop_verifier_mode = get_parameter("loop_verifier_mode").as_string();
    params.output_selection_mode = get_parameter("output_selection_mode").as_string();
    params.loop_belief_threshold = get_parameter("loop_belief_threshold").as_double();
    params.loop_translation_scale = get_parameter("loop_translation_scale").as_double();
    params.loop_rotation_scale = get_parameter("loop_rotation_scale").as_double();
    params.loop_max_fit_translation = get_parameter("loop_max_fit_translation").as_double();
    params.loop_max_fit_rotation = get_parameter("loop_max_fit_rotation").as_double();
    params.loop_branch_prior = get_parameter("loop_branch_prior").as_double();
    params.loop_null_compatibility = get_parameter("loop_null_compatibility").as_double();
    if (get_parameter("loop_max_verifications").as_int() < 0) throw std::invalid_argument("loop_max_verifications must be nonnegative");
    params.loop_max_verifications = static_cast<decltype(params.loop_max_verifications)>(get_parameter("loop_max_verifications").as_int());
    if (get_parameter("loop_trajectory_samples").as_int() < 0) throw std::invalid_argument("loop_trajectory_samples must be nonnegative");
    params.loop_trajectory_samples = static_cast<decltype(params.loop_trajectory_samples)>(get_parameter("loop_trajectory_samples").as_int());
    if (get_parameter("loop_min_points").as_int() < 0) throw std::invalid_argument("loop_min_points must be nonnegative");
    params.loop_min_points = static_cast<decltype(params.loop_min_points)>(get_parameter("loop_min_points").as_int());
    if (get_parameter("pgo_every_n_nodes").as_int() < 0) throw std::invalid_argument("pgo_every_n_nodes must be nonnegative");
    params.pgo_every_n_nodes = static_cast<decltype(params.pgo_every_n_nodes)>(get_parameter("pgo_every_n_nodes").as_int());
    params.pgo_max_iterations = get_parameter("pgo_max_iterations").as_int();
    params.pgo_analytic_jacobians = get_parameter("pgo_analytic_jacobians").as_bool();
    params.loop_robust_polish = get_parameter("loop_robust_polish").as_bool();
    if (get_parameter("random_seed").as_int() < 0) throw std::invalid_argument("random_seed must be nonnegative");
    params.random_seed = static_cast<decltype(params.random_seed)>(get_parameter("random_seed").as_int());
    params.loop_diagnostics_path = get_parameter("loop_diagnostics_path").as_string();

    const auto workers = get_parameter("worker_threads").as_int();
    if (workers < 1 || workers > std::numeric_limits<int>::max())
        throw std::invalid_argument("worker_threads must fit a positive int");
    params.worker_threads = static_cast<int>(workers);
    params.verbose_backend = get_parameter("verbose_backend").as_bool();
    map_publish_period_ = get_parameter("map_publish_period").as_double();
    visualization_publish_period_ = get_parameter("visualization_publish_period").as_double();
    for (double period : {map_publish_period_, visualization_publish_period_}) {
        if (!std::isfinite(period) || period < 0.001)
            throw std::invalid_argument("Publication periods must be finite and at least 0.001 seconds");
    }
    if (get_parameter("trajectory_max_poses").as_int() < 1)
        throw std::invalid_argument("trajectory_max_poses must be positive");
    trajectory_max_poses_ = static_cast<std::size_t>(get_parameter("trajectory_max_poses").as_int());
    if (uncertainty_map_publish_interval < 0)
        throw std::invalid_argument("uncertainty_map_publish_interval must be nonnegative (0 disables it)");
    const auto final_trajectory_path = get_parameter("final_trajectory_path").as_string();
    if (!final_trajectory_path.empty()) {
        // Opened now, written at shutdown: a run that cannot write its trajectory
        // should fail before processing the dataset, not after.
        final_trajectory_csv_.open(final_trajectory_path);
        if (!final_trajectory_csv_) throw std::runtime_error("Cannot open final_trajectory_path");
    }
    const auto performance_path = get_parameter("performance_diagnostics_path").as_string();
    if (!performance_path.empty()) {
        performance_csv_.open(performance_path);
        if (!performance_csv_) throw std::runtime_error("Cannot open performance_diagnostics_path");
        performance_csv_ << std::setprecision(17)
            << "received,stamp_ns,status,stamp_delta_ms,scan_period_ms,age_ms,total_ms,tf_convert_ms,"
               "motion_ms,matching_ms,insertion_ms,backend_ms,resample_ms,pose_publish_ms,"
               "baseline_pgo_ms,retrieval_ms,verification_ms,baseline_solves,candidates,trials,"
               "particles,hypotheses,processed,tf_errors,empty_scans,out_of_order_scans,"
               "map_publications,last_map_ms,visualization_ticks,last_visualization_ms,local_only_pgo_skips,loop_cache_bytes,selected_hypothesis,selection_changed,tracking_status,weak_scans,output_innovation_m,output_innovation_rad,output_x,output_y,output_yaw,output_selection_mode,map_hypothesis,map_position_risk_m2,selected_position_risk_m2,polish_solves,polish_work_ms\n";
    }

    params.tracking.sigma = static_cast<decltype(params.tracking.sigma)>(get_parameter("tracking_sigma").as_double());
    params.tracking.outlier_probability = static_cast<decltype(params.tracking.outlier_probability)>(get_parameter("tracking_outlier_probability").as_double());
    params.tracking.prior_translation_sigma = static_cast<decltype(params.tracking.prior_translation_sigma)>(get_parameter("tracking_translation_prior_sigma").as_double());
    params.tracking.prior_rotation_sigma = static_cast<decltype(params.tracking.prior_rotation_sigma)>(get_parameter("tracking_rotation_prior_sigma").as_double());
    params.tracking.max_translation = static_cast<decltype(params.tracking.max_translation)>(get_parameter("tracking_max_translation").as_double());
    params.tracking.max_rotation = static_cast<decltype(params.tracking.max_rotation)>(get_parameter("tracking_max_rotation").as_double());
    params.tracking.min_overlap = static_cast<decltype(params.tracking.min_overlap)>(get_parameter("tracking_min_overlap").as_double());
    params.tracking.inlier_distance = static_cast<decltype(params.tracking.inlier_distance)>(get_parameter("tracking_inlier_distance").as_double());
    params.tracking.effective_beams = static_cast<decltype(params.tracking.effective_beams)>(get_parameter("tracking_effective_beams").as_double());
    if (get_parameter("tracking_min_points").as_int() < 1 || get_parameter("tracking_min_points").as_int() > 100000) throw std::invalid_argument("Invalid tracking_min_points");
    params.tracking.min_points = static_cast<decltype(params.tracking.min_points)>(get_parameter("tracking_min_points").as_int());
    if (get_parameter("tracking_max_points").as_int() < 1 || get_parameter("tracking_max_points").as_int() > 100000) throw std::invalid_argument("Invalid tracking_max_points");
    params.tracking.max_points = static_cast<decltype(params.tracking.max_points)>(get_parameter("tracking_max_points").as_int());
    if (get_parameter("tracking_max_iterations").as_int() < 1 || get_parameter("tracking_max_iterations").as_int() > 100000) throw std::invalid_argument("Invalid tracking_max_iterations");
    params.tracking.max_iterations = static_cast<decltype(params.tracking.max_iterations)>(get_parameter("tracking_max_iterations").as_int());
    if (get_parameter("motion_proposal_samples").as_int() < 1 || get_parameter("motion_proposal_samples").as_int() > 100000) throw std::invalid_argument("Invalid motion_proposal_samples");
    params.motion_proposal_samples = static_cast<decltype(params.motion_proposal_samples)>(get_parameter("motion_proposal_samples").as_int());
    params.map_resolution = static_cast<decltype(params.map_resolution)>(get_parameter("map_resolution").as_double());
    params.split_min_mass = static_cast<decltype(params.split_min_mass)>(get_parameter("split_min_mass").as_double());
    if (get_parameter("split_min_particles").as_int() < 1 || get_parameter("split_min_particles").as_int() > 100000) throw std::invalid_argument("Invalid split_min_particles");
    params.split_min_particles = static_cast<decltype(params.split_min_particles)>(get_parameter("split_min_particles").as_int());
    if (get_parameter("split_persistence").as_int() < 1 || get_parameter("split_persistence").as_int() > 100000) throw std::invalid_argument("Invalid split_persistence");
    params.split_persistence = static_cast<decltype(params.split_persistence)>(get_parameter("split_persistence").as_int());
    if (get_parameter("loop_validation_scans").as_int() < 1 || get_parameter("loop_validation_scans").as_int() > 100000) throw std::invalid_argument("Invalid loop_validation_scans");
    params.loop_validation_scans = static_cast<decltype(params.loop_validation_scans)>(get_parameter("loop_validation_scans").as_int());
    params.tracking_diagnostics_path = static_cast<decltype(params.tracking_diagnostics_path)>(get_parameter("tracking_diagnostics_path").as_string());
    deskew_scan_ = get_parameter("deskew_scan").as_bool();
    params.recovery.enabled = static_cast<decltype(params.recovery.enabled)>(get_parameter("tracking_recovery").as_bool());
    params.recovery.translation_window = static_cast<decltype(params.recovery.translation_window)>(get_parameter("recovery_translation_window").as_double());
    params.recovery.rotation_window = static_cast<decltype(params.recovery.rotation_window)>(get_parameter("recovery_rotation_window").as_double());
    params.recovery.min_overlap = static_cast<decltype(params.recovery.min_overlap)>(get_parameter("recovery_min_overlap").as_double());
    params.recovery.ambiguity_margin = static_cast<decltype(params.recovery.ambiguity_margin)>(get_parameter("recovery_ambiguity_margin").as_double());
    if (get_parameter("recovery_after_failures").as_int()<0 || get_parameter("recovery_after_failures").as_int()>100000) throw std::invalid_argument("Invalid recovery_after_failures");
    params.recovery.after_failures = static_cast<decltype(params.recovery.after_failures)>(get_parameter("recovery_after_failures").as_int());
    if (get_parameter("recovery_interval").as_int()<0 || get_parameter("recovery_interval").as_int()>100000) throw std::invalid_argument("Invalid recovery_interval");
    params.recovery.interval = static_cast<decltype(params.recovery.interval)>(get_parameter("recovery_interval").as_int());
    if (get_parameter("recovery_confirmations").as_int()<0 || get_parameter("recovery_confirmations").as_int()>100000) throw std::invalid_argument("Invalid recovery_confirmations");
    params.recovery.confirmations = static_cast<decltype(params.recovery.confirmations)>(get_parameter("recovery_confirmations").as_int());
    if (get_parameter("loop_cache_budget_mb").as_int()<0 || get_parameter("loop_cache_budget_mb").as_int()>100000) throw std::invalid_argument("Invalid loop_cache_budget_mb");
    params.loop_cache_budget_mb = static_cast<decltype(params.loop_cache_budget_mb)>(get_parameter("loop_cache_budget_mb").as_int());


    /// BelugaSLAM instance
    slam_ = std::make_unique<BelugaSLAM> (motion_model, measurement_model, params);

    std::cout << "\033[1;32m[BelugaSLAM] SLAM setup completed with " << params.min_particles
              << " - " << params.max_particles << " particles\033[0m" << std::endl;
}

void BelugaSLAMNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto stamp = rclcpp::Time(msg->header.stamp);
    const auto elapsed = [](auto from, auto to) {
        return std::chrono::duration<double, std::milli>(to - from).count();
    };
    ++scans_received_;
    ScanTiming timing;
    timing.age_ms = (this->now() - stamp).seconds() * 1000.0;
    timing.scan_period_ms = std::isfinite(msg->scan_time) && msg->scan_time > 0 ? 1000.0 * msg->scan_time : 0.0;
    if (has_processed_scan_) timing.stamp_delta_ms = (stamp.nanoseconds() - last_processed_stamp_ns_) * 1.0e-6;
    if (has_processed_scan_ && stamp.nanoseconds() <= last_processed_stamp_ns_) {
        ++out_of_order_scans_;
        record_performance(stamp, "out_of_order", start, timing);
        return;
    }
    try {
        auto tf_now = tf_buffer_->lookupTransform(odom_f, base_f, msg->header.stamp, rclcpp::Duration::from_seconds(0.7));
        const auto current_odom = tf_to_se2(tf_now.transform);
        auto z = laser_to_cartesian(msg, current_odom);
        auto t0 = Clock::now();
        timing.tf_convert_ms = elapsed(start, t0);
        if (z.empty()) {
            ++empty_scans_;
            record_performance(stamp, "empty_scan", start, timing);
            return;
        }
        if (!first_odom_received_) {
            last_odom_ = current_odom;
            first_odom_received_ = true;
        }
        const auto u = std::make_tuple(current_odom, last_odom_);
        last_odom_ = current_odom;
        slam_->sample_motion_model(u);
        auto t1 = Clock::now(); timing.motion_ms = elapsed(t0, t1);
        slam_->measurement_model_map(z);
        auto t2 = Clock::now(); timing.matching_ms = elapsed(t1, t2);
        const auto finished_events = slam_->update_occupancy_grid(z, stamp.seconds());
        auto t3 = Clock::now(); timing.insertion_ms = elapsed(t2, t3);
        slam_->post_update(z, finished_events);
        auto t4 = Clock::now(); timing.backend_ms = elapsed(t3, t4);
        timing.backend = slam_->backend_timing();
        slam_->resample();
        auto t5 = Clock::now(); timing.resample_ms = elapsed(t4, t5);
        timing.selected_hypothesis=slam_->best_hypothesis_id();
        timing.tracking_status=slam_->best_tracking_status();
        timing.weak_scans=slam_->best_tracking_failures();
        if (has_processed_scan_) {
            timing.selection_changed=timing.selected_hypothesis!=last_output_hypothesis_;
            const auto predicted=last_output_pose_*(std::get<1>(u).inverse()*std::get<0>(u));
            const auto innovation=predicted.inverse()*slam_->best_pose();
            timing.output_innovation_m=innovation.translation().norm();
            timing.output_innovation_rad=std::abs(innovation.so2().log());
        }
        last_output_pose_=slam_->best_pose();last_output_hypothesis_=timing.selected_hypothesis;
        compute_se2_covariance();
        publish_best_pose(stamp);
        broadcast_map_to_odom(stamp, current_odom);
        timing.pose_publish_ms = elapsed(t5, Clock::now());
        latest_scan_stamp_ = stamp;
        last_processed_stamp_ns_ = stamp.nanoseconds();
        has_processed_scan_ = true;
        ++scans_processed_;
        // The core assigns a sequence only to scans it actually inserted, so pair the
        // stamp with the sequence it just took instead of assuming they stay in step.
        if (final_trajectory_csv_.is_open()) {
            const auto assigned = slam_->scan_sequence_count();
            if (assigned > scan_sequence_stamps_.size()) {
                scan_sequence_stamps_.resize(assigned, 0);
                scan_sequence_stamps_.back() = stamp.nanoseconds();
            }
        }
        record_performance(stamp, "processed", start, timing);
    } catch (const tf2::TransformException& ex) {
        ++tf_errors_;
        timing.tf_convert_ms = elapsed(start, Clock::now());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF rejected scan: %s", ex.what());
        record_performance(stamp, "tf_error", start, timing);
    }
}

void BelugaSLAMNode::record_performance(const rclcpp::Time& stamp, const char* status,
    std::chrono::steady_clock::time_point start, const ScanTiming& t) {
    const double total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (performance_csv_.is_open()) {
        performance_csv_ << scans_received_ << ',' << stamp.nanoseconds() << ',' << status << ','
            << t.stamp_delta_ms << ',' << t.scan_period_ms << ',' << t.age_ms << ',' << total_ms << ','
            << t.tf_convert_ms << ',' << t.motion_ms << ',' << t.matching_ms << ',' << t.insertion_ms << ','
            << t.backend_ms << ',' << t.resample_ms << ',' << t.pose_publish_ms << ','
            << t.backend.baseline_pgo_ms << ',' << t.backend.retrieval_ms << ',' << t.backend.verification_ms << ','
            << t.backend.baseline_solves << ',' << t.backend.candidates << ',' << t.backend.trials << ','
            << slam_->particles().size() << ',' << slam_->get_active_hypotheses_count() << ','
            << scans_processed_ << ',' << tf_errors_ << ',' << empty_scans_ << ',' << out_of_order_scans_ << ','
            << map_publications_ << ',' << last_map_ms_ << ',' << visualization_ticks_ << ',' << last_visualization_ms_ << ','
            << t.backend.local_only_skips << ',' << t.backend.loop_cache_bytes << ',' << t.selected_hypothesis << ','
            << t.selection_changed << ',' << t.tracking_status << ',' << t.weak_scans << ','
            << t.output_innovation_m << ',' << t.output_innovation_rad;
        // Export the exact online pose used for /best_pose at this scan stamp.
        // Rejected callbacks have no new estimate; leave all three fields empty.
        if (std::string_view(status) == "processed") {
            performance_csv_ << ',' << last_output_pose_.translation().x() << ','
                << last_output_pose_.translation().y() << ',' << last_output_pose_.so2().log() << ','
                << slam_->output_selection_mode() << ',' << slam_->map_hypothesis_id() << ','
                << slam_->map_position_risk_m2() << ',' << slam_->selected_position_risk_m2() << ','
                << t.backend.polish_solves << ',' << t.backend.polish_work_ms;
        } else {
            performance_csv_ << ",,,,,,,,,";
        }
        performance_csv_ << '\n';
        if (scans_received_ % 100 == 0) performance_csv_.flush();
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Scan %.1f ms (match %.1f, insert %.1f, backend %.1f, TF/convert %.1f); "
        "processed=%llu received=%llu TF_errors=%llu empty=%llu out_of_order=%llu; last map %.1f ms",
        total_ms, t.matching_ms, t.insertion_ms, t.backend_ms, t.tf_convert_ms,
        static_cast<unsigned long long>(scans_processed_), static_cast<unsigned long long>(scans_received_),
        static_cast<unsigned long long>(tf_errors_), static_cast<unsigned long long>(empty_scans_),
        static_cast<unsigned long long>(out_of_order_scans_), last_map_ms_);
}

void BelugaSLAMNode::write_final_trajectory() {
    if (!final_trajectory_csv_.is_open() || !slam_) return;
    // Read once, at the end: every loop closure the run ever accepted is already in
    // the optimized poses, including the ones that corrected scans logged long before.
    const auto trajectory = slam_->final_trajectory();
    final_trajectory_csv_ << std::setprecision(17)
        << "sequence,stamp_ns,submap,online_x,online_y,online_yaw,optimized_x,optimized_y,optimized_yaw\n";
    std::size_t written = 0;
    for (const auto& point : trajectory) {
        // Scans the core sequenced but the node never stamped, and any sequence from a
        // discarded hypothesis, have no timestamp to evaluate against.
        if (point.sequence >= scan_sequence_stamps_.size()) continue;
        const auto stamp_ns = scan_sequence_stamps_[point.sequence];
        if (stamp_ns == 0) continue;
        final_trajectory_csv_ << point.sequence << ',' << stamp_ns << ',' << point.submap_id << ','
            << point.online.translation().x() << ',' << point.online.translation().y() << ','
            << point.online.so2().log() << ','
            << point.optimized.translation().x() << ',' << point.optimized.translation().y() << ','
            << point.optimized.so2().log() << '\n';
        ++written;
    }
    final_trajectory_csv_.flush();
    RCLCPP_INFO(get_logger(), "Final trajectory: %zu poses written (hypothesis %zu, %zu samples held)",
                written, slam_->best_hypothesis_id(), trajectory.size());
}

void BelugaSLAMNode::publish_visualization() {
    if (!has_processed_scan_) return;
    const auto visualization_start = std::chrono::steady_clock::now();
    if (particle_cloud_pub_->get_subscription_count()) publish_particles(latest_scan_stamp_);
    if (entropy_pub_->get_subscription_count()) compute_entropy();
    if (publish_trajectory && trajectory_pub_->get_subscription_count()) {
        trajectory_msg_.header.stamp = latest_scan_stamp_;
        trajectory_pub_->publish(trajectory_msg_);
    }
    if (slam_->loop_closure_poses().size() != marker_loop_count_) {
        publish_loop_closure_markers(latest_scan_stamp_);
        marker_loop_count_ = slam_->loop_closure_poses().size();
    }
    if (slam_->spatial_split_poses().size() != marker_split_count_) {
        publish_spatial_split_markers(latest_scan_stamp_);
        marker_split_count_ = slam_->spatial_split_poses().size();
    }
    const auto now = std::chrono::steady_clock::now();
    if (map_publications_ == 0 || std::chrono::duration<double>(now - last_map_publish_).count() >= map_publish_period_) {
        publish_map();  // transient-local map stays available to late subscribers
        ++map_publications_;
        if (uncertainty_map_publish_interval > 0 && uncertainty_map_pub_->get_subscription_count() &&
            map_publications_ % static_cast<std::uint64_t>(uncertainty_map_publish_interval) == 0) publish_uncertainty_map();
        last_map_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - now).count();
        last_map_publish_ = std::chrono::steady_clock::now();
    }
    last_visualization_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - visualization_start).count();
    ++visualization_ticks_;
}

std::vector<std::pair<double, double>> BelugaSLAMNode::laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr msg, const state_type& start_odom) {
    std::vector<std::pair<double, double>> points;
    points.reserve(msg->ranges.size());
    if (!std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment)) return points;
    Sophus::SE2d::Tangent scan_motion = Sophus::SE2d::Tangent::Zero();
    bool deskew_available = false;
    if (deskew_scan_ && msg->ranges.size() > 1 && std::isfinite(msg->time_increment) && msg->time_increment > 0) {
        const auto end_stamp = rclcpp::Time(msg->header.stamp) + rclcpp::Duration::from_seconds(
            static_cast<double>(msg->time_increment) * (msg->ranges.size()-1));
        try {
            const auto end_tf = tf_buffer_->lookupTransform(odom_f, base_f, end_stamp, rclcpp::Duration::from_seconds(0));
            scan_motion = (start_odom.inverse() * tf_to_se2(end_tf.transform)).log();
            deskew_available = true;
        } catch (const tf2::TransformException&) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "End-of-scan odometry unavailable: processing this scan without deskew");
        }
    }
    auto tf_laser = tf_buffer_->lookupTransform(
        base_f,                      // target (base_link)
        msg->header.frame_id,        // source (laser)
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1)
    );

    Sophus::SE2d T_bl_laser = tf_to_se2(tf_laser.transform);

    const double sensor_max = std::isfinite(msg->range_max) && msg->range_max > 0 ? msg->range_max : range_max;
    const double sensor_min = std::isfinite(msg->range_min) ? std::max(0.1, static_cast<double>(msg->range_min)) : 0.1;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const float r = msg->ranges[i];
        if (std::isfinite(r) && r < std::min(range_max, sensor_max) && r > sensor_min) {
            const double angle = static_cast<double>(msg->angle_min) + i * static_cast<double>(msg->angle_increment);

            Eigen::Vector2d p_laser(
                r * std::cos(angle),
                r * std::sin(angle)
            );

            Eigen::Vector2d p_base = T_bl_laser * p_laser;
            if (deskew_available) p_base = Sophus::SE2d::exp(
                scan_motion * (static_cast<double>(i)/(msg->ranges.size()-1))) * p_base;

            points.emplace_back(p_base.x(), p_base.y());
        }
    }
    return points;
}

Sophus::SE2d BelugaSLAMNode::tf_to_se2(const geometry_msgs::msg::Transform& t) {
    double yaw = tf2::getYaw(t.rotation);
    return Sophus::SE2d{Sophus::SO2d{yaw}, Eigen::Vector2d{t.translation.x, t.translation.y}};
}

void BelugaSLAMNode::publish_map() {
    const auto& best_oc_grid = slam_->best_occupancy_grid();

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = latest_scan_stamp_;
    msg.header.frame_id = "map";
    
    msg.info.resolution = best_oc_grid.resolution();
    msg.info.width = best_oc_grid.width();
    msg.info.height = best_oc_grid.height();
    
    msg.info.origin.position.x = best_oc_grid.origin().translation().x();
    msg.info.origin.position.y = best_oc_grid.origin().translation().y();
    
    tf2::Quaternion q;
    q.setRPY(0, 0, best_oc_grid.origin().so2().log());
    msg.info.origin.orientation = tf2::toMsg(q);

    msg.data.assign(best_oc_grid.data().begin(), best_oc_grid.data().end());
    
    map_pub_->publish(msg);
}

void BelugaSLAMNode::publish_best_pose(const rclcpp::Time& stamp) {

    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = "map";

    const auto& best_pose = slam_->best_pose();

    // Pose
    msg.pose.pose.position.x = best_pose.translation().x();
    msg.pose.pose.position.y = best_pose.translation().y();
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, best_pose.so2().log());
    msg.pose.pose.orientation = tf2::toMsg(q);

    // Covarianza
    auto cov = covariance_;

    msg.pose.covariance.fill(0.0);

    msg.pose.covariance[0]  = cov(0,0) + 1e-6;  // x-x
    msg.pose.covariance[1]  = cov(0,1);  // x-y
    msg.pose.covariance[6]  = cov(1,0);  // y-x
    msg.pose.covariance[7]  = cov(1,1) + 1e-6;  // y-y

    msg.pose.covariance[14] = 1e-6; // z-z
    msg.pose.covariance[21] = 1e-6; // roll-roll
    msg.pose.covariance[28] = 1e-6; // pitch-pitch

    msg.pose.covariance[5]  = cov(0,2);  // x-yaw
    msg.pose.covariance[30] = cov(2,0);  // yaw-x

    msg.pose.covariance[11] = cov(1,2);  // y-yaw
    msg.pose.covariance[31] = cov(2,1);  // yaw-y

    msg.pose.covariance[35] = cov(2,2) + 1e-6;  // yaw-yaw

    pose_pub_->publish(msg);

    // Trajectory
    if (publish_trajectory) {
        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header = msg.header;
        pose_msg.pose = msg.pose.pose;

        trajectory_msg_.header.stamp = stamp;
        trajectory_msg_.header.frame_id = "map";

        trajectory_msg_.poses.push_back(pose_msg);

        if (trajectory_msg_.poses.size() > trajectory_max_poses_) {
            const auto remove = trajectory_msg_.poses.size() - trajectory_max_poses_;
            trajectory_msg_.poses.erase(trajectory_msg_.poses.begin(), trajectory_msg_.poses.begin() + remove);
        }
    }
}

void BelugaSLAMNode::publish_particles(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    
    auto poses = beluga::views::states(slam_->particles());
    for (const auto& pose : poses) {
        geometry_msgs::msg::Pose p;
        p.position.x = pose.translation().x();
        p.position.y = pose.translation().y();
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.so2().log());
        p.orientation = tf2::toMsg(q);
        msg.poses.push_back(p);
    }
    particle_cloud_pub_->publish(msg);
}

void BelugaSLAMNode::broadcast_map_to_odom(const rclcpp::Time& stamp, const Sophus::SE2d& current_odom) {
    auto best_pose = slam_->best_pose();
    auto map_to_odom = best_pose * current_odom.inverse();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = odom_f;
    t.transform.translation.x = map_to_odom.translation().x();
    t.transform.translation.y = map_to_odom.translation().y();
    tf2::Quaternion q;
    q.setRPY(0, 0, map_to_odom.so2().log());
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
}


void BelugaSLAMNode::compute_se2_covariance() {
    // Report the selected mode's second moment about the published frontend pose.
    // A covariance about the global mixture mean belongs to a different estimator.
    const auto best = slam_->best_pose();
    const auto selected = slam_->best_hypothesis_id();
    covariance_.setZero();
    double mass = 0;
    for (const auto& particle : slam_->particles()) {
        if (std::get<2>(particle)->id != selected) continue;
        const double weight = static_cast<double>(std::get<1>(particle));
        const auto& pose = std::get<0>(particle);
        const Eigen::Vector3d error{pose.translation().x() - best.translation().x(),
            pose.translation().y() - best.translation().y(),
            belugaslam::wrap_angle(pose.so2().log() - best.so2().log())};
        covariance_ += weight * error * error.transpose(); mass += weight;
    }
    if (mass > 0) covariance_ /= mass;
    else covariance_ = 1e3 * Sophus::Matrix3<double>::Identity();
}

void BelugaSLAMNode::compute_entropy() {
    auto weights = beluga::views::weights(slam_->particles());
    double entropy = 0.0;
    for (const auto& w : weights) {
        if (w > 1e-9) { // Avoid log(0)
            entropy -= w * std::log(w);
        }
    }
    std_msgs::msg::Float64 msg;
    msg.data = entropy;
    entropy_pub_->publish(msg);
}

void BelugaSLAMNode::publish_uncertainty_map() {
    const auto& best_lo_grid = slam_->best_log_odds_grid();

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = latest_scan_stamp_;
    msg.header.frame_id = "map";

    msg.info.resolution = best_lo_grid.resolution();
    msg.info.width = best_lo_grid.width();
    msg.info.height = best_lo_grid.height();
    
    msg.info.origin.position.x = best_lo_grid.origin().translation().x();
    msg.info.origin.position.y = best_lo_grid.origin().translation().y();
    
    tf2::Quaternion q;
    q.setRPY(0, 0, best_lo_grid.origin().so2().log());
    msg.info.origin.orientation = tf2::toMsg(q);

    constexpr double eps = 1e-9;
    msg.data.assign(best_lo_grid.data().size(), 0);
    for (long unsigned int i = 0; i < best_lo_grid.data().size(); ++i) {
        double p = 1.0f / (1.0f + std::exp(-best_lo_grid.data().at(i)));
        p = std::clamp(p, eps, 1.0 - eps);

        double H = -p * std::log(p) -(1.0 - p) * std::log(1.0 - p);

        msg.data[i] = static_cast<int8_t>(100.0 * H / std::log(2.0));
    }
    uncertainty_map_pub_->publish(msg);
}

void BelugaSLAMNode::publish_loop_closure_markers(const rclcpp::Time& stamp) {
    const auto& lc_poses = slam_->loop_closure_poses();
    if (lc_poses.empty()) return;

    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < lc_poses.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = stamp;
        marker.ns = "loop_closures";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = lc_poses[i].translation().x();
        marker.pose.position.y = lc_poses[i].translation().y();
        marker.pose.position.z = 0.3; // Slightly above the ground plane

        tf2::Quaternion q;
        q.setRPY(0, 0, lc_poses[i].so2().log());
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Bright green, fully opaque
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Never expire
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array.markers.push_back(marker);
    }

    loop_closure_markers_pub_->publish(marker_array);
}

void BelugaSLAMNode::publish_spatial_split_markers(const rclcpp::Time& stamp) {
    const auto& split_poses = slam_->spatial_split_poses();
    if (split_poses.empty()) return;

    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < split_poses.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = stamp;
        marker.ns = "spatial_splits";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = split_poses[i].translation().x();
        marker.pose.position.y = split_poses[i].translation().y();
        marker.pose.position.z = 0.35; // Slightly offset above ground plane

        tf2::Quaternion q;
        q.setRPY(0, 0, split_poses[i].so2().log());
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Bright red, fully opaque
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Never expire
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array.markers.push_back(marker);
    }

    spatial_split_markers_pub_->publish(marker_array);
}
