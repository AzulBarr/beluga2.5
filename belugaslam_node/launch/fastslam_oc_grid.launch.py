import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_min_particles = DeclareLaunchArgument(
        'min_particles',
        default_value='10',
        description='Minimum number of particles for FastSLAM'
    )

    declare_max_particles = DeclareLaunchArgument(
        'max_particles',
        default_value='50',
        description='Maximum number of particles for FastSLAM'
    )

    declare_odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame for FastSLAM (e.g., odom, odom_combined)'
    )

    declare_base_frame_cmd =DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame for FastSLAM (e.g., base_footprint, base_link)'
    )

    declare_slam_prefix_cmd = DeclareLaunchArgument(
        'slam_prefix',
        default_value='',
        description='Prefix for profiling tools'
    )

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )

    declare_range_max = DeclareLaunchArgument(
        'range_max',
        default_value='25.0',
        description='Maximum range for laser scan'
    )

    declare_kld_epsilon = DeclareLaunchArgument(
        'kld_epsilon',
        default_value='0.5',
        description='KLD sampling error bound'
    )

    declare_kld_z = DeclareLaunchArgument(
        'kld_z',
        default_value='3.0',
        description='KLD sampling confidence level'
    )

    declare_spatial_resolution_x = DeclareLaunchArgument(
        'spatial_resolution_x',
        default_value='0.5',
        description='Spatial resolution in x for KLD sampling'
    )

    declare_spatial_resolution_y = DeclareLaunchArgument(
        'spatial_resolution_y',
        default_value='0.5',
        description='Spatial resolution in y for KLD sampling'
    ) 

    declare_spatial_resolution_theta = DeclareLaunchArgument(
        'spatial_resolution_theta',
        default_value='0.17',
        description='Spatial resolution in theta for KLD sampling'
    )

    declare_min_update_angle = DeclareLaunchArgument(
        'min_update_angle',
        default_value='0.0',
        description='Deprecated and ignored; use keyframe_min_rotation for insertion'
    )

    declare_min_update_distance = DeclareLaunchArgument(
        'min_update_distance',
        default_value='0.0',
        description='Deprecated and ignored; use keyframe_min_translation for insertion'
    )

    declare_uncertainty_map_publish_interval = DeclareLaunchArgument(
        'uncertainty_map_publish_interval',
        default_value='10',
        description='Publish uncertainty every N map publications when subscribed; 0 disables'
    )

    declare_alpha1 = DeclareLaunchArgument('alpha1', default_value='0.1', description='Rotation noise from rotation')
    declare_alpha2 = DeclareLaunchArgument('alpha2', default_value='0.05', description='Rotation noise from translation')
    declare_alpha3 = DeclareLaunchArgument('alpha3', default_value='0.1', description='Translation noise from translation')
    declare_alpha4 = DeclareLaunchArgument('alpha4', default_value='0.05', description='Translation noise from rotation')
    declare_alpha5 = DeclareLaunchArgument('alpha5', default_value='0.1', description='Deprecated; use motion_distance_threshold (alpha1-alpha4 are the noise coefficients)')
    declare_likelihood_scaling_factor = DeclareLaunchArgument('likelihood_scaling_factor', default_value='0.05', description='Scaling factor for scan matching likelihood')
    declare_submap_num_range_data = DeclareLaunchArgument('submap_num_range_data', default_value='15', description='Accepted insertions before starting the next submap; frozen at twice this count')
    declare_keyframe_min_translation = DeclareLaunchArgument('keyframe_min_translation', default_value='0.15', description='Insertion motion filter translation threshold in meters')
    declare_keyframe_min_rotation = DeclareLaunchArgument('keyframe_min_rotation', default_value='0.0872665', description='Insertion motion filter rotation threshold in radians')
    declare_keyframe_max_time = DeclareLaunchArgument('keyframe_max_time', default_value='5.0', description='Maximum scan timestamp interval between insertions in seconds')
    declare_max_points_per_scan_node = DeclareLaunchArgument('max_points_per_scan_node', default_value='180', description='Maximum stored endpoints per graph scan node')
    declare_loop_recent_submaps = DeclareLaunchArgument('loop_recent_submaps', default_value='5', description='Recent submaps excluded from loop search')
    declare_loop_max_candidates = DeclareLaunchArgument('loop_max_candidates', default_value='6', description='Maximum retrieved submaps geometrically verified per event')
    declare_loop_max_branches = DeclareLaunchArgument('loop_max_branches', default_value='2', description='Maximum loop hypotheses spawned per event')
    declare_max_hypotheses = DeclareLaunchArgument('max_hypotheses', default_value='4', description='Global bound on graph hypotheses')
    declare_loop_candidate_distance = DeclareLaunchArgument('loop_candidate_distance', default_value='10.0', description='Maximum pose-prior distance for loop retrieval')
    declare_loop_search_translation = DeclareLaunchArgument('loop_search_translation', default_value='3.0', description='Correlative matcher translation half-window')
    declare_loop_search_rotation = DeclareLaunchArgument('loop_search_rotation', default_value='0.7', description='Correlative matcher rotation half-window')
    declare_loop_min_score = DeclareLaunchArgument('loop_min_score', default_value='0.55', description='Minimum distance-field match score')
    declare_loop_min_overlap = DeclareLaunchArgument('loop_min_overlap', default_value='0.35', description='Minimum fraction of endpoints within 0.30 m of a wall')

    declare_enable_loop_closure = DeclareLaunchArgument('enable_loop_closure', default_value='true', description='Enable loop candidate verification and branching')
    declare_enable_pgo = DeclareLaunchArgument('enable_pgo', default_value='true', description='Enable periodic PGO; false also prevents loop application')
    declare_loop_verifier_mode = DeclareLaunchArgument('loop_verifier_mode', default_value='belief', description='Loop verifier: belief, map, uniform or geometry')
    declare_output_selection_mode = DeclareLaunchArgument('output_selection_mode', default_value='map', choices=['map', 'pose_risk'], description='Published pose/map selection; pose_risk minimizes retained frontend squared position loss')
    declare_frontend_pose_mode = DeclareLaunchArgument('frontend_pose_mode', default_value='frontend', choices=['frontend', 'proposal_mean', 'proposal_seed'], description='Per-hypothesis pose: matcher, guarded proposal mean, or non-worsening proposal-seeded refinement')
    declare_proposal_pose_min_ess = DeclareLaunchArgument('proposal_pose_min_ess', default_value='5.0')
    declare_optimized_trajectory_path = DeclareLaunchArgument('optimized_trajectory_path', default_value='', description='Write the selected retrospective graph trajectory in TUM format on clean shutdown')
    declare_proposal_pose_min_local_mass = DeclareLaunchArgument('proposal_pose_min_local_mass', default_value='0.90')
    declare_proposal_pose_max_log_drop = DeclareLaunchArgument('proposal_pose_max_log_drop', default_value='0.02')
    declare_loop_belief_threshold = DeclareLaunchArgument('loop_belief_threshold', default_value='0.25', description='Minimum aggregated trajectory compatibility')
    declare_loop_translation_scale = DeclareLaunchArgument('loop_translation_scale', default_value='0.30', description='Trajectory alignment translation scale in meters')
    declare_loop_rotation_scale = DeclareLaunchArgument('loop_rotation_scale', default_value='0.10', description='Trajectory alignment rotation scale in radians')
    declare_loop_max_fit_translation = DeclareLaunchArgument('loop_max_fit_translation', default_value='0.30', description='Maximum trial loop residual in meters')
    declare_loop_max_fit_rotation = DeclareLaunchArgument('loop_max_fit_rotation', default_value='0.12', description='Maximum trial loop residual in radians')
    declare_loop_branch_prior = DeclareLaunchArgument('loop_branch_prior', default_value='0.5', description='Total prior mass assigned to categorical loop alternatives')
    declare_loop_null_compatibility = DeclareLaunchArgument('loop_null_compatibility', default_value='0.2', description='Compatibility factor for the no-loop alternative')
    declare_loop_max_verifications = DeclareLaunchArgument('loop_max_verifications', default_value='6', description='Maximum fixed candidates evaluated over the belief per event')
    declare_loop_trajectory_samples = DeclareLaunchArgument('loop_trajectory_samples', default_value='200', description='Maximum timestamp-aligned poses in trajectory verification')
    declare_loop_min_points = DeclareLaunchArgument('loop_min_points', default_value='30', description='Minimum points in a loop query scan')
    declare_pgo_every_n_nodes = DeclareLaunchArgument('pgo_every_n_nodes', default_value='20', description='Optimize each live hypothesis after this many inserted nodes')
    declare_pgo_max_iterations = DeclareLaunchArgument('pgo_max_iterations', default_value='50', description='Maximum Ceres iterations per PGO solve')
    declare_pgo_analytic_jacobians = DeclareLaunchArgument('pgo_analytic_jacobians', default_value='true', description='Use exact analytic SE2 residual derivatives; false restores AutoDiff')
    declare_loop_robust_polish = DeclareLaunchArgument('loop_robust_polish', default_value='true', description='Check loop trials under the same robust objective used after installation')
    declare_random_seed = DeclareLaunchArgument('random_seed', default_value='42', description='Reproducible core seed; zero requests random seeding')
    declare_loop_diagnostics_path = DeclareLaunchArgument('loop_diagnostics_path', default_value='', description='Optional CSV file for per-hypothesis verification scores')

    declare_worker_threads = DeclareLaunchArgument('worker_threads', default_value='2', description='Bounded concurrency for independent particle matches and loop trials')
    declare_verbose_backend = DeclareLaunchArgument('verbose_backend', default_value='false', description='Print individual backend events')
    declare_map_publish_period = DeclareLaunchArgument('map_publish_period', default_value='1.0', description='Minimum wall seconds between full map publications')
    declare_visualization_publish_period = DeclareLaunchArgument('visualization_publish_period', default_value='0.2', description='Wall seconds between visualization timer ticks')
    declare_trajectory_max_poses = DeclareLaunchArgument('trajectory_max_poses', default_value='5000', description='Maximum visualization path samples; core graph is retained')
    declare_scan_queue_depth = DeclareLaunchArgument('scan_queue_depth', default_value='50', description='DDS queue depth to absorb short bursts; does not raise processing throughput')
    declare_scan_reliable = DeclareLaunchArgument('scan_reliable', default_value='false', description='Request reliable scan delivery only with a reliable publisher')
    declare_performance_diagnostics_path = DeclareLaunchArgument('performance_diagnostics_path', default_value='', description='Optional per-scan timing and rejection CSV')
    declare_final_trajectory_path = DeclareLaunchArgument('final_trajectory_path', default_value='', description='Optional CSV with the online and pose-graph-optimized trajectory, written when the run ends')

    declare_tracking_sigma = DeclareLaunchArgument('tracking_sigma', default_value='0.15', description='See QUALITY_REVIEW.md for tracking_sigma')
    declare_tracking_outlier_probability = DeclareLaunchArgument('tracking_outlier_probability', default_value='0.05', description='See QUALITY_REVIEW.md for tracking_outlier_probability')
    declare_tracking_translation_prior_sigma = DeclareLaunchArgument('tracking_translation_prior_sigma', default_value='0.50', description='See QUALITY_REVIEW.md for tracking_translation_prior_sigma')
    declare_tracking_rotation_prior_sigma = DeclareLaunchArgument('tracking_rotation_prior_sigma', default_value='0.20', description='See QUALITY_REVIEW.md for tracking_rotation_prior_sigma')
    declare_tracking_max_translation = DeclareLaunchArgument('tracking_max_translation', default_value='0.50', description='See QUALITY_REVIEW.md for tracking_max_translation')
    declare_tracking_max_rotation = DeclareLaunchArgument('tracking_max_rotation', default_value='0.25', description='See QUALITY_REVIEW.md for tracking_max_rotation')
    declare_tracking_min_overlap = DeclareLaunchArgument('tracking_min_overlap', default_value='0.35', description='See QUALITY_REVIEW.md for tracking_min_overlap')
    declare_tracking_inlier_distance = DeclareLaunchArgument('tracking_inlier_distance', default_value='0.20', description='See QUALITY_REVIEW.md for tracking_inlier_distance')
    declare_tracking_effective_beams = DeclareLaunchArgument('tracking_effective_beams', default_value='20.0', description='See QUALITY_REVIEW.md for tracking_effective_beams')
    declare_tracking_prior_information_scale = DeclareLaunchArgument('tracking_prior_information_scale', default_value='1.0', description='Matcher prior scale; 1 preserves regularization, .05 is an experimental 20-beam MAP interpretation')
    declare_tracking_min_points = DeclareLaunchArgument('tracking_min_points', default_value='12', description='See QUALITY_REVIEW.md for tracking_min_points')
    declare_tracking_max_points = DeclareLaunchArgument('tracking_max_points', default_value='180', description='See QUALITY_REVIEW.md for tracking_max_points')
    declare_tracking_max_iterations = DeclareLaunchArgument('tracking_max_iterations', default_value='20', description='See QUALITY_REVIEW.md for tracking_max_iterations')
    declare_motion_proposal_samples = DeclareLaunchArgument('motion_proposal_samples', default_value='8', description='See QUALITY_REVIEW.md for motion_proposal_samples')
    declare_map_resolution = DeclareLaunchArgument('map_resolution', default_value='0.05', description='See QUALITY_REVIEW.md for map_resolution')
    declare_split_min_mass = DeclareLaunchArgument('split_min_mass', default_value='0.02', description='See QUALITY_REVIEW.md for split_min_mass')
    declare_split_min_particles = DeclareLaunchArgument('split_min_particles', default_value='2', description='See QUALITY_REVIEW.md for split_min_particles')
    declare_split_persistence = DeclareLaunchArgument('split_persistence', default_value='3', description='See QUALITY_REVIEW.md for split_persistence')
    declare_loop_validation_scans = DeclareLaunchArgument('loop_validation_scans', default_value='3', description='See QUALITY_REVIEW.md for loop_validation_scans')
    declare_tracking_diagnostics_path = DeclareLaunchArgument('tracking_diagnostics_path', default_value='', description='See QUALITY_REVIEW.md for tracking_diagnostics_path')
    declare_motion_distance_threshold = DeclareLaunchArgument('motion_distance_threshold', default_value='0.01', description='See QUALITY_REVIEW.md for motion_distance_threshold')
    declare_deskew_scan = DeclareLaunchArgument('deskew_scan', default_value='true', description='See QUALITY_REVIEW.md for deskew_scan')
    declare_tracking_recovery = DeclareLaunchArgument('tracking_recovery', default_value='true', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_translation_window = DeclareLaunchArgument('recovery_translation_window', default_value='1.0', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_rotation_window = DeclareLaunchArgument('recovery_rotation_window', default_value='0.35', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_min_overlap = DeclareLaunchArgument('recovery_min_overlap', default_value='0.55', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_ambiguity_margin = DeclareLaunchArgument('recovery_ambiguity_margin', default_value='0.05', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_after_failures = DeclareLaunchArgument('recovery_after_failures', default_value='1', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_interval = DeclareLaunchArgument('recovery_interval', default_value='3', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_recovery_confirmations = DeclareLaunchArgument('recovery_confirmations', default_value='2', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')
    declare_loop_cache_budget_mb = DeclareLaunchArgument('loop_cache_budget_mb', default_value='64', description='Late-run recovery/cache control; see LATE_RUN_REVIEW.md')

    belugaslam_node = Node(
        package="belugaslam_node",  
        executable="belugaslam_node", 
        name="belugaslam",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "min_particles": LaunchConfiguration('min_particles'),
            "max_particles": LaunchConfiguration('max_particles'),
            "odom_frame": LaunchConfiguration('odom_frame'), 
            "base_frame": LaunchConfiguration('base_frame'), 
            "publish_trajectory": False,
            #save_map": False,
            "range_max": LaunchConfiguration('range_max'),
            "kld_epsilon": LaunchConfiguration('kld_epsilon'),
            "kld_z": LaunchConfiguration('kld_z'),
            "spatial_resolution_x": LaunchConfiguration('spatial_resolution_x'),
            "spatial_resolution_y": LaunchConfiguration('spatial_resolution_y'),
            "spatial_resolution_theta": LaunchConfiguration('spatial_resolution_theta'),
            "min_update_angle": LaunchConfiguration('min_update_angle'),
            "min_update_distance": LaunchConfiguration('min_update_distance'),
            "uncertainty_map_publish_interval": LaunchConfiguration('uncertainty_map_publish_interval'),
            "alpha1": LaunchConfiguration('alpha1'),
            "alpha2": LaunchConfiguration('alpha2'),
            "alpha3": LaunchConfiguration('alpha3'),
            "alpha4": LaunchConfiguration('alpha4'),
            "alpha5": LaunchConfiguration('alpha5'),
            "likelihood_scaling_factor": LaunchConfiguration('likelihood_scaling_factor'),
            "submap_num_range_data": LaunchConfiguration('submap_num_range_data'),
            "keyframe_min_translation": LaunchConfiguration('keyframe_min_translation'),
            "keyframe_min_rotation": LaunchConfiguration('keyframe_min_rotation'),
            "keyframe_max_time": LaunchConfiguration('keyframe_max_time'),
            "max_points_per_scan_node": LaunchConfiguration('max_points_per_scan_node'),
            "loop_recent_submaps": LaunchConfiguration('loop_recent_submaps'),
            "loop_max_candidates": LaunchConfiguration('loop_max_candidates'),
            "loop_max_branches": LaunchConfiguration('loop_max_branches'),
            "max_hypotheses": LaunchConfiguration('max_hypotheses'),
            "loop_candidate_distance": LaunchConfiguration('loop_candidate_distance'),
            "loop_search_translation": LaunchConfiguration('loop_search_translation'),
            "loop_search_rotation": LaunchConfiguration('loop_search_rotation'),
            "loop_min_score": LaunchConfiguration('loop_min_score'),
            "loop_min_overlap": LaunchConfiguration('loop_min_overlap'),
            "enable_loop_closure": ParameterValue(LaunchConfiguration('enable_loop_closure'), value_type=bool),
            "enable_pgo": ParameterValue(LaunchConfiguration('enable_pgo'), value_type=bool),
            "loop_verifier_mode": ParameterValue(LaunchConfiguration('loop_verifier_mode'), value_type=str),
            "output_selection_mode": ParameterValue(LaunchConfiguration('output_selection_mode'), value_type=str),
            "frontend_pose_mode": ParameterValue(LaunchConfiguration('frontend_pose_mode'), value_type=str),
            "optimized_trajectory_path": ParameterValue(LaunchConfiguration('optimized_trajectory_path'), value_type=str),
            "proposal_pose_min_ess": ParameterValue(LaunchConfiguration('proposal_pose_min_ess'), value_type=float),
            "proposal_pose_min_local_mass": ParameterValue(LaunchConfiguration('proposal_pose_min_local_mass'), value_type=float),
            "proposal_pose_max_log_drop": ParameterValue(LaunchConfiguration('proposal_pose_max_log_drop'), value_type=float),
            "loop_belief_threshold": ParameterValue(LaunchConfiguration('loop_belief_threshold'), value_type=float),
            "loop_translation_scale": ParameterValue(LaunchConfiguration('loop_translation_scale'), value_type=float),
            "loop_rotation_scale": ParameterValue(LaunchConfiguration('loop_rotation_scale'), value_type=float),
            "loop_max_fit_translation": ParameterValue(LaunchConfiguration('loop_max_fit_translation'), value_type=float),
            "loop_max_fit_rotation": ParameterValue(LaunchConfiguration('loop_max_fit_rotation'), value_type=float),
            "loop_branch_prior": ParameterValue(LaunchConfiguration('loop_branch_prior'), value_type=float),
            "loop_null_compatibility": ParameterValue(LaunchConfiguration('loop_null_compatibility'), value_type=float),
            "loop_max_verifications": ParameterValue(LaunchConfiguration('loop_max_verifications'), value_type=int),
            "loop_trajectory_samples": ParameterValue(LaunchConfiguration('loop_trajectory_samples'), value_type=int),
            "loop_min_points": ParameterValue(LaunchConfiguration('loop_min_points'), value_type=int),
            "pgo_every_n_nodes": ParameterValue(LaunchConfiguration('pgo_every_n_nodes'), value_type=int),
            "pgo_max_iterations": ParameterValue(LaunchConfiguration('pgo_max_iterations'), value_type=int),
            "pgo_analytic_jacobians": ParameterValue(LaunchConfiguration('pgo_analytic_jacobians'), value_type=bool),
            "loop_robust_polish": ParameterValue(LaunchConfiguration('loop_robust_polish'), value_type=bool),
            "random_seed": ParameterValue(LaunchConfiguration('random_seed'), value_type=int),
            "loop_diagnostics_path": ParameterValue(LaunchConfiguration('loop_diagnostics_path'), value_type=str),

            "worker_threads": ParameterValue(LaunchConfiguration('worker_threads'), value_type=int),
            "verbose_backend": ParameterValue(LaunchConfiguration('verbose_backend'), value_type=bool),
            "map_publish_period": ParameterValue(LaunchConfiguration('map_publish_period'), value_type=float),
            "visualization_publish_period": ParameterValue(LaunchConfiguration('visualization_publish_period'), value_type=float),
            "trajectory_max_poses": ParameterValue(LaunchConfiguration('trajectory_max_poses'), value_type=int),
            "scan_queue_depth": ParameterValue(LaunchConfiguration('scan_queue_depth'), value_type=int),
            "scan_reliable": ParameterValue(LaunchConfiguration('scan_reliable'), value_type=bool),
            "performance_diagnostics_path": ParameterValue(LaunchConfiguration('performance_diagnostics_path'), value_type=str),
            "final_trajectory_path": ParameterValue(LaunchConfiguration('final_trajectory_path'), value_type=str),
            "tracking_sigma": ParameterValue(LaunchConfiguration('tracking_sigma'), value_type=float),
            "tracking_outlier_probability": ParameterValue(LaunchConfiguration('tracking_outlier_probability'), value_type=float),
            "tracking_translation_prior_sigma": ParameterValue(LaunchConfiguration('tracking_translation_prior_sigma'), value_type=float),
            "tracking_rotation_prior_sigma": ParameterValue(LaunchConfiguration('tracking_rotation_prior_sigma'), value_type=float),
            "tracking_max_translation": ParameterValue(LaunchConfiguration('tracking_max_translation'), value_type=float),
            "tracking_max_rotation": ParameterValue(LaunchConfiguration('tracking_max_rotation'), value_type=float),
            "tracking_min_overlap": ParameterValue(LaunchConfiguration('tracking_min_overlap'), value_type=float),
            "tracking_inlier_distance": ParameterValue(LaunchConfiguration('tracking_inlier_distance'), value_type=float),
            "tracking_effective_beams": ParameterValue(LaunchConfiguration('tracking_effective_beams'), value_type=float),
            "tracking_prior_information_scale": ParameterValue(LaunchConfiguration('tracking_prior_information_scale'), value_type=float),
            "tracking_min_points": ParameterValue(LaunchConfiguration('tracking_min_points'), value_type=int),
            "tracking_max_points": ParameterValue(LaunchConfiguration('tracking_max_points'), value_type=int),
            "tracking_max_iterations": ParameterValue(LaunchConfiguration('tracking_max_iterations'), value_type=int),
            "motion_proposal_samples": ParameterValue(LaunchConfiguration('motion_proposal_samples'), value_type=int),
            "map_resolution": ParameterValue(LaunchConfiguration('map_resolution'), value_type=float),
            "split_min_mass": ParameterValue(LaunchConfiguration('split_min_mass'), value_type=float),
            "split_min_particles": ParameterValue(LaunchConfiguration('split_min_particles'), value_type=int),
            "split_persistence": ParameterValue(LaunchConfiguration('split_persistence'), value_type=int),
            "loop_validation_scans": ParameterValue(LaunchConfiguration('loop_validation_scans'), value_type=int),
            "tracking_diagnostics_path": ParameterValue(LaunchConfiguration('tracking_diagnostics_path'), value_type=str),
            "motion_distance_threshold": ParameterValue(LaunchConfiguration('motion_distance_threshold'), value_type=float),
            "deskew_scan": ParameterValue(LaunchConfiguration('deskew_scan'), value_type=bool),
            "tracking_recovery": ParameterValue(LaunchConfiguration('tracking_recovery'), value_type=bool),
            "recovery_translation_window": ParameterValue(LaunchConfiguration('recovery_translation_window'), value_type=float),
            "recovery_rotation_window": ParameterValue(LaunchConfiguration('recovery_rotation_window'), value_type=float),
            "recovery_min_overlap": ParameterValue(LaunchConfiguration('recovery_min_overlap'), value_type=float),
            "recovery_ambiguity_margin": ParameterValue(LaunchConfiguration('recovery_ambiguity_margin'), value_type=float),
            "recovery_after_failures": ParameterValue(LaunchConfiguration('recovery_after_failures'), value_type=int),
            "recovery_interval": ParameterValue(LaunchConfiguration('recovery_interval'), value_type=int),
            "recovery_confirmations": ParameterValue(LaunchConfiguration('recovery_confirmations'), value_type=int),
            "loop_cache_budget_mb": ParameterValue(LaunchConfiguration('loop_cache_budget_mb'), value_type=int),
        }],
        remappings=[('/scan', LaunchConfiguration('scan_topic'))],
        arguments=["--ros-args", "--log-level", "INFO"],
        prefix=LaunchConfiguration('slam_prefix')
    )

    return LaunchDescription([
    declare_slam_prefix_cmd,
    declare_odom_frame_cmd,
    declare_base_frame_cmd,
    declare_scan_topic_cmd,
    declare_min_particles,
    declare_max_particles,
    declare_range_max,
    declare_kld_epsilon,
    declare_kld_z,
    declare_spatial_resolution_x,
    declare_spatial_resolution_y,
    declare_spatial_resolution_theta,
    declare_min_update_angle,
    declare_min_update_distance,
    declare_uncertainty_map_publish_interval,
    declare_use_sim_time,
    declare_alpha1,
    declare_alpha2,
    declare_alpha3,
    declare_alpha4,
    declare_alpha5,
    declare_likelihood_scaling_factor,
    declare_submap_num_range_data,
    declare_keyframe_min_translation,
    declare_keyframe_min_rotation,
    declare_keyframe_max_time,
    declare_max_points_per_scan_node,
    declare_loop_recent_submaps,
    declare_loop_max_candidates,
    declare_loop_max_branches,
    declare_max_hypotheses,
    declare_loop_candidate_distance,
    declare_loop_search_translation,
    declare_loop_search_rotation,
    declare_loop_min_score,
    declare_loop_min_overlap,
    declare_enable_loop_closure,
    declare_enable_pgo,
    declare_loop_verifier_mode,
    declare_output_selection_mode,
    declare_frontend_pose_mode,
    declare_optimized_trajectory_path,
    declare_proposal_pose_min_ess,
    declare_proposal_pose_min_local_mass,
    declare_proposal_pose_max_log_drop,
    declare_loop_belief_threshold,
    declare_loop_translation_scale,
    declare_loop_rotation_scale,
    declare_loop_max_fit_translation,
    declare_loop_max_fit_rotation,
    declare_loop_branch_prior,
    declare_loop_null_compatibility,
    declare_loop_max_verifications,
    declare_loop_trajectory_samples,
    declare_loop_min_points,
    declare_pgo_every_n_nodes,
    declare_pgo_max_iterations,
    declare_pgo_analytic_jacobians,
    declare_loop_robust_polish,
    declare_random_seed,
    declare_loop_diagnostics_path,
    declare_worker_threads,
    declare_verbose_backend,
    declare_map_publish_period,
    declare_visualization_publish_period,
    declare_trajectory_max_poses,
    declare_scan_queue_depth,
    declare_scan_reliable,
    declare_performance_diagnostics_path,
    declare_final_trajectory_path,
    declare_tracking_sigma,
    declare_tracking_outlier_probability,
    declare_tracking_translation_prior_sigma,
    declare_tracking_rotation_prior_sigma,
    declare_tracking_max_translation,
    declare_tracking_max_rotation,
    declare_tracking_min_overlap,
    declare_tracking_inlier_distance,
    declare_tracking_effective_beams,
    declare_tracking_prior_information_scale,
    declare_tracking_min_points,
    declare_tracking_max_points,
    declare_tracking_max_iterations,
    declare_motion_proposal_samples,
    declare_map_resolution,
    declare_split_min_mass,
    declare_split_min_particles,
    declare_split_persistence,
    declare_loop_validation_scans,
    declare_tracking_diagnostics_path,
    declare_motion_distance_threshold,
    declare_deskew_scan,
    declare_tracking_recovery,
    declare_recovery_translation_window,
    declare_recovery_rotation_window,
    declare_recovery_min_overlap,
    declare_recovery_ambiguity_margin,
    declare_recovery_after_failures,
    declare_recovery_interval,
    declare_recovery_confirmations,
    declare_loop_cache_budget_mb,

    belugaslam_node,
    ])
