import os
from launch import LaunchDescription
from launch_ros.actions import Node
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

    declare_random_seed = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random seed for reproducibility (0 for random seed)'
    )

    declare_min_update_angle = DeclareLaunchArgument(
        'min_update_angle',
        default_value='0.1',
        description='Minimum angle difference for resampling'
    )

    declare_min_update_distance = DeclareLaunchArgument(
        'min_update_distance',
        default_value='0.1',
        description='Minimum distance difference for resampling'
    )

    declare_uncertainty_map_publish_interval = DeclareLaunchArgument(
        'uncertainty_map_publish_interval',
        default_value='10',
        description='Publish uncertainty map every N iterations'
    )

    declare_alpha1 = DeclareLaunchArgument('alpha1', default_value='0.1', description='Rotation noise from rotation')
    declare_alpha2 = DeclareLaunchArgument('alpha2', default_value='0.05', description='Rotation noise from translation')
    declare_alpha3 = DeclareLaunchArgument('alpha3', default_value='0.1', description='Translation noise from translation')
    declare_alpha4 = DeclareLaunchArgument('alpha4', default_value='0.05', description='Translation noise from rotation')
    declare_alpha5 = DeclareLaunchArgument('alpha5', default_value='0.1', description='Translation noise (extra)')
    declare_likelihood_scaling_factor = DeclareLaunchArgument('likelihood_scaling_factor', default_value='0.05', description='Scaling factor for scan matching likelihood')
    declare_submap_num_range_data = DeclareLaunchArgument('submap_num_range_data', default_value='15', description='Scans before starting the next submap; a submap is frozen at twice this count')
    declare_keyframe_min_translation = DeclareLaunchArgument('keyframe_min_translation', default_value='0.15', description='Graph motion filter translation threshold')
    declare_keyframe_min_rotation = DeclareLaunchArgument('keyframe_min_rotation', default_value='0.0872665', description='Graph motion filter rotation threshold in radians')
    declare_max_points_per_scan_node = DeclareLaunchArgument('max_points_per_scan_node', default_value='180', description='Maximum stored endpoints per graph scan node')
    declare_loop_recent_submaps = DeclareLaunchArgument('loop_recent_submaps', default_value='5', description='Recent submaps excluded from loop search')
    declare_loop_max_candidates = DeclareLaunchArgument('loop_max_candidates', default_value='6', description='Maximum retrieved submaps geometrically verified per event')
    declare_loop_max_branches = DeclareLaunchArgument('loop_max_branches', default_value='2', description='Maximum loop hypotheses spawned per event')
    declare_max_hypotheses = DeclareLaunchArgument('max_hypotheses', default_value='4', description='Global bound on graph hypotheses')
    declare_loop_candidate_distance = DeclareLaunchArgument('loop_candidate_distance', default_value='10.0', description='Maximum pose-prior distance for loop retrieval')
    declare_loop_search_translation = DeclareLaunchArgument('loop_search_translation', default_value='3.0', description='Correlative matcher translation half-window')
    declare_loop_search_rotation = DeclareLaunchArgument('loop_search_rotation', default_value='0.7', description='Correlative matcher rotation half-window')
    declare_loop_min_score = DeclareLaunchArgument('loop_min_score', default_value='0.55', description='Minimum distance-field match score')
    declare_loop_min_overlap = DeclareLaunchArgument('loop_min_overlap', default_value='0.35', description='Minimum in-bounds scan overlap')

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
    declare_random_seed,
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
    belugaslam_node,
    ])
