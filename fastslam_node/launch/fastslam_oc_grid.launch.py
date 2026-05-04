import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_num_particles = DeclareLaunchArgument(
        'num_particles',
        default_value='500',
        description='Number of particles for FastSLAM'
    )

    declare_min_particles = DeclareLaunchArgument(
        'min_particles',
        default_value='500',
        description='Minimum number of particles for FastSLAM'
    )

    declare_max_particles = DeclareLaunchArgument(
        'max_particles',
        default_value='2000',
        description='Maximum number of particles for FastSLAM'
    )

    declare_odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom_combined',
        description='Odometry frame for FastSLAM (e.g., odom, odom_combined)'
    )

    declare_base_frame_cmd =DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame for FastSLAM (e.g., base_footprint, base_link)'
    )

    declare_slam_prefix_cmd = DeclareLaunchArgument(
        'slam_prefix',
        default_value='',
        description='Prefix for profiling tools'
    )

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/base_scan',
        description='Laser scan topic'
    )

    declare_range_max = DeclareLaunchArgument(
        'range_max',
        default_value='25.0',
        description='Maximum range for laser scan'
    )

    declare_kld_epsilon = DeclareLaunchArgument(
        'kld_epsilon',
        default_value='0.05',
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

    fastslam_node = Node(
        package="fastslam_node",  
        executable="fastslam_node", 
        name="fastslam_oc_grid",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "num_particles": LaunchConfiguration('num_particles'),
            "min_particles": LaunchConfiguration('min_particles'),
            "max_particles": LaunchConfiguration('max_particles'),
            "odom_frame": LaunchConfiguration('odom_frame'), 
            "base_frame": LaunchConfiguration('base_frame'), 
            "publish_trajectory": False,
            "save_map": True,
            "range_max": LaunchConfiguration('range_max'),
            "kld_epsilon": LaunchConfiguration('kld_epsilon'),
            "kld_z": LaunchConfiguration('kld_z'),
        }],
        remappings=[('/scan', LaunchConfiguration('scan_topic'))],
        arguments=["--ros-args", "--log-level", "WARN"],
        prefix=LaunchConfiguration('slam_prefix')
    )

    return LaunchDescription([
    declare_slam_prefix_cmd,
    declare_odom_frame_cmd,
    declare_base_frame_cmd,
    declare_scan_topic_cmd,
    declare_num_particles,
    declare_min_particles,
    declare_max_particles,
    declare_range_max,
    declare_kld_epsilon,
    declare_kld_z,
    fastslam_node,
    ])