import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_fastslam_prefix_cmd = DeclareLaunchArgument(
        'fastslam_prefix',
        default_value='',
        description='Set of commands to precede the node (e.g. "valgrind" or "gdb")'
    )

    declare_odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom_combined'
    )

    declare_base_frame_cmd =DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint'
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

    fastslam_node = Node(
        package="fastslam_node",  
        executable="fastslam_node", 
        name="fastslam_oc_grid",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "num_particles": 500,
            "odom_frame": LaunchConfiguration('odom_frame'), 
            "base_frame": LaunchConfiguration('base_frame'), 
            "publish_trajectory": False,
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
    fastslam_node,
    ])