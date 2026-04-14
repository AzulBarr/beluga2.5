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

    slam_prefix = LaunchConfiguration('slam_prefix')

    fastslam_node = Node(
        package="fastslam_node",  
        executable="fastslam_node", 
        name="fastslam_oc_grid",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "num_particles": 500,
            "odom_frame": "odom_combined", 
            "base_frame": "base_footprint", 
            "publish_trajectory": False,
        }],
        arguments=["--ros-args", "--log-level", "WARN"],
        remappings=[
            ("/scan", "/base_scan"), # para el bag del MIT
        ],
        #prefix=LaunchConfiguration('slam_prefix') # para el bag de Beluga
    )

    return LaunchDescription([
        declare_fastslam_prefix_cmd,
        fastslam_node
    ])