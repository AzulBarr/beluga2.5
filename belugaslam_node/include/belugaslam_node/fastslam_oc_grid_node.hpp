#ifndef __BELUGASLAM_NODE_HPP__
#define __BELUGASLAM_NODE_HPP__

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <fstream>

#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "belugaslam_core/fastslam_oc_grid_core.hpp"

using state_type = Sophus::SE2d;

/**
 * \file
 * \brief ROS 2 wrapper for the 2D FastSlam algorithm implementation.
 */

/**
 * \brief ROS 2 Node for 2D Simultaneous Localization and Mapping (SLAM).
 *
 * This node interfaces with sensor data and odometry to build a probabilistic 
 * occupancy grid map while estimating the robot's trajectory using a 
 * particle filter (FastSLAM 1.5).
 * 
 */
class BelugaSLAMNode : public rclcpp::Node {
public:
    /// Constructor.
    BelugaSLAMNode();
    ~BelugaSLAMNode() {
        // An exception escaping a destructor aborts the process; a failed export must not.
        try { write_final_trajectory(); }
        catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Final trajectory not written: %s", error.what());
        }
    }

private:
    void setup_slam();
    /**
     * \brief Processes incoming laser scans and triggers the SLAM update cycle.
     * \param msg Shared pointer to the incoming sensor_msgs::msg::LaserScan.
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /// Extracts and publishes the occupancy grid of the most likely particle.
    void publish_map();
    void publish_visualization();

    /// Extracts and publishes the pose of the most likely particle.
    void publish_best_pose(const rclcpp::Time& stamp);
    
    /**
     * \brief Publishes the current particle cloud for visualization.
     * \param stamp The timestamp to be used in the message header.
     */
    void publish_particles(const rclcpp::Time& stamp);
    
    /**
     * \brief Computes and broadcasts the map-to-odom transform.
     * \param stamp Current simulation/system time.
     * \param current_odom Current odometry pose in the odom frame.
     */
    void broadcast_map_to_odom(const rclcpp::Time& stamp, const state_type& current_odom);
    
    //void save_map();

    /// Writes the run trajectory, online and optimized, once the run is over.
    void write_final_trajectory();

    /**
     * \brief Converts polar laser readings to Cartesian coordinates in the robot's local frame.
     * \param msg The laser scan message.
     * \return A vector of (x, y) coordinates.
     */
    [[nodiscard]] std::vector<std::pair<double, double>> laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr msg, const state_type& start_odom);
    
    /**
     * \brief Converts a geometry_msgs Transform to a Sophus SE2 state.
     * \param t The transform message.
     * \return The equivalent state in SE2.
     */
    [[nodiscard]] state_type tf_to_se2(const geometry_msgs::msg::Transform& t);

    /**
     * \brief Computes the covariance of the best particle's pose estimate and returns it as a 3x3 matrix.
     * The covariance is derived from the distribution of particles around the best estimate, providing insight into the uncertainty of the pose estimation.
     */
    void compute_se2_covariance();

    void compute_entropy();

    void publish_uncertainty_map();

    /// Publishes persistent markers at loop closure detection poses
    void publish_loop_closure_markers(const rclcpp::Time& stamp);

    /// Publishes persistent red markers at spatial cluster split poses
    void publish_spatial_split_markers(const rclcpp::Time& stamp);

    std::unique_ptr<BelugaSLAM> slam_; // Pointer to the BelugaSLAM core implementation.
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr entropy_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr uncertainty_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_closure_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr spatial_split_markers_pub_;

    state_type last_odom_;
    bool first_odom_received_ = false;
    size_t best_idx_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    nav_msgs::msg::Path trajectory_msg_;

    std::string odom_f;
    std::string base_f;
    bool publish_trajectory;
    bool save_grid;
    double range_max;
    bool deskew_scan_ = true;
    int uncertainty_map_publish_interval;

    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};
    bool has_processed_scan_ = false;
    state_type last_output_pose_;
    std::size_t last_output_hypothesis_ = 0;
    std::int64_t last_processed_stamp_ns_ = 0;
    double map_publish_period_ = 1.0;
    double visualization_publish_period_ = 0.2;
    std::size_t trajectory_max_poses_ = 5000;
    std::size_t marker_loop_count_ = 0, marker_split_count_ = 0;
    std::chrono::steady_clock::time_point last_map_publish_{};
    double last_map_ms_ = 0.0, last_visualization_ms_ = 0.0;
    std::uint64_t visualization_ticks_ = 0;
    std::uint64_t map_publications_ = 0, scans_received_ = 0, scans_processed_ = 0;
    std::uint64_t tf_errors_ = 0, empty_scans_ = 0, out_of_order_scans_ = 0;
    std::ofstream performance_csv_;
<<<<<<< HEAD
    std::ofstream final_trajectory_csv_;
    // Scan sequence -> stamp of the scan the core gave that sequence to. The core
    // counts only the scans it inserted, so this cannot be derived from scans_received_.
    std::vector<std::int64_t> scan_sequence_stamps_;
=======
    std::ofstream optimized_trajectory_file_;
>>>>>>> 09d27d2 (mejoras generales)
    struct ScanTiming {
        double tf_convert_ms = 0.0, motion_ms = 0.0, matching_ms = 0.0;
        double insertion_ms = 0.0, backend_ms = 0.0, resample_ms = 0.0, pose_publish_ms = 0.0;
        double stamp_delta_ms = 0.0, scan_period_ms = 0.0, age_ms = 0.0;
        std::size_t selected_hypothesis=0, weak_scans=0;
        bool selection_changed=false;
        std::string tracking_status="unprocessed";
        double output_innovation_m=0, output_innovation_rad=0;
        BelugaSLAM::BackendTiming backend;
    };
    void record_performance(const rclcpp::Time& stamp, const char* status,
                            std::chrono::steady_clock::time_point start, const ScanTiming& timing);

    int it = 0; // Iteration counter for controlling the frequency of certain operations (e.g., publishing the uncertainty map).

    Sophus::Matrix3<double> covariance_ = 1e3 * Sophus::Matrix3<double>::Identity(); // Covariance matrix for the best particle's pose estimate.
}; 

#endif // __BELUGASLAM_NODE_HPP__
