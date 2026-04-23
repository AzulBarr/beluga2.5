// #ifndef __FASTSLAM_NODE_H__
// #define __FASTSLAM_NODE_H__

#include <opencv2/opencv.hpp>

#ifndef __FASTSLAM_NODE_HPP__
#define __FASTSLAM_NODE_HPP__

#include <iomanip>
#include <memory>
#include <vector>
#include <string>
#include <chrono>

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

#include "fastslam_core/fastslam_oc_grid.hpp"

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
 * particle filter (FastSLAM 1.0).
 * 
 */
class FastSLAMNode : public rclcpp::Node {
public:
    /// Constructor.
    FastSLAMNode();
    ~FastSLAMNode() {
        save_map();
        save_trajectory();
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
    
    void save_map();
    void save_trajectory();

    /**
     * \brief Converts polar laser readings to Cartesian coordinates in the robot's local frame.
     * \param msg The laser scan message.
     * \return A vector of (x, y) coordinates.
     */
    [[nodiscard]] std::vector<std::pair<double, double>> laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * \brief Converts a geometry_msgs Transform to a Sophus SE2 state.
     * \param t The transform message.
     * \return The equivalent state in SE2.
     */
    [[nodiscard]] state_type tf_to_se2(const geometry_msgs::msg::Transform& t);

    std::unique_ptr<FastSLAM> slam_; // Pointer to the FastSLAM core implementation.
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    state_type last_odom_;
    bool first_odom_received_ = false;
    size_t best_idx_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    nav_msgs::msg::Path trajectory_msg_;
    double min_distance_ = 0.2; 
    Eigen::Vector2d last_saved_pos_;

    std::string odom_f;
    std::string base_f;
    bool publish_trajectory;
    bool save_grid;
    double range_max;
};

#endif // __FASTSLAM_NODE_HPP__
