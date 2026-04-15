#include "fastslam_node/fastslam_oc_grid.hpp"
using namespace rclcpp;

FastSLAMNode::FastSLAMNode() : Node("fastslam_node") {
    this->declare_parameter("num_particles", 500);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("publish_trajectory", false);
    this->declare_parameter("save_map", true);
    
    setup_slam();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&FastSLAMNode::laser_callback, this, std::placeholders::_1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
    particle_cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/best_pose", 10);
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    trajectory_msg_.header.frame_id = "map";
    
    RCLCPP_INFO(this->get_logger(), "FastSLAM Node inicialized and waiting for data...");
}

void FastSLAMNode::setup_slam() {
    beluga::DifferentialDriveModelParam motion_params{0.1, 0.1, 0.1, 0.1};
    beluga::DifferentialDriveModel<state_type> motion_model{motion_params};

    beluga::LikelihoodFieldProbModelParam sensor_params{100.0, 2.0, 0.5, 0.5, 0.2};
    beluga::LikelihoodFieldProbModel<GridTypeOC> measurement_model(sensor_params, GridTypeOC());

    FastSLAMParams params;
    params.num_particles = this->get_parameter("num_particles").as_int();
    
    publish_trajectory = this->get_parameter("publish_trajectory").as_bool();
    save_grid = this->get_parameter("save_map").as_bool();
    odom_f = this->get_parameter("odom_frame").as_string();
    base_f = this->get_parameter("base_frame").as_string();

    /// FastSLAM instance
    slam_ = std::make_unique<FastSLAM> (motion_model, measurement_model, params);

    RCLCPP_INFO(this->get_logger(), "SLAM setup completed with %zu particles", params.num_particles); 
}

void FastSLAMNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {    
    try {
        auto tf_now = tf_buffer_->lookupTransform(odom_f, base_f, msg->header.stamp, rclcpp::Duration::from_seconds(0.7)); //0.1
        Sophus::SE2d current_odom = tf_to_se2(tf_now.transform);

        if (!first_odom_received_) {
            last_odom_ = current_odom;
            first_odom_received_ = true;
            return;
        }
        
        auto u = std::make_tuple(current_odom, last_odom_); // Control
        last_odom_ = current_odom;
        auto z = laser_to_cartesian(msg); // Measurement
    
        /// FAST SLAM
        auto t0 = std::chrono::high_resolution_clock::now();
        slam_->sample_motion_model(u);
        RCLCPP_INFO(this->get_logger(), "Sample completed");
        auto t1 = std::chrono::high_resolution_clock::now();

        slam_->measurement_model_map(z, best_idx_);
        RCLCPP_INFO(this->get_logger(), "Weights calculated");
        auto t2 = std::chrono::high_resolution_clock::now();

        slam_->update_occupancy_grid(z);
        RCLCPP_INFO(this->get_logger(), "Occupancy grid updated");
        auto t3 = std::chrono::high_resolution_clock::now();

        auto weights = beluga::views::weights(slam_->particles());
        auto max_weight_it = std::max_element(weights.begin(), weights.end());
        best_idx_ = std::distance(weights.begin(), max_weight_it);

        publish_map();
        RCLCPP_INFO(this->get_logger(), "Map published");
        auto t4 = std::chrono::high_resolution_clock::now();

        publish_particles(msg->header.stamp);
        RCLCPP_INFO(this->get_logger(), "Particles published");
        auto t5 = std::chrono::high_resolution_clock::now();

        broadcast_map_to_odom(msg->header.stamp, current_odom);
        RCLCPP_INFO(this->get_logger(), "Map to odom TF published");
        auto t6 = std::chrono::high_resolution_clock::now();

        slam_->resample();
        RCLCPP_INFO(this->get_logger(), "Resample completed");
        auto t7 = std::chrono::high_resolution_clock::now();

        auto d_sample = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        auto d_weight = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto d_map = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto d_pub_map = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        auto d_particles = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
        auto d_tf = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
        auto d_resample = std::chrono::duration_cast<std::chrono::milliseconds>(t7 - t6).count();

        RCLCPP_INFO(this->get_logger(),
            "Times [ms] | sample: %ld | weight: %ld | map: %ld | pub_map: %ld | particles: %ld | tf: %ld | resample: %ld",
            d_sample, d_weight, d_map, d_pub_map, d_particles, d_tf, d_resample);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
}

std::vector<std::pair<double, double>> FastSLAMNode::laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<double, double>> points;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];
        if (std::isfinite(r) && r < 25.0 && r > 0.1) {//msg->range_max && r > msg->range_min) {
            float angle = msg->angle_min + i * msg->angle_increment;
            points.push_back({r * std::cos(angle), r * std::sin(angle)});
        }
    }
    return points;
}

Sophus::SE2d FastSLAMNode::tf_to_se2(const geometry_msgs::msg::Transform& t) {
    double yaw = tf2::getYaw(t.rotation);
    return Sophus::SE2d{Sophus::SO2d{yaw}, Eigen::Vector2d{t.translation.x, t.translation.y}};
}

void FastSLAMNode::publish_map() {
    auto oc_grids = slam_->particles() | beluga::views::elements<3>;
    auto& best_oc_grid = oc_grids[best_idx_];
    
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
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

    /// Best pose estimation and publication
    geometry_msgs::msg::PoseStamped msg2;

    msg2.header.stamp = this->now();
    msg2.header.frame_id = "map";

    auto poses = beluga::views::states(slam_->particles());
    const auto& best_pose = poses[best_idx_];

    msg2.pose.position.x = best_pose.translation().x();
    msg2.pose.position.y = best_pose.translation().y();
    msg2.pose.position.z = 0.0;

    tf2::Quaternion q2;
    q2.setRPY(0, 0, best_pose.so2().log());
    msg2.pose.orientation = tf2::toMsg(q2);

    pose_pub_->publish(msg2);

    /// Publish the trajectory
    if (publish_trajectory){
        auto t = best_pose.translation();
        if (true) {
            trajectory_msg_.header.stamp = this->now();
            trajectory_msg_.poses.push_back(msg2);
            trajectory_pub_->publish(trajectory_msg_);                
            last_saved_pos_ = t;
        }
    }
}

void FastSLAMNode::publish_particles(const rclcpp::Time& stamp) {
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

void FastSLAMNode::broadcast_map_to_odom(const rclcpp::Time& stamp, const Sophus::SE2d& current_odom) {
    auto poses = beluga::views::states(slam_->particles());
    auto best_pose = poses[best_idx_];
    auto map_to_odom = best_pose * current_odom.inverse();
    std::string odom_f = this->get_parameter("odom_frame").as_string();

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

void FastSLAMNode::save_map() {
    if (!save_grid) return;
    auto lo_grids = slam_->particles() | beluga::views::elements<2>;
    auto& best_lo_grid = lo_grids[best_idx_];

    const int width = best_lo_grid.width();
    const int height = best_lo_grid.height();

    cv::Mat map_img(height, width, CV_8UC1);

    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            float lo = best_lo_grid.at(r * width + c);
            
            uint8_t pixel_val;
            if (std::abs(lo) < 0.01f) {
                pixel_val = 127; 
            } else {
                float p = 1.0f / (1.0f + std::exp(-lo));
                pixel_val = static_cast<uint8_t>((1.0f - p) * 255.0f);
            }
            map_img.at<uint8_t>(r, c) = pixel_val;
        }
    }

    cv::imwrite("final_map.png", map_img);
    RCLCPP_WARN(this->get_logger(), "Map saved successfully as PNG.");
}

void FastSLAMNode::save_trajectory() {
    if (!publish_trajectory) return;
    std::ofstream file("trajectory.txt");
    for (const auto& pose_i : trajectory_msg_.poses) {
        double x = pose_i.pose.position.x;
        double y = pose_i.pose.position.y;
        double z = 0.0;
        double qx = pose_i.pose.orientation.x;
        double qy = pose_i.pose.orientation.y;
        double qz = pose_i.pose.orientation.z;
        double qw = pose_i.pose.orientation.w;
        file << std::fixed << std::setprecision(9)
             << pose_i.header.stamp.sec + pose_i.header.stamp.nanosec * 1e-9 << " "
             << x << " " << y << " " << z << " "
             << qx << " " << qy << " " << qz << " " << qw
             << "\n";    
            
    }
    file.close();
}


