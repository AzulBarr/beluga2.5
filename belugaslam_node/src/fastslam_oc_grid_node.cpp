#include "belugaslam_node/fastslam_oc_grid_node.hpp"
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
    this->declare_parameter("min_update_distance", 0.1);
    this->declare_parameter("min_update_angle", 0.1);
    this->declare_parameter("uncertainty_map_publish_interval", 10);
    this->declare_parameter("alpha1", 0.1);
    this->declare_parameter("alpha2", 0.05);
    this->declare_parameter("alpha3", 0.1);
    this->declare_parameter("alpha4", 0.05);
    this->declare_parameter("alpha5", 0.1);
    this->declare_parameter("likelihood_scaling_factor", 0.05);

    setup_slam();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&BelugaSLAMNode::laser_callback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
    particle_cloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particle_cloud", 10);
    entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("/localization_entropy", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/best_pose", 10);
    uncertainty_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_uncertainty", 1);
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    trajectory_msg_.header.frame_id = "map";
    
    RCLCPP_INFO(this->get_logger(), "BelugaSLAM Node initialized and waiting for data...");
}

void BelugaSLAMNode::setup_slam() {
    double a1 = get_parameter("alpha1").as_double();
    double a2 = get_parameter("alpha2").as_double();
    double a3 = get_parameter("alpha3").as_double();
    double a4 = get_parameter("alpha4").as_double();
    double a5 = get_parameter("alpha5").as_double();
    beluga::DifferentialDriveModelParam motion_params{a1, a2, a3, a4, a5};
    beluga::DifferentialDriveModel<state_type> motion_model{motion_params};

    beluga::LikelihoodFieldProbModelParam sensor_params{100.0, 2.0, 0.5, 0.5, 0.2, true};
    beluga::LikelihoodFieldProbModel<GridTypeOC> measurement_model(sensor_params, GridTypeOC());

    auto params = FastSLAMParams{};
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
    min_update_angle = get_parameter("min_update_angle").as_double();
    min_update_distance = get_parameter("min_update_distance").as_double();
    uncertainty_map_publish_interval = get_parameter("uncertainty_map_publish_interval").as_int();

    /// BelugaSLAM instance
    slam_ = std::make_unique<BelugaSLAM> (motion_model, measurement_model, params);

    RCLCPP_INFO(this->get_logger(), "SLAM setup completed with %zu - %zu particles", params.min_particles, params.max_particles); 
}

void BelugaSLAMNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {    
    try {
        auto tf_now = tf_buffer_->lookupTransform(odom_f, base_f, msg->header.stamp, rclcpp::Duration::from_seconds(0.7));
        Sophus::SE2d current_odom = tf_to_se2(tf_now.transform);

        if (!first_odom_received_) {
            last_odom_ = current_odom;
            first_odom_received_ = true;
            return;
        }
        distance = (current_odom.translation() - last_odom_.translation()).norm();
        angle_diff = (last_odom_.so2().inverse() * current_odom.so2()).log();
        angle_diff = std::abs(angle_diff);

        if (distance < min_update_distance && angle_diff < min_update_angle) {
            publish_best_pose(msg->header.stamp);
            publish_map();
            return;
        }
        auto u = std::make_tuple(current_odom, last_odom_); // Control
        last_odom_ = current_odom;
        auto z = laser_to_cartesian(msg); // Measurement
    
        /// FAST SLAM
        const auto update_start_time = std::chrono::high_resolution_clock::now();
        auto t0 = std::chrono::high_resolution_clock::now();
        slam_->sample_motion_model(u);
        RCLCPP_INFO(this->get_logger(), "Sample completed");
        auto t1 = std::chrono::high_resolution_clock::now();

        slam_->measurement_model_map(z);
        RCLCPP_INFO(this->get_logger(), "Weights calculated");
        auto t2 = std::chrono::high_resolution_clock::now();

        auto finished_events = slam_->update_occupancy_grid(z);
        RCLCPP_INFO(this->get_logger(), "Occupancy grid updated");

        slam_->post_update(z, finished_events);
        RCLCPP_INFO(this->get_logger(), "Post update completed");
        auto t3 = std::chrono::high_resolution_clock::now();

        slam_->resample();
        RCLCPP_INFO(this->get_logger(), "Resample completed");
        auto t4 = std::chrono::high_resolution_clock::now();

        compute_se2_covariance();

        publish_particles(msg->header.stamp);
        RCLCPP_INFO(this->get_logger(), "Particles published");
        auto t5 = std::chrono::high_resolution_clock::now();
        
        compute_entropy();
        publish_best_pose(msg->header.stamp);
        RCLCPP_INFO(this->get_logger(), "Best pose published");

        publish_map();
        if (it % uncertainty_map_publish_interval == 0) {
            publish_uncertainty_map();
        }
        it ++;
        RCLCPP_INFO(this->get_logger(), "Map published");
        auto t6 = std::chrono::high_resolution_clock::now();

        broadcast_map_to_odom(msg->header.stamp, current_odom);
        RCLCPP_INFO(this->get_logger(), "Map to odom TF published");
        auto t7 = std::chrono::high_resolution_clock::now();

        const auto update_stop_time = std::chrono::high_resolution_clock::now();
        const auto update_duration = update_stop_time - update_start_time;

        auto d_sample = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        auto d_weight = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        auto d_resample = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        auto d_map = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
        auto d_pub_map = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
        auto d_particles = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
        auto d_tf = std::chrono::duration_cast<std::chrono::milliseconds>(t7 - t6).count();

        RCLCPP_INFO(this->get_logger(),
            "Times [ms] | sample: %ld | weight: %ld | resample: %ld | map: %ld | pub_map: %ld | particles: %ld | tf: %ld",
            d_sample, d_weight, d_resample, d_map, d_pub_map, d_particles, d_tf);

        RCLCPP_INFO(
            get_logger(), "Particle filter update iteration stats: %ld particles | %ld active hypotheses | %ld submaps | %.3fms",
            slam_->particles().size(),
            slam_->get_active_hypotheses_count(),
            slam_->get_submaps_count(),
            std::chrono::duration<double, std::milli>(update_duration).count());

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
}

std::vector<std::pair<double, double>> BelugaSLAMNode::laser_to_cartesian(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<double, double>> points;
    auto tf_laser = tf_buffer_->lookupTransform(
        base_f,                      // target (base_link)
        msg->header.frame_id,        // source (laser)
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1)
    );

    Sophus::SE2d T_bl_laser = tf_to_se2(tf_laser.transform);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float r = msg->ranges[i];

        if (std::isfinite(r) && r < range_max && r > 0.1) {
            float angle = msg->angle_min + i * msg->angle_increment;

            Eigen::Vector2d p_laser(
                r * std::cos(angle),
                r * std::sin(angle)
            );

            Eigen::Vector2d p_base = T_bl_laser * p_laser;

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
    auto best_oc_grid = slam_->best_occupancy_grid();

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

        trajectory_pub_->publish(trajectory_msg_);
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
    auto poses = beluga::views::states(slam_->particles());
    auto weights = beluga::views::weights(slam_->particles());

    const Sophus::Vector4<double> mean_vector = beluga::mean(poses, weights, [](const auto& value) {
      return Eigen::Map<const Sophus::Vector4<double>>{value.data()};
    });

    auto mean = Sophus::SE2<double>{Eigen::Map<const Sophus::SE2<double>>{mean_vector.data()}};

    // Clear the full matrix to avoid uninitialized memory in cross-covariances
    covariance_.setZero();

    // Compute the covariance of the translation part.
    covariance_.template topLeftCorner<2, 2>() = beluga::covariance(
        poses, weights, mean.translation(), [](const auto& value) { return value.translation(); });
    
    // Symmetrize just in case of floating point inaccuracies
    covariance_(0, 1) = (covariance_(0, 1) + covariance_(1, 0)) / 2.0;
    covariance_(1, 0) = covariance_(0, 1);

    // Compute the orientation variance and re-normalize the rotation component (after using the non-normal result).
    if (mean.so2().unit_complex().norm() < std::numeric_limits<double>::epsilon()) {
      // Handle the case where both averages are too close to zero.
      // Return infinite variance.
      covariance_.coeffRef(2, 2) = std::numeric_limits<double>::infinity();
    } else {
      // See circular standard deviation in
      // https://en.wikipedia.org/wiki/Directional_statistics#Dispersion.
      // 2*Var 
      covariance_.coeffRef(2, 2) = -2.0 * std::log(mean.so2().unit_complex().norm());
    }
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
    auto best_lo_grid = slam_->best_log_odds_grid();

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
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
