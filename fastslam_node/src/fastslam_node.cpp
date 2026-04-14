#include <rclcpp/rclcpp.hpp>
#include "fastslam_node/fastslam_oc_grid.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<FastSLAMNode>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}