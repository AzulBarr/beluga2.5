#include <rclcpp/rclcpp.hpp>
#include "belugaslam_node/fastslam_oc_grid_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<BelugaSLAMNode>();
    rclcpp::spin(node); 

    rclcpp::shutdown();
    return 0;
}