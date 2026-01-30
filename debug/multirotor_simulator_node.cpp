#include <rclcpp/rclcpp.hpp>
#include "multirotor_simulator.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<mrs_multirotor_simulator::MultirotorSimulator>(options);

    // IMPORTANT: mrs_lib::Node is NOT an rclcpp::Node
    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}