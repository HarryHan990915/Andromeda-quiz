#include "rclcpp/rclcpp.hpp"
#include "andromeda_proximity_filter/proximity_filter_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProximityFilterNode>());
    rclcpp::shutdown();
    return 0;
}
