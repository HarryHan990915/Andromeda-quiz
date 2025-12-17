#include "andromeda_proximity_filter/proximity_filter_lib.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>
#include <string>

class ProximityFilterNode : public rclcpp::Node
{
public:
    ProximityFilterNode();

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    ProximityFilterLib filter_lib_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;

    rclcpp::Time last_processed_time_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;
};

