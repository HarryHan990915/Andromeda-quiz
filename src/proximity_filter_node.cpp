#include "andromeda_proximity_filter/proximity_filter_node.hpp"

ProximityFilterNode::ProximityFilterNode()
    : Node("proximity_filter")
{
    // Parameters
    this->declare_parameter<std::vector<double>>("polygon_x", {0.5,0.25,-0.25,-0.5,-0.25,0.25});
    this->declare_parameter<std::vector<double>>("polygon_y", {0.0,0.433,0.433,0.0,-0.433,-0.433});
    this->declare_parameter<double>("max_range", 1.5);
    this->declare_parameter<double>("fov_center_deg", 0.0);
    this->declare_parameter<double>("fov_width_deg", 180.0);
    this->declare_parameter<std::string>("input_topic", "/points/raw");
    this->declare_parameter<std::string>("output_topic", "/points/filtered");
    this->declare_parameter<std::string>("polygon_topic", "/points/filter_polygon");

    rcl_interfaces::msg::ParameterDescriptor rate_desc;
    rate_desc.description = "Processing frequency in Hz";
    this->declare_parameter<int>("subscription_rate", 20, rate_desc);

    // Subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("input_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&ProximityFilterNode::pointcloudCallback, this, std::placeholders::_1));

    // Publishers
    filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("output_topic").as_string(), rclcpp::QoS(10).reliable());

    polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        this->get_parameter("polygon_topic").as_string(), rclcpp::QoS(1).transient_local());

    last_processed_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "ProximityFilterNode started");
}

void ProximityFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Throttle
    int rate_hz = this->get_parameter("subscription_rate").as_int();
    double min_dt = 1.0 / rate_hz;
    rclcpp::Time now = this->now();
    if ((now - last_processed_time_).seconds() < min_dt) return;
    last_processed_time_ = now;

    // Update lib parameters
    filter_lib_.setPolygon(this->get_parameter("polygon_x").as_double_array(),
                           this->get_parameter("polygon_y").as_double_array());
    filter_lib_.setMaxRange(this->get_parameter("max_range").as_double());
    filter_lib_.setFov(this->get_parameter("fov_center_deg").as_double(),
                       this->get_parameter("fov_width_deg").as_double());

    // Publish polygon for visualization
    std::vector<double> poly_x, poly_y;
    filter_lib_.getPolygon(poly_x, poly_y);
    geometry_msgs::msg::PolygonStamped poly_msg;
    poly_msg.header = msg->header;
    for (size_t i=0;i<poly_x.size();++i){
        geometry_msgs::msg::Point32 p;
        p.x = poly_x[i]; p.y = poly_y[i]; p.z = 0.0f;
        poly_msg.polygon.points.push_back(p);
    }
    polygon_pub_->publish(poly_msg);

    // Filter point cloud
    sensor_msgs::msg::PointCloud2 filtered = filter_lib_.filterPointCloud(*msg);
    filtered_pub_->publish(filtered);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Filtered cloud: %zu -> %u points",
                          msg->width*msg->height, filtered.width);
}
