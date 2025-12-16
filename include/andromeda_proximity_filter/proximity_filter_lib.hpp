#pragma once
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ProximityFilterLib
{
public:
    ProximityFilterLib();
    ~ProximityFilterLib() = default;

    // 设置参数
    void setPolygon(const std::vector<double>& x, const std::vector<double>& y);
    void setMaxRange(double range);
    void setFov(double center_deg, double width_deg);

    sensor_msgs::msg::PointCloud2 filterPointCloud(const sensor_msgs::msg::PointCloud2& cloud) const;

    void getPolygon(std::vector<double>& x, std::vector<double>& y) const;

private:
    bool pointInPolygonStrict(double x, double y) const;
    bool inFov(double x, double y) const;

private:
    std::vector<double> polygon_x_;
    std::vector<double> polygon_y_;
    double max_range_;
    double max_range_sq_;
    double fov_center_rad_;
    double fov_half_width_rad_;
};
