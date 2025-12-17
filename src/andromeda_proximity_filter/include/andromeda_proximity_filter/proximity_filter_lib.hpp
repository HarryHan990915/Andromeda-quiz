#pragma once

#include <vector>
#include <cstddef>

#include <sensor_msgs/msg/point_cloud2.hpp>

class ProximityFilterLib
{
public:

    // Self-designed pointcloud struct instead of using Ros message,
    // So that non-ROS google testing can be used to check the method works or not
    struct Point3D
    {
        double x;
        double y;
        double z;
    };

    ProximityFilterLib();


    void setPolygon(const std::vector<double>& x,
                    const std::vector<double>& y);

    void getPolygon(std::vector<double>& x,
                    std::vector<double>& y) const;

    void setMaxRange(double range);

    // center_deg: FOV center angle (deg)
    // width_deg : total FOV width (deg)
    void setFov(double center_deg, double width_deg);

    std::vector<Point3D>
    filterPoints(const std::vector<Point3D>& points) const;

    sensor_msgs::msg::PointCloud2
    filterPointCloud(const sensor_msgs::msg::PointCloud2& cloud) const;

private:

    bool rayCasting(double x, double y) const;
    bool inFov(double x, double y) const;

    std::vector<size_t>
    filterPointIndices(const std::vector<Point3D>& points) const;

    std::vector<double> polygon_x_;
    std::vector<double> polygon_y_;

    double max_range_;
    double max_range_sq_;

    double fov_center_rad_;
    double fov_half_width_rad_;
};
