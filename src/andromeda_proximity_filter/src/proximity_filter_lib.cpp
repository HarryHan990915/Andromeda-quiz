#include "andromeda_proximity_filter/proximity_filter_lib.hpp"

#include <cmath>
#include <cstring>

// Constructor

ProximityFilterLib::ProximityFilterLib()
    : max_range_(1.5),
      max_range_sq_(1.5 * 1.5),
      fov_center_rad_(0.0),
      fov_half_width_rad_(M_PI)
{
    // Default hexagon body footprint with orgin of (0,0,0)
    polygon_x_ = { 0.5,  0.25, -0.25, -0.5, -0.25,  0.25 };
    polygon_y_ = { 0.0,  0.433, 0.433, 0.0, -0.433, -0.433 };
}


void ProximityFilterLib::setPolygon(const std::vector<double>& x,
                                   const std::vector<double>& y)
{
    polygon_x_ = x;
    polygon_y_ = y;
}

void ProximityFilterLib::getPolygon(std::vector<double>& x,
                                   std::vector<double>& y) const
{
    x = polygon_x_;
    y = polygon_y_;
}

void ProximityFilterLib::setMaxRange(double range)
{
    max_range_ = range;
    max_range_sq_ = range * range;
}

void ProximityFilterLib::setFov(double center_deg, double width_deg)
{
    fov_center_rad_     = center_deg * M_PI / 180.0;
    fov_half_width_rad_ = (width_deg * M_PI / 180.0) * 0.5;
}


// The core algoirthm, where raycasting algoritm is applied to check if the point is inside the polygon or outside
// Select a point and let it spray a ray in one direction normally to the right. When it hits
// the polygon's edge, the oddness or the eveness of the point would reflect it is inside or outside
bool ProximityFilterLib::rayCasting(double x, double y) const
{
    bool inside = false;
    const size_t n = polygon_x_.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double x1 = polygon_x_[i];
        double y1 = polygon_y_[i];
        double x2 = polygon_x_[j];
        double y2 = polygon_y_[j];

        if (y1 == y2) continue;

        bool crosses = ((y1 > y) != (y2 > y));
        if (!crosses) continue;

        double x_intersect =
            x1 + (x2 - x1) * (y - y1) / (y2 - y1);

        if (x < x_intersect) {
            inside = !inside;
        }
    }

    // Treat edge points are also considered as inside
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;

        double x1 = polygon_x_[i];
        double y1 = polygon_y_[i];
        double x2 = polygon_x_[j];
        double y2 = polygon_y_[j];

        double dx = x - x1;
        double dy = y - y1;

        double dx_edge = x2 - x1;
        double dy_edge = y2 - y1;

        double cross = dx * dy_edge - dy * dx_edge;
        if (std::fabs(cross) < 1e-6) {
            double dot = dx * dx_edge + dy * dy_edge;
            double len_sq = dx_edge * dx_edge + dy_edge * dy_edge;
            if (dot >= 0 && dot <= len_sq) {
                return true;
            }
        }
    }

    return inside;
}

bool ProximityFilterLib::inFov(double x, double y) const
{
    // Calculate the angle of fov
    double angle = std::atan2(y, x);

    double diff = angle - fov_center_rad_;

    // Fix the angle and land it in [-pi, pi]
    if (diff > M_PI) {
        diff -= 2.0 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2.0 * M_PI;
    }

    return std::abs(diff) <= fov_half_width_rad_;
}

// Pure C++ interface with non-ROS for testing
std::vector<size_t>
ProximityFilterLib::filterPointIndices(
    const std::vector<Point3D>& points) const
{
    std::vector<size_t> indices;
    indices.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];

        if ((p.x * p.x + p.y * p.y + p.z * p.z) > max_range_sq_) continue;
        if (!inFov(p.x, p.y)) continue;
        if (rayCasting(p.x, p.y)) continue;

        indices.push_back(i);
    }
    return indices;
}

std::vector<ProximityFilterLib::Point3D>
ProximityFilterLib::filterPoints(
    const std::vector<Point3D>& points) const
{
    std::vector<Point3D> result;
    auto indices = filterPointIndices(points);

    result.reserve(indices.size());
    for (auto idx : indices) {
        result.push_back(points[idx]);
    }
    return result;
}

// Encapsulate with ROS
sensor_msgs::msg::PointCloud2
ProximityFilterLib::filterPointCloud(
    const sensor_msgs::msg::PointCloud2& cloud) const
{
    sensor_msgs::msg::PointCloud2 filtered = cloud;
    filtered.data.clear();
    filtered.width  = 0;
    filtered.height = 1;

    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto& f : cloud.fields) {
        if (f.name == "x") x_off = f.offset;
        if (f.name == "y") y_off = f.offset;
        if (f.name == "z") z_off = f.offset;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) {
        return filtered;
    }

    const size_t point_step = cloud.point_step;
    const size_t num_points = cloud.width * cloud.height;

    std::vector<Point3D> points;
    points.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* ptr = &cloud.data[i * point_step];
        float x = *reinterpret_cast<const float*>(ptr + x_off);
        float y = *reinterpret_cast<const float*>(ptr + y_off);
        float z = *reinterpret_cast<const float*>(ptr + z_off);
        points.push_back({x, y, z});
    }

    auto indices = filterPointIndices(points);

    for (auto idx : indices) {
        const uint8_t* ptr = &cloud.data[idx * point_step];
        filtered.data.insert(filtered.data.end(), ptr, ptr + point_step);
        filtered.width++;
    }

    filtered.row_step = filtered.width * point_step;
    return filtered;
}
