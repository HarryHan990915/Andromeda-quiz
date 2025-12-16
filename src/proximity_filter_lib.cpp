#include "andromeda_proximity_filter/proximity_filter_lib.hpp"
#include <cmath>

ProximityFilterLib::ProximityFilterLib()
    : max_range_(1.5), max_range_sq_(2.25),
      fov_center_rad_(0.0), fov_half_width_rad_(M_PI) // 180 deg
{
    polygon_x_ = {0.5, 0.25, -0.25, -0.5, -0.25, 0.25};
    polygon_y_ = {0.0, 0.433, 0.433, 0.0, -0.433, -0.433};
}

void ProximityFilterLib::setPolygon(const std::vector<double>& x, const std::vector<double>& y)
{
    polygon_x_ = x;
    polygon_y_ = y;
}

void ProximityFilterLib::setMaxRange(double range)
{
    max_range_ = range;
    max_range_sq_ = range * range;
}

void ProximityFilterLib::setFov(double center_deg, double width_deg)
{
    fov_center_rad_ = center_deg * M_PI / 180.0;
    fov_half_width_rad_ = width_deg * M_PI / 180.0 / 2.0;
}

void ProximityFilterLib::getPolygon(std::vector<double>& x, std::vector<double>& y) const
{
    x = polygon_x_;
    y = polygon_y_;
}

// =========================
// 核心算法
// =========================

bool ProximityFilterLib::pointInPolygonStrict(double x, double y) const
{
    bool inside = false;
    size_t n = polygon_x_.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (polygon_y_[i] == polygon_y_[j]) continue;
        bool intersect =
            ((polygon_y_[i] > y) != (polygon_y_[j] > y)) &&
            (x < (polygon_x_[j] - polygon_x_[i]) * (y - polygon_y_[i]) /
                     (polygon_y_[j] - polygon_y_[i]) +
                 polygon_x_[i]);
        if (intersect) inside = !inside;
    }

    for (size_t i = 0; i < n; ++i) {
        double dx = x - polygon_x_[i];
        double dy = y - polygon_y_[i];
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) return false;
        size_t j = (i + 1) % n;
        double dx1 = polygon_x_[j] - polygon_x_[i];
        double dy1 = polygon_y_[j] - polygon_y_[i];
        double cross = dx * dy1 - dy * dx1;
        if (std::abs(cross) < 1e-6) {
            double dot = dx * dx1 + dy * dy1;
            double len_sq = dx1 * dx1 + dy1 * dy1;
            if (dot >= 0 && dot <= len_sq) return false;
        }
    }
    return inside;
}

bool ProximityFilterLib::inFov(double x, double y) const
{
    double angle = std::atan2(y, x);
    double delta = angle - fov_center_rad_;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    return std::fabs(delta) <= fov_half_width_rad_;
}

sensor_msgs::msg::PointCloud2 ProximityFilterLib::filterPointCloud(const sensor_msgs::msg::PointCloud2& cloud) const
{
    sensor_msgs::msg::PointCloud2 filtered = cloud;
    filtered.data.clear();
    filtered.width = 0;
    filtered.height = 1;

    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : cloud.fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
        if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) return filtered;

    const size_t point_step = cloud.point_step;
    const size_t num_points = cloud.width * cloud.height;

    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* ptr = &cloud.data[i * point_step];
        float x = *reinterpret_cast<const float*>(ptr + x_offset);
        float y = *reinterpret_cast<const float*>(ptr + y_offset);
        float z = *reinterpret_cast<const float*>(ptr + z_offset);

        if ((x*x + y*y + z*z) > max_range_sq_) continue;
        if (!inFov(x, y)) continue;
        if (pointInPolygonStrict(x, y)) continue;

        filtered.data.insert(filtered.data.end(), ptr, ptr + point_step);
        filtered.width++;
    }
    filtered.row_step = filtered.width * point_step;
    return filtered;
}
