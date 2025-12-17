#include <gtest/gtest.h>
#include "andromeda_proximity_filter/proximity_filter_lib.hpp"

TEST(ProximityFilterLibTest, SimplePolygon)
{
    ProximityFilterLib filter;

    // square: (0,0) -> (1,1)
    filter.setPolygon({0, 1, 1, 0}, {0, 0, 1, 1});
    filter.setMaxRange(2.0);
    filter.setFov(0.0, 180.0);

    std::vector<ProximityFilterLib::Point3D> points = {
        {0.5,  0.5, 0.0},  // inside polygon 
        {1.5,  0.5, 0.0},  // outside polygon
        {0.2, -0.2, 0.0},  // outside polygon 
        {2.0,  0.0, 0.0}   // on range boundary
    };

    auto filtered = filter.filterPoints(points);

    ASSERT_EQ(filtered.size(), 3);

    EXPECT_DOUBLE_EQ(filtered[0].x, 1.5);
    EXPECT_DOUBLE_EQ(filtered[1].x, 0.2);
    EXPECT_DOUBLE_EQ(filtered[2].x, 2.0);
}
