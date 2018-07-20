#include "gtest/gtest.h"
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>

class OccupancyGrid2DTest : public ::testing::Test {
    public:
        static constexpr const char* incomplete_map = "/Users/tfh/Documents/motion_planning/test_map.png";
};

TEST_F(OccupancyGrid2DTest, CreateMap) {
    double resolution = 0.01;
    Pose2D origin = {.x=0, .y=0, .theta=0};

    OccupancyGrid2D<Pose2D> og(incomplete_map, resolution, origin);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
