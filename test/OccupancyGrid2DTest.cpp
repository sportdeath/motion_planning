#include "gtest/gtest.h"
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>

class OccupancyGrid2DTest : public ::testing::Test {
    public:
        static constexpr const char* classroom_incomplete = "maps/classroom_incomplete.png";
};

TEST_F(OccupancyGrid2DTest, CreateMap) {
    double resolution = 0.1;
    Pose2D origin = {.x=0, .y=0, .theta=0};

    OccupancyGrid2D<Pose2D> map(classroom_incomplete, resolution, origin);
}

TEST_F(OccupancyGrid2DTest, TestOccupancy) {
    double resolution = 0.1;
    Pose2D origin = {.x=0, .y=0, .theta=0};

    OccupancyGrid2D<Pose2D> map(classroom_incomplete, resolution, origin);

    Pose2D query = {.x=0, .y=0, .theta=0};
    map.occupancyProbability(&query);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
