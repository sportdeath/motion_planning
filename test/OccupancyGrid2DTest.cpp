#include "gtest/gtest.h"
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>

class OccupancyGrid2DTest : public ::testing::Test {
    public:
        static constexpr const char* tiny = "maps/tiny.png";
        static constexpr const char* classroom_incomplete = "maps/classroom_incomplete.png";
};

TEST_F(OccupancyGrid2DTest, CreateMap) {
    double resolution = 0.1;
    Pose2D origin = {.x=0, .y=0, .theta=0};

    OccupancyGrid2D<Pose2D> map(classroom_incomplete, resolution, origin);
}

TEST_F(OccupancyGrid2DTest, TestOccupancyProbability) {
    double resolution = 1.;
    Pose2D origin = {.x=0, .y=0, .theta=0};

    OccupancyGrid2D<Pose2D> map(tiny, resolution, origin);

    Pose2D query = {.x=0, .y=0, .theta=0};
    ASSERT_EQ(0, map.occupancyProbability(&query));
    query.x = 1;
    ASSERT_EQ(1, map.occupancyProbability(&query));
    query.x = 2;
    ASSERT_NEAR(0.5, map.occupancyProbability(&query), 0.01);
    query.y = 1;
    ASSERT_EQ(0, map.occupancyProbability(&query));
    query.x = 1;
    ASSERT_NEAR(0.5, map.occupancyProbability(&query), 0.01);
    query.x = 0;
    ASSERT_NEAR(0.75, map.occupancyProbability(&query), 0.01);
    query.y = 2;
    ASSERT_EQ(1, map.occupancyProbability(&query));
    query.x = 1;
    ASSERT_NEAR(0.25, map.occupancyProbability(&query), 0.01);
    query.x = 2;
    ASSERT_EQ(1, map.occupancyProbability(&query));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
