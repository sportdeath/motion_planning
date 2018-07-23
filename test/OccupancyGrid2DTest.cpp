#include "gtest/gtest.h"
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>

class OccupancyGrid2DTest : public ::testing::Test {
    public:
        static constexpr const char* tiny = "maps/tiny.png";
        static constexpr const char* classroom_incomplete = "maps/classroom_incomplete.png";
};

TEST_F(OccupancyGrid2DTest, CreateMap) {
    OccupancyGrid2D<Pose2D> occ;
}

TEST_F(OccupancyGrid2DTest, TestOccupancyProbability) {
    OccupancyGrid2D<Pose2D> occ;

    double resolution = 1.;
    Pose2D origin = {.x=0, .y=0, .theta=0};
    occ.setMap(tiny, resolution, origin);

    Pose2D query = {.x=0, .y=0, .theta=0};
    ASSERT_EQ(0, occ.occupancyProbability(&query));
    query.x = 1;
    ASSERT_EQ(1, occ.occupancyProbability(&query));
    query.x = 2;
    ASSERT_NEAR(0.5, occ.occupancyProbability(&query), 0.01);
    query.y = 1;
    ASSERT_EQ(0, occ.occupancyProbability(&query));
    query.x = 1;
    ASSERT_NEAR(0.5, occ.occupancyProbability(&query), 0.01);
    query.x = 0;
    ASSERT_NEAR(0.75, occ.occupancyProbability(&query), 0.01);
    query.y = 2;
    ASSERT_EQ(1, occ.occupancyProbability(&query));
    query.x = 1;
    ASSERT_NEAR(0.25, occ.occupancyProbability(&query), 0.01);
    query.x = 2;
    ASSERT_EQ(1, occ.occupancyProbability(&query));
}

TEST_F(OccupancyGrid2DTest, SampleFree) {
    OccupancyGrid2D<Pose2D> occ;

    double resolution = 1.;
    Pose2D origin = {.x=0, .y=0, .theta=0};
    occ.setMap(classroom_incomplete, resolution, origin);

    for (int i = 0; i < 100; i++) {
        Pose2D state = occ.sampleFree();
        ASSERT_TRUE(occ.isFree(&state));
        ASSERT_FALSE(occ.isUnknown(&state));
        ASSERT_FALSE(occ.isOccupied(&state));
    }
}

TEST_F(OccupancyGrid2DTest, SampleOccupiedPerimeter) {
    OccupancyGrid2D<Pose2D> occ;

    double resolution = 1.;
    Pose2D origin = {.x=0, .y=0, .theta=0};
    occ.setMap(classroom_incomplete, resolution, origin);

    for (int i = 0; i < 100; i++) {
        Pose2D state = occ.samplePerimeter();
        ASSERT_TRUE(occ.isOccupied(&state));
        ASSERT_FALSE(occ.isFree(&state));
        ASSERT_FALSE(occ.isUnknown(&state));
    }
}

TEST_F(OccupancyGrid2DTest, SampleUnknownPerimeter) {
    OccupancyGrid2D<Pose2D> occ;

    double resolution = 1.;
    Pose2D origin = {.x=0, .y=0, .theta=0};
    occ.setMap(classroom_incomplete, resolution, origin);

    for (int i = 0; i < 100; i++) {
        Pose2D state = occ.samplePerimeter(true);
        ASSERT_FALSE(occ.isOccupied(&state));
        ASSERT_FALSE(occ.isFree(&state));
        ASSERT_TRUE(occ.isUnknown(&state));
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
