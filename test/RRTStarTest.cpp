#include <functional>

#include "gtest/gtest.h"
#include <motion_planning/RRTStar.hpp>
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>
#include <motion_planning/Steer/DubinsSteer.hpp>

class RRTStarTest : public ::testing::Test {
    public:
        DubinsSteer dubinsSteer;
        OccupancyGrid2D<Pose2D> classroom;
        std::function<Pose2D(void)> sampleClassroom;
        std::function<bool(const Pose2D *)> goalClassroom;
        static constexpr double searchRadius = 3.;

        RRTStarTest() 
            : dubinsSteer(1.)
        {

            double resolution = 0.05;
            Pose2D origin = {.x=0, .y=0, .theta=0};
            classroom.setMap("maps/classroom_incomplete.png", resolution, origin);

            sampleClassroom = std::bind(&OccupancyGrid2D<Pose2D>::sampleFree, classroom);
            goalClassroom = std::bind(&OccupancyGrid2D<Pose2D>::isUnknown, classroom, std::placeholders::_1);
        }
};

TEST_F(RRTStarTest, InitializeRRTStar) {
    Pose2D start = {.x=0, .y=0, .theta=0};
    RRTStar<Pose2D> rrt(
        &dubinsSteer, 
        &classroom, 
        sampleClassroom,
        goalClassroom,
        start,
        searchRadius);
}

TEST_F(RRTStarTest, RRTStarIterate) {
    Pose2D start = {.x=1, .y=1, .theta=0.5};
    RRTStar<Pose2D> rrt(
        &dubinsSteer, 
        &classroom, 
        sampleClassroom,
        goalClassroom,
        start,
        searchRadius);

    int count = 0;
    for (int i = 0; i < 1000; i++) {
        if (rrt.iterate()) count++;
    }

    EXPECT_GT(count, 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
