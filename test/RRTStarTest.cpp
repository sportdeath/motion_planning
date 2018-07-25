#include <functional>

#include "gtest/gtest.h"
#include <motion_planning/RRTStar.hpp>
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>
#include <motion_planning/Steer/DubinsSteer.hpp>
#include <motion_planning/StateSampler/StateSampler.hpp>

class RRTStarTest : public ::testing::Test {
    public:
        DubinsSteer dubinsSteer;
        OccupancyGrid2D<Pose2D> classroom;
        std::function<bool(const Pose2D *)> goalClassroom;
        static constexpr double searchRadius = 3.;

        UniformSampler<Pose2D> sampleFree;

        RRTStarTest() :
            dubinsSteer(1.),
            sampleFree(&classroom)
        {

            double resolution = 0.05;
            Pose2D origin = {.x=0, .y=0, .theta=0};
            classroom.setMap("maps/classroom_incomplete.png", resolution, origin);

            goalClassroom = std::bind(&OccupancyGrid2D<Pose2D>::isUnknown, classroom, std::placeholders::_1);
        }
};

TEST_F(RRTStarTest, InitializeRRTStar) {
    Pose2D start = {.x=0, .y=0, .theta=0};
    RRTStar<Pose2D> rrt(
        &dubinsSteer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);
}

TEST_F(RRTStarTest, RRTStarIterate) {
    Pose2D start = {.x=1, .y=1, .theta=0.5};
    RRTStar<Pose2D> rrt(
        &dubinsSteer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);

    int count = 0;
    for (int i = 0; i < 1000; i++) {
        if (rrt.iterate()) count++;
    }

    EXPECT_GT(count, 0);
}

TEST_F(RRTStarTest, RRTStarSamplePath) {
    Pose2D start = {.x=5, .y=4, .theta=-0.5};
    RRTStar<Pose2D> rrt(
        &dubinsSteer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);


    RRTStar<Pose2D>::Node * lastNode;
    int count = 0;
    for (int i = 0; i < 1000; i++) {
        RRTStar<Pose2D>::Node * node = rrt.iterate();
        if (node == NULL) {
            count++;
        } else {
            lastNode = node;
        }
    }

    ASSERT_GT(count, 0);

    double sampleResolution = 0.1;

    std::vector<Pose2D> path = rrt.samplePath(lastNode, sampleResolution);

    EXPECT_GT(path.size(), 0);

    std::vector<std::vector<Pose2D>> tree = rrt.sampleTree(sampleResolution);
    size_t treeSize = 0;
    for (auto vec = tree.begin(); vec < tree.end(); vec++) { 
        treeSize += (*vec).size();
    }

    EXPECT_GT(treeSize, path.size());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
