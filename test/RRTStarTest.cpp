#include <functional>

#include "gtest/gtest.h"
#include <motion_planning/RRTStar.hpp>
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>
#include <motion_planning/Steer/ReedsSheppSteer.hpp>
#include <motion_planning/StateSampler/StateSampler.hpp>

class RRTStarTest : public ::testing::Test {
    public:
        ReedsSheppSteer steer;
        OccupancyGrid2D<Pose2D> classroom;
        std::function<bool(const Pose2D *)> goalClassroom;
        static constexpr double searchRadius = 3.;

        UniformSampler<Pose2D> sampleFree;

        RRTStarTest() :
            steer(1.),
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
        &steer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);
}

TEST_F(RRTStarTest, RRTStarIterate) {
    Pose2D start = {.x=1, .y=1, .theta=0.5};
    RRTStar<Pose2D> rrt(
        &steer, 
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
        &steer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);


    RRTStar<Pose2D>::Node * lastNode = NULL;
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
    std::vector<std::vector<Pose2D>> tree = rrt.sampleTree(sampleResolution);
    size_t treeSize = 0;
    for (auto vec = tree.begin(); vec < tree.end(); vec++) { 
        treeSize += (*vec).size();
    }


    if (lastNode != NULL) {
        std::vector<Pose2D> path = rrt.samplePath(lastNode, sampleResolution);

        EXPECT_GT(path.size(), 0);
        EXPECT_GT(treeSize, path.size());
    }
}

TEST_F(RRTStarTest, RRTStarCostConsistency) {

    // Create an RRT
    Pose2D start = {.x=5, .y=4, .theta=-0.5};
    RRTStar<Pose2D> rrt(
        &steer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);


    // Iterate
    for (int i = 0; i < 1000; i++) {
        rrt.iterate();
    }

    // For each node
    const RRTStar<Pose2D>::Node * n;
    for (const RRTStar<Pose2D>::Node & node : rrt.getNodes()) {
        double cost = 0;

        n = &node;

        // Accumulate the total cost
        while (n -> parent != NULL) {
            bool validSteer = steer.steer(&n -> parent -> state, &n -> state);

            if (validSteer) {
                cost += steer.cost();
            }

            n = n -> parent;
        }

        ASSERT_NEAR(cost, node.cost, 0.00001);
    }
}

TEST_F(RRTStarTest, RRTStarParentConsistency) {

    // Create an RRT
    Pose2D start = {.x=5, .y=4, .theta=-0.5};
    RRTStar<Pose2D> rrt(
        &steer, 
        &classroom, 
        sampleFree.sampleFunction(),
        goalClassroom,
        start,
        searchRadius);

    // Iterate
    for (int i = 0; i < 1000; i++) {
        rrt.iterate();
    }

    // For each node
    int nullParents = 0;
    for (const RRTStar<Pose2D>::Node & node : rrt.getNodes()) {
        if (node.parent == NULL) {
            nullParents += 1;
            continue;
        }

        // Make sure the node is in it's parents children
        std::vector<RRTStar<Pose2D>::Node *> parentChildren = node.parent -> children;
        ASSERT_NE(std::find(parentChildren.begin(), parentChildren.end(), &node), parentChildren.end());

        // For every child make sure the child points to the parent
        for (RRTStar<Pose2D>::Node * child : node.children) {
            ASSERT_EQ(&node, child -> parent);
        }
    }

    // Make sure only the root has a null parent
    ASSERT_EQ(nullParents, 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
