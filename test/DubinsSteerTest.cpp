#include <gtest/gtest.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Steer/DubinsSteer.hpp>

class DubinsSteerTest : public ::testing::Test {};

TEST_F(DubinsSteerTest, InitSteer) {
    double turningRadius = 1.;
    DubinsSteer steer(turningRadius);
}

TEST_F(DubinsSteerTest, CreateSteer) {
    double turningRadius = 1.;
    DubinsSteer steer(turningRadius);

    Pose2D start = {.x=0, .y=0, .theta=0};
    Pose2D end = {.x=1, .y=1, .theta=1};

    steer.steer(&start, &end);
}

TEST_F(DubinsSteerTest, ApproximatelyLinearSteer) {
    double turningRadius = 0.00001;
    DubinsSteer steer(turningRadius);

    // The length of this steer should
    // be approximately 5 (3-4-5 triangle)
    Pose2D start = {.x=0, .y=0, .theta=0};
    Pose2D end = {.x=3, .y=4, .theta=3.14};

    steer.steer(&start, &end);

    ASSERT_NEAR(5., steer.cost(), 0.0001);
}

TEST_F(DubinsSteerTest, HalfCircle) {
    // The optimal path should be a half circle
    // with radius 1
    double turningRadius = 1.;
    Pose2D start = {.x=0, .y=-1, .theta=0};
    Pose2D end = {.x=0, .y=1, .theta=M_PI};

    DubinsSteer steer(turningRadius);

    steer.steer(&start, &end);

    ASSERT_NEAR(M_PI, steer.cost(), 0.00001);

    // Sample the line in intervals of pi/4
    std::vector<Pose2D> samples = steer.sample(steer.cost()/4.);
    ASSERT_EQ(5, samples.size());

    // The midpoint should be (x=1, y=0, theta=pi/2)
    ASSERT_NEAR(1, samples[2].x, 0.0001);
    ASSERT_NEAR(0, samples[2].y, 0.0001);
    ASSERT_NEAR(M_PI/2., samples[2].theta, 0.0001);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
