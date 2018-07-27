#include <gtest/gtest.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Steer/ReedsSheppSteer.hpp>

class ReedsSheppSteerTest : public ::testing::Test {};

TEST_F(ReedsSheppSteerTest, InitSteer) {
    double turningRadius = 1.;
    ReedsSheppSteer steer(turningRadius);
}

TEST_F(ReedsSheppSteerTest, CreateSteer) {
    double turningRadius = 1.;
    ReedsSheppSteer steer(turningRadius);

    Pose2D start = {.x=0, .y=0, .theta=0};
    Pose2D end = {.x=1, .y=1, .theta=1};

    steer.steer(&start, &end);
}

TEST_F(ReedsSheppSteerTest, ApproximatelyLinearSteer) {
    double turningRadius = 0.00001;
    ReedsSheppSteer steer(turningRadius);

    // The length of this steer should
    // be approximately 5 (3-4-5 triangle)
    Pose2D start = {.x=0, .y=0, .theta=0};
    Pose2D end = {.x=3, .y=4, .theta=3.14};

    steer.steer(&start, &end);

    ASSERT_NEAR(5., steer.cost(), 0.0001);
}

TEST_F(ReedsSheppSteerTest, HalfCircle) {
    // The optimal path should be a half circle
    // with radius 1
    double turningRadius = 1.;
    Pose2D start = {.x=0, .y=-1, .theta=0};
    Pose2D end = {.x=0, .y=1, .theta=M_PI};

    ReedsSheppSteer steer(turningRadius);

    steer.steer(&start, &end);

    ASSERT_NEAR(M_PI, steer.cost(), 0.00001);

    // Sample the line in intervals of pi/4
    std::vector<Pose2D> samples = steer.sample(steer.cost()/4.);
    ASSERT_EQ(5, samples.size());

    // The midpoint should be (x=1, y=0, theta=pi/2)
    ASSERT_NEAR(1, samples[2].x, 0.0001);
    ASSERT_NEAR(0, samples[2].y, 0.0001);
    ASSERT_NEAR(M_PI/2., samples[2].theta, 0.0001);

    ASSERT_NEAR(end.x, samples[4].x, 0.0001);
    ASSERT_NEAR(end.y, samples[4].y, 0.0001);
    ASSERT_NEAR(end.theta, samples[4].theta, 0.0001);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
