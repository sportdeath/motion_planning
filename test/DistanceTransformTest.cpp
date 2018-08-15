#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "motion_planning/Occupancy/DistanceTransform.hpp"

class DistanceTransformTest : public ::testing::Test {
public:
    double inf = 999999;
};

TEST_F(DistanceTransformTest, 1DSquaredSimple) {
    
    std::vector<double> input = {0, inf, inf};

    std::vector<double> output(input.size());

    DistanceTransform(input.size()).distanceSquared1D(input, output);

    ASSERT_EQ(output[0], 0);
    ASSERT_EQ(output[1], 1);
    ASSERT_EQ(output[2], 4);
}

TEST_F(DistanceTransformTest, 1DSquaredSimpleRev) {
    
    std::vector<double> input = {inf, inf, 0};

    std::vector<double> output(input.size());

    DistanceTransform(input.size()).distanceSquared1D(input, output);

    ASSERT_EQ(output[0], 4);
    ASSERT_EQ(output[1], 1);
    ASSERT_EQ(output[2], 0);
}

TEST_F(DistanceTransformTest, 1DSquared) {
    std::vector<double> input = {0, inf, inf, inf, 0, inf, 0, inf, inf, inf, inf};

    std::vector<double> output(input.size());

    DistanceTransform(input.size()).distanceSquared1D(input, output);

    ASSERT_EQ(output[0], 0);
    ASSERT_EQ(output[1], 1);
    ASSERT_EQ(output[2], 4);
    ASSERT_EQ(output[3], 1);
    ASSERT_EQ(output[4], 0);
    ASSERT_EQ(output[5], 1);
    ASSERT_EQ(output[6], 0);
    ASSERT_EQ(output[7], 1);
    ASSERT_EQ(output[8], 4);
    ASSERT_EQ(output[9], 9);
    ASSERT_EQ(output[10], 16);
}

TEST_F(DistanceTransformTest, 2DSquaredSimple) {
    std::vector<double> input = {
          0, inf, inf, inf, 
        inf, inf, inf, inf, 
        inf, inf, inf, inf
    };

    int width = 4;
    int height = 3;

    DistanceTransform dt(std::max(width, height));
    dt.distanceSquared2D(input, width, height);

    ASSERT_EQ(input[0], 0);
    ASSERT_EQ(input[1], 1);
    ASSERT_EQ(input[2], 4);
    ASSERT_EQ(input[3], 9);

    ASSERT_EQ(input[4], 1);
    ASSERT_EQ(input[5], 2);
    ASSERT_EQ(input[6], 5);
    ASSERT_EQ(input[7], 10);

    ASSERT_EQ(input[8], 4);
    ASSERT_EQ(input[9], 5);
    ASSERT_EQ(input[10], 8);
    ASSERT_EQ(input[11], 13);
}

TEST_F(DistanceTransformTest, 2DSquared) {
    std::vector<double> input = {
        inf, inf, inf, inf, inf,
        inf, inf,   0, inf, inf,
        inf, inf, inf, inf, inf,
        inf,   0, inf, inf,   0
    };

    int width = 5;
    int height = 4;

    DistanceTransform dt(std::max(width, height));
    dt.distanceSquared2D(input, width, height);

    ASSERT_EQ(input[0], 5);
    ASSERT_EQ(input[1], 2);
    ASSERT_EQ(input[2], 1);
    ASSERT_EQ(input[3], 2);
    ASSERT_EQ(input[4], 5);

    ASSERT_EQ(input[5], 4);
    ASSERT_EQ(input[6], 1);
    ASSERT_EQ(input[7], 0);
    ASSERT_EQ(input[8], 1);
    ASSERT_EQ(input[9], 4);

    ASSERT_EQ(input[10], 2);
    ASSERT_EQ(input[11], 1);
    ASSERT_EQ(input[12], 1);
    ASSERT_EQ(input[13], 2);
    ASSERT_EQ(input[14], 1);

    ASSERT_EQ(input[15], 1);
    ASSERT_EQ(input[16], 0);
    ASSERT_EQ(input[17], 1);
    ASSERT_EQ(input[18], 1);
    ASSERT_EQ(input[19], 0);
}

TEST_F(DistanceTransformTest, 2D) {
    std::vector<double> input = {
        inf, inf, inf, inf, 
        inf, inf,   0, inf, 
        inf, inf, inf, inf,
        inf,   0, inf, inf
    };

    int width = 4;
    int height = 4;

    DistanceTransform dt(std::max(width, height));
    dt.distance2D(input, width, height);

    ASSERT_EQ(input[0], sqrt(5));
    ASSERT_EQ(input[1], sqrt(2));
    ASSERT_EQ(input[2], 1);
    ASSERT_EQ(input[3], sqrt(2));

    ASSERT_EQ(input[4], 2);
    ASSERT_EQ(input[5], 1);
    ASSERT_EQ(input[6], 0);
    ASSERT_EQ(input[7], 1);

    ASSERT_EQ(input[8], sqrt(2));
    ASSERT_EQ(input[9], 1);
    ASSERT_EQ(input[10], 1);
    ASSERT_EQ(input[11], sqrt(2));

    ASSERT_EQ(input[12], 1);
    ASSERT_EQ(input[13], 0);
    ASSERT_EQ(input[14], 1);
    ASSERT_EQ(input[15], 2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
