#include "gtest/gtest.h"
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>
#include <motion_planning/StateSampler/StateSampler.hpp>

class StateSamplerTest : public ::testing::Test {
    public:
        OccupancyGrid2D<Pose2D> occ;

        StateSamplerTest() {
            double resolution = 0.05;
            Pose2D origin = {.x=0, .y=0, .theta=0};
            occ.setMap("maps/classroom_incomplete.png", resolution, origin);
        }
};

TEST_F(StateSamplerTest, SampleFree) {
    UniformSampler<Pose2D> sampler(&occ);

    for (int i = 0; i < 100; i++) {
        Pose2D state = sampler.sample();
        ASSERT_TRUE(occ.isFree(&state));
        ASSERT_FALSE(occ.isUnknown(&state));
        ASSERT_FALSE(occ.isOccupied(&state));
    }
}

TEST_F(StateSamplerTest, SampleUnknown) {
    UniformSampler<Pose2D> sampler(&occ, false, true);

    for (int i = 0; i < 100; i++) {
        Pose2D state = sampler.sample();
        ASSERT_FALSE(occ.isFree(&state));
        ASSERT_TRUE(occ.isUnknown(&state));
        ASSERT_FALSE(occ.isOccupied(&state));
    }
}

TEST_F(StateSamplerTest, SampleOccupied) {
    UniformSampler<Pose2D> sampler(&occ, false, false, true);

    for (int i = 0; i < 100; i++) {
        Pose2D state = sampler.sample();
        ASSERT_FALSE(occ.isFree(&state));
        ASSERT_FALSE(occ.isUnknown(&state));
        ASSERT_TRUE(occ.isOccupied(&state));
    }
}

TEST_F(StateSamplerTest, SampleMixedPair) {
    UniformSampler<Pose2D> freeSampler(&occ);
    UniformSampler<Pose2D> occSampler(&occ, false, false, true);

    std::vector<StateSampler<Pose2D> *> samplers = {&freeSampler, &occSampler};
    std::vector<double> weights = {1., 2.};
    MixedSampler<Pose2D> mixedSampler(samplers, weights);

    int total = 10000;
    int occCount = 0;
    int freeCount = 0;
    for (int i = 0; i < total; i++) {
        Pose2D state = mixedSampler.sample();
        ASSERT_TRUE(occ.isOccupied(&state) or occ.isFree(&state));
        if (occ.isOccupied(&state)) occCount += 1;
        if (occ.isFree(&state)) freeCount += 1;
    }

    double occProb = occCount/(double) total;
    double freeProb = freeCount/(double) total;

    ASSERT_NEAR(freeProb, 0.333, 0.01);
    ASSERT_NEAR(occProb, 0.666, 0.01);
}

TEST_F(StateSamplerTest, SampleMixedTriple) {
    UniformSampler<Pose2D> freeSampler(&occ);
    UniformSampler<Pose2D> occSampler(&occ, false, false, true);
    UniformSampler<Pose2D> unknownSampler(&occ, false, true, false);

    std::vector<StateSampler<Pose2D> *> samplers = {&freeSampler, &occSampler, &unknownSampler};
    std::vector<double> weights = {1., 2., 3.};
    MixedSampler<Pose2D> mixedSampler(samplers, weights);

    int total = 50000;
    int occCount = 0;
    int freeCount = 0;
    int unknownCount = 0;
    for (int i = 0; i < total; i++) {
        Pose2D state = mixedSampler.sample();
        if (occ.isOccupied(&state)) occCount += 1;
        if (occ.isFree(&state)) freeCount += 1;
        if (occ.isUnknown(&state)) unknownCount += 1;
    }

    double occProb = occCount/(double) total;
    double freeProb = freeCount/(double) total;
    double unknownProb = unknownCount/(double) total;

    ASSERT_NEAR(freeProb, 1./6., 0.01);
    ASSERT_NEAR(occProb, 2./6., 0.01);
    ASSERT_NEAR(unknownProb, 3./6., 0.01);
}

TEST_F(StateSamplerTest, BridgeTest) {
    BridgeSampler sampler(&occ, 1.);

    for (int i = 0; i < 100; i++) {
        Pose2D state = sampler.sample();
        ASSERT_TRUE(occ.isFree(&state));
        ASSERT_FALSE(occ.isUnknown(&state));
        ASSERT_FALSE(occ.isOccupied(&state));
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
