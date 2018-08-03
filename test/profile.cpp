#include <iostream>
#include <functional>

#include <motion_planning/RRTStar.hpp>
#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/Occupancy/OccupancyGrid2D.hpp>
#include <motion_planning/Steer/ReedsSheppSteer.hpp>
#include <motion_planning/StateSampler/StateSampler.hpp>

int main() {
    ReedsSheppSteer steer(0.3);

    OccupancyGrid2D<Pose2D> classroom;
    double resolution = 0.05;
    Pose2D origin = {.x=0, .y=0, .theta=0};
    classroom.setMap("maps/classroom_incomplete.png", resolution, origin);

    Pose2D start = {.x=1, .y=1, .theta=0};
    double searchRadius = 5.;

    UniformSampler<Pose2D> sampleFree(&classroom);

    RRTStar<Pose2D> rrt(
        &steer,
        &classroom,
        sampleFree.sampleFunction(),
        start,
        searchRadius);

    int count = 0;
    for (int i = 0; i < 500; i++) {
        if (rrt.iterate()) count++;
    }
    std::cout << "Finished" << std::endl;

    return 0;
}
