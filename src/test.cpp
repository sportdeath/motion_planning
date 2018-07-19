#include <iostream>

#include "motion_planning/DubinsSteer.hpp"

int main() {
    Pose2D start = {.x=0, .y=0, .theta=0};
    Pose2D end = {.x=4, .y=4, .theta=3.14};
    double turning_radius = 1.0;
    DubinsSteer steer(start, end, turning_radius);

    double sampleResolution = 0.1;
    std::vector<Pose2D> samples = steer.sample(sampleResolution);

    for (std::vector<Pose2D>::iterator it = samples.begin(); it != samples.end(); it++) {
        Pose2D pose = *it;
        std::cout << "x: " << pose.x << ", y: " << pose.y << ", theta: " << pose.theta << std::endl;
    }

    return 0;
}
