#include <iostream>

#include "motion_planning/DubinsSteer.hpp"
#include "motion_planning/OccupancyGrid2D.hpp"

int main() {
    double turning_radius = 1.0;
    DubinsSteer steer(turning_radius);

    Pose2D start = {.x=0, .y=0, .theta=0};
    Pose2D end = {.x=4, .y=4, .theta=3.14};
    bool isValid = steer.steer(start, end);

    if (isValid) {
        double sampleResolution = 0.1;
        std::vector<Pose2D> samples = steer.sample(sampleResolution);

        for (std::vector<Pose2D>::iterator it = samples.begin(); it != samples.end(); it++) {
            Pose2D pose = *it;
            std::cout << "x: " << pose.x << ", y: " << pose.y << ", theta: " << pose.theta << std::endl;
        }
    } else {
        std::cout << "Invalid steer!" << std::endl;
    }

    std::string filename("/Users/tfh/Documents/motion_planning/test_map.png");
    double resolution = 0.05;
    OccupancyGrid2D<Pose2D> occupancyGrid(filename, resolution, start);
    std::cout << occupancyGrid.isFree(end) << std::endl;

    return 0;
}
