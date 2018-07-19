extern "C" {
#include "dubins.h"
}

#include "motion_planning/DubinsSteer.hpp"

DubinsSteer::DubinsSteer(Pose2D start, Pose2D end, double turning_radius) {
    double q0[] = {start.x, start.y, start.theta};
    double q1[] = {end.x, end.y, end.theta};
    dubins_shortest_path(&path, q0, q1, turning_radius);
}

int DubinsSteer::dubinsSampleCallback(double q[3], double x, void * user_data) {
    // Construct a pose
    Pose2D pose = {.x = q[0], .y = q[1], .theta = q[2]};
    
    // Add it to the vector
    DubinsSampleData * data = (DubinsSampleData *) user_data;
    (*data -> samples)[data -> sampleIndex] = pose;

    // Increase the index
    data -> sampleIndex ++;

    return 0;
}

std::vector<Pose2D> DubinsSteer::sample(double resolution) {
    // Initialize the output vector
    int numSamples = cost()/resolution;
    std::vector<Pose2D> samples(numSamples);

    // Initialize the data passed to the callback function
    DubinsSampleData data = {.samples = &samples, .sampleIndex = 0};

    // Sample the vector
    dubins_path_sample_many(&path, resolution, dubinsSampleCallback, (void *) &data);

    return samples;
}

double DubinsSteer::cost() {
    return dubins_path_length(&path);
}
