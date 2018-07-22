#include <iostream> 

extern "C" {
#include <dubins.h>
}
#include "motion_planning/Steer/DubinsSteer.hpp" 
bool DubinsSteer::steer(const Pose2D * start, const Pose2D * end) {
    double q0[] = {start -> x, start -> y, start -> theta};
    double q1[] = {end -> x, end -> y, end -> theta};
    int output = dubins_shortest_path(&path, q0, q1, turningRadius);

    return output == 0;
}

int DubinsSteer::dubinsSampleCallback(double q[3], double x, void * user_data) {
    // Construct a pose
    Pose2D pose = {.x = q[0], .y = q[1], .theta = q[2]};
    
    // Add it to the vector
    DubinsSampleData * data = (DubinsSampleData *) user_data;
    if (data -> sampleIndex < data -> samples -> size()) {
        (*data -> samples)[data -> sampleIndex] = pose;
    }

    // Increase the index
    data -> sampleIndex ++;

    return 0;
}

std::vector<Pose2D> DubinsSteer::sample(double resolution) {
    // Initialize the output vector
    int numSamples = cost()/resolution + 1;
    std::vector<Pose2D> samples(numSamples);

    // Initialize the data passed to the callback function
    DubinsSampleData data = {.samples = &samples, .sampleIndex = 0};

    // Sample the vector
    dubins_path_sample_many(&path, resolution, dubinsSampleCallback, (void *) &data);

    // Add the end point
    double endPoint[3];
    dubins_path_endpoint(&path, endPoint);
    Pose2D end = {.x=endPoint[0], .y=endPoint[1], .theta=endPoint[2]};
    samples[numSamples-1] = end;

    return samples;
}

double DubinsSteer::cost() {
    return dubins_path_length(&path);
}
