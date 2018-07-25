#include <cmath>
#include <random>
#include <chrono>
#include <iostream>

#include <motion_planning/State/Pose2D.hpp>
#include <motion_planning/StateSampler/StateSampler.hpp>

template<class State>
UniformSampler<State>::UniformSampler(
        Occupancy<State> * occupancy_, 
        bool sampleFree_, 
        bool sampleUnknown_, 
        bool sampleOccupied_) :
    occupancy(occupancy_),
    sampleFree(sampleFree_),
    sampleUnknown(sampleUnknown_),
    sampleOccupied(sampleOccupied_)
{}

template<class State>
State UniformSampler<State>::sample() {
    Pose2D state;

    while (true) {
        // Choose a random state in the map
        occupancy -> randomState(&state);

        if ((sampleFree && occupancy -> isFree(&state)) ||
            (sampleUnknown && occupancy -> isUnknown(&state)) ||
            (sampleOccupied && occupancy -> isOccupied(&state))) {
            return state;
        }
    }
}

BridgeSampler::BridgeSampler(Occupancy<Pose2D> * occupancy_, double stdDev) :
    occupancy(occupancy_),
    sampleOccupied(occupancy, false, false, true),
    normalDistribution(0., stdDev)
{
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

Pose2D BridgeSampler::sample() {
    while (true) {
        // Choose a random occupied state
        Pose2D occ = sampleOccupied.sample();
        
        // Choose a nearby point
        double x_noise = normalDistribution(generator);
        double y_noise = normalDistribution(generator);
        Pose2D near = {.x=occ.x + x_noise, .y=occ.y + y_noise, .theta=0.};

        // Check if the nearby point is occupied
        if (occupancy -> isOccupied(&near)) {

            // check if the midpoint is free
            Pose2D midpoint = {.x=(occ.x+near.x)/2., .y=(occ.y+near.y)/2., .theta=0.};

            if (occupancy -> isFree(&midpoint)) {
                 double theta = atan2(occ.x - near.x, near.y - occ.y);
                 midpoint.theta = theta;
                 return midpoint;
            }
        }
    }
}

template<class State>
MixedSampler<State>::MixedSampler(
        std::vector<StateSampler<State> *> & samplers_,
        std::vector<double> & weights) :
    samplers(samplers_)
{
    cumulativeProbabilities = std::vector<double>(weights.size());

    double totalWeight = 0;
    for (auto & weight : weights) {
        totalWeight += weight;
    }

    // Accumulate probability
    cumulativeProbabilities[0] = weights[0]/totalWeight;
    for (int i = 1; i < weights.size(); i++) {
        cumulativeProbabilities[i] = weights[i]/totalWeight + cumulativeProbabilities[i-1];
    }

    choiceDistribution = std::uniform_real_distribution<double>(0., 1.);
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

template<class State>
State MixedSampler<State>::sample() {
    double choice = choiceDistribution(generator);

    for (int i = 0; i < samplers.size(); i++) {
        if (choice < cumulativeProbabilities[i]) {
            return samplers[i] -> sample();
        }
    }
}

template class MixedSampler<Pose2D>;
