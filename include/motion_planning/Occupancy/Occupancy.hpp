#ifndef OCCUPANCY_HPP
#define OCCUPANCY_HPP

#include "motion_planning/Steer/Steer.hpp"

template <class State>
class Occupancy {
public:
    /**
     * Return the probability that the given
     * state is occupied.
     */
    virtual double occupancyProbability(const State * state) = 0;

    /**
     * The upper-threshold occupancy probability for which
     * states are considered free.
     */
    virtual double freeThreshold() = 0;

    /**
     * The lower-threshold occupancy probability for which
     * states are considered occupied.
     */
    virtual double occupiedThreshold() = 0;

    inline bool isFree(const State * state) {
        return occupancyProbability(state) < freeThreshold();
    }

    inline bool isOccupied(const State * state) {
        return occupancyProbability(state) > occupiedThreshold();
    }

    inline bool isUnknown(const State * state) {
        return (not isFree(state)) and (not isOccupied(state));
    }

    /**
     * Takes a steering function and 
     */
    virtual bool isFree(Steer<State> * steer) = 0;

    /**
     * Sample a free state uniformly at random.
     */
    virtual State state sampleFree() = 0;

    /**
     * Sample occupied or unknown states that border free states.
     */
    virtual State state samplePerimeter(bool unknown=false) = 0;
};

#endif // OCCUPANCY_HPP

// Uniform sampling -> free state
// Bridge sampling 
// What if we wanted to sample based on distance or something...
// I guess that could be implimented by the class specifically.
