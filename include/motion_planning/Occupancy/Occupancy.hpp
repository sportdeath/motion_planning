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
    virtual double occupancyProbability(const State * state) const = 0;

    /**
     * The upper-threshold occupancy probability for which
     * states are considered free.
     */
    virtual double freeThreshold() const = 0;

    /**
     * The lower-threshold occupancy probability for which
     * states are considered occupied.
     */
    virtual double occupiedThreshold() const = 0;

    inline bool isFree(const State * state) const {
        return occupancyProbability(state) < freeThreshold();
    }

    inline bool isOccupied(const State * state) const {
        return occupancyProbability(state) > occupiedThreshold();
    }

    inline bool isUnknown(const State * state) const {
        return (not isFree(state)) and (not isOccupied(state));
    }

    /**
     * Takes a steering function and 
     */
    virtual bool isSteerFree(Steer<State> * steer) const = 0;

    virtual void randomState(State * state) = 0;

    /**
     * Sample a free state uniformly at random.
     */
    virtual State sampleFree() = 0;

    /**
     * Sample occupied or unknown states that border free states.
     */
    virtual State samplePerimeter(bool unknown=false) = 0;
};

#endif // OCCUPANCY_HPP

// Uniform sampling -> free state
// Bridge sampling 
// What if we wanted to sample based on distance or something...
// I guess that could be implimented by the class specifically.
