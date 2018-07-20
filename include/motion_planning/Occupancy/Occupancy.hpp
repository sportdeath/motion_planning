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

    bool isFree(const State * state) {
        return occupancyProbability(state) < 0.5;
    }

    bool isUnknown(const State * state) {
        return occupancyProbability(state) == 0.5;
    }

    virtual bool isFree(Steer<State> * steer) = 0;
};

#endif // OCCUPANCY_HPP
