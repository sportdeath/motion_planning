#pragma once

#include <vector>

template <class State>
class Steer {
public:
    /**
     * Determine the optimal path in free space
     * between the start and end states.
     *
     * @returns True iff a valid steer exists.
     */
    virtual bool steer(const State * start, const State * end) = 0;


    /**
     * Samples the steer path until the cost between
     * adjacent points is less than or equal to the
     * resolution.
     */
    virtual std::vector<State> sample(double resolution) = 0;

    /**
     * Return a point that is a fraction of the cost
     * along the steering path.
     *
     * For example, if t=0.5 then the cost from the start
     * to the interpolated state should equal the cost from
     * the interpolated state to the end.
     */
    virtual State interpolate(double t) = 0;

    /**
     * Returns the cost of the steer.
     */
    virtual double cost() = 0;

    /**
     * Returns the physical distance of the steer.
     * Necessary for collision checking
     *
     * This is not the same as the cost which can be
     * and real valued function that increases over the
     * span of the steering path.
     */
    virtual double distance() = 0;

    /**
     * Return a lower bound on the cost between
     * two states. This should be easier to compute
     * than the actual cost.
     *
     * For example, for a Dubin's car moving at unit speed,
     * a lower bound on the cost is the distance between
     * two states.
     */
    virtual double lowerBoundCost(const State * sate, const State * end) const = 0;
};
