#include <vector>

template <class State>
class Steer {
public:
    /**
     * Samples the steer path until the cost between
     * adjacent points is less than or equal to the
     * resolution.
     */
    virtual std::vector<State> sample(double resolution) = 0;

    /**
     * Returns the cost of the steer.
     */
    virtual double cost() = 0;
};
