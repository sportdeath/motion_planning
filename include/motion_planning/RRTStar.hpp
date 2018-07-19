#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/StateSampler/StateSampler.hpp"

template <class State>
class RRTStar {

private:
    Steer<State> steer;
    Occupancy<State> occupancy;
    StateSampler<State> stateSampler;

    struct Node {
        State state;
        Node * parent;
        std::vector<Node *> children;
    };

    void insert(const Node &node);

    Node * growTree(const State &rand, std::vector<Node> const &nearNodes);

    void rewire(const Node &rand, std::vector<Node> const &nearNodes);

public:
    /** 
     * Initialize the RRTStar algorithm.
     *
     * @param steer Determines feasible paths between states in free space.
     * @param occupancy Determines whether a state is in free space.
     * @param stateSampler Samples states in free space.
     */
    RRTStar(Steer<State> &steer_, const Occupancy<State> &occupancy_, const StateSampler<State> &stateSampler_);

    /**
     * Perform one iteration of the RRTStar algorithm
     *
     * This generates a single random sample and attempts
     * to add it to the tree of paths.
     *
     * @returns True iff the random sample is added to the tree
     */
    bool iterate();

    /**
     */
    std::vector<State> getBestPaths(int N);
};
