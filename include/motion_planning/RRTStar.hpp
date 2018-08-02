#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <list>
#include <functional>

#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/Occupancy/Occupancy.hpp"

template <class State>
class RRTStar {
public:
    struct Node {
        State state;
        Node * parent;
        std::vector<Node *> children;
        double cost;
        double segmentCost; // the cost from parent to node
    };

private:
    Steer<State> * steer;
    const Occupancy<State> * occupancy;
    const std::function<State(void)> sampleState;
    const std::function<bool(const State *)> isGoal;

    double searchRadius;

    // nodes is a list so pointers don't get invalidated
    // as it changes size.
    std::list<Node> nodes;
    std::vector<Node *> goalNodes;

    Node * initNode(const State & state, Node * parent, double cost, double segmentCost);

    Node * growTree(const State & rand);
    void rewire(Node * rand);
    void updateCosts(Node * node);

public:
    /** 
     * Initialize the RRTStar algorithm.
     *
     * @param steer Determines feasible paths between states in free space.
     * @param occupancy Determines whether a state is in free space.
     * @param stateSampler Samples states in free space.
     * @param goalChecker Checks whether states are in the goal region.
     */
    RRTStar(
        Steer<State> * steer_, 
        const Occupancy<State> * occupancy_, 
        std::function<State(void)> sampleState_,
        std::function<bool(const State *)> isGoal_,
        const State & start,
        double searchRadius_);

    /**
     * Perform one iteration of the RRTStar algorithm
     *
     * This generates a single random sample and attempts
     * to add it to the tree of paths.
     *
     * @returns True iff the random sample is added to the tree
     */
    Node * iterate();

    const std::vector<Node *> getGoals() const {return goalNodes;};
    const std::list<Node> getNodes() const {return nodes;};

    /**
     * Sample the path at approximately the given resolution.
     */
    std::vector<State> samplePath(const Node * end, double resolution) const;

    std::vector<std::vector<State> > sampleTree(double resolution) const;
};

#endif // RRT_STAR_HPP
