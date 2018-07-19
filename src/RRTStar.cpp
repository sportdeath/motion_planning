#include <vector>

#include "motion_planning/RRTStar.hpp"

template <class State>
void RRTStar<State>::insert(const Node &node) {
    // Insert the node into the 
    // Will the RTree take care of memory?
}

template <class State>
typename RRTStar<State>::Node * RRTStar<State>::growTree(const State &rand, const std::vector<Node> &nearNodes) {

    // Initialize the cost
    double costMin = std::numeric_limits<double>::infinity();
    Node * parentMin = NULL;

    // Iterate over nearby nodes
    for (typename std::vector<Node>::iterator nearNode = nearNodes.begin(); nearNode < nearNodes.end(); nearNode++) {

        // Compute the steer from the nearest node to the random node
        bool validSteer = steer.steer(&nearNode, rand);

        // If the steer is valid and the path is free
        if (validSteer && occupancy.isFree(steer)) {

            // Compute the total cost from the root to the random node
            double cost = nearNode -> cost + steer.cost();

            if (cost < costMin) {
                costMin = cost;
                parentMin = nearNode;
            }
        }
    }

    if (parentMin != NULL) {
        // Create the node
        Node randNode = {.state=rand, .parent=parentMin, .cost=costMin};

        // Insert it into the graph
        insert(randNode);
    }
}

// Perform multidimensional scaling to
// make the cost in free space approximately equal to the cost in 

template <class State>
bool RRTStar<State>::iterate() {
    // Generate a random state
    State rand = stateSampler.sample();    

    // Determine the nearest nodes to the state
    std::vector<Node> nearNodes = nearestNodes(rand.state);

    // Attempt to grow the tree to the random state
    Node * randNode = growTree(rand, nearNodes);

    if (randNode != NULL) {
        // If the node has been added, perform rewire
        rewire(randNode, nearNodes);
    }
}
