#include <cmath>
#include <vector>
#include <limits>

#include "motion_planning/RRTStar.hpp"
#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/State/Pose2D.hpp"

template <class State>
RRTStar<State>::RRTStar(
        Steer<State> * steer_, 
        const Occupancy<State> * occupancy_, 
        const std::function<State(void)> sampleState_,
        const std::function<bool(const State *)> isGoal_) :
    steer(steer_),
    occupancy(occupancy_),
    sampleState(sampleState_),
    isGoal(isGoal_)
{}

template <class State>
typename RRTStar<State>::Node * RRTStar<State>::growTree(const State & rand, const std::vector<Node *> &nearNodes) {

    // Initialize the cost
    double costMin = std::numeric_limits<double>::infinity();
    Node * parentMin = NULL;

    // Iterate over nearby nodes
    for (auto nearNode = nearNodes.begin(); nearNode < nearNodes.end(); nearNode++) {

        // Compute the steer from the nearest node to the random node
        bool validSteer = steer -> steer(&((*nearNode) -> state), &rand);

        // If the steer is valid and the path is free
        if (validSteer && occupancy -> isSteerFree(steer)) {

            // Compute the total cost from the root to the random node
            double cost = (*nearNode) -> cost + steer -> cost();

            if (cost < costMin) {
                costMin = cost;
                parentMin = *nearNode;
            }
        }
    }

    if (parentMin != NULL) {
        // Create a new node
        Node * randNode = initNode(rand, parentMin, costMin);

        // Insert it into the tree for nearest neighbors
        //insert(randNode);

        return randNode;
    } else {
        return NULL;
    }
}

template <>
void RRTStar<Pose2D>::rewire(Node * randNode, const std::vector<Node *> & nearNodes) {
    // Iterate over nearby nodes
    for (auto nearNode = nearNodes.begin(); nearNode < nearNodes.end(); nearNode++) {

        // Compute the steer from the random node to the nearest node
        bool validSteer = steer -> steer(&(randNode -> state), &((*nearNode) -> state));

        // If the steer is valid and the path is free
        if (validSteer && occupancy -> isSteerFree(steer)) {

            // Compute the total cost from the root to the random node
            double cost = randNode -> cost + steer -> cost();

            if (cost < (*nearNode) -> cost) {
                // Remove nearNode from parent's children
                std::vector<Node *> & children = (*nearNode) -> parent -> children;
                children.erase(std::remove(children.begin(), children.end(), *nearNode), children.end());

                // Set nearNode's parent to randNode
                (*nearNode) -> parent = randNode; 

                // Add randNode's child to nearNode
                randNode -> children.push_back(*nearNode);
            }
        }
    }
}

// Init node
template <class State>
typename RRTStar<State>::Node * RRTStar<State>::initNode(const State & state, Node * parent, double cost) {
    // Add a new node to the list of nodes
    std::size_t old_size = nodes.size();
    nodes.resize(old_size + 1);
    Node * newNode = &nodes[old_size];

    // Initialize state
    newNode -> state = state;
    newNode -> parent = parent;
    newNode -> cost = cost;

    // Add child to the parent
    parent -> children.push_back(newNode);

    return newNode;
}

template <class State>
bool RRTStar<State>::iterate() {
    // Generate a random state
    State rand = sampleState();    

    // Determine the nearest nodes to the state
    std::vector<Node*> nearNodes = nearestNodes(&rand);

    // Attempt to grow the tree to the random state
    Node * randNode = growTree(rand, nearNodes);

    if (randNode != NULL) {
        // If the node has been added, perform rewire
        rewire(randNode, nearNodes);
        // Check if the added node is in the goal set
        if (isGoal(&randNode -> state)) {
            goalNodes.push_back(randNode);
        }

        return true;
    } else {
        return false;
    }
}

template <>
std::vector<typename RRTStar<Pose2D>::Node *> RRTStar<Pose2D>::nearestNodes(const Pose2D * state) {
    std::vector<Node *> nearNodes;

    for (auto node = nodes.begin(); node < nodes.end(); node++) {
        double dist = sqrt(pow(state -> x - ((*node).state.x), 2.) + pow(state -> y - ((*node).state.y), 2.));
        
        if (dist < 3.) {
            nearNodes.push_back(&*node);
        }
    }

    return nearNodes;
}

template class RRTStar<Pose2D>;
