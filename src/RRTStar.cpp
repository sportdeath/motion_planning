#include <cmath>
#include <vector>
#include <limits>
#include <iostream>
#include <algorithm>

#include "motion_planning/RRTStar.hpp"
#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/State/Pose2D.hpp"

template <class State>
RRTStar<State>::RRTStar(
        Steer<State> * steer_, 
        const Occupancy<State> * occupancy_, 
        const std::function<State(void)> sampleState_,
        const std::function<bool(const State *)> isGoal_,
        const State & start,
        double searchRadius_) :
    steer(steer_),
    occupancy(occupancy_),
    sampleState(sampleState_),
    isGoal(isGoal_),
    searchRadius(searchRadius_)
{
    // Add the root node
    initNode(start, NULL, 0.);
}

template <class State>
typename RRTStar<State>::Node * RRTStar<State>::growTree(const State & rand) {

    // Initialize the cost
    double costMin = std::numeric_limits<double>::infinity();
    Node * parentMin = NULL;

    for (auto node = nodes.begin(); node != nodes.end(); node++) { 

        // Compute the steer from the nearest node to the random node
        bool validSteer = steer -> steer(&((*node).state), &rand);

        // Filter out nodes based on the length of the steer
        if (steer -> cost() > searchRadius) continue;

        // If the steer is valid and the path is free
        if (validSteer && occupancy -> isSteerFree(steer)) {

            // Compute the total cost from the root to the random node
            double cost = (*node).cost + steer -> cost();

            // If the cost is better, update the parent
            if (cost < costMin) {
                costMin = cost;
                parentMin = &*node;
            }
        }
    }

    if (parentMin != NULL) {
        // Create a new node
        Node * randNode = initNode(rand, parentMin, costMin);

        return randNode;
    } else {
        return NULL;
    }
}

template <class State>
void RRTStar<State>::rewire(Node * randNode) {

    // Iterate over all nodes
    for (auto node = nodes.begin(); node != nodes.end(); node++) { 

        // Compute the steer from the random node to the nearest node
        bool validSteer = steer -> steer(&(randNode -> state), &((*node).state));

        // Filter out nodes based on the length of the steer
        if (steer -> cost() > searchRadius) continue;

        // If the steer is valid and the path is free
        if (validSteer && occupancy -> isSteerFree(steer)) {

            // Compute the total cost from the root to the random node
            double cost = randNode -> cost + steer -> cost();

            if (cost < (*node).cost) {
                // Remove nearNode from its parent's children
                std::vector<Node *> & children = (*node).parent -> children;
                children.erase(std::remove(children.begin(), children.end(), &*node), children.end());

                // Set nearNode's parent to randNode
                (*node).parent = randNode; 

                // Add nearNode to randNodes children
                randNode -> children.push_back(&*node);
            }
        }
    }
}

template <class State>
typename RRTStar<State>::Node * RRTStar<State>::initNode(const State & state, Node * parent, double cost) {
    // Add a new node to the list of nodes
    nodes.emplace_back();
    Node * newNode = &nodes.back();

    // Initialize state
    newNode -> state = state;
    newNode -> parent = parent;
    newNode -> cost = cost;

    // Add child to the parent
    if (parent != NULL) {
        parent -> children.push_back(newNode);
    }

    return newNode;
}

template <class State>
typename RRTStar<State>::Node * RRTStar<State>::iterate() {
    // Generate a random state
    State rand = sampleState();    

    // Attempt to grow the tree to the random state
    Node * randNode = growTree(rand);

    if (randNode != NULL) {
        // If the node has been added, perform rewire
        rewire(randNode);
        // Check if the added node is in the goal set
        if (isGoal(&randNode -> state)) {
            goalNodes.push_back(randNode);
        }
    }

    return randNode;
}

template <class State>
std::vector<State> RRTStar<State>::samplePath(const Node * end, double resolution) const {

    // Sample paths between every node
    std::vector<std::vector<State>> pathSegments;
    while (end -> parent != NULL) {
        bool validSteer = steer -> steer(&end -> parent -> state, &end -> state);

        if (validSteer) {
            pathSegments.push_back(steer -> sample(resolution));
        }

        end = end -> parent;
    }

    // Reverse the nodes
    std::reverse(pathSegments.begin(), pathSegments.end());

    // Determine the total path size
    size_t total_length = 0;
    for (auto vec = pathSegments.begin(); vec < pathSegments.end(); vec++) { 
        total_length += (*vec).size();
    }

    // Allocate the total path and merge
    std::vector<State> path;
    path.reserve(total_length);
    for (auto vec = pathSegments.begin(); vec < pathSegments.end(); vec++) { 
        path.insert(path.end(), (*vec).begin(), (*vec).end());
    }

    return path;
}

template <class State>
std::vector<std::vector<State> > RRTStar<State>::sampleTree(double resolution) const {

    // Sample paths between every node
    std::vector<std::vector<State>> pathSegments;
    for (auto node = nodes.begin(); node != nodes.end(); node++) { 
        if ((*node).parent != NULL)  {
            bool validSteer = steer -> steer(&(*node).parent -> state, &(*node).state);

            if (validSteer) {
                pathSegments.push_back(steer -> sample(resolution));
            }
        }
    }

    return pathSegments;
}

template class RRTStar<Pose2D>;
