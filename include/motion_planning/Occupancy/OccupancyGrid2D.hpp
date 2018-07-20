#ifndef OCCUPANCY_GRID_2D_HPP
#define OCCUPANCY_GRID_2D_HPP

#include <string>

#include <png.h>
#include <Eigen/Dense>

#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/Steer/Steer.hpp"

template <class State>
class OccupancyGrid2D : public Occupancy<State> {
private:
    double resolution;
    State origin;
    Eigen::Matrix<png_byte, Eigen::Dynamic, Eigen::Dynamic> map;

public:
    // @TODO add png for robot and convolve the map by it rotated in different directions
    // accounting for error in projection
    OccupancyGrid2D(std::string mapPngFile, double resolution, State origin);

    double occupancyProbability(const State * state);
    bool isFree(const State * state);
    bool isUnknown(const State * state);
    bool isFree(const Steer<State> * steer);
};

#endif // OCCUPANCY_GRID_2D_HPP
