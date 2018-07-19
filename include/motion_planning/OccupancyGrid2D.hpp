#include <string>

#include <png.h>
#include <Eigen/Dense>

#include "motion_planning/Occupancy.hpp"

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

    bool isFree(const State);
};
