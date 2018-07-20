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
    /**
     * Initialize an occupancy grid with a PNG.
     *
     * @param mapPngFile The input map file. The map must be grayscale.
     *      White pixels are considered free space and black pixels are
     *      occupied space.
     * @param resolution The resolution of cells in the map in units/pixel.
     * @param origin The pose of the top leftmost pixel (0, 0).
     */
    OccupancyGrid2D(std::string mapPngFile, double resolution, State origin);

    double occupancyProbability(const State * state);
    bool isFree(const State * state);
    bool isUnknown(const State * state);
    bool isFree(const Steer<State> * steer);
};

#endif // OCCUPANCY_GRID_2D_HPP
