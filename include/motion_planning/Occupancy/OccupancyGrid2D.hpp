#ifndef OCCUPANCY_GRID_2D_HPP
#define OCCUPANCY_GRID_2D_HPP

#include <string>
#include <random>

#include <png.h>
#include <Eigen/Dense>

#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/Steer/Steer.hpp"

template <class State>
class OccupancyGrid2D : public Occupancy<State> {
private:
    double resolution;
    State origin;
    Eigen::MatrixXd map;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> colDistribution;
    std::uniform_real_distribution<double> rowDistribution;
    std::uniform_real_distribution<double> thetaDistribution;

public:
    OccupancyGrid2D();

    /**
     * Set the map of the occupancy grid to a png.
     *
     * @param mapPngFile The input map file. The map must be grayscale.
     *      White pixels are considered free space and black pixels are
     *      occupied space.
     * @param resolution The resolution of cells in the map in units/pixel.
     * @param origin The pose of the top leftmost pixel (0, 0).
     */
    bool setMap(std::string mapPngFile, double resolution, State origin);

    double occupancyProbability(const State * state) const;
    bool isSteerFree(Steer<State> * steer) const;
    State sampleFree();
    State samplePerimeter(bool unknown=false);
    double freeThreshold() const {return 0.4;};
    double occupiedThreshold() const {return 0.6;};
    void randomState(State * state);
};

#endif // OCCUPANCY_GRID_2D_HPP
