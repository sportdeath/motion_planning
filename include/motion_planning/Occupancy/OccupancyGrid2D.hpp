#ifndef OCCUPANCY_GRID_2D_HPP
#define OCCUPANCY_GRID_2D_HPP

#include <string>
#include <random>

#include <png.h>

#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/Steer/Steer.hpp"

template <class State>
class OccupancyGrid2D : public Occupancy<State> {
private:
    double resolution;
    State origin;

    std::vector<double> map;
    size_t width;
    size_t height;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> colDistribution;
    std::uniform_real_distribution<double> rowDistribution;
    std::uniform_real_distribution<double> thetaDistribution;

    double intToProbability(uint8_t i);
    bool initializeMap(size_t width, size_t height, double resolution_, State origin_);

public:
    OccupancyGrid2D();
    size_t getWidth() {return width;};
    size_t getHeight() {return height;};
    double getResolution() {return resolution;};

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
    double occupancyProbability(int cell) const;
    double occupancyProbability(int row, int col) const;

    int rowColToCell(int row, int col) const;
    void stateToRowCol(const Pose2D * state, int & row, int & col) const;
    void xyToRowCol(double x, double y, int & row, int & col) const;

    bool isSteerFree(Steer<State> * steer) const;
    double freeThreshold() const {return 0.4;};
    double occupiedThreshold() const {return 0.6;};
    void randomState(State * state);

    template<typename T>
    bool setMap(const std::vector<T> & dataVec, size_t width, size_t height, double resolution_, State origin_);

    bool setMap(const int8_t * data, size_t width, size_t height, double resolution_, State origin_);
    bool setMap(const uint8_t * data, size_t width, size_t height, double resolution_, State origin_);
};

#endif // OCCUPANCY_GRID_2D_HPP
