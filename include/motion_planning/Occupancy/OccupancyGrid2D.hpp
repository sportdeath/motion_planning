#ifndef OCCUPANCY_GRID_2D_HPP
#define OCCUPANCY_GRID_2D_HPP

#include <string>
#include <random>

#include <png.h>

#include "motion_planning/State/Pose2D.hpp"
#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/Steer/Steer.hpp"

template <class State>
class OccupancyGrid2D : public Occupancy<State> {
private:
    double bufferedRadius;
    double collisionCheckingResolution;

    double resolution;
    State origin;

    std::vector<double> map;
    std::vector<double> dt; // distance transform
    size_t width;
    size_t height;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> colDistribution;
    std::uniform_real_distribution<double> rowDistribution;
    std::uniform_real_distribution<double> thetaDistribution;

    double intToProbability(uint8_t i) const;
    bool initializeMap(size_t width, size_t height, double resolution_, State origin_);
    void computeDistanceTransform();

public:
    OccupancyGrid2D();
    size_t getWidth() {return width;};
    size_t getHeight() {return height;};
    double getResolution() {return resolution;};
    const Pose2D & getOrigin() {return origin;};

    double entropy() const;

    /**
     * We assume the robot is a circle centered at the pose.
     * 
     * @param objectRadius The radius of the object.
     * @param searchBuffer A small value used to increase the search
     * radius. As this value approaches zero the number of collision
     * checks that need to be made will go to infinity.
     */
    void setObjectRadius(double objectRadius, double searchBuffer);
    double distanceTransform(const State * state) const;

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
    double occupancyProbability(size_t row, size_t col) const;

    inline bool isFree(const State * state) const {
        return occupancyProbability(state) < freeThreshold(); }
    inline bool isOccupied(const State * state) const {
        return occupancyProbability(state) > occupiedThreshold(); }
    inline bool isUnknown(const State * state) const {
        return (not isFree(state)) and (not isOccupied(state)); }
    inline bool isFree(size_t row, size_t col) const {
        return occupancyProbability(row, col) < freeThreshold(); }
    inline bool isFree(int cell) const {
        return occupancyProbability(cell) < freeThreshold(); }
    inline bool isOccupied(size_t row, size_t col) const {
        return occupancyProbability(row, col) > occupiedThreshold(); }
    inline bool isOccupied(int cell) const {
        return occupancyProbability(cell) > occupiedThreshold(); }
    inline bool isUnknown(size_t row, size_t col) const {
        return (not isFree(row, col)) and (not isOccupied(row, col)); }
    inline bool isUnknown(int cell) const {
        return (not isFree(cell)) and (not isOccupied(cell)); }

    int rowColToCell(size_t row, size_t col) const;
    void stateToRowCol(const Pose2D * state, size_t & row, size_t & col) const;
    void xyToRowCol(double x, double y, size_t & row, size_t & col) const;
    void rowColToXY(size_t row, size_t col, double & x, double & y) const;

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
