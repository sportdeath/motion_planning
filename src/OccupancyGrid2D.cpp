#include <cstdio>
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>

#include <png.h>

#include "motion_planning/State/Pose2D.hpp"
#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/Occupancy/OccupancyGrid2D.hpp"

template<class State>
OccupancyGrid2D<State>::OccupancyGrid2D() {
    thetaDistribution = std::uniform_real_distribution<double>(-M_PI, M_PI);
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

template<class State>
double OccupancyGrid2D<State>::intToProbability(uint8_t i) {
    return 1. - i/((double) std::numeric_limits<uint8_t>::max());
}

template<class State>
template<typename T>
bool OccupancyGrid2D<State>::setMap(const std::vector<T> & dataVec, size_t width, size_t height, double resolution_, State origin_) {
    const T * data = &dataVec[0];
    return setMap(data, width, height, resolution_, origin_);
}

template<class State>
bool OccupancyGrid2D<State>::setMap(const uint8_t * data, size_t width, size_t height, double resolution_, State origin_) {
    initializeMap(width, height, resolution_, origin_);

    // Fill the map matrix
    for (size_t i = 0; i < width * height; i++) {
        map[i] = intToProbability(data[i]);
    }

    return true;
}

template<class State>
bool OccupancyGrid2D<State>::setMap(const int8_t * data, size_t width, size_t height, double resolution_, State origin_) {
    initializeMap(width, height, resolution_, origin_);

    // Fill the map matrix
    double element;
    for (size_t i = 0; i < width * height; i++) {
        element = data[i]/100.;
        if (element < 0) {
            element = 0.5;
        }
        map[i] = element;
    }

    return true;
}

template<class State>
bool OccupancyGrid2D<State>::initializeMap(size_t width_, size_t height_, double resolution_, State origin_) {
    // Initialize the map
    width = width_;
    height = height_;
    map = std::vector<double>(width * height, 0);

    // Initialize the map parameters
    resolution = resolution_;
    origin = origin_;
    colDistribution = std::uniform_real_distribution<double>(0, width);
    rowDistribution = std::uniform_real_distribution<double>(0, height);

    return true;
}

template<class State>
bool OccupancyGrid2D<State>::setMap(std::string mapPngFilename, double resolution_, State origin_) {
    // Open the file
    std::FILE * mapPngFile = std::fopen(mapPngFilename.c_str(), "rb");

    // Create a png object
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    // Create an object to store the png info
    png_infop pngInfo = png_create_info_struct(png);

    // Open the file and read from it
    png_init_io(png, mapPngFile);
    png_read_info(png, pngInfo);

    // Extract basic image properties
    int width      = png_get_image_width(png, pngInfo);
    int height     = png_get_image_height(png, pngInfo);
    png_byte color_type = png_get_color_type(png, pngInfo);
    png_byte bit_depth  = png_get_bit_depth(png, pngInfo);

    // Make sure the image is gray scale
    if(color_type != PNG_COLOR_TYPE_GRAY) {
        std::cerr << mapPngFilename << " is not a grayscale image!" << std::endl;
        return false;
    } 

    // Use a bit depth of 8 regardless of the input
    if(bit_depth == 16) png_set_strip_16(png);
    png_read_update_info(png, pngInfo);

    // Initialize a place to read data to
    png_bytep mapData = new png_byte[width];

    // Initialize the map
    initializeMap(width, height, resolution_, origin_);

    for (int row = 0; row < height; row++) {
        // Read the data
        png_read_row(png, mapData, NULL);
        // Fill the map matrix
        for (int col = 0; col < width; col++) {
            map[row * width + col] = intToProbability(mapData[col]);
        }
    }

    // Close the file
    fclose(mapPngFile);

    return true;
}

template<class State>
double OccupancyGrid2D<State>::occupancyProbability(int cell) const {
    return map[cell];
}

template<class State>
double OccupancyGrid2D<State>::occupancyProbability(int row, int col) const {
    return occupancyProbability(rowColToCell(row, col));
}

template<class State>
int OccupancyGrid2D<State>::rowColToCell(int row, int col) const {
    return row * width + col;
}


template<class State>
double OccupancyGrid2D<State>::occupancyProbability(const State * state) const {
    // Convert the state to a cell
    int row, col;
    stateToRowCol(state, row, col);

    if ((0 <= col) and (col < width) and (0 <= row) and (row < height)) {
        // The cell is in the map, use the value from it
        return occupancyProbability(row, col);
    } else {
        // If the cell is outside of the map, assume it is unknown
        return 0.5;
   }
}

template<class State>
bool OccupancyGrid2D<State>::isSteerFree(Steer<State> * steer) const {
    // Sample the state at the resolution of the map
    std::vector<State> samples = steer -> sample(resolution);

    bool isFree_ = true;

    for (auto sample = samples.begin(); sample < samples.end(); sample++) {
        isFree_ &= Occupancy<State>::isFree(&*sample);
    }

    return isFree_;
}

template<>
void OccupancyGrid2D<Pose2D>::stateToRowCol(const Pose2D * state, int & row, int & col) const {
    return xyToRowCol(state -> x, state -> y, row, col);
}

template<class State>
void OccupancyGrid2D<State>::rowColToXY(int row, int col, double & x, double & y) const {
    double x_ = col * resolution;
    double y_ = row * resolution;
    x = x_ * cos(-origin.theta) - y_ * sin(-origin.theta) + origin.x;
    y = x_ * sin(-origin.theta) + y_ * cos(-origin.theta) + origin.y;
}

template<class State>
void OccupancyGrid2D<State>::xyToRowCol(double x, double y, int & row, int & col) const {
    // Translate the state by the origin
    double x_trans = x - origin.x;
    double y_trans = y - origin.y;

    // Rotate the state into the map
    double x_rot = x_trans * cos(origin.theta) - y_trans * sin(origin.theta);
    double y_rot = x_trans * sin(origin.theta) + y_trans * cos(origin.theta);

    // Discretize the state into a cell
    col = std::floor(x_rot/resolution);
    row = std::floor(y_rot/resolution);
}

template<>
void OccupancyGrid2D<Pose2D>::randomState(Pose2D * state) {
    int col = colDistribution(generator);
    int row = rowDistribution(generator);
    rowColToXY(row, col, state -> x, state -> y);
    state -> theta = thetaDistribution(generator);
}

template class OccupancyGrid2D<Pose2D>;
template bool OccupancyGrid2D<Pose2D>::setMap<uint8_t>(const std::vector<uint8_t> &, size_t, size_t, double, Pose2D);
template bool OccupancyGrid2D<Pose2D>::setMap<int8_t>(const std::vector<int8_t> &, size_t, size_t, double, Pose2D);
