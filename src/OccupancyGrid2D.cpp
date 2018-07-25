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

template<>
double OccupancyGrid2D<Pose2D>::occupancyProbability(const Pose2D * state) const {
    // Translate the state by the origin
    double x_trans = state -> x - origin.x;
    double y_trans = state -> y - origin.y;

    // Rotate the state into the map
    double x_rot = x_trans * cos(origin.theta) - y_trans * sin(origin.theta);
    double y_rot = x_trans * sin(origin.theta) + y_trans * cos(origin.theta);

    // Discretize the state into a cell
    int x_cell = std::floor(x_rot/resolution);
    int y_cell = std::floor(y_rot/resolution);

    if ((0 <= x_cell) and (x_cell < width) and (0 <= y_cell) and (y_cell < height)) {
        // The cell is in the map, use the value from it
        return map[y_cell * width + x_cell];
    } else {
        // If the cell is outside of the map, assume it is unknown
        return 0.5;
   }
}

template<>
bool OccupancyGrid2D<Pose2D>::isSteerFree(Steer<Pose2D> * steer) const {
    // Sample the state at the resolution of the map
    std::vector<Pose2D> samples = steer -> sample(resolution);

    bool isFree_ = true;

    for (auto sample = samples.begin(); sample < samples.end(); sample++) {
        isFree_ &= Occupancy::isFree(&*sample);
    }

    return isFree_;
}

template<>
void OccupancyGrid2D<Pose2D>::randomState(Pose2D * state) {
    double x = colDistribution(generator) * resolution;
    double y = rowDistribution(generator) * resolution;
    state -> x = x * cos(-origin.theta) - y * sin(-origin.theta) + origin.x;
    state -> y = x * sin(-origin.theta) + y * cos(-origin.theta) + origin.y;
    state -> theta = thetaDistribution(generator);
}

template class OccupancyGrid2D<Pose2D>;
template bool OccupancyGrid2D<Pose2D>::setMap<uint8_t>(const std::vector<uint8_t> &, size_t, size_t, double, Pose2D);
template bool OccupancyGrid2D<Pose2D>::setMap<int8_t>(const std::vector<int8_t> &, size_t, size_t, double, Pose2D);
