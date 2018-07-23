#include <cstdio>
#include <iostream>
#include <cmath>
#include <limits>

#include <png.h>
#include <Eigen/Dense>

#include "motion_planning/State/Pose2D.hpp"
#include "motion_planning/Steer/Steer.hpp"
#include "motion_planning/Occupancy/Occupancy.hpp"
#include "motion_planning/Occupancy/OccupancyGrid2D.hpp"

template<class State>
OccupancyGrid2D<State>::OccupancyGrid2D() {
    thetaDistribution = std::uniform_real_distribution<double>(-M_PI, M_PI);
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

    map = Eigen::MatrixXd(height, width);

    for (int row = 0; row < height; row++) {
        // Read the data
        png_read_row(png, mapData, NULL);
        // Fill the map matrix
        for (int col = 0; col < width; col++) {
            map(row, col) =  1. - mapData[col]/((double) std::numeric_limits<png_byte>::max());
        }
    }

    // Close the file
    fclose(mapPngFile);

    // Initialize the map parameters
    resolution = resolution_;
    origin = origin_;
    colDistribution = std::uniform_real_distribution<double>(0, map.cols());
    rowDistribution = std::uniform_real_distribution<double>(0, map.rows());

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
    int x_cell = std::round(x_rot/resolution);
    int y_cell = std::round(y_rot/resolution);

    if ((0 <= x_cell) and (x_cell < map.cols()) and (0 <= y_cell) and (y_cell < map.rows())) {
        // The cell is in the map, use the value from it
        return map(y_cell, x_cell);
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

    for (typename std::vector<Pose2D>::iterator sample = samples.begin(); sample < samples.end(); sample++) {
        isFree_ &= Occupancy::isFree(&*sample);
    }

    return isFree_;
}

template<>
void OccupancyGrid2D<Pose2D>::randomState(Pose2D * state) {
    state -> x = colDistribution(generator) * resolution;
    state -> y = rowDistribution(generator) * resolution;
    state -> theta = thetaDistribution(generator);
}

template<>
Pose2D OccupancyGrid2D<Pose2D>::sampleFree() {
    Pose2D state;

    while (true) {
        // Choose a random state in the map
        randomState(&state);

        if (Occupancy::isFree(&state)) {
            return state;
        }
    }
}

template<>
Pose2D OccupancyGrid2D<Pose2D>::samplePerimeter(bool unknown) {
    Pose2D state;
    Pose2D stateNear;
    
    while (true) {
        // Choose a random state in the map
        randomState(&state);

        if ((unknown and isUnknown(&state)) or ((not unknown) and isOccupied(&state))) {
            // Find an adjacent state that is free
            stateNear.theta = state.theta;

            for (int i = -1; i <= 1; i+=2) {
                stateNear.x = state.x + i * resolution;
                stateNear.y = state.y;
                if (Occupancy::isFree(&stateNear)) return state;

                stateNear.x = state.x;
                stateNear.y = state.y + i * resolution;
                if (Occupancy::isFree(&stateNear)) return state;
            }

        }
    }
}

template class OccupancyGrid2D<Pose2D>;
