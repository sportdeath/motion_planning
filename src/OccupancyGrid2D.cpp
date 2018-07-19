#include <cstdio>
#include <iostream>
#include <cmath>

#include <png.h>
#include <Eigen/Dense>

#include "motion_planning/Pose2D.hpp"
#include "motion_planning/OccupancyGrid2D.hpp"

template<class State>
OccupancyGrid2D<State>::OccupancyGrid2D(std::string mapPngFilename, double resolution_, State origin_) 
: resolution(resolution_),
  origin(origin_) {

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
        return;
    } 

    // Use a bit depth of 8 regardless of the input
    if(bit_depth == 16) png_set_strip_16(png);
    png_read_update_info(png, pngInfo);

    // Initialize an Eigen matrix
    map = Eigen::Matrix<png_byte, Eigen::Dynamic, Eigen::Dynamic>(height, width);
    png_bytep mapData = map.data();

    // Read the occupancy grid into the map
    for (int row = 0; row < height; row++) {
        png_read_row(png, mapData + row * width, NULL);
    }

    // Close the file
    fclose(mapPngFile);
}

template<>
bool OccupancyGrid2D<Pose2D>::isFree(const Pose2D state) {
    // @TODO use the origin to compute this
    double x = state.x;
    double y = state.y;

    int x_cell = std::round(x/resolution);
    int y_cell = std::round(y/resolution);

    if ((0 <= x_cell) and (x_cell < map.cols()) and (0 <= y_cell) and (y_cell < map.rows())) {
        // The cell is in the map, use the value from it
        return map(x_cell, y_cell) < 128;
    } else {
        // If the cell is outside of the map, assume it is occupied.
        return false;
    }
}

template class OccupancyGrid2D<Pose2D>;
