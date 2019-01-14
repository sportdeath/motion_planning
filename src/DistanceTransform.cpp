#include <iostream>
#include <cmath>

#include "motion_planning/Occupancy/DistanceTransform.hpp"

// http://people.cs.uchicago.edu/~pff/papers/dt.pdf

DistanceTransform::DistanceTransform(int maxSize) {
    // Initialize vectors to store indices and boundaries
    parabolaIdxs = std::vector<int>(maxSize);
    parabolaBoundaries = std::vector<double>(maxSize + 1);
}

void DistanceTransform::distanceSquared1D(const std::vector<double> & input, std::vector<double> & output) {
    // Each parabola has the form
    //
    //     input[index] + (query - index)^2

    // The grid location of the i-th parabola is parabolaIdxs[i]
    parabolaIdxs[0] = 0;

    // The range of the i-th parabola is
    // parabolaBoundaries[i] to parabolaBoundaries[i + 1]
    // Initialize all of the ranges to extend to infinity
    parabolaBoundaries[0] = -inf;
    parabolaBoundaries[1] = inf;

    // The number of parabolas in the lower envelope
    int numParabolas = 0;

    // Compute the lower envelope over all grid cells
    double intersectionPoint;
    for (size_t idx = 1; idx < input.size(); idx++) {
        numParabolas++;
        do {
            numParabolas--;
            // The location of the rightmost parabola in the lower envelope
            int parabolaIdx = parabolaIdxs[numParabolas];

            // Compute the intersection point between the current and rightmost parabola
            // by solving for the intersection point, p:
            //
            // input[idx] + (p - idx)^2 = input[parabolaIdx] - (p - parabolaIdx)^2
            //
            intersectionPoint = (
                    (input[idx] + idx * idx)
                    -
                    (input[parabolaIdx] + parabolaIdx * parabolaIdx))
                    /
                    (2 * (idx - parabolaIdx));

            // If the intersection point is before the boundary,
            // the rightmost parabola is not actually part of the
            // lower envelope, so decrease k and repeat.
        } while (intersectionPoint <= parabolaBoundaries[numParabolas]);

        // Move to the next parabola
        numParabolas ++;

        parabolaIdxs[numParabolas] = idx;
        parabolaBoundaries[numParabolas] = intersectionPoint;
        parabolaBoundaries[numParabolas+1] = inf;
    }

    int parabola = 0;
    for (size_t idx = 0; idx < input.size(); idx++) {
        // Find the parabola corresponding to the index
        while (parabolaBoundaries[parabola + 1] < idx) parabola++;

        // Compute the value of the parabola
        int idxDist = idx - parabolaIdxs[parabola];
        output[idx] = idxDist * idxDist + input[parabolaIdxs[parabola]];
    }
}

void DistanceTransform::distanceSquared2D(std::vector<double> & input, int width, int height) {
    // Transform along the columns
    std::vector<double> colVec(height);
    std::vector<double> colDT(height);
    for (int col = 0; col < width; col++) {
        for (int row = 0; row < height; row++) {
            colVec[row] = input[row * width + col];
        }
        distanceSquared1D(colVec, colDT);
        for (int row = 0; row < height; row++) {
            input[row * width + col] = colDT[row];
        }
    }

    // Transform along the rows
    std::vector<double> rowVec(width);
    std::vector<double> rowDT(width);
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            rowVec[col] = input[row * width + col];
        }
        distanceSquared1D(rowVec, rowDT);
        for (int col = 0; col < width; col++) {
            input[row * width + col] = rowDT[col];
        }
    }
}

void DistanceTransform::distance2D(std::vector<double> & input, int width, int height) {
    distanceSquared2D(input, width, height);
    for (size_t i = 0; i < input.size(); i++) {
        input[i] = sqrt(input[i]);
    }
}
