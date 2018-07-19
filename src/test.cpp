#include <stdio.h>
#include <iostream>

extern "C" {
#include "dubins.h"
}
#include <spatialindex/capi/sidx_api.h>

int printConfiguration(double q[3], double x, void* user_data) {
    printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    return 0;
}

int main() {
    double q0[] = { 0,0,0 };
    double q1[] = { 4,4,3.142 };
    double turning_radius = 1.0;
    DubinsPath path;
    dubins_shortest_path( &path, q0, q1, turning_radius);
    dubins_path_sample_many( &path, 0.1, printConfiguration, NULL);

    char* pszVersion = SIDX_Version();
    fprintf(stdout, "libspatialindex version: %s\n", pszVersion);
    fflush(stdout);
    free(pszVersion);
    return 0;
}
