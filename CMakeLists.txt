cmake_minimum_required(VERSION 3.0)

######################################
# Add external projects

include(ExternalProject)

# Add Dubin's Curves
ExternalProject_Add(dubins_proj
    GIT_REPOSITORY https://github.com/AndrewWalker/Dubins-Curves
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )
ExternalProject_Get_Property(dubins_proj SOURCE_DIR BINARY_DIR)
include_directories(${SOURCE_DIR}/include)
link_directories(${BINARY_DIR})
set(LIBS ${LIBS} dubins)
set(DEPS ${DEPS} dubins_proj)

# Add Spatial Index
ExternalProject_Add(spatialindex_proj
    GIT_REPOSITORY https://github.com/libspatialindex/libspatialindex
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )
ExternalProject_Get_Property(spatialindex_proj SOURCE_DIR BINARY_DIR)
include_directories(${SOURCE_DIR}/include)
link_directories(${BINARY_DIR}/bin)
set(LIBS ${LIBS} spatialindex spatialindex_c)
set(DEPS ${DEPS} spatialindex_proj)

# Add Eigen
ExternalProject_Add(eigen_proj
    GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )
ExternalProject_Get_Property(eigen_proj SOURCE_DIR)
include_directories(${SOURCE_DIR})
set(DEPS ${DEPS} eigen_proj)

# Add PNG
ExternalProject_Add(png_proj
    GIT_REPOSITORY git://git.code.sf.net/p/libpng/code
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )
ExternalProject_Get_Property(png_proj SOURCE_DIR BINARY_DIR)
include_directories(${SOURCE_DIR})
link_directories(${BINARY_DIR})
set(LIBS ${LIBS} png)
set(DEPS ${DEPS} png_proj)

# Finished adding external projects
######################################

# Add includes
include_directories(include)

add_library(motion_planning
    src/DubinsSteer.cpp
    src/OccupancyGrid2D.cpp
    )
set(LIBS ${LIBS} motion_planning)

# Make the test executable
add_executable(test src/test.cpp)
add_dependencies(test ${DEPS})
target_link_libraries(test ${LIBS})
