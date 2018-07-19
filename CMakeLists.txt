cmake_minimum_required(VERSION 3.0)

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
    )
ExternalProject_Get_Property(spatialindex_proj SOURCE_DIR BINARY_DIR)
include_directories(${SOURCE_DIR}/include)
link_directories(${BINARY_DIR}/bin)
set(LIBS ${LIBS} spatialindex spatialindex_c)
set(DEPS ${DEPS} spatialindex_proj)

# Make the test executable
add_executable(test src/test.cpp)
add_dependencies(test ${DEPS})
target_link_libraries(test ${LIBS})
