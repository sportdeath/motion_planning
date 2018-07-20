cmake_minimum_required(VERSION 3.1)

project(motion_planning)

set (CMAKE_CXX_STANDARD 11)

######################################
# Add external projects >
######################################

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

######################################
# < End add external projects
######################################

######################################
# Compile the library >
######################################

# Add includes
include_directories(include)
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
add_dependencies(${PROJECT_NAME} ${DEPS})
set(LIBS ${LIBS} ${PROJECT_NAME})

######################################
# < End compile the library
######################################

######################################
# Testing >
######################################

# Add Google Test
ExternalProject_Add(gtest_proj
    GIT_REPOSITORY https://github.com/google/googletest.git
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )
ExternalProject_Get_Property(gtest_proj SOURCE_DIR BINARY_DIR)
include_directories(${SOURCE_DIR}/googletest/include)
set(LIBS ${LIBS} ${BINARY_DIR}/googlemock/gtest/libgtest_main.a)
set(LIBS ${LIBS} ${BINARY_DIR}/googlemock/gtest/libgtest.a)

# Add the tests
enable_testing()

file(GLOB TEST_SRC_FILES test/*.cpp)

# from list of files we'll create tests test_name.cpp -> test_name
foreach(_test_file ${TEST_SRC_FILES})
    get_filename_component(_test_name ${_test_file} NAME_WE)
    add_executable(${_test_name} ${_test_file})
    add_dependencies(${_test_name} ${DEPS})
    target_link_libraries(${_test_name} ${LIBS})
    add_test(${_test_name} ${_test_name})
endforeach()

######################################
# < End testing
######################################
