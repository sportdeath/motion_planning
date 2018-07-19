cmake_minimum_required(VERSION 3.1)

project(motion_planning)

set(CMAKE_CXX_STANDARD 11)

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
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${LIBS} ${PROJECT_NAME})

######################################
# < End compile the library
######################################

######################################
# Testing >
######################################

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# Add the google test library
set(LIBS ${LIBS} gtest_main)

# Add the tests
enable_testing()

# from list of files we'll create tests test_name.cpp -> test_name
file(GLOB TEST_SRC_FILES test/*.cpp)
foreach(_test_file ${TEST_SRC_FILES})
    get_filename_component(_test_name ${_test_file} NAME_WE)
    add_executable(${_test_name} ${_test_file})
    add_dependencies(${_test_name} ${DEPS})
    target_link_libraries(${_test_name} ${LIBS})
    add_test(${_test_name} ${_test_name})
endforeach()

# Copy the maps into the build directory
file(GLOB MAP_FILES test/maps/*.png)
foreach(_map_file ${MAP_FILES})
    get_filename_component(_map_name ${_map_file} NAME)
    message(${_map_name})
    configure_file(${_map_file} maps/${_map_name} COPYONLY)
endforeach()

######################################
# < End testing
######################################
