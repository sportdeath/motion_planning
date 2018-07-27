cmake_minimum_required(VERSION 3.1)

project(motion_planning)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Add external projects
add_subdirectory(external)

######################################
# Compile the library >
######################################

# Add includes
include_directories(include)
file(GLOB SRC_FILES src/*.cpp)
add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${LIBS} ${PROJECT_NAME})

######################################
# < End compile the library
######################################

######################################
# Testing >
######################################

# Add the tests
enable_testing()

# from list of files we'll create tests test_name.cpp -> test_name
file(GLOB TEST_SRC_FILES test/*.cpp)
foreach(_test_file ${TEST_SRC_FILES})
    get_filename_component(_test_name ${_test_file} NAME_WE)
    add_executable(${_test_name} ${_test_file})
    target_link_libraries(${_test_name} ${LIBS})
    add_test(${_test_name} ${_test_name})
endforeach()

# Copy the maps into the build directory
file(GLOB MAP_FILES test/maps/*.png)
foreach(_map_file ${MAP_FILES})
    get_filename_component(_map_name ${_map_file} NAME)
    configure_file(${_map_file} maps/${_map_name} COPYONLY)
endforeach()

######################################
# < End testing
######################################
