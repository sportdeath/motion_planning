cmake_minimum_required(VERSION 3.1)

project(motion_planning)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

######################################
# Add external projects >
######################################

include(ExternalProject)

# Add PNG
ExternalProject_Add(png_proj
    GIT_REPOSITORY git://git.code.sf.net/p/libpng/code
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS_RELEASE}
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
    configure_file(${_map_file} maps/${_map_name} COPYONLY)
endforeach()

######################################
# < End testing
######################################
