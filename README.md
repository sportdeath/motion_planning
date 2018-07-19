# Building

To build the library independently do:

    cd motion_planning
    mkdir build
    cmake ..
    make

To integrate this project into another CMake project add the following to ```CMakeLists.txt```:

    ExternalProject_Add(motion_planning_proj
        GIT_REPOSITORY https://github.com/sportdeath/motion_planning
        UPDATE_COMMAND ""
        INSTALL_COMMAND ""
        )
    ExternalProject_Get_Property(motion_planning_proj SOURCE_DIR BINARY_DIR)
    include_directories(${SOURCE_DIR}/include)
    link_directories(${BINARY_DIR})
    set(LIBS ${LIBS} dubins)
    set(DEPS ${DEPS} dubins_proj)
