# Add googletest
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/googletest/CMakeLists.txt)
  message("Compiling googletest from git submodule.")
  message("If you do not want this (e.g. you are using catkin_make), run")
  message("git submodule deinit external/googletest")
  add_subdirectory(googletest)
else()
  message("googletest git submodule does not exist, using shared library.")
  message("If you do not want this (e.g. you are not using catkin_make)")
  message("git submodule update --init --recursive")
endif()
set(LIBS ${LIBS} gtest_main)

# Add libpng
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
set(LIBS ${LIBS} ${PNG_LIBRARY})

# Propagate the libraries to the root
set(LIBS ${LIBS} PARENT_SCOPE)
