# Add googletest
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/googletest/CMakeLists.txt)
  set(GTEST_FOUND TRUE)
  add_subdirectory(googletest)
endif()
set(LIBS ${LIBS} gtest_main)

# Add libpng
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
set(LIBS ${LIBS} ${PNG_LIBRARY})

# Propagate the libraries to the root
set(LIBS ${LIBS} PARENT_SCOPE)
