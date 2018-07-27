# Add googletest
add_subdirectory(googletest)
set(LIBS ${LIBS} gtest_main)

# Add libpng
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
set(LIBS ${LIBS} ${PNG_LIBRARY})

# Propagate the libraries to the root
set(LIBS ${LIBS} PARENT_SCOPE)
