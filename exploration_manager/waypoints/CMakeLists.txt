cmake_minimum_required(VERSION 3.10)

# Project name
project(waypoints)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/waypoints.cpp
)

# Add the library
add_library(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)