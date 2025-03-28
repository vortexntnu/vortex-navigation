cmake_minimum_required(VERSION 3.10)

# Project name
project(waypoints)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

include_directories(include)

# Source files
set(SOURCES
    src/waypointFinder.cpp
)

# Add the library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Eigen3::Eigen)

target_include_directories(waypoints PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
        EXPORT waypoints_export
        LIBRARY DESTINATION lib)

install(DIRECTORY include/ 
DESTINATION include)

export(EXPORT waypoints_export FILE "${CMAKE_CURRENT_BINARY_DIR}/waypointsTargets.cmake")