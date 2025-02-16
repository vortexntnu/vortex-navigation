cmake_minimum_required(VERSION 3.8)
project(wall_detector)

# === C++ standard ===
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  endif()
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (wall_detector src/wall_detector_ros.cpp)
target_link_libraries (wall_detector ${PCL_LIBRARIES})

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/wall_detector_ros.cpp
)

target_link_libraries(${LIB_NAME} PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_action
  rclcpp_components
)

rclcpp_components_register_node(
    ${LIB_NAME}
    PLUGIN "vortex::wall_detector::WallDetectorNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

# Export the target for other packages to use
ament_export_targets(export_${LIB_NAME})

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
