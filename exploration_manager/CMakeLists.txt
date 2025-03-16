cmake_minimum_required(VERSION 3.5)
project(exploration_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

# Add subdirectory for the Grid module (handles its own CUDA setup)
add_subdirectory(voxel_mapping)

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/exploration_manager_ros.cpp
  src/exploration_manager.cpp
)

# Link against the voxel_mapping CUDA library
target_link_libraries(${LIB_NAME} PUBLIC voxel_mapping_cuda)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  tf2_eigen
  visualization_msgs
  vortex_msgs
)

rclcpp_components_register_node(
    ${LIB_NAME}
    PLUGIN "ExplorationManagerNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME} HAS_LIBRARY_TARGET)
install(
  TARGETS ${LIB_NAME} voxel_mapping_cuda
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
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
