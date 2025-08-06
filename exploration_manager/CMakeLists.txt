cmake_minimum_required(VERSION 3.5)
project(exploration_manager)

option(USE_NVTX "Enable NVTX profiling support" OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(voxel_mapping REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/exploration_manager_ros.cpp
)


if(USE_NVTX)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") # Jetson ARM 64-bit
    set(NVTX_LIBRARY_PATH /usr/lib/aarch64-linux-gnu/libnvToolsExt.so)
    set(NVTX_INCLUDE_DIR /usr/include)
  else() # Desktop x86_64 
    set(NVTX_LIBRARY_PATH /usr/local/cuda-12.8/targets/x86_64-linux/lib/libnvToolsExt.so)
    set(NVTX_INCLUDE_DIR /usr/local/cuda-12.8/include)
  endif()

  if(EXISTS ${NVTX_LIBRARY_PATH})
    message(STATUS "NVTX enabled. Found at ${NVTX_LIBRARY_PATH}")
    set(NVTX_LIBRARY ${NVTX_LIBRARY_PATH})

    target_link_libraries(${LIB_NAME} PRIVATE ${NVTX_LIBRARY})
    target_compile_definitions(${LIB_NAME} PRIVATE USE_NVTX)
    target_include_directories(${LIB_NAME} PRIVATE ${NVTX_INCLUDE_DIR})

  else()
    message(FATAL_ERROR "NVTX requested but not found at ${NVTX_LIBRARY_PATH}")
  endif()
endif()

target_link_libraries(${LIB_NAME} PUBLIC voxel_mapping::voxel_mapping
  spdlog::spdlog
  fmt::fmt
)

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
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  tf2_eigen
  visualization_msgs
  voxel_mapping
)

rclcpp_components_register_node(
    ${LIB_NAME}
    PLUGIN "ExplorationManagerNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME} HAS_LIBRARY_TARGET)
install(
  TARGETS ${LIB_NAME}
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
