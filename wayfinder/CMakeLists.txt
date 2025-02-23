cmake_minimum_required(VERSION 3.5)
project(pipeline_line_fitting)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#find correct dicts
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

set(LIB_NAME "${PROJECT_NAME}_component")


#add correct modules
add_library(${LIB_NAME} SHARED
    src/wayfinder.cpp
    src/grid.cpp
)

target_link_libraries(${LIB_NAME} PUBLIC
  ${OpenCV_LIBS}
  ${ARMADILLO_LIBRARIES}
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  cv_bridge
)

rclcpp_components_register_node(
    ${LIB_NAME}
    PLUGIN "WayfinderNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

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
