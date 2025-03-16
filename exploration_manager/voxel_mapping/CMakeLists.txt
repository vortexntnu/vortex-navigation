cmake_minimum_required(VERSION 3.10)
project(voxel_mapping LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "CUDA architectures" FORCE)
set(CMAKE_CUDA_ARCHITECTURES 72;87)

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

include_directories(include)

add_executable(test_voxel_mapping tests/test.cpp)

set(SOURCES
    src/voxel_mapping.cpp
    src/voxel_mapping.cu
)

add_library(voxel_mapping_cuda SHARED ${SOURCES})

target_link_libraries(voxel_mapping_cuda PRIVATE ${CUDA_LIBRARIES} ${CUDA_cudart_LIBRARY})
target_link_libraries(test_voxel_mapping voxel_mapping_cuda ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} Eigen3::Eigen)

target_include_directories(voxel_mapping_cuda PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)

install(
  TARGETS voxel_mapping_cuda
  EXPORT export_voxel_mapping_cuda
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

export(EXPORT export_voxel_mapping_cuda FILE "${CMAKE_CURRENT_BINARY_DIR}/voxel_mapping_cudaTargets.cmake")