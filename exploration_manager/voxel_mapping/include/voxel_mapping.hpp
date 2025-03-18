#ifndef VOXEL_MAPPING_HPP
#define VOXEL_MAPPING_HPP

#include <cstdint>
#include <cuda_runtime.h>
#include <Eigen/Dense>

extern "C" void set_intrinsics_d(const float* intrinsics, cudaStream_t stream);
extern "C" void set_image_size_d(int width, int height, cudaStream_t stream);
extern "C" void set_grid_constants_d(float resolution, uint grid_size_x, uint grid_size_y, uint grid_size_z, cudaStream_t stream);
extern "C" void set_depth_range_d(float min_depth, float max_depth, cudaStream_t stream);
extern "C" void set_log_odds_properties_d(float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, float occupancy_threshold, float free_threshold, cudaStream_t stream);

extern "C" void launch_process_depth_kernels(
    const float* d_depth, int width, int height,
    const float* d_transform, float* d_voxel_grid, char* d_aabb,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z,
    cudaStream_t stream);

extern "C" void launch_extract_2d_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z, cudaStream_t stream);


extern "C" void launch_extract_dilated_2d_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z, int dilation_size,
    cudaStream_t stream); 

class VoxelMapping {
public:
    VoxelMapping(float resolution, uint size_x, uint size_y, uint size_z, float min_depth, float max_depth, float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, float occupancy_threshold, float free_threshold);
    VoxelMapping();
    ~VoxelMapping();
    void set_K(float fx, float fy, float cx, float cy);
    void set_image_size(int width, int height);
    void set_log_odds_properties(float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, float occupancy_threshold, float free_threshold);

    void integrate_depth(const float* depth_image, const Eigen::Matrix4f& transform, const Eigen::VectorXi& aabb_indices);
    
    std::vector<float> get_grid_block(const Eigen::VectorXi& aabb_indices);

    void extract_slice(const Eigen::VectorXi& indices, std::vector<float>& slice);

    void extract_dialated_slice(const Eigen::VectorXi& indices, std::vector<float>& slice, int dialation_size);
    
private:
    void init_grid();
    void allocate_aabb_device(const Eigen::VectorXi& aabb_indices);

    float resolution_;
    uint size_x_, size_y_, size_z_;
    float *d_voxel_grid_;
    float *d_buffer_;
    char *d_aabb_;
    cudaStream_t stream_;
    float fx_, fy_, cx_, cy_;
    float min_depth_, max_depth_;
    int image_width_, image_height_;
    float log_odds_occupied_, log_odds_free_, log_odds_min_, log_odds_max_, occupancy_threshold_, free_threshold_;
};

#endif // VOXEL_MAPPING_HPP