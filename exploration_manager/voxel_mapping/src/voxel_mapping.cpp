#include <voxel_mapping.hpp>
#include <iostream>
#include <vector>

VoxelMapping::VoxelMapping(float resolution, uint size_x, uint size_y, uint size_z, float min_depth, float max_depth, float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, float occupancy_threshold, float free_threshold)
    : size_x_(size_x), size_y_(size_y), size_z_(size_z), min_depth_(min_depth), max_depth_(max_depth), log_odds_occupied_(log_odds_occupied), log_odds_free_(log_odds_free), log_odds_min_(log_odds_min), log_odds_max_(log_odds_max), occupancy_threshold_(occupancy_threshold), free_threshold_(free_threshold) {
    if (resolution <= 0) {
        throw std::invalid_argument("Resolution must be positive");
    }
    resolution_ = resolution;
    cudaStreamCreate(&stream_);
    init_grid();
}

VoxelMapping::VoxelMapping() {

}

void VoxelMapping::init_grid() {
    size_t matrix_size = size_x_ * size_y_ * size_z_ * sizeof(float);

    cudaError_t err1 = cudaMalloc((void**)&d_voxel_grid_, matrix_size);

    cudaMemset(d_voxel_grid_, 0, matrix_size);

    if (err1 != cudaSuccess) {
        std::cerr << "CUDA malloc failed for voxel grid: " << cudaGetErrorString(err1) << std::endl;
        exit(EXIT_FAILURE);
    }

    set_grid_constants_d(resolution_, size_x_, size_y_, size_z_, stream_);
    set_depth_range_d(min_depth_, max_depth_, stream_);
    set_log_odds_properties_d(log_odds_occupied_, log_odds_free_, log_odds_min_, log_odds_max_, occupancy_threshold_, free_threshold_, stream_);

    std::cout << "Voxel grid initialized on GPU with size " << size_x_ << "x" << size_y_ << "x" << size_z_ << std::endl;
}

void VoxelMapping::set_K(float fx, float fy, float cx, float cy) {
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    float intrinsics[4] = {static_cast<float>(fx), static_cast<float>(fy), 
                          static_cast<float>(cx), static_cast<float>(cy)};
    set_intrinsics_d(intrinsics, stream_);
}

void VoxelMapping::set_image_size(int width, int height) {
    image_width_ = width;
    image_height_ = height;
    set_image_size_d(width, height, stream_);
}

void VoxelMapping::set_log_odds_properties(float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, float occupancy_threshold, float free_threshold) {
    set_log_odds_properties_d(log_odds_occupied, log_odds_free, log_odds_min, log_odds_max, occupancy_threshold, free_threshold, stream_);
}

void VoxelMapping::allocate_aabb_device(const Eigen::VectorXi& aabb_indices) {
    int min_x = aabb_indices[0];
    int max_x = aabb_indices[1];
    int min_y = aabb_indices[2];
    int max_y = aabb_indices[3];
    int min_z = aabb_indices[4];
    int max_z = aabb_indices[5];

    int aabb_size_x = max_x - min_x + 1;
    int aabb_size_y = max_y - min_y + 1;
    int aabb_size_z = max_z - min_z + 1;

    size_t total_size = aabb_size_x * aabb_size_y * aabb_size_z * sizeof(char);

    cudaError_t err = cudaMalloc((void**)&d_aabb_, total_size);
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc failed for 3D AABB: " << cudaGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }
    err = cudaMemset(d_aabb_, 0, total_size);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memset failed for 3D AABB: " << cudaGetErrorString(err) << std::endl;
        cudaFree(d_aabb_);
        exit(EXIT_FAILURE);
    }
}

std::vector<float> VoxelMapping::get_grid_block(const Eigen::VectorXi& aabb_indices) {
    cudaStreamSynchronize(stream_); // Ensure all kernel operations are complete

    int grid_min_x = aabb_indices[0];
    int grid_max_x = aabb_indices[1];
    int grid_min_y = aabb_indices[2];
    int grid_max_y = aabb_indices[3];
    int grid_min_z = aabb_indices[4];
    int grid_max_z = aabb_indices[5];

    size_t aabb_size_x = grid_max_x - grid_min_x + 1;
    size_t aabb_size_y = grid_max_y - grid_min_y + 1;
    size_t aabb_size_z = grid_max_z - grid_min_z + 1;

    size_t total_elements = aabb_size_x * aabb_size_y * aabb_size_z;
    std::vector<float> block(total_elements);

    cudaMemcpy3DParms copyParams = {0};

    // Source: d_voxel_grid (z-major order: z varies fastest, then y, then x)
    // The pitch is the stride between y-planes (size_x * size_z * sizeof(float))
    copyParams.srcPtr = make_cudaPitchedPtr(
        d_voxel_grid_ + (grid_min_x * size_y_ * size_z_ + grid_min_y * size_z_ + grid_min_z),
        size_z_ * sizeof(float), // Pitch (stride in bytes between y-planes, z-major)
        size_z_,                 // Width in elements (z-dimension)
        size_y_                  // Height in elements (y-dimension)
    );

    // Destination: block (z-major order)
    // Since block is a flat vector, the pitch matches the z-dimension size
    copyParams.dstPtr = make_cudaPitchedPtr(
        block.data(),
        aabb_size_z * sizeof(float), // Pitch (stride in bytes between y-planes in AABB)
        aabb_size_z,                 // Width in elements (z-dimension of AABB)
        aabb_size_y                  // Height in elements (y-dimension of AABB)
    );

    // Extent of the region to copy (AABB subregion in z-major order)
    copyParams.extent = make_cudaExtent(
        aabb_size_z * sizeof(float), // Width in bytes (z-dimension)
        aabb_size_y,                 // Height in elements (y-dimension)
        aabb_size_x                  // Depth in elements (x-dimension)
    );

    copyParams.kind = cudaMemcpyDeviceToHost;

    cudaError_t err = cudaMemcpy3D(&copyParams);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy3D failed in get_grid_block: " << cudaGetErrorString(err) << std::endl;
        throw std::runtime_error("Failed to copy AABB block from GPU");
    }

    return block;
}

void VoxelMapping::extract_slice(const Eigen::VectorXi& indices, std::vector<float>& slice) {
    int min_x = indices[0];
    int max_x = indices[1];
    int min_y = indices[2];
    int max_y = indices[3];
    int min_z = indices[4];
    int max_z = indices[5];

    size_t slice_size_x = max_x - min_x + 1;
    size_t slice_size_y = max_y - min_y + 1;
    size_t slice_size_z = max_z - min_z + 1;

    size_t slice_size = slice_size_x * slice_size_y;

    slice.resize(slice_size);

    float* d_slice;

    cudaError_t err = cudaMalloc(&d_slice, slice_size * sizeof(float));
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc failed for 2d slice: " << cudaGetErrorString(err) << std::endl;
    }

    err = cudaMemset(d_slice, -1.0, slice_size * sizeof(float));
    if (err != cudaSuccess) {
        std::cerr << "CUDA memset failed for 2d slice: " << cudaGetErrorString(err) << std::endl;
    }

    launch_extract_2d_slice_kernel(d_voxel_grid_, d_slice, min_x, max_x, min_y, max_y, min_z, max_z, stream_);

    cudaStreamSynchronize(stream_);

    err = cudaMemcpy(slice.data(), d_slice, slice_size * sizeof(float), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy failed for 2d slice: " << cudaGetErrorString(err) << std::endl;
    }

    cudaFree(d_slice);
}

void VoxelMapping::extract_dialated_slice(const Eigen::VectorXi& indices, std::vector<float>& slice, int dilation_size) {
    int min_x = indices[0];
    int max_x = indices[1];
    int min_y = indices[2];
    int max_y = indices[3];
    int min_z = indices[4];
    int max_z = indices[5];

    size_t slice_size_x = max_x - min_x + 1;
    size_t slice_size_y = max_y - min_y + 1;
    size_t slice_size_z = max_z - min_z + 1;

    size_t slice_size = slice_size_x * slice_size_y;

    slice.resize(slice_size);

    float* d_slice;

    cudaError_t err = cudaMalloc(&d_slice, slice_size * sizeof(float));
    if (err != cudaSuccess) {
        std::cerr << "CUDA malloc failed for 2d slice: " << cudaGetErrorString(err) << std::endl;
    }

    err = cudaMemset(d_slice, -1.0, slice_size * sizeof(float));
    if (err != cudaSuccess) {
        std::cerr << "CUDA memset failed for 2d slice: " << cudaGetErrorString(err) << std::endl;
    }

    launch_extract_dilated_2d_slice_kernel(d_voxel_grid_, d_slice, min_x, max_x, min_y, max_y, min_z, max_z, dilation_size, stream_);

    cudaStreamSynchronize(stream_);

    err = cudaMemcpy(slice.data(), d_slice, slice_size * sizeof(float), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy failed for 2d slice: " << cudaGetErrorString(err) << std::endl;
    }

    cudaFree(d_slice);
}

void VoxelMapping::integrate_depth(const float* depth_image, const Eigen::Matrix4f& transform, const Eigen::VectorXi& aabb_indices) {
    size_t depth_size = image_width_ * image_height_ * sizeof(float);
    size_t transform_size = 16 * sizeof(float);
    size_t total_size = transform_size + depth_size;

    std::vector<float> host_buffer(16 + image_width_ * image_height_);
    memcpy(host_buffer.data(), transform.data(), transform_size);
    memcpy(host_buffer.data() + 16, depth_image, depth_size);
    if (!d_buffer_) {
        cudaError_t err = cudaMalloc(&d_buffer_, total_size);
        if (err != cudaSuccess) {
            std::cerr << "CUDA malloc failed for buffer: " << cudaGetErrorString(err) << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    cudaError_t err = cudaMemcpyAsync(d_buffer_, host_buffer.data(), total_size, cudaMemcpyHostToDevice, stream_);
    if (err != cudaSuccess) {
        std::cerr << "CUDA memcpy failed for buffer: " << cudaGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }

    float* d_transform = d_buffer_;
    float* d_depth = d_buffer_ + 16;

    allocate_aabb_device(aabb_indices);

    launch_process_depth_kernels(d_depth, image_width_, image_height_,
        d_transform, d_voxel_grid_, d_aabb_,
        aabb_indices[0], aabb_indices[1], aabb_indices[2], aabb_indices[3], aabb_indices[4], aabb_indices[5],
        stream_);

    // Synchronize before freeing memory
    err = cudaStreamSynchronize(stream_);
    if (err != cudaSuccess) {
        std::cerr << "CUDA stream synchronize failed: " << cudaGetErrorString(err) << std::endl;
        cudaFree(d_aabb_);
        cudaFree(d_buffer_);
        exit(EXIT_FAILURE);
    }
    cudaFree(d_buffer_);
    d_buffer_ = nullptr; // Reset pointer to avoid dangling references
    cudaFree(d_aabb_);
    d_aabb_ = nullptr; // Reset pointer to avoid dangling references
}

VoxelMapping::~VoxelMapping() {
    cudaStreamDestroy(stream_);
    cudaFree(d_voxel_grid_);
    if (d_buffer_) {
        cudaFree(d_buffer_);
    }
    if (d_aabb_) {
        cudaFree(d_aabb_);
    }
}