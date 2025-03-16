#include <cuda_runtime.h>
#include <cuda/atomic>

__constant__ float d_intrinsics[4];
__constant__ uint d_image_width;
__constant__ uint d_image_height;
__constant__ uint d_grid_size_x;
__constant__ uint d_grid_size_y;
__constant__ uint d_grid_size_z;
__constant__ float d_resolution;
__constant__ float d_min_depth;
__constant__ float d_max_depth;
__constant__ float d_log_odds_occupied;
__constant__ float d_log_odds_free;
__constant__ float d_log_odds_min;
__constant__ float d_log_odds_max;

// aabb is z-major for locality in grid integration
#define AABB_INDEX(x, y, z, size_x, size_y, size_z) ((x) * (size_y) * (size_z) + (y) * (size_z) + (z))
#define VOXEL_INDEX(x, y, z, size_x, size_y, size_z) ((x) * (size_y) * (size_z) + (y) * (size_z) + (z))

extern "C" void set_intrinsics_d(const float* intrinsics, cudaStream_t stream) {
    cudaMemcpyToSymbolAsync(d_intrinsics, intrinsics, 4 * sizeof(float), 0, cudaMemcpyHostToDevice, stream);
}

extern "C" void set_image_size_d(uint width, uint height, cudaStream_t stream) {
    cudaMemcpyToSymbolAsync(d_image_width, &width, sizeof(uint), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_image_height, &height, sizeof(uint), 0, cudaMemcpyHostToDevice, stream);
}

extern "C" void set_grid_constants_d(uint grid_size_x, uint grid_size_y, uint grid_size_z, float resolution, cudaStream_t stream) {
    cudaMemcpyToSymbolAsync(d_grid_size_x, &grid_size_x, sizeof(uint), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_grid_size_y, &grid_size_y, sizeof(uint), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_grid_size_z, &grid_size_z, sizeof(uint), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_resolution, &resolution, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
}

extern "C" void set_depth_range_d(float min_depth, float max_depth, cudaStream_t stream) {
    cudaMemcpyToSymbolAsync(d_min_depth, &min_depth, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_max_depth, &max_depth, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
}

extern "C" void set_log_odds_properties_d(float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, cudaStream_t stream) {
    cudaMemcpyToSymbolAsync(d_log_odds_occupied, &log_odds_occupied, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_log_odds_free, &log_odds_free, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_log_odds_min, &log_odds_min, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyToSymbolAsync(d_log_odds_max, &log_odds_max, sizeof(float), 0, cudaMemcpyHostToDevice, stream);
}

__global__ void aabb_raycasting_kernel(
    const float* d_depth,
    const float* d_transform,
    char* d_aabb_3d,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= d_image_width || y >= d_image_height) return;

    float fx = d_intrinsics[0];
    float fy = d_intrinsics[1];
    float cx = d_intrinsics[2];
    float cy = d_intrinsics[3];

    float depth = d_depth[y * d_image_width + x];
    if (depth < d_min_depth || depth > d_max_depth || depth <= 0.0f) return;

    float cam_x = (x - cx) * depth / fx;
    float cam_y = (y - cy) * depth / fy;
    float cam_z = depth;

    float world_x = d_transform[0] * cam_x + d_transform[4] * cam_y + d_transform[8] * cam_z + d_transform[12];
    float world_y = d_transform[1] * cam_x + d_transform[5] * cam_y + d_transform[9] * cam_z + d_transform[13];
    float world_z = d_transform[2] * cam_x + d_transform[6] * cam_y + d_transform[10] * cam_z + d_transform[14];
    float w = d_transform[3] * cam_x + d_transform[7] * cam_y + d_transform[11] * cam_z + d_transform[15];
    if (w != 1.0f) {
        world_x /= w;
        world_y /= w;
        world_z /= w;
    }

    float start_x = d_transform[12];
    float start_y = d_transform[13];
    float start_z = d_transform[14];
    float step_size = d_resolution * 0.5f;
    float dx = world_x - start_x;
    float dy = world_y - start_y;
    float dz = world_z - start_z;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    int steps = static_cast<int>(distance / step_size);
    int aabb_size_x = max_x - min_x + 1;
    int aabb_size_y = max_y - min_y + 1;
    int aabb_size_z = max_z - min_z + 1;

    for (int i = 0; i < steps; ++i) {
        float t = static_cast<float>(i) / steps;
        float free_x = start_x + t * dx;
        float free_y = start_y + t * dy;
        float free_z = start_z + t * dz;
        int free_grid_x = static_cast<int>(free_x / d_resolution) - min_x;
        int free_grid_y = static_cast<int>(free_y / d_resolution) - min_y;
        int free_grid_z = static_cast<int>(free_z / d_resolution) - min_z;

        if (free_grid_x < 0 || free_grid_x >= aabb_size_x ||
            free_grid_y < 0 || free_grid_y >= aabb_size_y ||
            free_grid_z < min_z || free_grid_z > max_z) continue;

        int aabb_idx = AABB_INDEX(free_grid_x, free_grid_y, free_grid_z, aabb_size_x, aabb_size_y, aabb_size_z);
        d_aabb_3d[aabb_idx] = 1;
    }

    int end_x = static_cast<int>(world_x / d_resolution) - min_x;
    int end_y = static_cast<int>(world_y / d_resolution) - min_y;
    int end_z = static_cast<int>(world_z / d_resolution) - min_z;

    if (end_x >= 0 && end_x < aabb_size_x &&
        end_y >= 0 && end_y < aabb_size_y &&
        end_z >= min_z && end_z <= max_z) {
        int end_aabb_idx = AABB_INDEX(end_x, end_y, end_z, aabb_size_x, aabb_size_y, aabb_size_z);
        d_aabb_3d[end_aabb_idx] = 2;
    }
}

__global__ void aabb_integration_kernel(
    char* d_aabb_3d,
    float* d_voxel_grid,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
    
    int aabb_x = blockIdx.x * blockDim.x + threadIdx.x;
    int aabb_y = blockIdx.y * blockDim.y + threadIdx.y;
    int aabb_size_x = max_x - min_x + 1;
    int aabb_size_y = max_y - min_y + 1;
    int aabb_size_z = max_z - min_z + 1;

    if (aabb_x >= aabb_size_x || aabb_y >= aabb_size_y) return;

    int global_x = aabb_x + min_x;
    int global_y = aabb_y + min_y;

    if (global_x < 0 || global_x >= d_grid_size_x ||
        global_y < 0 || global_y >= d_grid_size_y) return;

    for (int z = 0; z <= aabb_size_z; ++z) {
        int aabb_idx = AABB_INDEX(aabb_x, aabb_y, z, aabb_size_x, aabb_size_y, aabb_size_z);
        if (d_aabb_3d[aabb_idx] == 0) continue;

        int grid_idx = VOXEL_INDEX(global_x, global_y, z + min_z, d_grid_size_x, d_grid_size_y, d_grid_size_z);
        float update = (d_aabb_3d[aabb_idx] == 1) ? d_log_odds_free : d_log_odds_occupied;
        float new_log_odds = d_voxel_grid[grid_idx] + update;
        new_log_odds = min(max(new_log_odds, d_log_odds_min), d_log_odds_max);
        d_voxel_grid[grid_idx] = new_log_odds;

    }
}

extern "C" void launch_process_depth_kernels(
    const float* d_depth, int width, int height,
    const float* d_transform, float* d_voxel_grid, char* d_aabb,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z,
    cudaStream_t stream) {

    dim3 threads(16, 16);

    dim3 ray_casting_blocks((width + 15) / 16, (height + 15) / 16);
    aabb_raycasting_kernel<<<ray_casting_blocks, threads, 0, stream>>>(
        d_depth, d_transform, d_aabb,
        min_x, max_x, min_y, max_y, min_z, max_z);

    cudaStreamSynchronize(stream);

    int aabb_size_x = max_x - min_x + 1;
    int aabb_size_y = max_y - min_y + 1;
    dim3 integration_blocks((aabb_size_x + 15) / 16, (aabb_size_y + 15) / 16);
    aabb_integration_kernel<<<integration_blocks, threads, 0, stream>>>(
        d_aabb, d_voxel_grid,
        min_x, max_x, min_y, max_y, min_z, max_z);
}