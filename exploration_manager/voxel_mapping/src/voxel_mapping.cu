#include <cuda_runtime.h>
#include <cuda/atomic>
#include <iostream>

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
__constant__ float d_occupancy_threshold;
__constant__ float d_free_threshold;

// aabb is z-major for locality in grid integration
#define AABB_INDEX(x, y, z, size_x, size_y, size_z) ((x) * (size_y) * (size_z) + (y) * (size_z) + (z))
#define VOXEL_INDEX(x, y, z, size_x, size_y, size_z) ((x) * (size_y) * (size_z) + (y) * (size_z) + (z))
#define SLICE_INDEX(x, y, size_y) ((x) * (size_y) + (y))

extern "C" void set_intrinsics_d(const float* intrinsics) {
    cudaMemcpyToSymbol(d_intrinsics, intrinsics, 4 * sizeof(float), 0, cudaMemcpyHostToDevice);
}

extern "C" void set_image_size_d(uint width, uint height) {
    cudaMemcpyToSymbol(d_image_width, &width, sizeof(uint), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_image_height, &height, sizeof(uint), 0, cudaMemcpyHostToDevice);
}

extern "C" void set_grid_constants_d(uint grid_size_x, uint grid_size_y, uint grid_size_z, float resolution) {
    cudaMemcpyToSymbol(d_grid_size_x, &grid_size_x, sizeof(uint), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_grid_size_y, &grid_size_y, sizeof(uint), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_grid_size_z, &grid_size_z, sizeof(uint), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_resolution, &resolution, sizeof(float), 0, cudaMemcpyHostToDevice);
}

extern "C" void set_depth_range_d(float min_depth, float max_depth) {
    cudaMemcpyToSymbol(d_min_depth, &min_depth, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_max_depth, &max_depth, sizeof(float), 0, cudaMemcpyHostToDevice);
}

extern "C" void set_log_odds_properties_d(float log_odds_occupied, float log_odds_free, float log_odds_min, float log_odds_max, float occupancy_threshold, float free_threshold) {
    cudaMemcpyToSymbol(d_log_odds_occupied, &log_odds_occupied, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_log_odds_free, &log_odds_free, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_log_odds_min, &log_odds_min, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_log_odds_max, &log_odds_max, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_occupancy_threshold, &occupancy_threshold, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(d_free_threshold, &free_threshold, sizeof(float), 0, cudaMemcpyHostToDevice);
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
    if (depth < d_min_depth || depth <= 0.0f) return;

    bool max_depth = false;
    if (depth > d_max_depth) {
        depth = d_max_depth;
        max_depth = true;
    }

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
            free_grid_z < 0 || free_grid_z > aabb_size_z) continue;

        int aabb_idx = AABB_INDEX(free_grid_x, free_grid_y, free_grid_z, aabb_size_x, aabb_size_y, aabb_size_z);
        d_aabb_3d[aabb_idx] = 1;
    }

    if (max_depth) return;

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

__global__ void extract_2d_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= max_x - min_x + 1 || y >= max_y - min_y + 1) return;

    int global_x = x + min_x;
    int global_y = y + min_y;

    if (global_x < 0 || global_x >= d_grid_size_x ||
        global_y < 0 || global_y >= d_grid_size_y) return;

    int slice_size_x = max_x - min_x + 1;
    int slice_size_y = max_y - min_y + 1;

    for (int z = min_z; z <= max_z; ++z) {
        int grid_idx = VOXEL_INDEX(global_x, global_y, z, d_grid_size_x, d_grid_size_y, d_grid_size_z);
        float log_odds = d_voxel_grid[grid_idx];
        
        if (log_odds >= d_occupancy_threshold) {
            d_slice[SLICE_INDEX(x, y, slice_size_y)] = 1.0f;
            break;
        }
        else if (log_odds <= d_free_threshold) {
            d_slice[SLICE_INDEX(x, y, slice_size_y)] = 0.0f;
        }
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

    cudaError_t err = cudaStreamSynchronize(stream);
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error after aabb_raycasting_kernel: " << cudaGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }

    int aabb_size_x = max_x - min_x + 1;
    int aabb_size_y = max_y - min_y + 1;
    dim3 integration_blocks((aabb_size_x + 15) / 16, (aabb_size_y + 15) / 16);
    aabb_integration_kernel<<<integration_blocks, threads, 0, stream>>>(
        d_aabb, d_voxel_grid,
        min_x, max_x, min_y, max_y, min_z, max_z);
}

extern "C" void launch_extract_2d_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z,
    cudaStream_t stream) {

    dim3 threads(16, 16);
    dim3 blocks((max_x - min_x + 15) / 16, (max_y - min_y + 15) / 16);
    extract_2d_slice_kernel<<<blocks, threads, 0, stream>>>(
        d_voxel_grid, d_slice,
        min_x, max_x, min_y, max_y, min_z, max_z);
}

__global__ void extract_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
    
    int slice_x = blockIdx.x * blockDim.x + threadIdx.x;
    int slice_y = blockIdx.y * blockDim.y + threadIdx.y;
    int slice_size_x = max_x - min_x + 1;
    int slice_size_y = max_y - min_y + 1;

    if (slice_x >= slice_size_x || slice_y >= slice_size_y) return;
    
    int global_x = slice_x + min_x;
    int global_y = slice_y + min_y;
    
    if (global_x < 0 || global_x >= d_grid_size_x ||
        global_y < 0 || global_y >= d_grid_size_y) return;

    float state = -1.0f;  // Default: unknown
    for (int z = min_z; z <= max_z; ++z) {
        int grid_idx = VOXEL_INDEX(global_x, global_y, z, d_grid_size_x, d_grid_size_y, d_grid_size_z);
        float log_odds = d_voxel_grid[grid_idx];

        if (log_odds >= d_occupancy_threshold) {
            state = 1.0f;  // Occupied
            break;
        } else if (log_odds <= d_free_threshold) {
            state = 0.0f;  // Free
        }
    }

    d_slice[SLICE_INDEX(slice_x, slice_y, slice_size_x)] = state;
}

__global__ void dilate_x_kernel(
    float* src, float* dst, int radio, int width, int height, int tile_w, int tile_h) {
    extern __shared__ float smem[];
    int tx = threadIdx.x;
    int ty = threadIdx.y;
    int bx = blockIdx.x;
    int by = blockIdx.y;
    int x = bx * tile_w + tx - radio;
    int y = by * tile_h + ty;
    smem[ty * blockDim.x + tx] = -1.0f;
    __syncthreads();
    if (x < 0 || x >= width || y >= height) return;
    smem[ty * blockDim.x + tx] = src[y * width + x];
    __syncthreads();
    if (x < (bx * tile_w) || x >= ((bx + 1) * tile_w)) return;
    float* smem_thread = &smem[ty * blockDim.x + tx - radio];
    float val = smem_thread[0];
    for (int xx = 1; xx <= 2 * radio; xx++) {
        val = max(val, smem_thread[xx]);
    }
    dst[y * width + x] = val;
}

__global__ void dilate_y_kernel(
    float* src, float* dst, int radio, int width, int height, int tile_w, int tile_h) {
    extern __shared__ float smem[];
    int tx = threadIdx.x;
    int ty = threadIdx.y;
    int bx = blockIdx.x;
    int by = blockIdx.y;
    int x = bx * tile_w + tx;
    int y = by * tile_h + ty - radio;
    smem[ty * blockDim.x + tx] = -1.0f;
    __syncthreads();
    if (x >= width || y < 0 || y >= height) return;
    smem[ty * blockDim.x + tx] = src[y * width + x];
    __syncthreads();
    if (y < (by * tile_h) || y >= ((by + 1) * tile_h)) return;
    float* smem_thread = &smem[(ty - radio) * blockDim.x + tx];
    float val = smem_thread[0];
    for (int yy = 1; yy <= 2 * radio; yy++) {
        val = max(val, smem_thread[yy * blockDim.x]);
    }
    dst[y * width + x] = val;
}



void launch_dilate_slice_2d(
    float* src, float* dst, float* temp, int width, int height, int dilation_size) {
    // X-Kernel: Ensure blockDim.x is a multiple of 32
    int base_tile_w_x = 512;  // Starting point, power of 2
    int halo_size = 2 * dilation_size;
    int remainder = halo_size % 32;
    int tile_w_x = base_tile_w_x - remainder;  // Adjust to make blockDim.x a multiple of 32
    if (tile_w_x + halo_size > 1024) {
        tile_w_x = 1024 - halo_size;  // Cap at max threads per block
    }
    int tile_h_x = 1;
    dim3 block_x(tile_w_x + halo_size, tile_h_x);  // blockDim.x = tile_w + 2 * dilation_size
    dim3 grid_x(ceil((float)width / tile_w_x), ceil((float)height / tile_h_x));
    
    // Verify blockDim.x is a multiple of 32
    assert((tile_w_x + halo_size) % 32 == 0);
    assert((tile_w_x + halo_size) <= 1024);

    dilate_x_kernel<<<grid_x, block_x, block_x.y * block_x.x * sizeof(float)>>>(
        src, temp, dilation_size, width, height, tile_w_x, tile_h_x);
    cudaDeviceSynchronize();

    // Y-Kernel: Ensure blockDim.y is a multiple of 32, blockDim.x is small and multiple of 2
    int tile_w_y = 16;  // Fixed, multiple of 2, balances total threads
    int base_tile_h_y = 64;  // Starting point, multiple of 32
    int tile_h_y = base_tile_h_y - remainder;  // Adjust for blockDim.y
    if (tile_w_y * (tile_h_y + halo_size) > 1024) {
        tile_h_y = (1024 / tile_w_y) - halo_size;  // Cap total threads
    }
    dim3 block_y(tile_w_y, tile_h_y + halo_size);
    dim3 grid_y(ceil((float)width / tile_w_y), ceil((float)height / tile_h_y));
    
    // Verify blockDim.y is a multiple of 32 and total threads <= 1024
    assert((tile_h_y + halo_size) % 32 == 0);
    assert(tile_w_y * (tile_h_y + halo_size) <= 1024);

    dilate_y_kernel<<<grid_y, block_y, block_y.y * block_y.x * sizeof(float)>>>(
        temp, dst, dilation_size, width, height, tile_w_y, tile_h_y);
    cudaDeviceSynchronize();
}

__global__ void extract_dilated_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z, int dilation_size) {
    
    extern __shared__ float shared_voxel[];

    int slice_x = blockIdx.x * blockDim.x + threadIdx.x;
    int slice_y = blockIdx.y * blockDim.y + threadIdx.y;
    int slice_size_x = max_x - min_x + 1;
    int slice_size_y = max_y - min_y + 1;

    if (slice_x >= slice_size_x || slice_y >= slice_size_y) return;
    
    int global_x = slice_x + min_x;
    int global_y = slice_y + min_y;
    
    if (global_x < 0 || global_x >= d_grid_size_x ||
        global_y < 0 || global_y >= d_grid_size_y) return;

    for (int z = min_z; z <= max_z; ++z) {
        int grid_idx = VOXEL_INDEX(global_x, global_y, z, d_grid_size_x, d_grid_size_y, d_grid_size_z);
        float log_odds = d_voxel_grid[grid_idx];
        
        if (log_odds >= d_occupancy_threshold) {
            d_slice[SLICE_INDEX(slice_x, slice_y, slice_size_y)] = 1.0f;
            break;
        }
        else if (log_odds <= d_free_threshold) {
            d_slice[SLICE_INDEX(slice_x, slice_y, slice_size_y)] = 0.0f;
        }
    }
        
    __syncthreads();
    
    int shared_size_y = blockDim.y + dilation_size * 2;
                                
    #define SHARED_INDEX(x, y) ((x + dilation_size) * shared_size_y + (y + dilation_size))

    int block_x = threadIdx.x;
    int block_y = threadIdx.y;

    if (block_x < dilation_size && slice_x - dilation_size >= 0) {
        shared_voxel[SHARED_INDEX(block_x - dilation_size, block_y)] = d_slice[SLICE_INDEX(slice_x - dilation_size, slice_y, slice_size_y)];
    }
    if (block_x >= blockDim.x - dilation_size && slice_x + dilation_size < d_grid_size_x) {
        shared_voxel[SHARED_INDEX(block_x + dilation_size, block_y)] = d_slice[SLICE_INDEX(slice_x + dilation_size, slice_y, slice_size_y)];
    }
    if (block_y < dilation_size && slice_y - dilation_size >= 0) {
        shared_voxel[SHARED_INDEX(block_x, block_y - dilation_size)] = d_slice[SLICE_INDEX(slice_x, slice_y - dilation_size, slice_size_y)];
    }
    if (block_y >= blockDim.y - dilation_size && slice_y + dilation_size < d_grid_size_y) {
        shared_voxel[SHARED_INDEX(block_x, block_y + dilation_size)] = d_slice[SLICE_INDEX(slice_x, slice_y + dilation_size, slice_size_y)];
    }

    shared_voxel[SHARED_INDEX(block_x, block_y)] = d_slice[SLICE_INDEX(slice_x, slice_y, slice_size_y)];

    __syncthreads();

    if(shared_voxel[SHARED_INDEX(block_x, block_y)] == 1.0f) {
        int count = 0;
        count += shared_voxel[SHARED_INDEX(block_x - 1, block_y - 1)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x, block_y - 1)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x + 1, block_y - 1)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x - 1, block_y)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x + 1, block_y)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x - 1, block_y + 1)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x, block_y + 1)] == 1.0f;
        count += shared_voxel[SHARED_INDEX(block_x + 1, block_y + 1)] == 1.0f;
        if (count < 2) {
            shared_voxel[SHARED_INDEX(block_x, block_y)] = 0.0f;
        }
    }
    if(shared_voxel[SHARED_INDEX(block_x, block_y)] == 1.0f) {
        for (int i = -dilation_size; i <= dilation_size; ++i) {
            for (int j = -dilation_size; j <= dilation_size; ++j) {
                shared_voxel[SHARED_INDEX(block_x + i, block_y + j)] = 1.0f;
            }    
        }
    }

    __syncthreads();

    d_slice[SLICE_INDEX(slice_x, slice_y, slice_size_y)] = shared_voxel[SHARED_INDEX(block_x, block_y)];
}

extern "C" void launch_extract_dilated_2d_slice_kernel(
    const float* d_voxel_grid, float* d_slice,
    int min_x, int max_x, int min_y, int max_y, int min_z, int max_z, int dilation_size,
    cudaStream_t stream) {

    dim3 threads(16, 16);
    dim3 blocks((max_x - min_x + threads.x - 1) / threads.x, 
                (max_y - min_y + threads.y - 1) / threads.y);

    int shared_memory_size = (threads.x + dilation_size * 2) * (threads.y + dilation_size * 2) * sizeof(float);

    extract_dilated_slice_kernel<<<blocks, threads, shared_memory_size, stream>>>(
        d_voxel_grid, d_slice, min_x, max_x, min_y, max_y, min_z, max_z, dilation_size);
}

