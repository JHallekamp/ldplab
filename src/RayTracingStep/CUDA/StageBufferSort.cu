#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageBufferSort.hpp"

namespace
{
    constexpr size_t block_size = 256;

    __device__ size_t num_rays;
    __global__ void countPivotLocal(
        int32_t pivot_element,
        int32_t* ray_particle_index,
        uint32_t* local_rank,
        uint32_t* block_sizes)
    {
        // Shared memory
        extern __shared__ uint32_t sbuf[];

        const uint32_t tid = threadIdx.x;
        const uint32_t gi = blockIdx.x * blockDim.x + tid;

        if (gi >= num_rays)
            return;

        sbuf[tid] = (ray_particle_index[gi] >= pivot_element ? 1 : 0);
        __syncthreads();

        // Count local pivot elements
        uint32_t rank = sbuf[tid];
        for (uint32_t i = 0; i < tid; ++i)
            rank += sbuf[i];

        // Write back results
        local_rank[gi] = sbuf[tid] * rank;
        if (tid + 1 == blockDim.x)
            block_sizes[blockIdx.x] = rank;
    }

    __device__ uint32_t num_pivot_elements;
    __global__ void countPivotSum(uint32_t* block_sizes)
    {
        const size_t num_blocks =
            num_rays / block_size + (num_rays % block_size ? 1 : 0);
        uint32_t ctr = 0;
        for (size_t i = 0; i < num_blocks; ++i)
        {
            uint32_t t = block_sizes[i];
            block_sizes[i] = ctr;
            ctr += t;
        }
        num_pivot_elements = ctr;
    }

    __global__ void buildRanks(
        uint32_t* local_rank,
        uint32_t* block_sizes,
        uint32_t* rank_index_range)
    {
        // Shared memory
        extern __shared__ uint32_t sbuf[];

        const uint32_t sz = num_rays - num_pivot_elements;
        const uint32_t tid = threadIdx.x;
        const uint32_t gi = blockIdx.x * blockDim.x + tid;

        if (gi >= num_rays)
            return;

        sbuf[tid] = (local_rank[gi] > 0 ? 0 : 1);
        __syncthreads();

        if (gi >= sz)
        {
            // Build local rank
            uint32_t rank = sbuf[tid];
            for (uint32_t i = 0; i < tid; ++i)
                rank += sbuf[i];

            // Save results
            rank_index_range[gi] = sbuf[tid] * rank;
            if (tid + 1 == blockDim.x)
                block_sizes[blockIdx.x + 1] = rank;
        }
        else if (sbuf[tid] == 0)
        {
            // Build global rank and write into rank index range
            uint32_t rank = local_rank[gi] + block_sizes[blockIdx.x];
            rank_index_range[rank - 1] = gi;
        }
    }

    __global__ void swapBuffer(
        uint32_t* rank_index_range,
        uint32_t* block_sizes,
        int32_t* ray_buffer_particle_index,
        ldplab::Vec3* ray_buffer_origin,
        ldplab::Vec3* ray_buffer_direction,
        double* ray_buffer_intensity,
        double* ray_buffer_min_bv_dist,
        int32_t* intersection_buffer_index,
        ldplab::Vec3* intersection_buffer_point,
        ldplab::Vec3* intersection_buffer_normal,
        ldplab::Vec3* output_buffer_force_per_ray,
        ldplab::Vec3* output_buffer_torque_per_ray,
        bool swap_isec_and_output_data)
    {
        const uint32_t sz = num_rays - num_pivot_elements;
        const uint32_t gi = blockIdx.x * blockDim.x + threadIdx.x;

        if (gi < sz || gi >= num_rays)
            return;

        uint32_t rank = rank_index_range[gi];
        if (rank == 0)
            return;

        const uint32_t blk0 = sz / blockDim.x + 1;
        const uint32_t blki = gi / blockDim.x + 1;
        for (uint32_t i = blk0; i < blki; ++i)
            rank += block_sizes[i];

        const uint32_t target_idx = rank_index_range[rank - 1];

        // Swap buffer contents
        const auto t_particle_index = ray_buffer_particle_index[target_idx];
        ray_buffer_particle_index[target_idx] = ray_buffer_particle_index[gi];
        ray_buffer_particle_index[gi] = t_particle_index;
        
        const auto t_origin = ray_buffer_origin[target_idx];
        ray_buffer_origin[target_idx] = ray_buffer_origin[gi];
        ray_buffer_origin[gi] = t_origin;
        
        const auto t_direction = ray_buffer_direction[target_idx];
        ray_buffer_direction[target_idx] = ray_buffer_direction[gi];
        ray_buffer_direction[gi] = t_direction;
        
        const auto t_intensity = ray_buffer_intensity[target_idx];
        ray_buffer_intensity[target_idx] = ray_buffer_intensity[gi];
        ray_buffer_intensity[gi] = t_intensity;
        
        const auto t_min_bv_dist = ray_buffer_min_bv_dist[target_idx];
        ray_buffer_min_bv_dist[target_idx] = ray_buffer_min_bv_dist[gi];
        ray_buffer_min_bv_dist[gi] = t_min_bv_dist;

        if (swap_isec_and_output_data)
        {
            const auto t_index = intersection_buffer_index[target_idx];
            intersection_buffer_index[target_idx] = intersection_buffer_index[gi];
            intersection_buffer_index[gi] = t_index;

            const auto t_point = intersection_buffer_point[target_idx];
            intersection_buffer_point[target_idx] = intersection_buffer_point[gi];
            intersection_buffer_point[gi] = t_point;

            const auto t_normal = intersection_buffer_normal[target_idx];
            intersection_buffer_normal[target_idx] = intersection_buffer_normal[gi];
            intersection_buffer_normal[gi] = t_normal;

            const auto t_force_per_ray = output_buffer_force_per_ray[target_idx];
            output_buffer_force_per_ray[target_idx] = output_buffer_force_per_ray[gi];
            output_buffer_force_per_ray[gi] = t_force_per_ray;

            const auto t_torque_per_ray = output_buffer_torque_per_ray[target_idx];
            output_buffer_torque_per_ray[target_idx] = output_buffer_torque_per_ray[gi];
            output_buffer_torque_per_ray[gi] = t_torque_per_ray;
        }
    }
}

void ldplab::rtscuda::BufferSort::execute(
    StreamContext& stream_context, 
    PipelineData& pipeline_data, 
    size_t buffer_index, 
    size_t active_rays,
    bool isec_or_output_contains_data)
{
    const size_t blksz = block_size;
    const size_t memsz = blksz * sizeof(uint32_t);
    const size_t gridsz = active_rays / blksz + (active_rays % blksz ? 1 : 0);

    const size_t num_particles = stream_context.simulationParameter().num_particles;
    if (num_particles > 1)
    {
        cudaMemcpyToSymbolAsync(
            num_rays,
            &active_rays,
            sizeof(active_rays),
            0,
            cudaMemcpyKind::cudaMemcpyHostToDevice,
            stream_context.cudaStream());
        for (size_t p = num_particles; p > 1; --p)
        {
            const int32_t pivot = static_cast<int32_t>(p) - 1;
            countPivotLocal<<<gridsz, blksz, memsz, stream_context.cudaStream()>>>(
                pivot,
                stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
                pipeline_data.buffer_sort_local_rank.getDeviceBuffer(),
                pipeline_data.buffer_sort_block_size.getDeviceBuffer());
            countPivotSum<<<1, 1, 0, stream_context.cudaStream()>>>(
                pipeline_data.buffer_sort_block_size.getDeviceBuffer());
            buildRanks<<<gridsz, blksz, memsz, stream_context.cudaStream()>>>(
                pipeline_data.buffer_sort_local_rank.getDeviceBuffer(),
                pipeline_data.buffer_sort_block_size.getDeviceBuffer(),
                pipeline_data.buffer_sort_rank_index_range.getDeviceBuffer());
            swapBuffer<<<gridsz, blksz, 0, stream_context.cudaStream()>>>(
		        pipeline_data.buffer_sort_rank_index_range.getDeviceBuffer(),
                pipeline_data.buffer_sort_block_size.getDeviceBuffer(),
		        stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
		        stream_context.rayDataBuffers().origin_buffers.getDeviceBuffer(buffer_index),
		        stream_context.rayDataBuffers().direction_buffers.getDeviceBuffer(buffer_index),
		        stream_context.rayDataBuffers().intensity_buffers.getDeviceBuffer(buffer_index),
		        stream_context.rayDataBuffers().min_bv_distance_buffers.getDeviceBuffer(buffer_index),
		        stream_context.intersectionDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
		        stream_context.intersectionDataBuffers().point_buffers.getDeviceBuffer(buffer_index),
		        stream_context.intersectionDataBuffers().normal_buffers.getDeviceBuffer(buffer_index),
		        stream_context.outputDataBuffers().force_per_ray_buffer.getDeviceBuffer(buffer_index),
		        stream_context.outputDataBuffers().torque_per_ray_buffer.getDeviceBuffer(buffer_index),
                isec_or_output_contains_data);
        }
    }
}

bool ldplab::rtscuda::BufferSort::allocateData(
    const SharedStepData& shared_data, 
    PipelineData& data)
{
    return true;
}

#endif
