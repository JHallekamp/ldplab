#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageBufferSort.hpp"

#include "../../Utils/Log.hpp"

namespace
{
    constexpr size_t block_size = 256;

    __global__ void checkConflictingWarps(
        const int32_t* particle_index_buffer,
        uint32_t* conflicting_warp,
        size_t num_rays)
    {
        extern __shared__ int32_t sbuf[];
        const uint32_t tid = threadIdx.x;
        const uint32_t ri = blockIdx.x * blockDim.x + tid;
        if (ri < num_rays)
            sbuf[tid] = particle_index_buffer[ri];
        else
            sbuf[tid] = -1;
        __syncthreads();

        if (tid == 0)
        {
            uint32_t conflict = 0;
            int32_t pi = sbuf[0];
            for (size_t i = 1; i < blockDim.x; ++i)
            {
                if (pi != sbuf[i])
                {
                    if (pi == -1)
                        pi = sbuf[i];
                    else if (sbuf[i] != 0)
                    {
                        ++conflict;
                        break;
                    }
                }
            }
            conflicting_warp[blockIdx.x] = conflict;
        }
    }

    __global__ void countConflictingWarps(
        const uint32_t* conflicting_warp,
        size_t* num_conflicing_warps,
        size_t num_warps)
    {
        __shared__ size_t count[block_size];

        const uint32_t tid = threadIdx.x;
        const size_t num_blocks = 
            (num_warps / block_size) + (num_warps % block_size ? 1 : 0);
        count[tid] = 0;

        for (size_t i = 0; i < num_blocks; ++i)
        {
            const size_t wi = i * block_size + tid;
            if (wi < num_warps)
                count[tid] += conflicting_warp[wi] ? 1 : 0;
        }
        __syncthreads();

        for (unsigned int lim = block_size; lim > 1; lim /= 2)
        {
            unsigned int ofs = lim / 2;
            if (tid + ofs < lim)
                count[tid] += count[tid + ofs];
            __syncthreads();
        }

        if (tid == 0)
            *num_conflicing_warps = count[0];
    }

    __global__ void createGlobalOffsets(
        size_t* global_offset_per_particle,
        ldplab::rtscuda::PipelineData::BufferSortLocalRank* block_local_rank_per_ray,
        const int32_t* particle_index_buffer,
        size_t num_particles,
        size_t num_rays)
    {
        __shared__ size_t pcount[block_size];
        __shared__ ldplab::rtscuda::PipelineData::BufferSortLocalRank temp[block_size];
        
        const uint32_t tid = threadIdx.x;
        const uint32_t pi = blockIdx.x * blockDim.x + tid;
        const size_t num_blocks = 
            (num_rays / block_size) +
            (num_rays % block_size ? 1 : 0);
        const ldplab::rtscuda::PipelineData::BufferSortLocalRank invalid{ -1, 0 };
        
        pcount[tid] = 0;
        for (size_t i = 0; i < num_blocks; ++i)
        {
            const size_t ro = i * block_size;
            const size_t ri = ro + tid;

            // Load current
            if (ri < num_rays)
                temp[tid] = ldplab::rtscuda::PipelineData::BufferSortLocalRank{
                    particle_index_buffer[ri], 0 };
            else
                temp[tid] = invalid;
            __syncthreads();

            // Iterate over
            if (pi <= num_particles)
            {
                for (size_t j = 0; j < block_size; ++j)
                {
                    if (temp[j].particle_index == pi)
                    {
                        temp[j].rank = pcount[tid];
                        ++pcount[tid];
                    }
                }
            }
            __syncthreads();

            if (ri < num_rays)
                block_local_rank_per_ray[ri] = temp[tid];
        }

        if (pi <= num_particles)
            global_offset_per_particle[pi] = pcount[tid];
    }

    __global__ void prepareGlobalOffsetBuffer(
        size_t* global_offset_per_particle, 
        size_t num_particles)
    {
        if (threadIdx.x == 0)
        {
            size_t count = 0;
            for (size_t i = 0; i <= num_particles; ++i)
            {
                const size_t t = global_offset_per_particle[i];
                global_offset_per_particle[i] = count;
                count += t;
            }
            global_offset_per_particle[num_particles + 1] = count;
        }
    }

    __global__ void swapStage1(
        const int32_t* og_buffer_pi,
        const ldplab::Vec3* og_buffer_origin,
        const ldplab::Vec3* og_buffer_direction,
        const double* og_buffer_intensity,
        const double* og_buffer_bv_dist,
        const ldplab::Vec3* og_buffer_isec_point,
        const ldplab::Vec3* og_buffer_isec_normal,
        const int32_t* og_buffer_isec_pi,
        int32_t* swap_buffer_pi,
        ldplab::Vec3* swap_buffer_origin,
        ldplab::Vec3* swap_buffer_direction,
        double* swap_buffer_intensity,
        double* swap_buffer_bv_dist,
        ldplab::Vec3* swap_buffer_isec_point,
        ldplab::Vec3* swap_buffer_isec_normal,
        int32_t* swap_buffer_isec_pi,
        const ldplab::rtscuda::PipelineData::BufferSortLocalRank* block_local_rank_per_ray,
        const size_t* global_offset_per_particle,
        size_t num_particles,
        size_t num_rays)
    {
        const uint32_t ri = blockIdx.x * blockDim.x + threadIdx.x;
        if (ri >= num_rays)
            return;
        if (ri >= global_offset_per_particle[num_particles + 1])
            swap_buffer_pi[ri] = -1;
        else
        {
            const ldplab::rtscuda::PipelineData::BufferSortLocalRank rank = 
                block_local_rank_per_ray[ri];
            if (rank.particle_index >= 0)
            {
                const size_t target = 
                    global_offset_per_particle[rank.particle_index] + rank.rank;
                swap_buffer_pi[target] = og_buffer_pi[ri];
                swap_buffer_origin[target] = og_buffer_origin[ri];
                swap_buffer_direction[target] = og_buffer_direction[ri];
                swap_buffer_intensity[target] = og_buffer_intensity[ri];
                swap_buffer_bv_dist[target] = og_buffer_bv_dist[ri];
                swap_buffer_isec_point[target] = og_buffer_isec_point[ri];
                swap_buffer_isec_normal[target] = og_buffer_isec_normal[ri];
                swap_buffer_isec_pi[target] = og_buffer_isec_pi[ri];
            }
        }
    }

    __global__ void swapStage2(
        int32_t* og_buffer_pi,
        ldplab::Vec3* og_buffer_origin,
        ldplab::Vec3* og_buffer_direction,
        double* og_buffer_intensity,
        double* og_buffer_bv_dist,
        ldplab::Vec3* og_buffer_isec_point,
        ldplab::Vec3* og_buffer_isec_normal,
        int32_t* og_buffer_isec_pi,
        const int32_t* swap_buffer_pi,
        const ldplab::Vec3* swap_buffer_origin,
        const ldplab::Vec3* swap_buffer_direction,
        const double* swap_buffer_intensity,
        const double* swap_buffer_bv_dist,
        const ldplab::Vec3* swap_buffer_isec_point,
        const ldplab::Vec3* swap_buffer_isec_normal,
        const int32_t* swap_buffer_isec_pi,
        size_t num_rays)
    {
        const uint32_t ri = blockIdx.x * blockDim.x + threadIdx.x;
        if (ri >= num_rays)
            return;
        og_buffer_pi[ri] = swap_buffer_pi[ri];
        og_buffer_origin[ri] = swap_buffer_origin[ri];
        og_buffer_direction[ri] = swap_buffer_direction[ri];
        og_buffer_intensity[ri] = swap_buffer_intensity[ri];
        og_buffer_bv_dist[ri] = swap_buffer_bv_dist[ri];
        og_buffer_isec_point[ri] = swap_buffer_isec_point[ri];
        og_buffer_isec_normal[ri] = swap_buffer_isec_normal[ri];
        og_buffer_isec_pi[ri] = swap_buffer_isec_pi[ri];
    }
}

void ldplab::rtscuda::BufferSort::execute(
    StreamContext& stream_context, 
    PipelineData& pipeline_data, 
    size_t buffer_index, 
    size_t active_rays,
    bool isec_or_output_contains_data)
{
    const size_t num_particles = stream_context.simulationParameter().num_particles;
    if (num_particles > 1)
    {
        const size_t warp_size = stream_context.deviceProperties().warp_size;
        const size_t mem_size = warp_size * sizeof(int32_t);
        size_t grid_size = (active_rays / warp_size) + (active_rays % warp_size ? 1 : 0);

        size_t conflict_threshold = static_cast<size_t>(
            static_cast<double>(num_particles) *
            stream_context.simulationParameter().buffer_sort_abort_threshold);

        if (conflict_threshold > 0)
        {
            if (grid_size <= conflict_threshold)
                return;
            checkConflictingWarps << <grid_size, warp_size, mem_size, stream_context.cudaStream() >> > (
                stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
                pipeline_data.buffer_sort_conflicting_buffer.getDeviceBuffer(),
                active_rays);
            countConflictingWarps << <1, block_size, 0, stream_context.cudaStream() >> > (
                pipeline_data.buffer_sort_conflicting_buffer.getDeviceBuffer(),
                pipeline_data.buffer_sort_num_conflicting_buffers.getDeviceBuffer(),
                grid_size);
            if (!pipeline_data.buffer_sort_num_conflicting_buffers.downloadAsync(
                0, 1, stream_context.cudaStream()))
            {
                LDPLAB_LOG_ERROR("RTSCUDA: Ray buffer sort pipeline step "\
                    "failed to download conflicting warp count from device");
                return;
            }
            stream_context.synchronizeOnStream();
            size_t num_conflicting_warps =
                pipeline_data.buffer_sort_num_conflicting_buffers.getHostBuffer()[0];
            if (num_conflicting_warps <= conflict_threshold)
                return;
        }

        grid_size = 
            ((num_particles + 1) / block_size) + 
            ((num_particles + 1) % block_size ? 1 : 0);
        createGlobalOffsets<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
            pipeline_data.buffer_sort_global_offset_per_particle.getDeviceBuffer(),
            pipeline_data.buffer_sort_block_local_rank_per_ray.getDeviceBuffer(),
            stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
            num_particles,
            active_rays);
        prepareGlobalOffsetBuffer<<<1, 1, 0, stream_context.cudaStream()>>>(
            pipeline_data.buffer_sort_global_offset_per_particle.getDeviceBuffer(),
            num_particles);
        grid_size = (active_rays / block_size) + (active_rays % block_size ? 1 : 0);
        swapStage1<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
            stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().origin_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().direction_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().intensity_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().min_bv_distance_buffers.getDeviceBuffer(buffer_index),
            stream_context.intersectionDataBuffers().point_buffers.getDeviceBuffer(buffer_index),
            stream_context.intersectionDataBuffers().normal_buffers.getDeviceBuffer(buffer_index),
            stream_context.intersectionDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
            pipeline_data.buffer_sort_swap_ray_pi.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_origin.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_direction.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_intensity.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_min_bv_distance.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_isec_point.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_isec_normal.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_isec_pi.getDeviceBuffer(),
            pipeline_data.buffer_sort_block_local_rank_per_ray.getDeviceBuffer(),
            pipeline_data.buffer_sort_global_offset_per_particle.getDeviceBuffer(),
            num_particles,
            active_rays);
        swapStage2<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
            stream_context.rayDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().origin_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().direction_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().intensity_buffers.getDeviceBuffer(buffer_index),
            stream_context.rayDataBuffers().min_bv_distance_buffers.getDeviceBuffer(buffer_index),
            stream_context.intersectionDataBuffers().point_buffers.getDeviceBuffer(buffer_index),
            stream_context.intersectionDataBuffers().normal_buffers.getDeviceBuffer(buffer_index),
            stream_context.intersectionDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
            pipeline_data.buffer_sort_swap_ray_pi.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_origin.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_direction.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_intensity.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_ray_min_bv_distance.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_isec_point.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_isec_normal.getDeviceBuffer(),
            pipeline_data.buffer_sort_swap_isec_pi.getDeviceBuffer(),
            active_rays);
    }
}

bool ldplab::rtscuda::BufferSort::allocateData(
    size_t stream_id,
    const SharedStepData& shared_data, 
    PipelineData& data)
{
    const size_t num_particles = shared_data.simulation_parameter.num_particles;
    const size_t num_rays = shared_data.simulation_parameter.num_rays_per_batch;
    const size_t warp_size = shared_data.execution_model.stream_contexts[stream_id].
        deviceContext().deviceProperties().warp_size;
    const size_t num_warps = (num_rays / warp_size) + (num_rays % warp_size ? 1 : 0);
    return
        data.buffer_sort_num_conflicting_buffers.allocate(1, true) &&
        data.buffer_sort_conflicting_buffer.allocate(num_warps, false) &&
        data.buffer_sort_block_local_rank_per_ray.allocate(num_rays, false) &&
        data.buffer_sort_global_offset_per_particle.allocate(num_particles + 2, false) &&
        data.buffer_sort_swap_ray_pi.allocate(num_rays, false) &&
        data.buffer_sort_swap_ray_origin.allocate(num_rays, false) &&
        data.buffer_sort_swap_ray_direction.allocate(num_rays, false) &&
        data.buffer_sort_swap_ray_intensity.allocate(num_rays, false) &&
        data.buffer_sort_swap_ray_min_bv_distance.allocate(num_rays, false) &&
        data.buffer_sort_swap_isec_point.allocate(num_rays, false) &&
        data.buffer_sort_swap_isec_normal.allocate(num_rays, false) &&
        data.buffer_sort_swap_isec_pi.allocate(num_rays, false);
}

#endif
