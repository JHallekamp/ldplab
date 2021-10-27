#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageRayBufferReduce.hpp"

#include "../../Utils/Log.hpp"

namespace
{
    constexpr size_t rayBufferReduceKernelBlockSize = 128;

    __global__ void rayBufferReduceKernel(
        ldplab::rtscuda::PipelineData::RayStateCountingResult* result_buffer,
        int32_t* ray_index_buffer,
        size_t num_rays_per_batch,
        size_t num_particles)
    {
        using namespace ldplab;
        using namespace rtscuda;

        // Shared memory
        extern __shared__ PipelineData::RayStateCountingResult sbuf[];

        // ====================================================================
        // Preperation step: Prepare shared buffer
        unsigned int tid = threadIdx.x;
        unsigned int ri = blockIdx.x * blockDim.x + tid;
        int32_t particle_index = -1;
        if (ri < num_rays_per_batch)
            particle_index = ray_index_buffer[ri];
        sbuf[tid].num_active_rays = particle_index < 0 ? 0 : 1;
        sbuf[tid].num_world_space_rays =
            particle_index < static_cast<int32_t>(num_particles) ? 0 : 1;
        __syncthreads();

        // ====================================================================
        // Reduce step: Loop over the buffer and reduce its content
        unsigned int ofs;
        for (unsigned int lim = blockDim.x; lim > 1; lim = ofs)
        {
            ofs = lim / 2 + lim % 2;
            if (tid + ofs < lim)
            {
                sbuf[tid].num_active_rays += sbuf[tid + ofs].num_active_rays;
                sbuf[tid].num_world_space_rays += sbuf[tid + ofs].num_world_space_rays;
            }
            __syncthreads();
        }

        // ====================================================================
        // Final step: Write the result from shared buffer in result_buffer
        if (tid == 0)
        {
            //atomicAdd(&global_ray_counter.num_active_rays, sbuf[0].num_active_rays);
            //atomicAdd(&global_ray_counter.num_world_space_rays, sbuf[0].num_world_space_rays);
            result_buffer[blockIdx.x] = sbuf[0];
        }
    }

    __global__ void rayBufferReduceKernelStep2(
        ldplab::rtscuda::PipelineData::RayStateCountingResult* result_buffer,
        size_t buffer_size)
    {
        using namespace ldplab;
        using namespace rtscuda;

        // Shared memory
        extern __shared__ PipelineData::RayStateCountingResult sbuf[];

        // ====================================================================
        // Preperation step: Prepare shared buffer
        const unsigned int tid = threadIdx.x;
        const unsigned int elem_count = 
            buffer_size / blockDim.x + (buffer_size % blockDim.x ? 1 : 0);

        sbuf[tid].num_active_rays = 0;
        sbuf[tid].num_world_space_rays = 0;
        for (unsigned int i = 0; i < elem_count; ++i)
        {
            const unsigned int buf_idx = tid + i * blockDim.x;
            if (buf_idx < buffer_size)
            {
                sbuf[tid].num_active_rays += result_buffer[buf_idx].num_active_rays;
                sbuf[tid].num_world_space_rays += result_buffer[buf_idx].num_world_space_rays;
            }
        }
        __syncthreads();

        // ====================================================================
        // Reduction step: Reduce shared memory
        unsigned int ofs;
        for (unsigned int lim = blockDim.x; lim > 1; lim = ofs)
        {
            ofs = lim / 2 + lim % 2;
            if (tid + ofs < lim)
            {
                sbuf[tid].num_active_rays += sbuf[tid + ofs].num_active_rays;
                sbuf[tid].num_world_space_rays += sbuf[tid + ofs].num_world_space_rays;
            }
            __syncthreads();
        }

        // ====================================================================
        // Final step: Write the result from shared buffer in result_buffer
        if (tid == 0)
            result_buffer[0] = sbuf[0];
    }
}

ldplab::rtscuda::PipelineData::RayStateCountingResult
	ldplab::rtscuda::RayStateCounting::execute(
        StreamContext& smctx,
        PipelineData& pipeline_data,
		size_t ray_buffer_index,
        size_t num_rays)
{
    constexpr size_t block_size = rayBufferReduceKernelBlockSize;
    const size_t k1_grid_size = num_rays / block_size + (num_rays % block_size ? 1 : 0);
    const size_t k1_mem_size = block_size * sizeof(PipelineData::RayStateCountingResult);

    rayBufferReduceKernel<<<k1_grid_size, block_size, k1_mem_size, smctx.cudaStream()>>>(
        pipeline_data.ray_buffer_reduction_result_buffer.getDeviceBuffer(),
        smctx.rayDataBuffers().particle_index_buffers.getDeviceBuffer(ray_buffer_index),
        num_rays,
        smctx.simulationParameter().num_particles);

    size_t k2_block_size = block_size;
    size_t k2_grid_size = k1_grid_size;
    size_t k2_mem_size = k1_mem_size;
    if (k2_grid_size < k2_block_size)
    {
        k2_block_size = k2_grid_size;
        k2_mem_size = k2_block_size * sizeof(PipelineData::RayStateCountingResult);
    }
    k2_grid_size = 1;
    rayBufferReduceKernelStep2<<<k2_grid_size, k2_block_size, k2_mem_size, smctx.cudaStream() >>>(
        pipeline_data.ray_buffer_reduction_result_buffer.getDeviceBuffer(),
        k1_grid_size);

    if (!pipeline_data.ray_buffer_reduction_result_buffer.downloadAsync(0, 1, smctx.cudaStream()))
    {
        LDPLAB_LOG_ERROR("RTSCUDA: Ray buffer reduce pipeline step "\
            "failed to download reduction results from device");
    }
    smctx.synchronizeOnStream();
    return pipeline_data.ray_buffer_reduction_result_buffer.getHostBuffer()[0];
}

bool ldplab::rtscuda::RayStateCounting::allocateData(
    const SharedStepData& shared_data,
    PipelineData& data)
{
    constexpr size_t block_size = rayBufferReduceKernelBlockSize;
    size_t grid_size = 
        shared_data.simulation_parameter.num_rays_per_batch / block_size +
        (shared_data.simulation_parameter.num_rays_per_batch % block_size ? 1 : 0);
    return data.ray_buffer_reduction_result_buffer.allocate(grid_size, true);
}

#endif