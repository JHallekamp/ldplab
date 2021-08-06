#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageRayBufferReduce.hpp"

namespace
{
    __device__ ldplab::rtscuda::RayBufferReduceResult global_ray_counter;

    __global__ void rayBufferReduceKernel(
        ldplab::rtscuda::RayBufferReduceResult* result_buffer,
        int32_t* ray_index_buffer,
        size_t num_rays_per_batch,
        size_t num_particles)
    {
        using namespace ldplab;
        using namespace rtscuda;

        // Shared memory
        extern __shared__ RayBufferReduceResult sbuf[];

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
        for (unsigned int lim = blockDim.x; lim > 1; lim /= 2)
        {
            unsigned int ofs = lim / 2;
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
        ldplab::rtscuda::RayBufferReduceResult* result_buffer,
        size_t buffer_size)
    {
        using namespace ldplab;
        using namespace rtscuda;

        // Shared memory
        extern __shared__ RayBufferReduceResult sbuf[];

        // ====================================================================
        // Preperation step: Prepare shared buffer
        const unsigned int tid = threadIdx.x;
        const unsigned int elem_count = buffer_size / blockDim.x +
            (buffer_size % blockDim.x ? 1 : 0);

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
        for (unsigned int lim = blockDim.x; lim > 1; lim /= 2)
        {
            unsigned int ofs = lim / 2;
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
            global_ray_counter = sbuf[0];
    }
}

ldplab::rtscuda::RayBufferReduceResult 
	ldplab::rtscuda::RayBufferReduce::execute(
		BatchData& batch_data, 
		size_t ray_buffer_index)
{
    KernelLaunchParameter lp = getLaunchParameterStep1();
    rayBufferReduceKernel << <lp.grid_size, lp.block_size, lp.shared_memory_size >> > (
        m_context.resources.pipeline.reduction_result_buffer.get(),
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.parameters.num_rays_per_batch,
        m_context.parameters.num_particles);

    const size_t buffer_size = lp.grid_size.x;
    lp = getLaunchParameterStep2();
    rayBufferReduceKernelStep2 << <lp.grid_size, lp.block_size, lp.shared_memory_size >> > (
        m_context.resources.pipeline.reduction_result_buffer.get(),
        buffer_size);

    RayBufferReduceResult result;
    if (cudaMemcpyFromSymbol(
        &result,
        global_ray_counter,
        sizeof(RayBufferReduceResult)) != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Ray buffer reduce pipeline step "\
            "failed to download reduction results from device",
            m_context.uid);
    }
    return result;
}

#endif