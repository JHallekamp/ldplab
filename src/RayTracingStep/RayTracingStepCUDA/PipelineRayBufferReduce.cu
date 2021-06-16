#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineRayBufferReduce.hpp"

#include "Context.hpp"
#include "../../Utils/Log.hpp"

/**
 * @brief Ray buffer index reduction kernel.
 * @note Cuda threads can only synchronize with threads in the same
 *       block. Due to this, the reduction is performed block-wise
 *       and the result_buffer is assumed to contain a result per
 *       block, not a single overall result.
 * @warning Block size needs to be a power of 2 for this to work!
 */
__global__ void rayBufferReduceKernel(
    ldplab::rtscuda::RayBufferReduceResult* result_buffer, 
    int32_t* ray_index_buffer, 
    size_t num_rays_per_batch,
    size_t num_particles);

ldplab::rtscuda::RayBufferReduceResult 
    ldplab::rtscuda::PipelineRayBufferReduceStage::execute(
        size_t ray_buffer_index)
{
    // Execute kernel
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    const size_t shared_mem_size = block_size * sizeof(RayBufferReduceResult);
    rayBufferReduceKernel<<<grid_size, block_size, shared_mem_size>>>(
        m_context.resources.pipeline.reduction_result_buffer.get(),
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.parameters.num_rays_per_batch,
        m_context.parameters.num_particles);
    // Download data and reduce into single result buffer
    RayBufferReduceResult result{ 0, 0 };
    std::vector<RayBufferReduceResult>& host_results =
        m_context.resources.pipeline.host_reduction_result_buffer;
    if (m_context.resources.pipeline.reduction_result_buffer.download(host_results.data()))
    {
        for (size_t i = 0; i < host_results.size(); ++i)
        {
            result.num_active_rays += host_results[i].num_active_rays;
            result.num_world_space_rays += host_results[i].num_world_space_rays;
        }
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Ray buffer reduce pipeline step "\
            "failed to download reduction results from device",
            m_context.uid);
    }
    return result;
}

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

    // ========================================================================
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

    // ========================================================================
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

    // ========================================================================
    // Final step: Write the result from shared buffer in result_buffer
    if (tid == 0)
        result_buffer[blockIdx.x] = sbuf[0];
}

#endif
