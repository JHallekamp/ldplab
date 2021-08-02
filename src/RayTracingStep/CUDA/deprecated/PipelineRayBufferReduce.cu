#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineRayBufferReduce.hpp"

#include "Context.hpp"
#include "../../../Utils/Log.hpp"
#include "../../../Utils/Profiler.hpp"

//#include <atomic>

__device__ ldplab::rtscuda::RayBufferReduceResult global_ray_counter;

ldplab::rtscuda::KernelLaunchParameter 
    ldplab::rtscuda::PipelineRayBufferReduceStage::getLaunchParameterStep1()
{
    KernelLaunchParameter lp;
    lp.block_size.x = 128;
    lp.grid_size.x = m_context.parameters.num_rays_per_batch / lp.block_size.x +
        (m_context.parameters.num_rays_per_batch % lp.block_size.x ? 1 : 0);
    lp.shared_memory_size = lp.block_size.x * sizeof(RayBufferReduceResult);
    return lp;
}

ldplab::rtscuda::KernelLaunchParameter 
    ldplab::rtscuda::PipelineRayBufferReduceStage::getLaunchParameterStep2()
{
    KernelLaunchParameter lp = getLaunchParameterStep1();
    if (lp.grid_size.x < lp.block_size.x)
    {
        lp.block_size.x = lp.grid_size.x;
        lp.shared_memory_size = lp.block_size.x * sizeof(RayBufferReduceResult);
    }
    lp.grid_size.x = 1;
    return lp;
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

    // ========================================================================
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

    // ========================================================================
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

    // ========================================================================
    // Final step: Write the result from shared buffer in result_buffer
    if (tid == 0)
        global_ray_counter = sbuf[0];
}


ldplab::rtscuda::RayBufferReduceResult
ldplab::rtscuda::PipelineRayBufferReduceStage::execute(
    size_t ray_buffer_index)
{
    // ========================================================================
    // Execute kernel
    LDPLAB_PROFILING_START(pipeline_ray_buffer_reduce_kernel_execution);
    /* VERSION BETA: Use atomics */
    //RayBufferReduceResult result{ 0, 0 };
    //cudaMemcpyToSymbol(global_ray_counter, &result, sizeof(RayBufferReduceResult));

    /* VERSION ALPHA: Always required */
    KernelLaunchParameter lp = getLaunchParameterStep1();
    rayBufferReduceKernel << <lp.grid_size, lp.block_size, lp.shared_memory_size >> > (
        m_context.resources.pipeline.reduction_result_buffer.get(),
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.parameters.num_rays_per_batch,
        m_context.parameters.num_particles);

    /* VERSION GAMMA: Use seperate kernel to gather sum. */
    const size_t buffer_size = lp.grid_size.x;
    lp = getLaunchParameterStep2();
    rayBufferReduceKernelStep2 << <lp.grid_size, lp.block_size, lp.shared_memory_size >> > (
        m_context.resources.pipeline.reduction_result_buffer.get(),
        buffer_size);
    cudaDeviceSynchronize();
    LDPLAB_PROFILING_STOP(pipeline_ray_buffer_reduce_kernel_execution);

    // ========================================================================
    // Download data and reduce into single result buffer
    LDPLAB_PROFILING_START(pipeline_ray_buffer_reduce_data_download);

    /* VERSION ALPHA: Gather sum on host */
    //RayBufferReduceResult result{ 0, 0 };
    //std::vector<RayBufferReduceResult>& host_results =
    //    m_context.resources.pipeline.host_reduction_result_buffer;
    //if (m_context.resources.pipeline.reduction_result_buffer.download(host_results.data()))
    //{
    //    for (size_t i = 0; i < host_results.size(); ++i)
    //    {
    //        result.num_active_rays += host_results[i].num_active_rays;
    //        result.num_world_space_rays += host_results[i].num_world_space_rays;
    //    }
    //}
    //else
    //{
    //    LDPLAB_LOG_ERROR("RTSCUDA context %i: Ray buffer reduce pipeline step "\
    //        "failed to download reduction results from device",
    //        m_context.uid);
    //}

    /* VERSION BETA: Use atomics and global counter buffer. */
    //cudaMemcpyFromSymbol(&result, global_ray_counter, sizeof(RayBufferReduceResult)); 

    /* VERSION GAMMA: Use seperate kernel to gather sum. */
    RayBufferReduceResult result;
    //if (!m_context.resources.pipeline.reduction_result_buffer.download(&result, 0, 1))
    if (cudaMemcpyFromSymbol(
        &result,
        global_ray_counter,
        sizeof(RayBufferReduceResult)) != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Ray buffer reduce pipeline step "\
            "failed to download reduction results from device",
            m_context.uid);
    }
    LDPLAB_PROFILING_STOP(pipeline_ray_buffer_reduce_data_download);
    return result;
}

__device__ ldplab::rtscuda::RayBufferReduceResult 
    ldplab::rtscuda::executeRayBufferReduceKernel(
        DevicePipelineResources& resources, 
        size_t ray_buffer_index)
{
    const dim3 step1_grid_sz = resources.launch_params.rayBufferReduceStep1.grid_size;
    const dim3 step1_block_sz = resources.launch_params.rayBufferReduceStep1.block_size;
    const unsigned int step1_mem_sz = resources.launch_params.rayBufferReduceStep1.shared_memory_size;
    rayBufferReduceKernel<<<step1_grid_sz, step1_block_sz, step1_mem_sz>>>(
        resources.reduction.reduction_result_buffer,
        resources.ray_buffer.indices[ray_buffer_index],
        resources.parameters.num_rays_per_batch,
        resources.parameters.num_particles);

    const dim3 step2_grid_sz = resources.launch_params.rayBufferReduceStep2.grid_size;
    const dim3 step2_block_sz = resources.launch_params.rayBufferReduceStep2.block_size;
    const unsigned int step2_mem_sz = resources.launch_params.rayBufferReduceStep2.shared_memory_size;
    const size_t buffer_sz = step1_grid_sz.x;
    rayBufferReduceKernelStep2<<<step2_grid_sz, step2_block_sz, step2_mem_sz>>>(
        resources.reduction.reduction_result_buffer,
        buffer_sz);
    cudaDeviceSynchronize();

    return global_ray_counter;
}

#endif