#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageRayBufferReduce.hpp"

#include "../../Utils/Log.hpp"

namespace
{
    __global__ void rayBufferReduceKernel(
        ldplab::rtscuda::PipelineData::RayBufferReductionResult* result_buffer,
        int32_t* ray_index_buffer,
        size_t num_rays_per_batch,
        size_t num_particles)
    {
        using namespace ldplab;
        using namespace rtscuda;

        // Shared memory
        extern __shared__ PipelineData::RayBufferReductionResult sbuf[];

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
        ldplab::rtscuda::PipelineData::RayBufferReductionResult* result_buffer,
        size_t buffer_size)
    {
        using namespace ldplab;
        using namespace rtscuda;

        // Shared memory
        extern __shared__ PipelineData::RayBufferReductionResult sbuf[];

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
            result_buffer[0] = sbuf[0];
    }
}

ldplab::rtscuda::PipelineData::RayBufferReductionResult 
	ldplab::rtscuda::RayBufferReduce::execute(
        const GlobalData& global_data,
        BatchData& batch_data,
        PipelineData& pipeline_data,
		size_t ray_buffer_index)
{
    const PipelineData::KernelLaunchParameter& klp1 = 
        pipeline_data.ray_buffer_reduction_1_klp;
    rayBufferReduceKernel<<<klp1.grid_size, klp1.block_size, klp1.shared_memory_size>>>(
        pipeline_data.ray_buffer_reduction_result_buffer.getDeviceBuffer(),
        batch_data.ray_data_buffers.particle_index_buffers.getDeviceBuffer(ray_buffer_index),
        global_data.simulation_parameter.num_rays_per_batch,
        global_data.simulation_parameter.num_particles);

    const PipelineData::KernelLaunchParameter& klp2 =
        pipeline_data.ray_buffer_reduction_2_klp;
    rayBufferReduceKernelStep2<<<klp2.grid_size, klp2.block_size, klp2.shared_memory_size>>>(
        pipeline_data.ray_buffer_reduction_result_buffer.getDeviceBuffer(),
        klp1.grid_size.x);

    if (!pipeline_data.ray_buffer_reduction_result_buffer.download(0, 1))
    {
        LDPLAB_LOG_ERROR("RTSCUDA context %i: Ray buffer reduce pipeline step "\
            "failed to download reduction results from device",
            global_data.instance_uid);
    }
    return pipeline_data.ray_buffer_reduction_result_buffer.getHostBuffer()[0];
}

bool ldplab::rtscuda::RayBufferReduce::allocateData(
    const GlobalData& global_data,
    PipelineData& data)
{
    constexpr size_t block_size = 128;
    PipelineData::KernelLaunchParameter& klp1 = data.ray_buffer_reduction_1_klp;
    klp1.block_size.x = block_size;
    klp1.grid_size.x =
        global_data.simulation_parameter.num_rays_per_batch / block_size +
        (global_data.simulation_parameter.num_rays_per_batch % block_size ? 1 : 0);
    klp1.shared_memory_size = 
        klp1.block_size.x * sizeof(PipelineData::RayBufferReductionResult);

    PipelineData::KernelLaunchParameter& klp2 = data.ray_buffer_reduction_2_klp;
    klp2 = klp1;
    if (klp2.grid_size.x < klp2.block_size.x)
    {
        klp2.block_size.x = klp2.grid_size.x;
        klp2.shared_memory_size = 
            klp2.block_size.x * sizeof(PipelineData::RayBufferReductionResult);
    }
    klp2.grid_size.x = 1;

    return data.ray_buffer_reduction_result_buffer.allocate(klp1.grid_size.x, true);
}

#endif