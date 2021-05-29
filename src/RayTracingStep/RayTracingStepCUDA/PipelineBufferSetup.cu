#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineBufferSetup.hpp"

#include "Context.hpp"

void ldplab::rtscuda::PipelineBufferSetup::execute()
{
    // Execute kernel
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    bufferSetupKernel <<<grid_size, block_size>>> (
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_rays_per_buffer);
}

__global__ void ldplab::rtscuda::PipelineBufferSetup::bufferSetupKernel(
    int32_t* intersection_particle_index_buffer, 
    Vec3* output_force_per_ray, 
    Vec3* output_torque_per_ray, 
    size_t num_rays_per_batch)
{
    unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;
    intersection_particle_index_buffer[ri] = -1;
    output_force_per_ray[ri] = Vec3(0, 0, 0);
    output_torque_per_ray[ri] = Vec3(0, 0, 0);
}

#endif