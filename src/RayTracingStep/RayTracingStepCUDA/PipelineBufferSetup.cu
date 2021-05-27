#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineBufferSetup.hpp"

void ldplab::rtscuda::PipelineBufferSetup::execute()
{
    // Execute kernel
    const size_t block_size = 128;
    const size_t grid_size = m_context.parameters.num_rays_per_buffer / block_size;
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
    size_t num_rays_per_buffer)
{
    int gi = blockIdx.x * blockDim.x + threadIdx.x;
    if (gi >= num_rays_per_buffer)
        return;
    intersection_particle_index_buffer[gi] = -1;
    output_force_per_ray[gi] = Vec3(0, 0, 0);
    output_torque_per_ray[gi] = Vec3(0, 0, 0);
}

#endif