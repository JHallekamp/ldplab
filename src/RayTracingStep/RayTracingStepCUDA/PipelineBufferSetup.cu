#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineBufferSetup.hpp"

#include "Context.hpp"

/** @brief Kernel for setting up the batch buffers. */
__global__ void initialSetupKernel(
    ldplab::Vec3* output_force_per_particle,
    ldplab::Vec3* output_torque_per_particle,
    size_t num_particles);
/** @brief Buffer setup device kernel. */
__global__ void bufferSetupKernel(
    int32_t* intersection_particle_index_buffer,
    ldplab::Vec3* output_force_per_ray,
    ldplab::Vec3* output_torque_per_ray,
    size_t num_rays_per_batch);

void ldplab::rtscuda::PipelineBufferSetup::executeInitial()
{
    // Execute kernel
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_particles / block_size +
        (m_context.parameters.num_particles % block_size ? 1 : 0);
    initialSetupKernel<<<grid_size, block_size>>>(
        m_context.resources.output_buffer.force_per_particle.get(),
        m_context.resources.output_buffer.torque_per_particle.get(),
        m_context.parameters.num_particles);
}

void ldplab::rtscuda::PipelineBufferSetup::execute()
{
    // Execute kernel
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    bufferSetupKernel <<<grid_size, block_size>>> (
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_rays_per_batch);
}

__global__ void initialSetupKernel(
    ldplab::Vec3* output_force_per_particle, 
    ldplab::Vec3* output_torque_per_particle, 
    size_t num_particles)
{
    const size_t pi = blockIdx.x * blockDim.x + threadIdx.x;
    if (pi >= num_particles)
        return;
    output_force_per_particle[pi] = ldplab::Vec3(0, 0, 0);
    output_torque_per_particle[pi] = ldplab::Vec3(0, 0, 0);
}

__global__ void bufferSetupKernel(
    int32_t* intersection_particle_index_buffer, 
    ldplab::Vec3* output_force_per_ray, 
    ldplab::Vec3* output_torque_per_ray, 
    size_t num_rays_per_batch)
{
    const size_t ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;
    intersection_particle_index_buffer[ri] = -1;
    output_force_per_ray[ri] = ldplab::Vec3(0, 0, 0);
    output_torque_per_ray[ri] = ldplab::Vec3(0, 0, 0);
}

#endif