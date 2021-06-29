#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineBufferSetup.hpp"

#include "Context.hpp"

ldplab::rtscuda::KernelLaunchParameter 
    ldplab::rtscuda::PipelineBufferSetup::getLaunchParameterInitialSetup()
{
    KernelLaunchParameter p;
    p.block_size.x = 128; //m_context.device_properties.max_num_threads_per_block;
    p.grid_size.x = m_context.parameters.num_rays_per_batch / p.block_size.x +
        (m_context.parameters.num_rays_per_batch % p.block_size.x ? 1 : 0);
    return p;
}

ldplab::rtscuda::KernelLaunchParameter 
    ldplab::rtscuda::PipelineBufferSetup::getLaunchParameterBufferSetup()
{
    KernelLaunchParameter p;
    p.block_size.x = 128; //m_context.device_properties.max_num_threads_per_block;
    p.grid_size.x = m_context.parameters.num_rays_per_batch / p.block_size.x +
        (m_context.parameters.num_rays_per_batch % p.block_size.x ? 1 : 0);
    return p;
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

void ldplab::rtscuda::PipelineBufferSetup::executeInitial()
{
    // Execute kernel
    //const size_t block_size = m_context.parameters.num_threads_per_block;
    //const size_t grid_size = m_context.parameters.num_particles / block_size +
    //    (m_context.parameters.num_particles % block_size ? 1 : 0);
    const KernelLaunchParameter lp = getLaunchParameterInitialSetup();
    initialSetupKernel << <lp.grid_size, lp.block_size >> > (
        m_context.resources.output_buffer.force_per_particle.get(),
        m_context.resources.output_buffer.torque_per_particle.get(),
        m_context.parameters.num_particles);
}

void ldplab::rtscuda::PipelineBufferSetup::execute()
{
    // Execute kernel
    //const size_t block_size = m_context.parameters.num_threads_per_block;
    //const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    const KernelLaunchParameter lp = getLaunchParameterBufferSetup();
    bufferSetupKernel << <lp.grid_size, lp.block_size >> > (
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_rays_per_batch);
}

__device__ void ldplab::rtscuda::executeInitialSetupKernel(
    DevicePipelineResources& resources)
{
    const dim3 grid_sz = resources.launch_params.initialBufferSetup.grid_size;
    const dim3 block_sz = resources.launch_params.initialBufferSetup.block_size;
    initialSetupKernel<<<grid_sz, block_sz>>>(
        resources.output_buffer.force_per_particle,
        resources.output_buffer.torque_per_particle,
        resources.parameters.num_particles);
}

__device__ void ldplab::rtscuda::executeBufferSetupKernel(
    DevicePipelineResources& resources)
{
    const dim3 grid_sz = resources.launch_params.bufferSetup.grid_size;
    const dim3 block_sz = resources.launch_params.bufferSetup.block_size;
    bufferSetupKernel<<<grid_sz, block_sz>>>(
        resources.intersection_buffer.isec_indices,
        resources.output_buffer.force_per_ray,
        resources.output_buffer.torque_per_ray,
        resources.parameters.num_rays_per_batch);
}

#endif