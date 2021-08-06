#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "StageBufferSetup.hpp"

namespace
{
    __global__ void bufferStepSetupKernel(
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

    __global__ void bufferLayerSetupKernel(
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
}

void ldplab::rtscuda::BufferSetup::executeStepSetup(
    BatchData& batch_data)
{
    const KernelLaunchParameter lp = getLaunchParameterInitialSetup();
    initialSetupKernel<<<lp.grid_size, lp.block_size>>>(
        m_context.resources.output_buffer.force_per_particle.get(),
        m_context.resources.output_buffer.torque_per_particle.get(),
        m_context.parameters.num_particles);
}

void ldplab::rtscuda::BufferSetup::executeLayerSetup(
    BatchData& batch_data, 
    size_t buffer_index)
{
    const KernelLaunchParameter lp = getLaunchParameterBufferSetup();
    bufferSetupKernel<<<lp.grid_size, lp.block_size>>>(
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_rays_per_batch);
}

#endif