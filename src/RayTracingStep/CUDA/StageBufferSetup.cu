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
    StreamContext& stream_context,
    PipelineData& data)
{
    constexpr size_t block_size = 256;
    const size_t num_particles = stream_context.simulationParameter().num_particles;
    const size_t grid_size = (num_particles / block_size) + (num_particles % block_size ? 1 : 0);
    bufferStepSetupKernel<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
        stream_context.outputDataBuffers().force_per_particle_buffer.getDeviceBuffer(),
        stream_context.outputDataBuffers().torque_per_particle_buffer.getDeviceBuffer(),
        num_particles);
}

void ldplab::rtscuda::BufferSetup::executeLayerSetup(
    StreamContext& stream_context,
    PipelineData& data,
    size_t buffer_index,
    size_t output_buffer_index,
    size_t active_rays)
{
    constexpr size_t block_size = 256;
    const size_t grid_size = (active_rays / block_size) + (active_rays % block_size ? 1 : 0);
    bufferLayerSetupKernel<<<grid_size, block_size, 0, stream_context.cudaStream()>>>(
        stream_context.intersectionDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
        stream_context.outputDataBuffers().force_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        stream_context.outputDataBuffers().torque_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        active_rays);
}

bool ldplab::rtscuda::BufferSetup::allocateData(
    const SharedStepData& shared_data, 
    PipelineData& data)
{
    return true;
}

#endif