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
    const PipelineData::KernelLaunchParameter& klp = data.buffer_setup_step_klp;
    bufferStepSetupKernel<<<klp.grid_size, klp.block_size, klp.shared_memory_size, stream_context.cudaStream()>>>(
        stream_context.outputDataBuffers().force_per_particle_buffer.getDeviceBuffer(),
        stream_context.outputDataBuffers().torque_per_particle_buffer.getDeviceBuffer(),
        stream_context.simulationParameter().num_particles);
}

void ldplab::rtscuda::BufferSetup::executeLayerSetup(
    StreamContext& stream_context,
    PipelineData& data,
    size_t buffer_index,
    size_t output_buffer_index)
{
    const PipelineData::KernelLaunchParameter& klp = data.buffer_setup_layer_klp;
    bufferLayerSetupKernel<<<klp.grid_size, klp.block_size, klp.shared_memory_size, stream_context.cudaStream()>>>(
        stream_context.intersectionDataBuffers().particle_index_buffers.getDeviceBuffer(buffer_index),
        stream_context.outputDataBuffers().force_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        stream_context.outputDataBuffers().torque_per_ray_buffer.getDeviceBuffer(output_buffer_index),
        stream_context.simulationParameter().num_rays_per_batch);
}

bool ldplab::rtscuda::BufferSetup::allocateData(
    const SharedStepData& shared_data, 
    PipelineData& data)
{
    constexpr size_t block_size = 128;
    PipelineData::KernelLaunchParameter& klp1 = data.buffer_setup_layer_klp;
    klp1.block_size.x = block_size;
    klp1.grid_size.x =
        shared_data.simulation_parameter.num_rays_per_batch / klp1.block_size.x +
        (shared_data.simulation_parameter.num_rays_per_batch % klp1.block_size.x ? 1 : 0);
    PipelineData::KernelLaunchParameter& klp2 = data.buffer_setup_step_klp;
    klp2.block_size.x = block_size;
    klp2.grid_size =
        shared_data.simulation_parameter.num_particles / klp1.block_size.x +
        (shared_data.simulation_parameter.num_particles % klp1.block_size.x ? 1 : 0);
    return true;
}

#endif