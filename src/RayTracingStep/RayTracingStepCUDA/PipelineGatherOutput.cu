#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineGatherOutput.hpp"

#include "Context.hpp"

/** @brief Gather output kernel. */
__global__ void gatherOutputKernel(
    int32_t* ray_index_buffer,
    ldplab::Vec3* force_per_ray,
    ldplab::Vec3* torque_per_ray,
    size_t num_rays_per_batch,
    ldplab::Vec3* force_per_particle,
    ldplab::Vec3* torque_per_particle,
    ldplab::Mat3* p2w_transformations,
    size_t num_particles,
    bool particle_space_output);

void ldplab::rtscuda::PipelineGatherOutput::execute(size_t ray_buffer_index)
{
    const size_t grid_size = m_context.parameters.num_particles;
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t shared_mem_size = block_size * sizeof(Vec3) * 2;
    gatherOutputKernel<<<grid_size, block_size, shared_mem_size>>>(
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.resources.output_buffer.force_per_ray.get(),
        m_context.resources.output_buffer.torque_per_ray.get(),
        m_context.parameters.num_rays_per_batch,
        m_context.resources.output_buffer.force_per_particle.get(),
        m_context.resources.output_buffer.torque_per_particle.get(),
        m_context.resources.transformations.p2w_transformation.get(),
        m_context.parameters.num_particles,
        m_context.parameters.output_in_particle_space);
}

__global__ void gatherOutputKernel(
    int32_t* ray_index_buffer,
    ldplab::Vec3* force_per_ray,
    ldplab::Vec3* torque_per_ray,
    size_t num_rays_per_batch,
    ldplab::Vec3* force_per_particle,
    ldplab::Vec3* torque_per_particle,
    ldplab::Mat3* p2w_transformations,
    size_t num_particles,
    bool particle_space_output)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

    // Shared memory
    extern __shared__ Vec3 sbuf[];

    // ========================================================================
    // Preparation step: Prepare shared buffer and needed variables
    const unsigned int tid = threadIdx.x;
    const unsigned int pi = blockIdx.x;
    const unsigned int force_idx = tid;
    const unsigned int torque_idx = tid + blockDim.x;
    size_t ray_count = num_rays_per_batch / blockDim.x;
    if (num_rays_per_batch % blockDim.x != 0)
        ++ray_count;
    sbuf[force_idx] = Vec3(0, 0, 0);
    sbuf[torque_idx] = Vec3(0, 0, 0);
    unsigned int ri = tid;
    for (size_t i = 0; i <= ray_count; ++i)
    {
        if (ri < num_rays_per_batch)
        {
            if (ray_index_buffer[ri] == static_cast<int32_t>(pi))
            {
                sbuf[force_idx] += force_per_ray[ri];
                sbuf[torque_idx] += torque_per_ray[ri];
            }
        }
        ri += blockDim.x;
    }
    __syncthreads();

    // ========================================================================
    // Reduce step: Loop over the buffer and reduce its content
    for (unsigned int lim = blockDim.x; lim > 1; lim /= 2)
    {
        unsigned int ofs = lim / 2;
        if (tid + ofs < lim)
            sbuf[tid] += sbuf[tid + ofs];
        __syncthreads();
    }
    for (unsigned int lim = blockDim.x; lim > 1; lim /= 2)
    {
        unsigned int ofs = lim / 2;
        if (tid + ofs < lim)
            sbuf[tid + blockDim.x] += sbuf[tid + blockDim.x + ofs];
        __syncthreads();
    }

    // ========================================================================
    // Final step: Write the result from shared buffer in output buffers
    if (tid == 0)
    {
        if (!particle_space_output)
        {
            sbuf[force_idx] = p2w_transformations[pi] * sbuf[force_idx];
            sbuf[torque_idx] = p2w_transformations[pi] * sbuf[torque_idx];
        }
        force_per_particle[pi] += sbuf[force_idx];
        torque_per_particle[pi] += sbuf[torque_idx];
    }
}

#endif