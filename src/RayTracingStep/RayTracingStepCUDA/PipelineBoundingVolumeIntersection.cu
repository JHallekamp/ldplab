#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineBoundingVolumeIntersection.hpp"

#include "Context.hpp"

std::shared_ptr<ldplab::rtscuda::IPipelineBoundingVolumeIntersectionStage> 
    ldplab::rtscuda::IPipelineBoundingVolumeIntersectionStage::createInstance(
        const RayTracingStepCUDAInfo& info, Context& context)
{
    // Currently just create the bruteforce stage
    return std::make_shared<PipelineBoundingVolumeIntersectionBruteforce>(
        context);
}

namespace bruteforce_cuda
{
    /** @brief Bounding volume intersection device kernel. */
    __global__ void bvIntersectionKernel(
        int32_t* ray_index_buffer,
        ldplab::Vec3* ray_origin_buffer,
        ldplab::Vec3* ray_direction_buffer,
        double* ray_min_bv_dist_buffer,
        size_t num_rays_per_batch,
        ldplab::rtscuda::GenericBoundingVolumeData* bounding_volumes,
        ldplab::Mat3* w2p_transformation,
        ldplab::Vec3* w2p_translation,
        size_t num_particles);
    /** @brief Device function pointer to the actual kernel. */
    __device__ ldplab::rtscuda::pipelineBoundingVolumeIntersectionStageKernel_t
        bv_intersection_kernel_ptr = bvIntersectionKernel;
}

ldplab::rtscuda::PipelineBoundingVolumeIntersectionBruteforce::
    PipelineBoundingVolumeIntersectionBruteforce(Context& context)
    :
    m_context{ context }
{ }

void ldplab::rtscuda::PipelineBoundingVolumeIntersectionBruteforce::execute(
    size_t ray_buffer_index)
{
    const size_t block_size = m_context.parameters.num_threads_per_block;
    const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    bruteforce_cuda::bvIntersectionKernel<<<grid_size, block_size>>>(
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.min_bv_dist_buffers[ray_buffer_index].get(),
        m_context.parameters.num_rays_per_batch,
        m_context.resources.bounding_volumes.bounding_volume_per_particle.get(),
        m_context.resources.transformations.w2p_transformation.get(),
        m_context.resources.transformations.w2p_translation.get(),
        m_context.parameters.num_particles);
}

ldplab::rtscuda::pipelineBoundingVolumeIntersectionStageKernel_t 
    ldplab::rtscuda::PipelineBoundingVolumeIntersectionBruteforce::getKernel()
{
    // Copy the function pointer to the host
    pipelineBoundingVolumeIntersectionStageKernel_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        bruteforce_cuda::bv_intersection_kernel_ptr,
        sizeof(bruteforce_cuda::bv_intersection_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

__global__ void bruteforce_cuda::bvIntersectionKernel(
        int32_t* ray_index_buffer, 
        ldplab::Vec3* ray_origin_buffer, 
        ldplab::Vec3* ray_direction_buffer, 
        double* ray_min_bv_dist_buffer, 
        size_t num_rays_per_batch,
        ldplab::rtscuda::GenericBoundingVolumeData* bounding_volumes, 
        ldplab::Mat3* w2p_transformation, 
        ldplab::Vec3* w2p_translation, 
        size_t num_particles)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;
    unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;

    // Check if the ray already is in a particle space or is invalid
    if (ray_index_buffer[ri] < static_cast<int32_t>(num_particles))
        return;
    double dist;
    double min_dist = -1.0;
    int32_t min_idx = -1;

    Vec3 ray_origin = ray_origin_buffer[ri];
    Vec3 ray_direction = ray_direction_buffer[ri];

    // Check each bounding volume sequentially for intersections
    for (int32_t i = 0; i < static_cast<int32_t>(num_particles); ++i)
    {
        // Transform into particle space
        Vec3 pspace_origin = w2p_transformation[i] *
            (ray_origin + w2p_translation[i]);
        Vec3 pspace_direction = glm::normalize(
            w2p_transformation[i] * ray_direction);
        if (bounding_volumes->intersect_ray_bounding_volume(
            pspace_origin,
            pspace_direction,
            bounding_volumes->data,
            dist))
        {
            if (dist < min_dist &&
                dist > ray_min_bv_dist_buffer[ri])
            {
                min_dist = dist;
                min_idx = i;
            }
        }
    }
    // Check if the ray hits a particle bounding sphere
    if (min_idx >= 0)
    {
        // Ray hits particle with index min_idx
        ray_index_buffer[ri] = min_idx;
        ray_min_bv_dist_buffer[ri] = min_dist;
        // Transform ray from world to particle space
        ray_origin_buffer[ri] = w2p_transformation[min_idx] *
            (ray_origin + w2p_translation[min_idx]);
        ray_direction_buffer[ri] = glm::normalize(
            w2p_transformation[min_idx] * ray_direction);
    }
    else
    {
        // Ray exits the scene
        ray_index_buffer[ri] = -1;
    }
}

#endif