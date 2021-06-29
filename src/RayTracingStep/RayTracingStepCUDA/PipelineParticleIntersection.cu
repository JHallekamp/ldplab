#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "PipelineParticleIntersection.hpp"

#include "Context.hpp"

std::shared_ptr<ldplab::rtscuda::IPipelineParticleIntersectionStage> 
    ldplab::rtscuda::IPipelineParticleIntersectionStage::createInstance(
        const RayTracingStepCUDAInfo& info, 
        Context& context)
{
    // Currently we always create the generic particle geometry stage
    std::shared_ptr<ldplab::rtscuda::IPipelineParticleIntersectionStage> impl =
        std::make_shared<PipelineParticleIntersectionGenericParticleGeometry>(
            context);
    return impl;
}

namespace generic_particle_geometry_cuda
{
    __global__ void intersectionKernel(
        int32_t* ray_index_buffer,
        ldplab::Vec3* ray_origin_buffer,
        ldplab::Vec3* ray_direction_buffer,
        int32_t* intersection_index_buffer,
        ldplab::Vec3* intersection_point_buffer,
        ldplab::Vec3* intersection_normal_buffer,
        size_t num_rays_per_batch,
        ldplab::rtscuda::GenericParticleGeometryData* geometry_per_particle,
        ldplab::Mat3* p2w_transformation,
        ldplab::Vec3* p2w_translation,
        size_t num_particles);
    __device__ void executeKernel(
        ldplab::rtscuda::DevicePipelineResources& resources,
        size_t ray_buffer_index);
    __device__ ldplab::rtscuda::pipelineExecuteParticleIntersectionStage_t
        execution_kernel_ptr = executeKernel;
}

ldplab::rtscuda::PipelineParticleIntersectionGenericParticleGeometry::
    PipelineParticleIntersectionGenericParticleGeometry(Context& context)
    :
    m_context{ context }
{ }

ldplab::rtscuda::pipelineExecuteParticleIntersectionStage_t
    ldplab::rtscuda::PipelineParticleIntersectionGenericParticleGeometry::
        getKernel()
{
    using namespace generic_particle_geometry_cuda;
    // Copy the function pointer to the host
    pipelineExecuteParticleIntersectionStage_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        execution_kernel_ptr,
        sizeof(execution_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

void ldplab::rtscuda::PipelineParticleIntersectionGenericParticleGeometry::
    execute(size_t ray_buffer_index)
{
    using namespace generic_particle_geometry_cuda;
    //const size_t block_size = m_context.parameters.num_threads_per_block;
    //const size_t grid_size = m_context.parameters.num_rays_per_batch / block_size;
    const KernelLaunchParameter lp = getLaunchParameter();
    intersectionKernel<<<lp.grid_size, lp.block_size>>>(
        m_context.resources.ray_buffer.index_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.origin_buffers[ray_buffer_index].get(),
        m_context.resources.ray_buffer.direction_buffers[ray_buffer_index].get(),
        m_context.resources.intersection_buffer.intersection_particle_index_buffer.get(),
        m_context.resources.intersection_buffer.intersection_point_buffer.get(),
        m_context.resources.intersection_buffer.intersection_normal_buffer.get(),
        m_context.parameters.num_rays_per_batch,
        m_context.resources.particles.geometry_per_particle.get(),
        m_context.resources.transformations.p2w_transformation.get(),
        m_context.resources.transformations.p2w_translation.get(),
        m_context.parameters.num_particles);
}

__device__ void generic_particle_geometry_cuda::executeKernel(
    ldplab::rtscuda::DevicePipelineResources& resources,
    size_t ray_buffer_index)
{
    const dim3 grid_sz = resources.launch_params.particleIntersection.grid_size;
    const dim3 block_sz = resources.launch_params.particleIntersection.block_size;
    const unsigned int mem_sz = resources.launch_params.particleIntersection.shared_memory_size;
    intersectionKernel<<<grid_sz, block_sz, mem_sz>>>(
        resources.ray_buffer.indices[ray_buffer_index],
        resources.ray_buffer.origins[ray_buffer_index],
        resources.ray_buffer.directions[ray_buffer_index],
        resources.intersection_buffer.isec_indices,
        resources.intersection_buffer.points,
        resources.intersection_buffer.normals,
        resources.parameters.num_rays_per_batch,
        resources.particles.geometry_per_particle,
        resources.transformations.p2w_transformation,
        resources.transformations.p2w_translation,
        resources.parameters.num_particles);
}

ldplab::rtscuda::KernelLaunchParameter 
    ldplab::rtscuda::PipelineParticleIntersectionGenericParticleGeometry::
        getLaunchParameter()
{
    KernelLaunchParameter p;
    p.block_size.x = 128; //m_context.device_properties.max_num_threads_per_block;
    p.grid_size.x = m_context.parameters.num_rays_per_batch / p.block_size.x +
        (m_context.parameters.num_rays_per_batch % p.block_size.x ? 1 : 0);
    return p;
}

__global__ void generic_particle_geometry_cuda::intersectionKernel(
        int32_t* ray_index_buffer,
        ldplab::Vec3* ray_origin_buffer,
        ldplab::Vec3* ray_direction_buffer,
        int32_t* intersection_index_buffer,
        ldplab::Vec3* intersection_point_buffer,
        ldplab::Vec3* intersection_normal_buffer,
        size_t num_rays_per_batch,
        ldplab::rtscuda::GenericParticleGeometryData* geometry_per_particle, 
        ldplab::Mat3* p2w_transformation,
        ldplab::Vec3* p2w_translation,
        size_t num_particles)
{
    using namespace ldplab;
    using namespace rtscuda;

    const unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;
    int32_t particle_index = ray_index_buffer[ri];
    if (particle_index < 0 || 
        particle_index >= static_cast<int32_t>(num_particles) ||
        particle_index == intersection_index_buffer[ri])
        return;

    Vec3 isec_pt, isec_norm;
    double dist;
    bool intersect_outside;
    if (geometry_per_particle[particle_index].intersect_ray_particle(
        ray_origin_buffer[ri],
        ray_direction_buffer[ri],
        geometry_per_particle[particle_index].data,
        isec_pt,
        isec_norm,
        dist,
        intersect_outside))
    {
        // Intersects particle
        intersection_index_buffer[ri] = particle_index;
        intersection_normal_buffer[ri] = isec_norm;
        intersection_point_buffer[ri] = isec_pt;
    }
    else
    {
        // No intersection => transform ray to world space
        ray_index_buffer[ri] = static_cast<int32_t>(num_particles);
        ray_origin_buffer[ri] = 
            p2w_transformation[particle_index] * ray_origin_buffer[ri] + 
            p2w_translation[particle_index];
        ray_direction_buffer[ri] = glm::normalize(
            p2w_transformation[particle_index] * ray_direction_buffer[ri]);
    }
}

#endif
