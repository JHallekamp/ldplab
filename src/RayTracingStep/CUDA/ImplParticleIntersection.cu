#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplParticleIntersection.hpp"

#include <LDPLAB/RayTracingStep/CUDA/IGenericGeometry.hpp>

namespace particle_intersection
{
    using namespace ldplab;
    using namespace rtscuda;
    __global__ void intersectionKernel(
        int32_t* ray_index_buffer,
        Vec3* ray_origin_buffer,
        Vec3* ray_direction_buffer,
        int32_t* intersection_index_buffer,
        Vec3* intersection_point_buffer,
        Vec3* intersection_normal_buffer,
        size_t num_rays_per_batch,
        IGenericGeometry::intersectRay* geometry_intersect_ray,
        void** geometry_data,
        Mat3* p2w_transformation,
        Vec3* p2w_translation,
        size_t num_particles);
}

void ldplab::rtscuda::ParticleIntersection::execute(
	const GlobalData& global_data, 
	BatchData& batch_data, 
	size_t ray_buffer_index, 
	size_t intersection_buffer_index)
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

__global__ void particle_intersection::intersectionKernel(
    int32_t* ray_index_buffer, 
    Vec3* ray_origin_buffer, 
    Vec3* ray_direction_buffer, 
    int32_t* intersection_index_buffer, 
    Vec3* intersection_point_buffer, 
    Vec3* intersection_normal_buffer, 
    size_t num_rays_per_batch,
    IGenericGeometry::intersectRay* geometry_intersect_ray, 
    void** geometry_data, 
    Mat3* p2w_transformation, 
    Vec3* p2w_translation, 
    size_t num_particles)
{
    const unsigned int ri = blockIdx.x * blockDim.x + threadIdx.x;
    if (ri >= num_rays_per_batch)
        return;
    int32_t particle_index = ray_index_buffer[ri];
    if (particle_index < 0 ||
        particle_index >= static_cast<int32_t>(num_particles) ||
        particle_index == intersection_index_buffer[ri])
        return;

    Vec3 isec_pt, isec_norm;
    bool intersect_outside;
    if (geometry_intersect_ray[particle_index](
        ray_origin_buffer[ri],
        ray_direction_buffer[ri],
        geometry_data[particle_index],
        isec_pt,
        isec_norm,
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