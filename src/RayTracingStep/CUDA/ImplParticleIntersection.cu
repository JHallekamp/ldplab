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
    using namespace particle_intersection;
    const size_t block_size = 128;
    const size_t grid_size =
        global_data.simulation_parameter.num_rays_per_batch / block_size +
        (global_data.simulation_parameter.num_rays_per_batch % block_size ? 1 : 0);
    intersectionKernel<<<grid_size, block_size>>>(
        batch_data.ray_data_buffers.particle_index_buffers.getDeviceBuffer(ray_buffer_index),
        batch_data.ray_data_buffers.origin_buffers.getDeviceBuffer(ray_buffer_index),
        batch_data.ray_data_buffers.direction_buffers.getDeviceBuffer(ray_buffer_index),
        batch_data.intersection_data_buffers.particle_index_buffers.getDeviceBuffer(intersection_buffer_index),
        batch_data.intersection_data_buffers.point_buffers.getDeviceBuffer(intersection_buffer_index),
        batch_data.intersection_data_buffers.normal_buffers.getDeviceBuffer(intersection_buffer_index),
        global_data.simulation_parameter.num_rays_per_batch,
        global_data.particle_data_buffers.intersect_ray_fptr_buffer.getDeviceBuffer(),
        global_data.particle_data_buffers.geometry_data_buffer.getDeviceBuffer(),
        global_data.particle_data_buffers.p2w_transformation_buffer.getDeviceBuffer(),
        global_data.particle_data_buffers.p2w_translation_buffer.getDeviceBuffer(),
        global_data.simulation_parameter.num_particles);
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