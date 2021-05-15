#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "CudaContext.hpp"
#include "../../Utils/Log.hpp"

bool ldplab::rtscuda::CudaContext::create(
    const RayTracingStepCUDAInfo& info, 
    const ExperimentalSetup& setup)
{
    // Create ray buffer data (including dummy and initial ray buffer)
    const size_t num_ray_buffers = info.maximum_branching_depth * 2 + 2;
    if (!ray_buffer_data.index_buffers.allocate(
        info.number_rays_per_buffer, num_ray_buffers))
        return false;
    if (!ray_buffer_data.origin_buffers.allocate(
        info.number_rays_per_buffer, num_ray_buffers))
        return false;
    if (!ray_buffer_data.direction_buffers.allocate(
        info.number_rays_per_buffer, num_ray_buffers))
        return false;
    if (!ray_buffer_data.intensity_buffers.allocate(
        info.number_rays_per_buffer, num_ray_buffers))
        return false;
    if (!ray_buffer_data.min_bounding_volume_distance_buffers.allocate(
        info.number_rays_per_buffer, num_ray_buffers))
        return false;
    // Create intersection buffer
    if (!intersection_buffer_data.indices.allocate(
        info.number_rays_per_buffer))
        return false;
    if (!intersection_buffer_data.intersection_points.allocate(
        info.number_rays_per_buffer))
        return false;
    if (!intersection_buffer_data.intersection_normal.allocate(
        info.number_rays_per_buffer))
        return false;
    // Create output buffer data
    if (!output_buffer_data.force_per_particle.allocate(
        setup.particles.size()))
        return false;
    if (!output_buffer_data.torque_per_particle.allocate(
        setup.particles.size()))
        return false;
    // Create Transformation data
    if (!transformation_data.p2w_scale_rotation_matrices.allocate(
        setup.particles.size()))
        return false;
    if (!transformation_data.p2w_translation_vectors.allocate(
        setup.particles.size()))
        return false;
    if (!transformation_data.w2p_rotation_scale_matrices.allocate(
        setup.particles.size()))
        return false;
    if (!transformation_data.w2p_translation_vectors.allocate(
        setup.particles.size()))
        return false;
    // Create bounding volume data
    if (setup.particles[0].bounding_volume->type() ==
        IBoundingVolume::Type::sphere)
    {
        bounding_volume_data = std::make_shared<BoundingSphereCudaData>();
        BoundingSphereCudaData* bounding_sphere_data =
            static_cast<BoundingSphereCudaData*>(bounding_volume_data.get());
        if (!bounding_sphere_data->bounding_sphere_centers.allocate(
            setup.particles.size()))
            return false;
        if (!bounding_sphere_data->bounding_sphere_radii.allocate(
            setup.particles.size()))
            return false;
    }
    else
    {
        LDPLAB_LOG_ERROR("RTSCUDA: Context creation failed, bounding volume "\
            "type not supported");
        return false;
    }
    // No errors occured
    return true;
}

bool ldplab::rtscuda::CudaContext::update(
    const SimulationState& input)
{
    return false;
}

#endif