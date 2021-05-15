#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "Kernel/RayBoundingVolumeIntersectionTestStage.cuh"

#include "RayBoundingVolumeIntersectionTestStage.hpp"
#include "Context.hpp"
#include "../../Utils/Log.hpp"

#include <limits>

ldplab::rtscuda::RayBoundingSphereIntersectionTestStageBruteForce::
    RayBoundingSphereIntersectionTestStageBruteForce(
        Context& context)
    :
    m_context{ context },
    m_bounding_spheres{ ((BoundingSphereData*) 
        context.bounding_volume_data.get())->sphere_data }
{
    LDPLAB_LOG_INFO("RTSCUDA context %i: "\
        "RayBoundingSphereIntersectionTestStageBruteForce instance created",
        m_context.uid);
}

void ldplab::rtscuda::RayBoundingSphereIntersectionTestStageBruteForce::setup()
{
    // For the brute force method, no setup is needed
}

size_t 
    ldplab::rtscuda::RayBoundingSphereIntersectionTestStageBruteForce::execute(
        RayBuffer& buffer)
{
    LDPLAB_LOG_TRACE("RTSCUDA context %i: Test bounding sphere intersections "\
        "for batch buffer %i",
        m_context.uid,
        buffer.uid);

    // Upload data
    std::vector<Vec3> ray_origins;
    std::vector<Vec3> ray_directions;
    std::vector<Vec3> bounding_sphere_centers;
    std::vector<double> bounding_sphere_radii;
    std::vector<Vec3> w2p_translation_vectors;
    std::vector<Mat3> w2p_rotation_scale_matrices;
    for (size_t i = 0; i < buffer.size; ++i)
    {
        ray_origins.push_back(buffer.ray_data[i].origin);
        ray_directions.push_back(buffer.ray_data[i].direction);
    }

    BoundingSphereData* bsdata = static_cast<BoundingSphereData*>(
        m_context.bounding_volume_data.get());
    for (size_t i = 0; i < m_context.particles.size(); ++i)
    {
        bounding_sphere_centers.push_back(bsdata->sphere_data[i].center);
        bounding_sphere_radii.push_back(bsdata->sphere_data[i].radius);
        w2p_rotation_scale_matrices.push_back(
            m_context.particle_transformations[i].w2p_rotation_scale);
        w2p_translation_vectors.push_back(
            m_context.particle_transformations[i].w2p_translation);
    }

    m_context.cuda_context.ray_buffer_data.origin_buffers.upload(
        ray_origins.data(), 0);
    m_context.cuda_context.ray_buffer_data.direction_buffers.upload(
        ray_directions.data(), 0);
    m_context.cuda_context.ray_buffer_data.index_buffers.upload(
        buffer.index_data, 0);
    m_context.cuda_context.ray_buffer_data.
        min_bounding_volume_distance_buffers.upload(
            buffer.min_bounding_volume_distance_data, 0);

    m_context.cuda_context.transformation_data.w2p_translation_vectors.upload(
        w2p_translation_vectors.data());
    m_context.cuda_context.transformation_data.w2p_rotation_scale_matrices.upload(
        w2p_rotation_scale_matrices.data());

    BoundingSphereCudaData* bscuda = static_cast<BoundingSphereCudaData*>(
        m_context.cuda_context.bounding_volume_data.get());
    bscuda->bounding_sphere_centers.upload(
        bounding_sphere_centers.data());
    bscuda->bounding_sphere_radii.upload(
        bounding_sphere_radii.data());

    // Execute kernel
    const size_t block_size = 128;
    const size_t grid_size = m_context.parameters.number_rays_per_buffer / block_size;
    cudaDeviceSynchronize();
    executeRayBoundingSphereIntersectionTestBruteForce<<<grid_size, block_size>>> (
        m_context.cuda_context.ray_buffer_data.origin_buffers.get(),
        m_context.cuda_context.ray_buffer_data.direction_buffers.get(),
        m_context.cuda_context.ray_buffer_data.index_buffers.get(),
        m_context.cuda_context.ray_buffer_data.min_bounding_volume_distance_buffers.get(),
        m_context.parameters.number_rays_per_buffer,
        bscuda->bounding_sphere_centers.get(),
        bscuda->bounding_sphere_radii.get(),
        m_context.cuda_context.transformation_data.w2p_translation_vectors.get(),
        m_context.cuda_context.transformation_data.w2p_rotation_scale_matrices.get(),
        m_context.particles.size());
    cudaDeviceSynchronize();

    // Reduce
    std::vector<int32_t> indices(buffer.size);
    m_context.cuda_context.ray_buffer_data.index_buffers.download(
        indices.data(), 0);
    m_context.cuda_context.ray_buffer_data.origin_buffers.download(
        ray_origins.data(), 0);
    m_context.cuda_context.ray_buffer_data.direction_buffers.download(
        ray_directions.data(), 0);
    m_context.cuda_context.ray_buffer_data.min_bounding_volume_distance_buffers.download(
        buffer.min_bounding_volume_distance_data, 0);
    cudaDeviceSynchronize();

    size_t num_rays_hitting_boundary_sphere = 0;
    size_t num_rays_exiting_scene = 0;
    for (size_t i = 0; i < buffer.size; ++i)
    {
        if (buffer.index_data[i] >= m_context.particles.size())
        {
            if (indices[i] < 0)
                ++num_rays_exiting_scene;
            else
                ++num_rays_hitting_boundary_sphere;
            buffer.index_data[i] = indices[i];
            buffer.ray_data[i].origin = ray_origins[i];
            buffer.ray_data[i].direction = ray_directions[i];
        }
    }

    buffer.active_rays -= num_rays_exiting_scene;
    buffer.world_space_rays = 0;

    LDPLAB_LOG_TRACE("RTSCUDA context %i: Bounding sphere intersection "\
        "test on batch buffer %i completed, of %i tested rays %i hit "\
        "bounding spheres and %i rays exited the scene",
        m_context.uid,
        buffer.uid,
        num_rays_hitting_boundary_sphere + num_rays_exiting_scene,
        num_rays_hitting_boundary_sphere,
        num_rays_exiting_scene);

    return num_rays_hitting_boundary_sphere;
}

inline void ldplab::rtscuda::RayBoundingSphereIntersectionTestStageBruteForce::
    transformRayFromWorldToParticleSpace(Ray& ray, size_t pidx) const
{
    ray.origin = m_context.particle_transformations[pidx].w2p_rotation_scale *
        (ray.origin + 
            m_context.particle_transformations[pidx].w2p_translation);
    ray.direction = glm::normalize(
        m_context.particle_transformations[pidx].w2p_rotation_scale *
            ray.direction);
}

#endif