#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "Data.hpp"

bool ldplab::rtscuda::RayBufferResources::allocateResources(
    size_t num_buffers, 
    size_t num_rays_per_buffer)
{
    std::vector<int32_t*> temp_index_buffer_ptrs;
    std::vector<Vec3*> temp_origin_buffer_ptrs;
    std::vector<Vec3*> temp_direction_buffer_ptrs;
    std::vector<double*> temp_intensity_buffer_ptrs;
    std::vector<double*> temp_min_bv_dist_buffer_ptrs;
    for (size_t i = 0; i < num_buffers; ++i)
    {
        // Allocate index buffers
        index_buffers.emplace_back();
        if (!index_buffers.back().allocate(num_rays_per_buffer))
            return false;
        temp_index_buffer_ptrs.push_back(index_buffers.back().get());
        // Allocate origin buffers
        origin_buffers.emplace_back();
        if (!origin_buffers.back().allocate(num_rays_per_buffer))
            return false;
        temp_origin_buffer_ptrs.push_back(origin_buffers.back().get());
        // Allocate direction buffers
        direction_buffers.emplace_back();
        if (!direction_buffers.back().allocate(num_rays_per_buffer))
            return false;
        temp_direction_buffer_ptrs.push_back(direction_buffers.back().get());
        // Allocate intensity buffers
        intensity_buffers.emplace_back();
        if (!intensity_buffers.back().allocate(num_rays_per_buffer))
            return false;
        temp_intensity_buffer_ptrs.push_back(intensity_buffers.back().get());
        // Allocate min bounding volume distance buffers
        min_bv_dist_buffers.emplace_back();
        if (!min_bv_dist_buffers.back().allocate(num_rays_per_buffer))
            return false;
        temp_min_bv_dist_buffer_ptrs.push_back(min_bv_dist_buffers.back().get());
    }
    // Allocate and upload index buffer pointer array
    if (!index_buffer_pointers.allocate(num_buffers))
        return false;
    if (!index_buffer_pointers.upload(temp_index_buffer_ptrs.data()))
        return false;
    // Allocate and upload origin buffer pointer array
    if (!origin_buffer_pointers.allocate(num_buffers))
        return false;
    if (!origin_buffer_pointers.upload(temp_origin_buffer_ptrs.data()))
        return false;
    // Allocate and upload direction buffer pointer array
    if (!direction_buffer_pointers.allocate(num_buffers))
        return false;
    if (!direction_buffer_pointers.upload(temp_direction_buffer_ptrs.data()))
        return false;
    // Allocate and upload intensity buffer pointer array
    if (!intensity_buffer_pointers.allocate(num_buffers))
        return false;
    if (!intensity_buffer_pointers.upload(temp_intensity_buffer_ptrs.data()))
        return false;
    // Allocate and upload min bounding volume distance buffer pointer array
    if (!min_bv_dist_buffer_pointers.allocate(num_buffers))
        return false;
    if (!min_bv_dist_buffer_pointers.upload(temp_min_bv_dist_buffer_ptrs.data()))
        return false;
    // All done, no errors occured
    return true;
}

bool ldplab::rtscuda::IntersectionBufferResources::allocateResources(
    size_t num_rays_per_buffer)
{
    if (!intersection_point_buffer.allocate(num_rays_per_buffer))
        return false;
    if (!intersection_normal_buffer.allocate(num_rays_per_buffer))
        return false;
    if (!intersection_particle_index_buffer.allocate(num_rays_per_buffer))
        return false;
    return true;
}

bool ldplab::rtscuda::OutputBufferResources::allocateResources(
    size_t num_particles, 
    size_t num_rays_per_buffer)
{
    // Allocate host memory
    host_force_per_particle.resize(num_particles);
    host_torque_per_particle.resize(num_particles);
    // Allocate device memory
    if (!force_per_particle.allocate(num_particles))
        return false;
    if (!force_per_ray.allocate(num_rays_per_buffer))
        return false;
    if (!torque_per_particle.allocate(num_particles))
        return false;
    if (!torque_per_ray.allocate(num_rays_per_buffer))
        return false;
    return true;
}

bool ldplab::rtscuda::TransformationResources::allocateResource(
    size_t num_particles)
{
    // Allocate host memory
    host_p2w_transformation.resize(num_particles);
    host_p2w_translation.resize(num_particles);
    host_w2p_transformation.resize(num_particles);
    host_w2p_translation.resize(num_particles);
    // Allocate device memory
    if (!p2w_transformation.allocate(num_particles))
        return false;
    if (!p2w_translation.allocate(num_particles))
        return false;
    if (!w2p_transformation.allocate(num_particles))
        return false;
    if (!w2p_translation.allocate(num_particles))
        return false;
    return true;
}

bool ldplab::rtscuda::BoundingVolumeResources::allocateResource(
    const std::vector<Particle>& particles)
{
    std::vector<GenericBoundingVolumeData> host_data;
    bounding_volumes.resize(particles.size());
    for (size_t i = 0; i < particles.size(); ++i)
    {
        // Allocate bounding volume container
        std::shared_ptr<GenericBoundingVolume> bv =
            GenericBoundingVolume::create(particles[i].bounding_volume);
        if (bv == nullptr)
            return false;
        bounding_volumes[i] = bv;
        // Get data that later is written into generic bv data array
        host_data.push_back(bv->getData());
    }
    // Allocate generic bv data array
    if (!bounding_volume_per_particle.allocate(particles.size()))
        return false;
    // Upload generic bv data
    if (!bounding_volume_per_particle.upload(host_data.data()))
        return false;
    return true;
}

bool ldplab::rtscuda::ParticleResources::allocateResource(
    const std::vector<Particle>& particles)
{
    return allocateGeometries(particles) &&
        allocateMaterials(particles) &&
        allocateCenterOfMass(particles);
}

bool ldplab::rtscuda::ParticleResources::allocateGeometries(const std::vector<Particle>& particles)
{
    std::vector<GenericParticleGeometryData> host_data;
    geometries.resize(particles.size());
    for (size_t i = 0; i < particles.size(); ++i)
    {
        // Allocate particle geometry container
        std::shared_ptr<GenericParticleGeometry> pg =
            GenericParticleGeometry::create(particles[i].geometry);
        if (pg == nullptr)
            return false;
        geometries[i] = pg;
        // Get data that later is written into generic particle array
        host_data.push_back(pg->getData());
    }
    // Allocate generic particle geometry data array
    if (!geometry_per_particle.allocate(particles.size()))
        return false;
    // Upload generic particle geometry data
    if (!geometry_per_particle.upload(host_data.data()))
        return false;
    return true;
}

bool ldplab::rtscuda::ParticleResources::allocateMaterials(const std::vector<Particle>& particles)
{
    std::vector<GenericParticleMaterialData> host_data;
    materials.resize(particles.size());
    for (size_t i = 0; i < particles.size(); ++i)
    {
        // Allocate particle material container
        std::shared_ptr<GenericParticleMaterial> pm =
            GenericParticleMaterial::create(particles[i].material);
        if (pm == nullptr)
            return false;
        materials[i] = pm;
        // Get data that is later written into generic particle array
        host_data.push_back(pm->getData());
    }
    // Allocate generic particle material data array
    if (!material_per_particle.allocate(particles.size()))
        return false;
    // Upload generic particle material data
    if (!material_per_particle.upload(host_data.data()))
        return false;
    return true;
}

bool ldplab::rtscuda::ParticleResources::allocateCenterOfMass(const std::vector<Particle>& particles)
{
    // Allocate buffer
    if (!center_of_mass_per_particle.allocate(particles.size()))
        return false;
    // Collect center of masses and upload data
    std::vector<Vec3> host_data;
    for (size_t i = 0; i < particles.size(); ++i)
        host_data.push_back(particles[i].centre_of_mass);
    if (!center_of_mass_per_particle.upload(host_data.data()))
        return false;
    return true;
}

#endif