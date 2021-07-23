#include "ImplBoundingVolumeIntersection.hpp"

#include "Factory.hpp"
#include "RayTracingStepCPU.hpp"
#include "../../Utils/Log.hpp"

void ldplab::rtscpu::BoundingSphereIntersectionBruteforce::stepSetup(
    const ExperimentalSetup& setup, 
    const SimulationState& simulation_state, 
    const InterfaceMapping& interface_mapping)
{
    m_particle_bounding_spheres.resize(setup.particles.size());
    for (size_t i = 0; i < setup.particles.size(); ++i)
    {
        // Get the particle instance for particle i using the interface mapping
        const ParticleInstance& particle_instance = 
            simulation_state.particle_instances.find(
                interface_mapping.particle_index_to_uid.at(i))->second;
        // Get the bounding sphere in pspace
        m_particle_bounding_spheres[i] = *static_cast<BoundingVolumeSphere*>(
            setup.particles[i].bounding_volume.get());
        // Translate bounding volume center to world space
        m_particle_bounding_spheres[i].center += particle_instance.position;
    }
}

size_t ldplab::rtscpu::BoundingSphereIntersectionBruteforce::execute(
    RayBuffer& ray_data, 
    const std::vector<ParticleTransformation>& transformation_data, 
    const SimulationParameter& simulation_parameter, 
    void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Test bounding sphere intersections "\
        "for batch buffer %i",
        getParentRayTracingStepUID(),
        buffer.uid);

    size_t num_rays_exiting_scene = 0;
    size_t num_rays_hitting_boundary_sphere = 0;

    const Vec3 unit = { 1, 1, 1 };
    for (size_t i = 0; i < ray_data.size; ++i)
    {
        if (ray_data.index_data[i] < simulation_parameter.ray_world_space_index ||
            ray_data.index_data[i] == simulation_parameter.ray_invalid_index)
            continue;

        double min_d = -1.0;
        int32_t min_j = 0;

        const Vec3& o = ray_data.ray_data[i].origin;
        for (int32_t j = 0; j < simulation_parameter.ray_world_space_index; ++j)
        {
            const Vec3& oc = o - m_particle_bounding_spheres[j].center;
            const double rr = m_particle_bounding_spheres[j].radius *
                m_particle_bounding_spheres[j].radius;

            const double q = glm::dot(oc, oc) - rr;
            // Check if the ray origin lies within the sphere
            if (q < 1e-9)
                continue;

            const double p = glm::dot(ray_data.ray_data[i].direction, oc);
            const double discriminant = p * p - q;
            if (discriminant < 1e-9)
                continue;

            const double d_root = sqrt(discriminant);
            const double dist = -p - d_root;

            double t = ray_data.min_bounding_volume_distance_data[i];
            if (dist <= ray_data.min_bounding_volume_distance_data[i])
                continue;

            if (dist < min_d || min_d < 0)
            {
                min_d = dist;
                min_j = j;
            }
        }

        if (min_d < 0)
        {
            // ray exits scene
            ray_data.index_data[i] = -1;
            ++num_rays_exiting_scene;
        }
        else
        {
            // ray hits particle min_j boundary volume
            ray_data.index_data[i] = min_j;
            ray_data.min_bounding_volume_distance_data[i] = min_d;
            ray_data.ray_data[i].origin = 
                transformation_data[min_j].w2p_rotation_scale *
                    (ray_data.ray_data[i].origin +
                        transformation_data[min_j].w2p_translation);
            ray_data.ray_data[i].direction = glm::normalize(
                transformation_data[min_j].w2p_rotation_scale *
                    ray_data.ray_data[i].direction);
            ++num_rays_hitting_boundary_sphere;
        }
    }

    ray_data.active_rays -= num_rays_exiting_scene;
    ray_data.world_space_rays = 0;

    LDPLAB_LOG_TRACE("RTSCPU context %i: Bounding sphere intersection "\
        "test on batch buffer %i completed, of %i tested rays %i hit "\
        "bounding spheres and %i rays exited the scene",
        getParentRayTracingStepUID(),
        ray_data.uid,
        num_rays_hitting_boundary_sphere + num_rays_exiting_scene,
        num_rays_hitting_boundary_sphere,
        num_rays_exiting_scene);

    return num_rays_hitting_boundary_sphere;
}
