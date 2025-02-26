#include "ImplParticleIntersection.hpp"

#include <LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp>
#include <LDPLAB/RayTracingStep/CPU/DefaultInitialStageFactories.hpp>

#include "Factory.hpp"
#include "RayTracingStepCPU.hpp"
#include "../../Utils/Log.hpp"

void ldplab::rtscpu::ParticleIntersection::execute(
    RayBuffer& ray_data, 
    IntersectionBuffer& intersection_data, 
    const std::vector<ParticleTransformation>& transformation_data, 
    const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data, 
    const SimulationParameter& simulation_parameter, 
    void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        getParentRayTracingStepUID(),
        ray_data.uid);
    size_t num_hit_rays = 0;
    size_t num_missed_rays = 0;
    for (size_t i = 0; i < ray_data.size; i++)
    {
        if (ray_data.index_data[i] < 0 ||
            ray_data.index_data[i] >= simulation_parameter.ray_world_space_index ||
            ray_data.index_data[i] == intersection_data.particle_index[i])
            continue;

        Ray& ray = ray_data.ray_data[i];
        Vec3& inter_point = intersection_data.point[i];
        Vec3& inter_normal = intersection_data.normal[i];

        if (!geometry_data[ray_data.index_data[i]]->intersectRay(
            ray,
            inter_point,
            inter_normal))
        {
            // Transformation to world space
            const ParticleTransformation& trans = 
                transformation_data[ray_data.index_data[i]];
            ray.origin = trans.p2w_scale_rotation * ray.origin +
                trans.p2w_translation;
            ray.direction =
                glm::normalize(trans.p2w_scale_rotation * ray.direction);
            // Ray missed particle
            ray_data.index_data[i] = simulation_parameter.ray_world_space_index;
            num_missed_rays++;
        }
        else
        {
            intersection_data.particle_index[i] = ray_data.index_data[i];
            num_hit_rays++;
        }
    }

    ray_data.world_space_rays += num_missed_rays;

    LDPLAB_LOG_TRACE("RTSCPU %i: Ray particle intersection test on "\
        "batch buffer %i completed, of %i tested rays %i rays hit particles "\
        "and %i rays missed",
        getParentRayTracingStepUID(),
        ray_data.uid,
        num_hit_rays + num_missed_rays,
        num_hit_rays,
        num_missed_rays);
}

void ldplab::rtscpu::PolarizedLightParticleIntersection::execute(
    RayBuffer& ray_data,
    IntersectionBuffer& intersection_data,
    const std::vector<ParticleTransformation>& transformation_data,
    const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data,
    const SimulationParameter& simulation_parameter,
    void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        getParentRayTracingStepUID(),
        ray_data.uid);
    size_t num_hit_rays = 0;
    size_t num_missed_rays = 0;
    auto polarization_data = (default_factories::InitialStageHomogenousPolarizedLightBoundingSphereProjectionFactory::PolarizationData*)
        stage_dependent_data;
    for (size_t i = 0; i < ray_data.size; i++)
    {
        if (ray_data.index_data[i] < 0 ||
            ray_data.index_data[i] >= simulation_parameter.ray_world_space_index ||
            ray_data.index_data[i] == intersection_data.particle_index[i])
            continue;

        Ray& ray = ray_data.ray_data[i];
        Vec3& inter_point = intersection_data.point[i];
        Vec3& inter_normal = intersection_data.normal[i];
        auto& polarization = polarization_data->polarization_buffers[ray_data.depth].polarization_data[i];

        if (!geometry_data[ray_data.index_data[i]]->intersectRay(
            ray,
            inter_point,
            inter_normal))
        {
            // Transformation to world space
            const ParticleTransformation& trans =
                transformation_data[ray_data.index_data[i]];
            ray.origin = trans.p2w_scale_rotation * ray.origin +
                trans.p2w_translation;
            ray.direction =
                glm::normalize(trans.p2w_scale_rotation * ray.direction);
            if (polarization.is_particle_system)
            {
                polarization.direction = glm::normalize(trans.p2w_scale_rotation * polarization.direction);
                polarization.is_particle_system = false;
            }

            // Ray missed particle
            ray_data.index_data[i] = simulation_parameter.ray_world_space_index;
            num_missed_rays++;
        }
        else
        {
            if (!polarization.is_particle_system)
            {
                const ParticleTransformation& trans =
                    transformation_data[ray_data.index_data[i]];
                polarization.direction = glm::normalize(trans.w2p_rotation_scale * polarization.direction);
                polarization.is_particle_system = true;
            }
            intersection_data.particle_index[i] = ray_data.index_data[i];
            num_hit_rays++;
        }
    }

    ray_data.world_space_rays += num_missed_rays;

    LDPLAB_LOG_TRACE("RTSCPU %i: Ray particle intersection test on "\
        "batch buffer %i completed, of %i tested rays %i rays hit particles "\
        "and %i rays missed",
        getParentRayTracingStepUID(),
        ray_data.uid,
        num_hit_rays + num_missed_rays,
        num_hit_rays,
        num_missed_rays);
}