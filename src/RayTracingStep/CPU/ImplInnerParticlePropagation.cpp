#include "ImplInnerParticlePropagation.hpp"

#include "../../Utils/Log.hpp"

#include <LDPLAB/RayTracingStep/CPU/IGenericGeometry.hpp>

ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::EikonalSolverRK4LinearIndexGradient(
    RK4Parameter parameters)
    :
    m_parameters{ parameters }
{
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::execute(
    RayBuffer& ray_data, 
    IntersectionBuffer& intersection_data, 
    OutputBuffer& output_data, 
    const std::vector<std::shared_ptr<IGenericGeometry>>& geometry_data, 
    const std::vector<std::shared_ptr<IParticleMaterial>>& material_data, 
    const std::vector<Vec3>& center_of_mass, 
    const SimulationParameter& simulation_parameter, 
    void* stage_dependent_data)
{
    LDPLAB_LOG_TRACE("RTSCPU %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        getParentRayTracingStepUID(), ray_data.uid);

    for (size_t i = 0; i < ray_data.size; i++)
    {
        if (ray_data.index_data[i] <= simulation_parameter.ray_invalid_index ||
            ray_data.index_data[i] >= simulation_parameter.ray_world_space_index)
            continue;

        const size_t particle_index = ray_data.index_data[i];
        rayPropagation(
            particle_index,
            ray_data.ray_data[i],
            intersection_data.point[i],
            intersection_data.normal[i],
            geometry_data[particle_index],
            material_data[particle_index],
            center_of_mass[particle_index],
            output_data);
    }

    LDPLAB_LOG_TRACE("RTSCPU %i: Inner particle ray propagation on "\
        "buffer %i completed",
        getParentRayTracingStepUID(),
        ray_data.uid);
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::
rayPropagation(
    size_t particle_index,
    Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal,
    const std::shared_ptr<IGenericGeometry>& particle_geometry,
    const std::shared_ptr<IParticleMaterial>& particle_material,
    const Vec3& particle_center_of_mass,
    OutputBuffer& output) const
{
    bool intersected = false;
    bool is_inside = false;
    Arg x{
        ray.direction * particle_material->indexOfRefraction(ray.origin),
        ray.origin };
    Arg x_new{};
    while (!intersected)
    {
        rk4(particle_material, x, m_parameters.step_size, x_new);
        intersected = particle_geometry->intersectSegment(
            x.r,
            x_new.r,
            inter_point,
            inter_normal,
            is_inside);
        if (intersected || !is_inside)
        {
            if (!intersected)
            {
                // The following check is neccessary to test if the ray hits in
                // an extreme point (volume so small that it lies within the 
                // epsilon region). If that is the case, we assume the ray 
                // tunnels through the particle.
                bool intersect_outside = false;
                Vec3 t_ip, t_in;
                intersected = particle_geometry->intersectRay(
                    ray,
                    t_ip,
                    t_in,
                    intersect_outside);
                if (!intersected || intersect_outside)
                {
                    // We have found a case, where the ray tunnels through the
                    // Particle. We use the original ray and invalidate the 
                    // surface normal.
                    inter_point = ray.origin;
                    inter_normal = Vec3(0, 0, 0);
                }
                else
                {
                    // We have found a case, where the initial ray is bend 
                    // out of the particle in the initial step due to the 
                    // particle material gradient. In this case we assume that 
                    // it hits in the original intersection point.
                    // We use the previous normal and have to flip it, to 
                    // ensure that we have correct behaviour in the interaction
                    // stage.
                    // To receive the previous normal, we simply perform the
                    // Intersection test again, but this time we reverse the
                    // segment directions. Then we flip the normal.
                    particle_geometry->intersectSegment(
                        x_new.r,
                        x.r,
                        inter_point,
                        inter_normal,
                        is_inside);
                    inter_point = ray.origin;
                    inter_normal = -inter_normal;
                    ray.direction = glm::normalize(x.w);
                }
            }
            else
            {
                ray.direction = glm::normalize(x.w);
                const double nx = particle_material->indexOfRefraction(x.r);
                const double ny = particle_material->indexOfRefraction(inter_point);
                const Vec3 delta_momentum = (nx - ny) * ray.direction;
                const Vec3 r = inter_point - particle_center_of_mass;
                output.force[particle_index] += ray.intensity *
                    delta_momentum;
                output.torque[particle_index] += ray.intensity *
                    glm::cross(r, delta_momentum);
            }
            return;
        }
        else
        {
            const double nx = particle_material->indexOfRefraction(x.r);
            const double ny = particle_material->indexOfRefraction(x_new.r);
            const Vec3 t_old_direction = glm::normalize(x.w);
            const Vec3 t_new_direction = glm::normalize(x_new.w);
            const Vec3 delta_momentum = nx * t_old_direction -
                ny * t_new_direction;
            const Vec3 r = x_new.r - particle_center_of_mass;
            output.force[particle_index] += ray.intensity *
                delta_momentum;
            output.torque[particle_index] += ray.intensity *
                glm::cross(r, delta_momentum);
            x = x_new;
        }
    }
}

void ldplab::rtscpu::EikonalSolverRK4LinearIndexGradient::rk4(
    const std::shared_ptr<IParticleMaterial>& particle_material, 
    const Arg& x, 
    const double h, 
    Arg& x_new) const
{
    const ParticleMaterialLinearOneDirectional* raw_material =
        static_cast<ParticleMaterialLinearOneDirectional*>(particle_material.get());
    Arg k[4]{};
    const double beta[4] = { 1.0, 0.5, 0.5, 1.0 };
    const double c[4] = { 1.0, 2.0, 2.0, 1.0 };
    x_new = { {0,0,0}, {0,0,0} };
    for (size_t i = 0; i < 4; ++i)
    {
        Arg x_step = x;
        if (i > 0)
        {
            const double hb = h * beta[i];
            x_step.w.x += k[i - 1].w.x * hb;
            x_step.w.y += k[i - 1].w.y * hb;
            x_step.w.z += k[i - 1].w.z * hb;
            x_step.r.x += k[i - 1].r.x * hb;
            x_step.r.y += k[i - 1].r.y * hb;
            x_step.r.z += k[i - 1].r.z * hb;
        }
        k[i].w = raw_material->direction_times_gradient;
        const double index_of_refraction =
            1.0 / raw_material->indexOfRefraction(x_step.r);
        k[i].r.x = x_step.w.x * index_of_refraction;
        k[i].r.y = x_step.w.y * index_of_refraction;
        k[i].r.z = x_step.w.z * index_of_refraction;

        if (c[i] != 0.0)
        {
            x_new.w.x += k[i].w.x * c[i];
            x_new.w.y += k[i].w.y * c[i];
            x_new.w.z += k[i].w.z * c[i];
            x_new.r.x += k[i].r.x * c[i];
            x_new.r.y += k[i].r.y * c[i];
            x_new.r.z += k[i].r.z * c[i];
        }
    }
    x_new *= h / 6.0;
    x_new += x;
}
