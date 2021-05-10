#include "InnerParticlePropagationStage.hpp"

#include "Data.hpp"
#include "Context.hpp"
#include <LDPLAB/ExperimentalSetup/Particle.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleMaterial.hpp>

#include "../../Utils/Log.hpp"

#include <cmath>

ldplab::rtscuda::EikonalSolverRK4LinearIndexGradient::EikonalSolverRK4LinearIndexGradient(
    Context& context,
    RK4Parameter parameters)
    :
    m_context{ context },
    m_parameters{ parameters }
{
    LDPLAB_LOG_INFO("RTSCUDA context %i: "\
        "EikonalSolverRK4 instance created",
        m_context.uid);
}

void ldplab::rtscuda::EikonalSolverRK4LinearIndexGradient::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSCUDA context %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        m_context.uid, rays.uid);

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context.particles.size())
            continue;

        rayPropagation(
            rays.index_data[i],
            rays.ray_data[i],
            intersection.point[i],
            intersection.normal[i],
            output);
    }

    LDPLAB_LOG_TRACE("RTSCUDA context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context.uid,
        rays.uid);
}

void ldplab::rtscuda::EikonalSolverRK4LinearIndexGradient::
    rayPropagation(
        const size_t particle,
        Ray& ray,
        Vec3& inter_point,
        Vec3& inter_normal,
        OutputBuffer& output)
{
    ParticleMaterialLinearOneDirectional* material =
        (ParticleMaterialLinearOneDirectional*)m_context.particles[particle]
        .material.get();

    bool intersected = false;
    bool is_inside = false;
    Arg x{
        ray.direction * material->indexOfRefraction(ray.origin),
        ray.origin };
    Arg x_new{};
    while (!intersected)
    {
        rk4(material, x, m_parameters.step_size, x_new);
        intersected = m_context.particle_data->geometries[particle]->intersectSegment(
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
                intersected = m_context.particle_data->geometries[particle]->intersectRay(
                    ray,
                    t_ip,
                    t_in,
                    intersect_outside);
                if (!intersected || intersect_outside)
                {
                    // We have found a case, where the ray tunnels through the
                    // Particle. We use the original ray and invalidate the 
                    // surface normal.
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
                    m_context.particle_data->geometries[particle]->intersectSegment(
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
                ray.direction = glm::normalize(x.w);
            return;
        }
        else
        {
            const double nx = material->indexOfRefraction(x.r);
            const double ny = material->indexOfRefraction(x_new.r);
            const Vec3 t_old_direction = glm::normalize(x.w);
            const Vec3 t_new_direction = glm::normalize(x_new.w);
            const Vec3 delta_momentum = nx * t_old_direction -
                ny * t_new_direction;
            const Vec3 r = x_new.r -
                m_context.particles[particle].centre_of_mass;
            output.force[particle] += ray.intensity *
                delta_momentum;
            output.torque[particle] += ray.intensity *
                glm::cross(r, delta_momentum);
            x = x_new;
        }
    }
}

void ldplab::rtscuda::EikonalSolverRK4LinearIndexGradient::rk4(
    const ParticleMaterialLinearOneDirectional* particle,
    const Arg& x,
    const double h,
    Arg& x_new) const
{
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
            x_step.w.x += k[i-1].w.x * hb;
            x_step.w.y += k[i-1].w.y * hb;
            x_step.w.z += k[i-1].w.z * hb;
            x_step.r.x += k[i-1].r.x * hb;
            x_step.r.y += k[i-1].r.y * hb;
            x_step.r.z += k[i-1].r.z * hb;
        }
        // eikonal(particle, x_step)
        k[i].w = particle->direction_times_gradient;
        const double index_of_refraction =
            1.0 / particle->indexOfRefraction(x_step.r);
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
    x_new *= h/6.0;
    x_new += x;
}

inline ldplab::rtscuda::EikonalSolverRK4LinearIndexGradient::Arg
    ldplab::rtscuda::EikonalSolverRK4LinearIndexGradient::eikonal(
        const ParticleMaterialLinearOneDirectional* particle,
        const Arg& x) const
{
    //Arg f_of_x{ 
    //    particle->direction * particle->gradient, 
    //    x.w / particle->indexOfRefraction(x.r) };
    return Arg{
        particle->direction_times_gradient,
        x.w / particle->indexOfRefraction(x.r) };
}

ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::EikonalSolverRK45LinearIndexGradient(
    Context& context, 
    RK45Parameter parameters)
    :
    m_context{ context },
    m_parameters{ parameters }
{
    LDPLAB_LOG_INFO("RTSCUDA context %i: "\
        "EikonalSolverRK45 instance created",
        context.uid);
}

void ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSCUDA context %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        m_context.uid, rays.uid);

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context.particles.size())
            continue;

        rayPropagation(
            rays.index_data[i], 
            rays.ray_data[i],
            intersection.point[i], 
            intersection.normal[i],
            output);
    }

    LDPLAB_LOG_TRACE("RTSCUDA context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context.uid,
        rays.uid);
}

void ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::
    rayPropagation(
        const size_t particle, 
        Ray& ray, 
        Vec3& inter_point,
        Vec3& inter_normal,
        OutputBuffer& output)
{
    ParticleMaterialLinearOneDirectional* material =
        (ParticleMaterialLinearOneDirectional*) m_context.particles[particle]
        .material.get();

    bool intersected = false;
    bool is_inside = false;
    Arg x{
        ray.direction * material->indexOfRefraction(ray.origin),
        ray.origin };
    Arg x_new{};
    double h = m_parameters.initial_step_size;
    while (!intersected)
    {
        double error = rk45(material, x, h, x_new);
        if (error <= m_parameters.epsilon)
        {
            intersected =
                m_context.particle_data->geometries[particle]->intersectSegment(
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
                    intersected = m_context.particle_data->geometries[particle]->intersectRay(
                        ray,
                        t_ip,
                        t_in,
                        intersect_outside);
                    if (!intersected || intersect_outside)
                    {
                        // We have found a case, where the ray tunnels through the
                        // Particle. We use the original ray and invalidate the 
                        // surface normal.
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
                        m_context.particle_data->geometries[particle]->intersectSegment(
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
                    ray.direction = glm::normalize(x.w);
                return;
            }
            else
            {
                const double nx = material->indexOfRefraction(x.r);
                const double ny = material->indexOfRefraction(x_new.r);
                Vec3 t_old_direction = glm::normalize(x.w);
                Vec3 t_new_direction = glm::normalize(x_new.w);
                const Vec3 delta_momentum = nx * t_old_direction -
                    ny * t_new_direction;
                const Vec3 r = x_new.r -
                    m_context.particles[particle].centre_of_mass;
                output.force[particle] += ray.intensity *
                    delta_momentum;
                output.torque[particle] += ray.intensity *
                    glm::cross(r, delta_momentum);
                x = x_new;
                h = m_parameters.safety_factor * h * 
                    std::pow(m_parameters.epsilon / error, 0.2);
            }
        }
        else
        {
            h = m_parameters.safety_factor * h * 
                std::pow(m_parameters.epsilon / error, 0.25);
            LDPLAB_LOG_TRACE("RTSCUDA context %i: RK45 Step discarded with"\
                " error = %f, new step size = %f", m_context.uid, error, h);
        }
    }
}

double ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::rk45(
    const ParticleMaterialLinearOneDirectional* particle,
    const Arg& x,
    const double h,
    Arg& x_new) const
{
    Arg k[6]{};
    Arg error{ {0,0,0}, {0,0,0} };
    Arg t{};
    x_new = { {0,0,0}, {0,0,0} };
    for (size_t i = 0; i < 6; ++i)
    {
        Arg x_step = x;
        for (size_t j = 0; j < i; ++j)
        {
            const double hb = h * beta[i * 6 + j];
            x_step.w.x += k[j].w.x * hb;
            x_step.w.y += k[j].w.y * hb;
            x_step.w.z += k[j].w.z * hb;
            x_step.r.x += k[j].r.x * hb;
            x_step.r.y += k[j].r.y * hb;
            x_step.r.z += k[j].r.z * hb;
        }
        // eikonal(particle, x_step)
        k[i].w = particle->direction_times_gradient;
        const double index_of_refraction =
            1.0 / particle->indexOfRefraction(x_step.r);
        k[i].r.x = x_step.w.x * index_of_refraction;
        k[i].r.y = x_step.w.y * index_of_refraction;
        k[i].r.z = x_step.w.z * index_of_refraction;

        if (cerr[i] != 0.0)
        {
            error.w.x += k[i].w.x * cerr[i];
            error.w.y += k[i].w.y * cerr[i];
            error.w.z += k[i].w.z * cerr[i];
            error.r.x += k[i].r.x * cerr[i];
            error.r.y += k[i].r.y * cerr[i];
            error.r.z += k[i].r.z * cerr[i];
        }

        if (c_star[i] != 0.0)
        {
            x_new.w.x += k[i].w.x * c_star[i];
            x_new.w.y += k[i].w.y * c_star[i];
            x_new.w.z += k[i].w.z * c_star[i];
            x_new.r.x += k[i].r.x * c_star[i];
            x_new.r.y += k[i].r.y * c_star[i];
            x_new.r.z += k[i].r.z * c_star[i];
        }
    }
    x_new *= h;
    x_new += x;

    error *= h;
    return error.absoluteMax();

    //k[0] = eikonal(particle, x);
    //k[1] = eikonal(particle, x + k[0] * h * beta[1, 0]);
    //k[2] = eikonal(particle, x + (k[0] * beta[2, 0] + k[1] * beta[2, 1]) * h);
    //k[3] = eikonal(particle, x + (k[0] * beta[3, 0] + k[1] * beta[3, 1] + k[2] * beta[3, 2]) * h);
    //k[4] = eikonal(particle, x + (k[0] * beta[4, 0] + k[1] * beta[4, 1] + k[2] * beta[4, 2] + k[3] * beta[4, 3]) * h);
    //k[5] = eikonal(particle, x + (k[0] * beta[5, 0] + k[1] * beta[5, 1] + k[2] * beta[5, 2] + k[3] * beta[5, 3] + k[4] * beta[5, 4]) * h);
    //Arg errorfield = (k[0] * cerr[0] + k[1] * cerr[1] + k[2] * cerr[2] + k[3] * cerr[3] + k[4] * cerr[4] + k[5] * cerr[5]) * h;
}

inline ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::Arg 
    ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::eikonal(
        const ParticleMaterialLinearOneDirectional* particle,
        const Arg& x ) const
{
    //Arg f_of_x{ 
    //    particle->direction * particle->gradient, 
    //    x.w / particle->indexOfRefraction(x.r) };
    return Arg{
        particle->direction_times_gradient,
        x.w / particle->indexOfRefraction(x.r) };
}

double ldplab::rtscuda::EikonalSolverRK45LinearIndexGradient::Arg::absoluteMax()
{
    double max = std::abs(w.x);
    if (max < std::abs(w.y))
        max = std::abs(w.y);
    if (max < std::abs(w.z))
        max = std::abs(w.z);
    if (max < std::abs(r.x))
        max = std::abs(r.x);
    if (max < std::abs(r.y))
        max = std::abs(r.y);
    if (max < std::abs(r.z))
        max = std::abs(r.z);
    return max;
}