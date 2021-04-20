#include "InnerParticlePropagationStage.hpp"

#include "Data.hpp"
#include "Context.hpp"
#include <LDPLAB/ExperimentalSetup/Particle.hpp>
#include <LDPLAB/ExperimentalSetup/ParticleMaterial.hpp>

#include "../../Utils/Log.hpp"

#include <cmath>

ldplab::rtscpu::EikonalSolverRK4::EikonalSolverRK4(
    Context& context,
    RK4Parameter parameters)
    :
    m_context{ context },
    m_parameters{ parameters }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "EikonalSolverRK4 instance created",
        m_context.uid);
}

void ldplab::rtscpu::EikonalSolverRK4::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute inner particle ray propagation "\
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

    LDPLAB_LOG_TRACE("RTSCPU context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context.uid,
        rays.uid);
}

void ldplab::rtscpu::EikonalSolverRK4::
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
    Arg x{
        ray.direction * material->indexOfRefraction(ray.origin),
        ray.origin };
    Arg x_new{};
    while (!intersected)
    {
        rk4(material, x, m_parameters.step_size, x_new);
        if (isOutsideParticle(particle, x_new.r))
        {
            intersected = true;
            intersection(
                particle,
                x.r,
                x_new.r,
                inter_point,
                inter_normal);
            ray.direction = glm::normalize(x.w);
            ray.origin = x.r;
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
        }
    }
}

void ldplab::rtscpu::EikonalSolverRK4::rk4(
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

inline ldplab::rtscpu::EikonalSolverRK4::Arg
    ldplab::rtscpu::EikonalSolverRK4::eikonal(
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


ldplab::rtscpu::EikonalSolverRK45::
    EikonalSolverRK45(
        Context& context,
        RK4Parameter parameters)
    :
    m_context{ context },
    m_parameters{parameters}
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "EikonalSolverRK45 instance created",
        context.uid);
}

void ldplab::rtscpu::EikonalSolverRK45::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute inner particle ray propagation "\
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

    LDPLAB_LOG_TRACE("RTSCPU context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context.uid,
        rays.uid);
}

void ldplab::rtscpu::EikonalSolverRK45::
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
            if (isOutsideParticle(particle, x_new.r))
            {
                intersected = true;
                intersection(
                    particle,
                    x.r,
                    x_new.r,
                    inter_point, 
                    inter_normal);
                ray.direction = glm::normalize(x.w);
                ray.origin = x.r;
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
            LDPLAB_LOG_TRACE("RTSCPU context %i: RK45 Step discarded with"\
                " error = %f, new step size = %f", m_context.uid, error, h);
        }
    }
}

double ldplab::rtscpu::EikonalSolverRK45::rk45(
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

inline ldplab::rtscpu::EikonalSolverRK45::Arg 
    ldplab::rtscpu::EikonalSolverRK45::eikonal(
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

double ldplab::rtscpu::EikonalSolverRK45::Arg::absoluteMax()
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

ldplab::rtscpu::RK45RodParticlePropagation::RK45RodParticlePropagation(
    Context& context,
    RK4Parameter parameters)
    :
    EikonalSolverRK45(context, parameters),
    IPPRodParticle(context)
{
}

bool ldplab::rtscpu::RK45RodParticlePropagation::
isOutsideParticle(const size_t particle, const Vec3& r)
{
    return IPPRodParticle::isOutsideParticle(particle, r);
}

void ldplab::rtscpu::RK45RodParticlePropagation::intersection(
    const size_t particle,
    const Vec3& ray_in,
    const Vec3& ray_out,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    IPPRodParticle::intersection(particle, ray_in, ray_out, inter_point, inter_normal);
}

ldplab::rtscpu::RK4RodParticlePropagation::RK4RodParticlePropagation(
    Context& context,
    RK4 parameters)
    :
    EikonalSolverRK4(context, parameters),
    IPPRodParticle(context)
{
}

bool ldplab::rtscpu::RK4RodParticlePropagation::isOutsideParticle(
    const size_t particle, 
    const Vec3& r)
{
    return IPPRodParticle::isOutsideParticle(particle, r);
}

void ldplab::rtscpu::RK4RodParticlePropagation::intersection(
    const size_t particle,
    const Vec3& ray_in,
    const Vec3& ray_out,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    IPPRodParticle::intersection(
        particle,
        ray_in,
        ray_out,
        inter_point,
        inter_normal);
}

ldplab::rtscpu::RK4SphericalParticlePropagation::
RK4SphericalParticlePropagation(
    Context& context,
    RK4 parameters)
    :
    EikonalSolverRK4(context, parameters),
    IPPSphereParticle(context)
{
}

bool ldplab::rtscpu::RK4SphericalParticlePropagation::
    isOutsideParticle(const size_t particle, const Vec3& r)
{
    return IPPSphereParticle::isOutsideParticle(particle, r);
}

void ldplab::rtscpu::RK4SphericalParticlePropagation::
intersection(
    const size_t particle,
    const Vec3& ray_in,
    const Vec3& ray_out,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    IPPSphereParticle::intersection(particle, ray_in, ray_out, inter_point, inter_normal);
}

ldplab::rtscpu::RK45SphericalParticlePropagation::
RK45SphericalParticlePropagation(
    Context& context,
    RK4Parameter parameters)
    :
    EikonalSolverRK45(context, parameters),
    IPPSphereParticle(context)
{
}

bool ldplab::rtscpu::RK45SphericalParticlePropagation::
isOutsideParticle(const size_t particle, const Vec3& r)
{
    return IPPSphereParticle::isOutsideParticle(particle, r);
}

void ldplab::rtscpu::RK45SphericalParticlePropagation::
intersection(
    const size_t particle,
    const Vec3& ray_in,
    const Vec3& ray_out,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    IPPSphereParticle::intersection(particle, ray_in, ray_out, inter_point, inter_normal);
}


ldplab::rtscpu::IPPRodParticle::IPPRodParticle(Context& context)
    :
    m_rod_particles{ ((RodParticleData*)
        context.particle_data.get())->particle_data }
{
}

bool ldplab::rtscpu::IPPRodParticle::isOutsideParticle(
    const size_t particle, const Vec3& r)
{
    const RodParticle& geometry = m_rod_particles[particle];
    if (r.x * r.x + r.y * r.y >
        geometry.cylinder_radius * geometry.cylinder_radius)
        return true;

    if (r.z <= geometry.origin_cap.z + geometry.sphere_radius && r.z >= 0)
    {
        if (r.z > geometry.cylinder_length)
        {
            double norm_r2 = r.x * r.x + r.y * r.y;
            double radius2 = geometry.sphere_radius * geometry.sphere_radius -
                std::pow(r.z - geometry.origin_cap.z, 2.0);
            if (norm_r2 > radius2)
                return true;
            else
                return false;
        }
        else if (r.z < geometry.origin_indentation.z + geometry.sphere_radius)
        {
            double norm_r2 = r.x * r.x + r.y * r.y;
            double radius2 = geometry.sphere_radius * geometry.sphere_radius -
                std::pow(r.z - geometry.origin_indentation.z, 2.0);
            if (norm_r2 < radius2)
                return true;
            else
                return false;
        }
        else
            return false;
    }
    else
        return true;
}

void ldplab::rtscpu::IPPRodParticle::intersection(
    const size_t particle, 
    const Vec3& origin_in, 
    const Vec3& origin_out, 
    Vec3& inter_point,
    Vec3& inter_normal)
{
    const RodParticle& geometry = m_rod_particles[particle];
    const Ray ray{origin_out, glm::normalize(origin_in-origin_out) };

    double intersec_first = 0;
    double intersec_second = 0;
    // Check cylinder intersection
    if (cylinderIntersection(
        geometry,
        ray,
        intersec_first,
        intersec_second))
    {
        if (intersec_first >= 0) // Ray origin outside infinite cylinder
        {
            inter_point = ray.origin + intersec_first * ray.direction;
            if (inter_point.z >= 0 &&
                inter_point.z <= geometry.cylinder_length)
            {
                inter_normal = { -inter_point.x, -inter_point.y, 0 };
                inter_normal = glm::normalize(inter_normal);
                return;
            }
            else if (inter_point.z < 0) // First intersection under the cylinder
            {
                indentationIntersection(
                    geometry,
                    ray,
                    inter_point,
                    inter_normal);
                return;
            }
            else // First intersection over the cylinder
            {
                capIntersection(
                    geometry,
                    ray,
                    inter_point,
                    inter_normal);
                return;
            }
        }
        else if (intersec_second > 0) // Ray origin inside infinite cylinder
        {
            bottomTopIntersection(
                geometry,
                ray,
                inter_point,
                inter_normal);
            return;
        }
    }
    // Check for ray inside the infinite cylinder with orthogonal direction
    else
    {
        double distance =
            ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y;
        if (distance <= geometry.cylinder_radius *
            geometry.cylinder_radius)
        {
            bottomTopIntersection(
                geometry,
                ray,
                inter_point,
                inter_normal);
            return;
        }
    }
}


bool ldplab::rtscpu::IPPRodParticle::cylinderIntersection(
    const RodParticle& geometry,
    const Ray& ray,
    double& distance_min,
    double& distance_max)
{
    double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
        (ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y);
    double q = ((ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) -
        geometry.cylinder_radius * geometry.cylinder_radius) /
        (ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y);

    double discriminant = p * p - q;
    if (discriminant < 0.0)
        return false;
    distance_min = -p - std::sqrt(discriminant);
    distance_max = -p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtscpu::IPPRodParticle::bottomTopIntersection(
    const RodParticle& particle,
    const Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    if (ray.origin.z <= 0) // Ray origin below the particle
    {
        if (ray.direction.z <= 0)
            return false;
        else
            return indentationIntersection(
                particle,
                ray,
                inter_point,
                inter_normal);
    }

    const double particle_height =
        particle.origin_cap.z + particle.sphere_radius;
    if (ray.origin.z >= particle_height) // Ray origin above the particle
    {
        if (ray.direction.z >= 0)
            return false;
        else
            return capIntersection(
                particle,
                ray,
                inter_point,
                inter_normal);
    }

    const double dist =
        glm::length(particle.origin_indentation - ray.origin);
    if (dist <= particle.sphere_radius + 1e-9)
        return indentationIntersection(
            particle,
            ray,
            inter_point,
            inter_normal);
    else
        return capIntersection(
            particle,
            ray,
            inter_point,
            inter_normal);
}

bool ldplab::rtscpu::IPPRodParticle::sphereIntersection(
    const Vec3& origin,
    const double& radius,
    const Ray& ray,
    double& distance_min,
    double& distance_max)
{
    const Vec3 o_minus_c = ray.origin - origin;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) - (radius * radius);

    const double discriminant = (p * p) - q;
    if (discriminant < 1e-9)
        return false;

    distance_min = -p - std::sqrt(discriminant);
    distance_max = -p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtscpu::IPPRodParticle::capIntersection(
    const RodParticle& geometry,
    const Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    if (geometry.origin_indentation.z + geometry.sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray.direction.z == 0)
            return false;
        const double t = (geometry.cylinder_length - ray.origin.z) /
            ray.direction.z;
        if (t < 0)
            return false;
        inter_point = ray.origin + t * ray.direction;
        if (inter_point.x * inter_point.x + inter_point.y * inter_point.y >
            geometry.cylinder_radius * geometry.cylinder_radius)
            return false;
        inter_normal = Vec3(0, 0, -1);

        return true;
    }

    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        geometry.origin_cap,
        geometry.sphere_radius,
        ray,
        intersec_first,
        intersec_second))
    {
        if (intersec_first < 0)
            return false;
        inter_point = ray.origin + intersec_first *
            ray.direction;
        if (inter_point.z > geometry.cylinder_length &&
            inter_point.z <= geometry.origin_cap.z + geometry.sphere_radius)
        {
            inter_normal = glm::normalize(geometry.origin_cap - inter_point);
            return true;
        }
    }
    return false;
}

bool ldplab::rtscpu::IPPRodParticle::indentationIntersection(
    const RodParticle& geometry,
    const Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    if (geometry.origin_indentation.z + geometry.sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray.direction.z == 0)
            return false;
        const double t = -ray.origin.z / ray.direction.z;
        if (t < 0)
            return false;
        inter_point = ray.origin + t * ray.direction;
        if (inter_point.x * inter_point.x + inter_point.y * inter_point.y >
            geometry.cylinder_radius * geometry.cylinder_radius)
            return false;
        inter_normal = Vec3(0, 0, 1);
        return true;
    }
    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        geometry.origin_indentation,
        geometry.sphere_radius,
        ray,
        intersec_first,
        intersec_second))
    {
        inter_point = ray.origin + intersec_second * ray.direction;

        if (inter_point.z > 0 &&
            inter_point.z <= geometry.origin_indentation.z +
            geometry.sphere_radius)
        {
            inter_normal = glm::normalize(
                inter_point - geometry.origin_indentation);
            return true;
        }
    }
    return false;
}


ldplab::rtscpu::IPPSphereParticle::IPPSphereParticle(
    Context& context)
    :
    m_context{context}
{
}

bool ldplab::rtscpu::IPPSphereParticle::isOutsideParticle(const size_t particle, const Vec3& r)
{
    const SphericalParticleGeometry* geometry = (SphericalParticleGeometry*)
        m_context.particles[particle].geometry.get();
    if (r.x * r.x + r.y * r.y + r.z * r.z >
        geometry->radius * geometry->radius)
        return true;
    return false;
}

void ldplab::rtscpu::IPPSphereParticle::intersection(
    const size_t particle, 
    const Vec3& origin_in, 
    const Vec3& origin_out, 
    Vec3& inter_point, 
    Vec3& inter_normal)
{
    const SphericalParticleGeometry* geometry = (SphericalParticleGeometry*)
        m_context.particles[particle].geometry.get();

    Ray t_ray{ origin_out, glm::normalize(origin_in - origin_out), -1 };
    const double p = glm::dot(t_ray.direction, t_ray.origin);
    const double q = glm::dot(t_ray.origin, t_ray.origin) -
        (geometry->radius * geometry->radius);
    const double discriminant = (p * p) - q;
    const double distance = -p - std::sqrt(discriminant);
    inter_point = t_ray.origin + distance * t_ray.direction;
    inter_normal = -glm::normalize(inter_point);
}

ldplab::rtscpu::IPPMeshParticle::IPPMeshParticle(
    std::shared_ptr<Context> context)
    :
    m_context{context}
{
}

void ldplab::rtscpu::IPPMeshParticle::intersection(
    const size_t particle, 
    const Vec3& origin_in, 
    const Vec3& origin_out, 
    Vec3& inter_point, 
    Vec3& inter_normal)
{
}
