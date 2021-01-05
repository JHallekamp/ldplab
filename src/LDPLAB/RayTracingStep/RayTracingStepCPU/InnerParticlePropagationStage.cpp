#include "InnerParticlePropagationStage.hpp"

#include "Data.hpp"
#include "Context.hpp"
#include "../../ExperimentalSetup/Particle.hpp"
#include "../../ExperimentalSetup/ParticleMaterial.hpp"

#include "../../Log.hpp"

#include <cmath>

ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    LinearIndexGradientRodeParticlePropagation(
        std::shared_ptr<Context> context,
        RK45 parameters)
    :
    m_parameters{parameters},
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "LinearIndexGradientRodeParticlePropagation instance created",
        m_context->uid);
}

void ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        m_context->uid, rays.uid);

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context->particles.size())
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
        m_context->uid,
        rays.uid);
}

void ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    rayPropagation(
        const size_t particle, 
        Ray& ray, 
        Vec3& inter_point,
        Vec3& inter_normal,
        OutputBuffer& output)
{
    ParticleMaterialLinearOneDirectional* material =
        (ParticleMaterialLinearOneDirectional*) m_context->particles[particle]
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
            if (isOutsideParticle(
                m_context->rode_particle_geometry[particle], x_new.r))
            {
                Vec3 t_new_direction = glm::normalize(x.w);
                output.force[particle] += ray.intensity *
                    (t_new_direction - ray.direction);
                output.torque[particle] += ray.intensity *
                    glm::cross(
                        m_context->particles[particle].centre_of_mass,
                        (t_new_direction - ray.direction));
                intersected = true;
                intersection(
                    m_context->rode_particle_geometry[particle], 
                    x, 
                    inter_point, 
                    inter_normal);
                ray.direction = t_new_direction;
                ray.origin = x.r;
                return;
            }
            else
            {
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
                " error = %f, new step size = %f", m_context->uid, error, h);
        }
    }
}

bool ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    isOutsideParticle(const RodeParticle& geometry, const Vec3& r)
{
    if (r.x * r.x + r.y * r.y >
        geometry.cylinder_radius * geometry.cylinder_radius)
        return true;

    if (r.z <= geometry.origin_cap.z + geometry.sphere_radius && r.z >= 0)
    {
        if (r.z > geometry.cylinder_length)
        {
            double norm_r2 = r.x * r.x + r.y * r.y;
            double radius2 = geometry.sphere_radius * geometry.sphere_radius -
                std::pow(r.z-geometry.origin_cap.z,2.0);
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

double ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::rk45(
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
        k[i].r = x_step.w / particle->indexOfRefraction(x_step.r);

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

inline ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::Arg 
    ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::eikonal(
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

bool ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    cylinderIntersection(
        const RodeParticle& geometry, 
        const Ray& ray, 
        Vec3& inter_point,
        Vec3& inter_normal)
{ 
    const double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
        (ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y);
    const double q = ((ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) -
        geometry.cylinder_radius * geometry.cylinder_radius) /
        (ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y);
    double discriminant = p * p - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + std::sqrt(discriminant);

    inter_point = ray.origin + ray.direction * t;
    if (inter_point.z <= geometry.cylinder_length &&
        inter_point.z >= 0)
    {
        inter_normal = { -inter_point.x, -inter_point.y,0 };
        inter_normal = glm::normalize(inter_normal);
        return true;
    }
    return false;
    
}

bool ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    capIntersection(
        const RodeParticle& geometry,
        const Ray& ray, 
        Vec3& inter_point, 
        Vec3& inter_normal)
{
    Vec3 o_minus_c = ray.origin - geometry.origin_cap;
    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = dot(o_minus_c, o_minus_c) -
        (geometry.sphere_radius * geometry.sphere_radius);
    double discriminant = (p * p) - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + std::sqrt(discriminant);

    inter_point = ray.origin + t * ray.direction;
    if (inter_point.z > geometry.cylinder_length &&
        inter_point.z <= geometry.origin_cap.z +
        geometry.sphere_radius)
    {
        inter_normal = glm::normalize(
            geometry.origin_cap - inter_point);
        return true;
    }
    return false;
}

bool ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
indentationIntersection(
    const RodeParticle& geometry,
    const Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    Vec3 o_minus_c = ray.origin - geometry.origin_indentation;
    double p = glm::dot(ray.direction, o_minus_c);
    double q = dot(o_minus_c, o_minus_c) - 
        (geometry.sphere_radius * geometry.sphere_radius);
    double discriminant = (p * p) - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p - std::sqrt(discriminant);

    inter_point = ray.origin + t * ray.direction;
    if (inter_point.z > 0 &&
        inter_point.z <= geometry.origin_indentation.z +
        geometry.sphere_radius)
    {
        inter_normal = glm::normalize(
            inter_point - geometry.origin_indentation);
        return true;
    }
    return false;
}

void ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::intersection(
    const RodeParticle& geometry,
    const Arg& ray, 
    Vec3& inter_point,
    Vec3& inter_normal)
{
    Ray t_ray{ ray.r, glm::normalize(ray.w), -1};
    if (indentationIntersection(geometry, t_ray, inter_point, inter_normal))
        return;
    if (cylinderIntersection(geometry, t_ray, inter_point, inter_normal))
        return;
    if (capIntersection(geometry, t_ray, inter_point, inter_normal))
        return;
}

double ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::Arg::absoluteMax()
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
