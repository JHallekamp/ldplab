#include "InnerParticlePropagationStage.hpp"

#include "Data.hpp"
#include "Context.hpp"
#include "../../../ExperimentalSetup/Particle.hpp"
#include "../../../ExperimentalSetup/ParticleMaterial.hpp"

#include "../../../Log.hpp"

#include <cmath>

ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    LinearIndexGradientRodParticlePropagation(
        std::shared_ptr<Context> context,
        RK45 parameters)
    :
    m_context{ context },
    m_rod_particles{ ((RodParticleData*)
        context->particle_data.get())->particle_data },
    m_parameters{parameters}
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "LinearIndexGradientRodParticlePropagation instance created",
        m_context->uid);
}

void ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute inner particle ray propagation "\
        "on batch buffer %i",
        m_context->uid, rays.uid);

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context->particles.size())
            continue;

        rayPropagation(
            rays.index_data[i], 
            rays.ray_origin_data[i],
            rays.ray_direction_data[i],
            rays.ray_intensity_data[i],
            intersection.point_data[i], 
            intersection.normal_data[i],
            output);
    }

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context->uid,
        rays.uid);
}

void ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    rayPropagation(
        const size_t particle, 
        Vec3& ray_origin, 
        Vec3& ray_direction, 
        double ray_intensity,
        Vec3& inter_point,
        Vec3& inter_normal,
        OutputBuffer& output)
{
    ParticleMaterialLinearOneDirectional* material =
        (ParticleMaterialLinearOneDirectional*) m_context->particles[particle]
        .material.get();

    bool intersected = false;
    Arg x{
        ray_direction * material->indexOfRefraction(ray_origin),
        ray_origin };
    Arg x_new{};
    double h = m_parameters.initial_step_size;
    while (!intersected)
    {
        double error = rk45(material, x, h, x_new);
        if (error <= m_parameters.epsilon)
        {
            if (isOutsideParticle(m_rod_particles[particle], x_new.r))
            {
                Vec3 t_new_direction = glm::normalize(x.w);
                output.force_data[particle] += ray_intensity *
                    (t_new_direction - ray_direction);
                output.torque_data[particle] += ray_intensity *
                    glm::cross(
                        m_context->particles[particle].centre_of_mass,
                        (ray_direction - t_new_direction));
                intersected = true;
                intersection(
                    m_rod_particles[particle], 
                    x, 
                    inter_point, 
                    inter_normal);
                ray_direction = t_new_direction;
                ray_origin = x.r;
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
            LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: RK45 Step discarded with"\
                " error = %f, new step size = %f", m_context->uid, error, h);
        }
    }
}

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    isOutsideParticle(const RodParticle& geometry, const Vec3& r)
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

double ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::rk45(
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

inline ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::Arg 
    ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::eikonal(
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

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    cylinderIntersection(
        const RodParticle& geometry, 
        const Vec3& ray_origin, 
        const Vec3& ray_direction, 
        Vec3& inter_point,
        Vec3& inter_normal)
{ 
    const double p =
        (ray_origin.x * ray_direction.x + ray_origin.y * ray_direction.y) /
        (ray_direction.x * ray_direction.x + ray_direction.y * ray_direction.y);
    const double q = ((ray_origin.x * ray_origin.x + ray_origin.y * ray_origin.y) -
        geometry.cylinder_radius * geometry.cylinder_radius) /
        (ray_direction.x * ray_direction.x + ray_direction.y * ray_direction.y);
    double discriminant = p * p - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + std::sqrt(discriminant);
    if (t <= 1e-9)
        return false;
    inter_point = ray_origin + ray_direction * t;
    if (inter_point.z <= geometry.cylinder_length &&
        inter_point.z >= 0)
    {
        inter_normal = { -inter_point.x, -inter_point.y,0 };
        inter_normal = glm::normalize(inter_normal);
        return true;
    }
    return false;   
}

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    capIntersection(
        const RodParticle& geometry,
        const Vec3& ray_origin, 
        const Vec3& ray_direction, 
        Vec3& inter_point, 
        Vec3& inter_normal)
{
    if (geometry.origin_indentation.z + geometry.sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_direction.z == 0)
            return false;
        const double t = (geometry.cylinder_length - ray_origin.z) /
            ray_direction.z;
        if (t <= 1e-9)
            return false;
        inter_point = ray_origin + t * ray_direction;
        if (inter_point.x * inter_point.x + inter_point.y * inter_point.y >
            geometry.cylinder_radius * geometry.cylinder_radius)
            return false;
        inter_normal = Vec3(0, 0, -1);
        return true;
    }

    Vec3 o_minus_c = ray_origin - geometry.origin_cap;
    const double p = glm::dot(ray_direction, o_minus_c);
    const double q = dot(o_minus_c, o_minus_c) -
        (geometry.sphere_radius * geometry.sphere_radius);
    double discriminant = (p * p) - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p + std::sqrt(discriminant);
    if (t <= 1e-9)
        return false;
    inter_point = ray_origin + t * ray_direction;
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

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
indentationIntersection(
    const RodParticle& geometry,
    const Vec3& ray_origin,
    const Vec3& ray_direction,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    if (geometry.origin_indentation.z + geometry.sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_direction.z == 0)
            return false;
        const double t = -ray_origin.z /
            ray_direction.z;
        if (t <= 1e-9)
            return false;
        inter_point = ray_origin + t * ray_direction;
        if (inter_point.x * inter_point.x + inter_point.y * inter_point.y >
            geometry.cylinder_radius * geometry.cylinder_radius)
            return false;
        inter_normal = Vec3(0, 0, 1);
        return true;
    }

    Vec3 o_minus_c = ray_origin - geometry.origin_indentation;
    double p = glm::dot(ray_direction, o_minus_c);
    double q = dot(o_minus_c, o_minus_c) -
        (geometry.sphere_radius * geometry.sphere_radius);
    double discriminant = (p * p) - q;

    if (discriminant < 0.0)
        return false;
    const double t = -p - std::sqrt(discriminant);
    if (t <= 1e-9)
        return false;
    inter_point = ray_origin + t * ray_direction;
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

void ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::intersection(
    const RodParticle& geometry,
    const Arg& ray, 
    Vec3& inter_point,
    Vec3& inter_normal)
{
    const Vec3 t_ray_direction = glm::normalize(ray.w);
    if (indentationIntersection(
            geometry, ray.r, t_ray_direction, inter_point, inter_normal))
        return;
    if (cylinderIntersection(
            geometry, ray.r, t_ray_direction, inter_point, inter_normal))
        return;
    if (capIntersection(
            geometry, ray.r, t_ray_direction, inter_point, inter_normal))
        return;
}

double ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::Arg::absoluteMax()
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
