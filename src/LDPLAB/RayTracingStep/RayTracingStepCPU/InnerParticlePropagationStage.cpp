#include "InnerParticlePropagationStage.hpp"

#include "Data.hpp"
#include "Context.hpp"
#include "../../ExperimentalSetup/Particle.hpp"
#include "../../ExperimentalSetup/ParticleMaterial.hpp"

#include "../../Log.hpp"

ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    LinearIndexGradientRodeParticlePropagation(
        std::shared_ptr<Context> context,
        double initial_step_size,
        double epsilon,
        double safety_factor)
    :
    initial_step_size{ initial_step_size },
    epsilon{ epsilon },
    safety_factor{ safety_factor },
    m_context{ context }
{
}

void ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 && 
            rays.index_data[i] >= m_context->particles.size())
            continue;

        rayPropagation(
            rays.index_data[i], 
            rays.ray_data[i],
            intersection.point[i], 
            intersection.normal[i],
            output);
    }
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
    double h = initial_step_size;
    while (!intersected)
    {
        double error = rk45(material, x, h, x_new);
        if (error <= epsilon)
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
                h = safety_factor * h * std::pow(epsilon / error, 0.2);
            }
        }
        else
        {
            h = safety_factor * h * std::pow(epsilon / error, 0.25);
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
        return false;

    if (r.z < geometry.origin_cap.z + geometry.sphere_radius && r.z < 0)
    {
        if (r.z > geometry.cylinder_length)
        {
            double norm_r2 = r.x * r.x + r.y * r.y;
            double radius2 = geometry.sphere_radius * geometry.sphere_radius -
                std::pow(r.z-geometry.origin_cap.z,2.0);
            if (norm_r2 > radius2)
                return false;
            else
                return true;
        }
        else if (r.z < geometry.origin_indentation.z + geometry.sphere_radius)
        {
            double norm_r2 = r.x * r.x + r.y * r.y;
            double radius2 = geometry.sphere_radius * geometry.sphere_radius -
                std::pow(r.z - geometry.origin_indentation.z, 2.0);
            if (norm_r2 < radius2)
                return false;
            else
                return true;
        }
        else
            return true;
    }
    else
        return false;
}

double ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::rk45(
    const ParticleMaterialLinearOneDirectional* particle,
    const Arg& x,
    const double h,
    Arg& x_new)
{
    Arg k[6] {};
    Arg error{ {0,0,0}, {0,0,0} };
    for (size_t i = 0; i < 6; ++i)
    {
        Arg x_step = x;
        for (size_t j = 0; j < i; ++j)
            x_step = x_step + k[j] * h * beta[(i+1) * 6 + j];
        k[i] = eikonal(particle, x_step);
      
        error = error + k[i] * cerr[i];
        x_new = x_new + k[i] * c_star[i];
    }
    x_new = x_new * h + x;
    error = error * h;
    return error.absoluteMax();

    //k[0] = eikonal(particle, x);
    //k[1] = eikonal(particle, x + k[0] * h * beta[1, 0]);
    //k[2] = eikonal(particle, x + (k[0] * beta[2, 0] + k[1] * beta[2, 1]) * h);
    //k[3] = eikonal(particle, x + (k[0] * beta[3, 0] + k[1] * beta[3, 1] + k[2] * beta[3, 2]) * h);
    //k[4] = eikonal(particle, x + (k[0] * beta[4, 0] + k[1] * beta[4, 1] + k[2] * beta[4, 2] + k[3] * beta[4, 3]) * h);
    //k[5] = eikonal(particle, x + (k[0] * beta[5, 0] + k[1] * beta[5, 1] + k[2] * beta[5, 2] + k[3] * beta[5, 3] + k[4] * beta[5, 4]) * h);
    //Arg errorfield = (k[0] * cerr[0] + k[1] * cerr[1] + k[2] * cerr[2] + k[3] * cerr[3] + k[4] * cerr[4] + k[5] * cerr[5]) * h;
}

ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::Arg 
    ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::eikonal(
        const ParticleMaterialLinearOneDirectional* particle,
        Arg& x )
{
    Arg f_of_x{ 
        particle->direction * particle->gradient, 
        x.w / particle->indexOfRefraction(x.r) };
    return f_of_x;
}

bool ldplab::rtscpu::LinearIndexGradientRodeParticlePropagation::
    cylinderIntersection(
        const RodeParticle& geometry, 
        const Ray& ray, 
        Vec3& inter_point,
        Vec3& inter_normal)
{
    double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
        (ray.direction.x * ray.direction.x + ray.direction.y + ray.direction.y);
    double q = (ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) -
        geometry.cylinder_radius;
    double discriminant = p * p - q;

    if (discriminant < 0.0)
        return false;
    double t = -p - std::sqrt(discriminant);
    if (t < 0)
        double t = -p + std::sqrt(discriminant);

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
    double p = glm::dot(ray.direction, o_minus_c);
    double q = dot(o_minus_c, o_minus_c) -
        (geometry.sphere_radius * geometry.sphere_radius);
    double discriminant = (p * p) - q;

    if (discriminant < 0.0)
        return false;
    double t = -p - std::sqrt(discriminant);
    if (t >= 0)
        t = -p + std::sqrt(discriminant);

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
    double t = -p - std::sqrt(discriminant);
    if (t >= 0)
        t = -p + std::sqrt(discriminant);

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
