#include "RayParticleIntersectionTestStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../SimulationState.hpp"
#include "../../Log.hpp"
#include "../../Utils/Assert.hpp"

#include "../../../../libs/glm/glm.hpp"

#include <cmath>

ldplab::rtscpu::RodeParticleIntersectionTest::
    RodeParticleIntersectionTest(std::shared_ptr<Context> context)
    :
    m_context{ context }
{
}

void ldplab::rtscpu::RodeParticleIntersectionTest::execute(
    const SimulationState* state,
    const size_t particle,
    RayBuffer& input_hit_rays,
    RayBuffer& missed_rays,
    IntersectionBuffer& intersection)
{
    LDPLAB_LOG_DEBUG("RTSCPU context %i: Execute ray particle intersection"\
        " test on batch %i",
        m_context->uid, input_hit_rays.index);

    RodeParticle& particle_geometrie = 
        m_context->rode_particle_geometry[state->particles[particle].type];
    double cap_hight = particle_geometrie.origin_cap.z + 
        particle_geometrie.sphere_radius - particle_geometrie.cylinder_length;

    for (int i = 0; i < input_hit_rays.num_rays; i++)
    {
        Ray& ray = input_hit_rays.data[i];
        Ray& missed_ray = missed_rays.data[i];
        Vec3& inter_point = intersection.point[i];
        Vec3& inter_normal = intersection.normal[i];

        if(!intersectionTest(
            ray,
            particle_geometrie, 
            inter_point,
            inter_normal))
        {
            // Ray missed particle
            missed_ray = ray;
            ray.intensity = -1;
            input_hit_rays.num_rays--;
            missed_rays.num_rays++;
        }
    }

    LDPLAB_LOG_TRACE("RTSCPU context %i: RayBuffer %i contains %i rays that "\
        "hit the particle", m_context->uid, input_hit_rays.num_rays, 
        input_hit_rays.index);
    LDPLAB_LOG_TRACE("RTSCPU context %i: RayBuffer %i contains %i rays that "\
        "missed the particle", m_context->uid, missed_rays.num_rays,
        missed_rays.index);
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::intersectionTest(
    const Ray& ray,
    const RodeParticle& particle_geometrie,
    Vec3& inter_point, 
    Vec3& inter_normal)
{
    double intersec_first = 0;
    double intersec_second = 0;

    // Check cylinder intersection
    if (cylinderIntersection(
        particle_geometrie,
        ray,
        intersec_first,
        intersec_second))
    {
        if (intersec_first >= 0) // Ray origin outside infinite cylinder
        {
            inter_point = ray.origin + intersec_first * ray.direction;
            if (inter_point.z >= 0 &&
                inter_point.z <= particle_geometrie.cylinder_length)
            {
                inter_normal = { inter_point.x,inter_point.y,0 };
                inter_normal = glm::normalize(inter_normal);
                return true;
            }
            else if (inter_point.z < 0) // First intersection under the cylinder
            {
                inter_point = ray.origin + intersec_second * ray.direction;
                if (inter_point.z >= 0)
                {
                    return indentationIntersection(
                        particle_geometrie,
                        ray,
                        inter_point,
                        inter_normal);
                }
            }
            else // First intersection over the cylinder
            {
                return capIntersection(
                    particle_geometrie,
                    ray,
                    inter_point,
                    inter_normal);
            }
        }
        else if (intersec_second > 0) // Ray origin inside infinite cylinder
        {
            inter_point = ray.origin + intersec_first * ray.direction;
            if (inter_point.z < 0) // First intersection under the cylinder
            {
                inter_point = ray.origin + intersec_second * ray.direction;
                if (inter_point.z > 0)
                {
                    return indentationIntersection(
                        particle_geometrie,
                        ray,
                        inter_point,
                        inter_normal);
                }
            }
            // First intersection over the cylinder
            else if (inter_point.z > particle_geometrie.cylinder_length) 
            {
                return capIntersection(
                    particle_geometrie,
                    ray,
                    inter_point,
                    inter_normal);
            }
        }
    }
    // Check for ray inside the infinite cylinder with orthogonal direction
    else
    {
        double distance = 
            ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y;
        if (distance <= particle_geometrie.cylinder_radius * 
                particle_geometrie.cylinder_radius)
        {
            if (ray.origin.z >= particle_geometrie.cylinder_length && 
                ray.direction.z < 0)
            {
                return capIntersection(
                    particle_geometrie,
                    ray,
                    inter_point,
                    inter_normal);
            }
            else if ( (ray.origin.z <= 
                particle_geometrie.origin_indentation.z + 
                particle_geometrie.cylinder_radius) &&
                ray.direction.z > 0)
            {
                return indentationIntersection(
                    particle_geometrie,
                    ray,
                    inter_point,
                    inter_normal);
            }
        }
    }
    return false;
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::cylinderIntersection(
    const RodeParticle& particle,
    const Ray& ray,
    double& distance_min, 
    double& distance_max)
{
    double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
        (ray.direction.x * ray.direction.x + ray.direction.y + ray.direction.y);
    double q = (ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) - 
        particle.cylinder_radius;

    double discriminant = p * p - q;
    if (discriminant < 0.0)
        return false;
    distance_min = - p - std::sqrt(discriminant);
    distance_max = - p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::sphereIntersection(
    const Vec3& origin, 
    const double& raduis, 
    const Ray& ray, 
    double& distance_min, 
    double& distance_max)
{
    Vec3 o_minus_c = ray.origin - origin;

    double p = glm::dot(ray.direction, o_minus_c);
    double q = dot(o_minus_c, o_minus_c) - (raduis * raduis);

    double discriminant = (p * p) - q;
    if (discriminant < 0.0)
        return false;

    distance_min = -p - std::sqrt(discriminant);
    distance_max = -p + std::sqrt(discriminant);

    return true;
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::capIntersection(
    const RodeParticle& particle_geometrie,
    const Ray& ray, 
    Vec3& inter_point,
    Vec3& inter_normal)
{
    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        particle_geometrie.origin_cap,
        particle_geometrie.cylinder_radius,
        ray,
        intersec_first,
        intersec_second))
    {
        inter_point = ray.origin + intersec_first *
            ray.direction;
        if (inter_point.z > particle_geometrie.cylinder_length &&
            inter_point.z <= particle_geometrie.origin_cap.z +
            particle_geometrie.sphere_radius)
        {
            inter_normal = glm::normalize(
                particle_geometrie.origin_cap - inter_point);
            return true;
        }
    }
    return false;
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::indentationIntersection(
    const RodeParticle& particle_geometrie,
    const Ray& ray,
    Vec3& intersection_point,
    Vec3& intersection_normal)
{
    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        particle_geometrie.origin_indentation,
        particle_geometrie.cylinder_radius,
        ray,
        intersec_first,
        intersec_second))
    {
        intersection_point = ray.origin + intersec_second * ray.direction;
        intersection_normal = glm::normalize(
            particle_geometrie.origin_indentation - intersection_point);
        return true;
    }
    return false;
}
