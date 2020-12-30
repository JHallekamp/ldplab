#include "RayParticleIntersectionTestStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../SimulationState.hpp"
#include "../../Log.hpp"

#include "../../../../libs/glm/glm.hpp"

#include <cmath>

ldplab::rtscpu::RodeParticleIntersectionTest::
    RodeParticleIntersectionTest(std::shared_ptr<Context> context)
    :
    m_context{ context }
{
}

void ldplab::rtscpu::RodeParticleIntersectionTest::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection)
{
    LDPLAB_LOG_TRACE("RTSCPU context %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        m_context->uid, rays.uid);
    size_t num_hit_rays = 0;
    size_t num_missed_rays = 0;

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context->particles.size())
            continue;

        RodeParticle& geometry = m_context->
            rode_particle_geometry[rays.index_data[i]];
        double cap_hight = geometry.origin_cap.z +
            geometry.sphere_radius - geometry.cylinder_length;

        Ray& ray = rays.ray_data[i];
        Vec3& inter_point = intersection.point[i];
        Vec3& inter_normal = intersection.normal[i];

        if (!intersectionTest(
            geometry,
            ray,
            inter_point,
            inter_normal))
        {
            // Transformation to world space
            ParticleTransformation& trans = m_context->
                particle_transformations[rays.index_data[i]];
            ray.origin = trans.p2w_scale_rotation * ray.origin +
                trans.p2w_translation;
            ray.direction =
                glm::normalize(trans.p2w_scale_rotation * ray.direction);
            // Ray missed particle
            rays.index_data[i] = m_context->particles.size();
            num_missed_rays++;
        }
        else
            num_hit_rays++;
    }

    LDPLAB_LOG_TRACE("RTSCPU context %i: Ray particle intersection test on "\
        "batch buffer %i completed, of %i tested rays %i rays hit particles "\
        "and %i rays missed",
        m_context->uid, 
        rays.uid, 
        num_hit_rays + num_missed_rays,
        num_hit_rays, 
        num_missed_rays);
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::intersectionTest(
    const RodeParticle& geometry,
    const Ray& ray,
    Vec3& inter_point, 
    Vec3& inter_normal)
{
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
                        geometry,
                        ray,
                        inter_point,
                        inter_normal);
                }
            }
            else // First intersection over the cylinder
            {
                return capIntersection(
                    geometry,
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
                        geometry,
                        ray,
                        inter_point,
                        inter_normal);
                }
            }
            // First intersection over the cylinder
            else if (inter_point.z > geometry.cylinder_length) 
            {
                return capIntersection(
                    geometry,
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
        if (distance <= geometry.cylinder_radius * 
                geometry.cylinder_radius)
        {
            if (ray.origin.z >= geometry.cylinder_length && 
                ray.direction.z < 0)
            {
                return capIntersection(
                    geometry,
                    ray,
                    inter_point,
                    inter_normal);
            }
            else if ( (ray.origin.z <= 
                geometry.origin_indentation.z + 
                geometry.cylinder_radius) &&
                ray.direction.z > 0)
            {
                return indentationIntersection(
                    geometry,
                    ray,
                    inter_point,
                    inter_normal);
            }
        }
    }
    return false;
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::cylinderIntersection(
    const RodeParticle& geometry,
    const Ray& ray,
    double& distance_min, 
    double& distance_max)
{
    double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
        (ray.direction.x * ray.direction.x + ray.direction.y + ray.direction.y);
    double q = (ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) - 
        geometry.cylinder_radius;

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
    const RodeParticle& geometry,
    const Ray& ray, 
    Vec3& inter_point,
    Vec3& inter_normal)
{
    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        geometry.origin_cap,
        geometry.cylinder_radius,
        ray,
        intersec_first,
        intersec_second))
    {
        inter_point = ray.origin + intersec_first *
            ray.direction;
        if (inter_point.z > geometry.cylinder_length &&
            inter_point.z <= geometry.origin_cap.z +
            geometry.sphere_radius)
        {
            inter_normal = glm::normalize(
                geometry.origin_cap - inter_point);
            return true;
        }
    }
    return false;
}

bool ldplab::rtscpu::RodeParticleIntersectionTest::indentationIntersection(
    const RodeParticle& geometry,
    const Ray& ray,
    Vec3& inter_point,
    Vec3& inter_normal)
{
    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        geometry.origin_indentation,
        geometry.cylinder_radius,
        ray,
        intersec_first,
        intersec_second))
    {
        inter_point = ray.origin + intersec_second * ray.direction;
        inter_normal = glm::normalize(
            geometry.origin_indentation - inter_point);
        return true;
    }
    return false;
}
