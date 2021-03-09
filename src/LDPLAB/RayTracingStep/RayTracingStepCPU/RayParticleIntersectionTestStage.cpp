#include "RayParticleIntersectionTestStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "Debug/Debug.hpp"

#include "../../SimulationState.hpp"
#include "../../Utils/Log.hpp"

#include "../../../../libs/glm/glm.hpp"

#include <cmath>

ldplab::rtscpu::RodParticleIntersectionTest::
    RodParticleIntersectionTest(std::shared_ptr<Context> context)
    :
    m_context{ context },
    m_rod_particles{ ((RodParticleData*)
        context->particle_data.get())->particle_data }
{ }

void ldplab::rtscpu::RodParticleIntersectionTest::execute(
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
            rays.index_data[i] >= m_context->particles.size() ||
            rays.index_data[i] == intersection.particle_index[i])
            continue;

        const RodParticle& geometry = m_rod_particles[rays.index_data[i]];
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
            rays.index_data[i] =
                static_cast<int32_t>(m_context->particles.size());
            num_missed_rays++;
        }
        else
        {
            intersection.particle_index[i] = rays.index_data[i];
            num_hit_rays++;
        }
    }

    rays.world_space_rays += num_missed_rays;

    LDPLAB_LOG_TRACE("RTSCPU context %i: Ray particle intersection test on "\
        "batch buffer %i completed, of %i tested rays %i rays hit particles "\
        "and %i rays missed",
        m_context->uid, 
        rays.uid, 
        num_hit_rays + num_missed_rays,
        num_hit_rays, 
        num_missed_rays);
}

bool ldplab::rtscpu::RodParticleIntersectionTest::intersectionTest(
    const RodParticle& geometry,
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
                inter_normal = { inter_point.x, inter_point.y, 0 };
                inter_normal = glm::normalize(inter_normal);
                return true;
            }
            else if (inter_point.z < 0) // First intersection under the cylinder
            {
                return indentationIntersection(
                    geometry,
                    ray,
                    inter_point,
                    inter_normal);
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
            return bottomTopIntersection(
                geometry,
                ray,
                inter_point,
                inter_normal);
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
            return bottomTopIntersection(
                geometry, 
                ray, 
                inter_point, 
                inter_normal);
        }
    }
    return false;
}

bool ldplab::rtscpu::RodParticleIntersectionTest::cylinderIntersection(
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
    distance_min = - p - std::sqrt(discriminant);
    distance_max = - p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtscpu::RodParticleIntersectionTest::bottomTopIntersection(
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

bool ldplab::rtscpu::RodParticleIntersectionTest::sphereIntersection(
    const Vec3& origin, 
    const double& raduis, 
    const Ray& ray, 
    double& distance_min, 
    double& distance_max)
{
    const Vec3 o_minus_c = ray.origin - origin;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) - (raduis * raduis);

    const double discriminant = (p * p) - q;
    if (discriminant < 1e-9)
        return false;

    distance_min = -p - std::sqrt(discriminant);
    distance_max = -p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtscpu::RodParticleIntersectionTest::capIntersection(
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
        inter_normal = Vec3(0, 0, 1);

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
            inter_normal = glm::normalize(inter_point - geometry.origin_cap);
            return true;
        }
    }
    return false;
}

bool ldplab::rtscpu::RodParticleIntersectionTest::indentationIntersection(
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
        inter_normal = Vec3(0, 0, -1);
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
                geometry.origin_indentation - inter_point);
            return true;
        }
    }
    return false;
}
