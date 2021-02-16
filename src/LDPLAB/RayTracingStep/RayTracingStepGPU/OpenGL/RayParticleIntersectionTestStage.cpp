#include "RayParticleIntersectionTestStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../../SimulationState.hpp"
#include "../../../Log.hpp"

#include <glm/glm.hpp>
#include <cmath>

ldplab::rtsgpu_ogl::RodParticleIntersectionTest::
    RodParticleIntersectionTest(std::shared_ptr<Context> context)
    :
    m_context{ context },
    m_rod_particles{ ((RodParticleData*)
        context->particle_data.get())->particle_data }
{ }

void ldplab::rtsgpu_ogl::RodParticleIntersectionTest::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute ray particle intersection"\
        " test on batch buffer %i",
        m_context->uid, rays.uid);
    size_t num_hit_rays = 0;
    size_t num_missed_rays = 0;

    for (size_t i = 0; i < rays.size; i++)
    {
        if (rays.index_data[i] < 0 ||
            rays.index_data[i] >= m_context->particles.size() ||
            rays.index_data[i] == intersection.particle_index_data[i])
            continue;

        const RodParticle& geometry = m_rod_particles[rays.index_data[i]];
        Vec4& ray_origin = rays.ray_origin_data[i];
        Vec4& ray_direction = rays.ray_direction_data[i];
        Vec4& inter_point = intersection.point_data[i];
        Vec4& inter_normal = intersection.normal_data[i];

        if (!intersectionTest(
            geometry,
            ray_origin,
            ray_direction,
            inter_point,
            inter_normal))
        {
            // Transformation to world space
            const size_t pidx = rays.index_data[i];
            const Mat4& scale_rotation = 
                m_context->particle_transformation_data.p2w_scale_rotation_data[pidx];
            const Vec4& translation =
                m_context->particle_transformation_data.p2w_translation_data[pidx];
            ray_origin = scale_rotation * ray_origin + translation;
            ray_direction = glm::normalize(scale_rotation * ray_direction);
            // Ray missed particle
            rays.index_data[i] = 
                static_cast<int32_t>(m_context->particles.size());
            num_missed_rays++;
        }
        else
        {
            intersection.particle_index_data[i] = rays.index_data[i];
            num_hit_rays++;
        }
    }

    rays.world_space_rays += num_missed_rays;

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Ray particle intersection test on "\
        "batch buffer %i completed, of %i tested rays %i rays hit particles "\
        "and %i rays missed",
        m_context->uid, 
        rays.uid, 
        num_hit_rays + num_missed_rays,
        num_hit_rays, 
        num_missed_rays);
}

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::intersectionTest(
    const RodParticle& geometry,
    const Vec4& ray_origin,
    const Vec4& ray_direction,
    Vec4& inter_point, 
    Vec4& inter_normal)
{
    double intersec_first = 0;
    double intersec_second = 0;

    // Check cylinder intersection
    if (cylinderIntersection(
        geometry,
        ray_origin,
        ray_direction,
        intersec_first,
        intersec_second))
    {
        if (intersec_first >= 0) // Ray origin outside infinite cylinder
        {
            inter_point = ray_origin + intersec_first * ray_direction;
            if (inter_point.z >= 0 &&
                inter_point.z <= geometry.cylinder_length)
            {
                inter_normal = { inter_point.x, inter_point.y, 0, 0 };
                inter_normal = glm::normalize(inter_normal);
                return true;
            }
            else if (inter_point.z < 0) // First intersection under the cylinder
            {
                return indentationIntersection(
                    geometry,
                    ray_origin,
                    ray_direction,
                    inter_point,
                    inter_normal);
            }
            else // First intersection over the cylinder
            {
                return capIntersection(
                    geometry,
                    ray_origin,
                    ray_direction,
                    inter_point,
                    inter_normal);
            }
        }
        else if (intersec_second > 0) // Ray origin inside infinite cylinder
        {
            return bottomTopIntersection(
                geometry,
                ray_origin,
                ray_direction,
                inter_point,
                inter_normal);
        }
    }
    // Check for ray inside the infinite cylinder with orthogonal direction
    else
    {
        double distance =
            ray_origin.x * ray_origin.x + ray_origin.y * ray_origin.y;
        if (distance <= geometry.cylinder_radius *
            geometry.cylinder_radius)
        {
            return bottomTopIntersection(
                geometry,
                ray_origin,
                ray_direction,
                inter_point,
                inter_normal);
        }
    }
    return false;
}

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::bottomTopIntersection(
    const RodParticle& particle,
    const Vec4& ray_origin,
    const Vec4& ray_direction,
    Vec4& inter_point,
    Vec4& inter_normal)
{
    if (ray_origin.z <= 0) // Ray origin below the particle
    {
        if (ray_direction.z <= 0)
            return false;
        else
            return indentationIntersection(
                particle,
                ray_origin,
                ray_direction,
                inter_point,
                inter_normal);
    }

    const double particle_height =
        particle.origin_cap.z + particle.sphere_radius;
    if (ray_origin.z >= particle_height) // Ray origin above the particle
    {
        if (ray_direction.z >= 0)
            return false;
        else
            return capIntersection(
                particle,
                ray_origin,
                ray_direction,
                inter_point,
                inter_normal);
    }

    const double dist =
        glm::length(particle.origin_indentation - ray_origin);
    if (dist <= particle.sphere_radius + 1e-9)
        return indentationIntersection(
            particle,
            ray_origin,
            ray_direction,
            inter_point,
            inter_normal);
    else
        return capIntersection(
            particle,
            ray_origin,
            ray_direction,
            inter_point,
            inter_normal);
}

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::cylinderIntersection(
    const RodParticle& geometry,
    const Vec4& ray_origin,
    const Vec4& ray_direction,
    double& distance_min, 
    double& distance_max)
{
    double p =
        (ray_origin.x * ray_direction.x + ray_origin.y * ray_direction.y) /
        (ray_direction.x * ray_direction.x + ray_direction.y * ray_direction.y);
    double q = ((ray_origin.x * ray_origin.x + ray_origin.y * ray_origin.y) -
        geometry.cylinder_radius * geometry.cylinder_radius) /
        (ray_direction.x * ray_direction.x + ray_direction.y * ray_direction.y);

    double discriminant = p * p - q;
    if (discriminant < 0.0)
        return false;
    distance_min = - p - std::sqrt(discriminant);
    distance_max = - p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::sphereIntersection(
    const Vec4& sphere_origin, 
    const double& sphere_raduis, 
    const Vec4& ray_origin, 
    const Vec4& ray_direction, 
    double& distance_min, 
    double& distance_max)
{
    const Vec4 o_minus_c = ray_origin - sphere_origin;

    const double p = glm::dot(ray_direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) - 
        (sphere_raduis * sphere_raduis);

    const double discriminant = (p * p) - q;
    if (discriminant < 1e-9)
        return false;

    distance_min = -p - std::sqrt(discriminant);
    distance_max = -p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::capIntersection(
    const RodParticle& geometry,
    const Vec4& ray_origin, 
    const Vec4& ray_direction, 
    Vec4& inter_point,
    Vec4& inter_normal)
{
    if (geometry.origin_indentation.z + geometry.sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_direction.z == 0)
            return false;
        const double t = (geometry.cylinder_length - ray_origin.z) /
            ray_direction.z;
        if (t < 0)
            return false;
        inter_point = ray_origin + t * ray_direction;
        if (inter_point.x * inter_point.x + inter_point.y * inter_point.y >
            geometry.cylinder_radius * geometry.cylinder_radius)
            return false;
        inter_normal = Vec4(0, 0, 1, 0);
        return true;
    }

    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        geometry.origin_cap,
        geometry.sphere_radius,
        ray_origin,
        ray_direction,
        intersec_first,
        intersec_second))
    {
        if (intersec_first < 0)
            return false;
        inter_point = ray_origin + intersec_first *
            ray_direction;
        if (inter_point.z > geometry.cylinder_length &&
            inter_point.z <= geometry.origin_cap.z + geometry.sphere_radius)
        {
            inter_normal = glm::normalize(inter_point - geometry.origin_cap);
            return true;
        }
    }
    return false;
}

bool ldplab::rtsgpu_ogl::RodParticleIntersectionTest::indentationIntersection(
    const RodParticle& geometry,
    const Vec4& ray_origin,
    const Vec4& ray_direction,
    Vec4& inter_point,
    Vec4& inter_normal)
{
    if (geometry.origin_indentation.z + geometry.sphere_radius < 1e-3)
    {
        // Kappa is too small (or 0) and therefore assume the shape as perfect
        // cylinder.
        if (ray_direction.z == 0)
            return false;
        const double t = -ray_origin.z / ray_direction.z;
        if (t < 0)
            return false;
        inter_point = ray_origin + t * ray_direction;
        if (inter_point.x * inter_point.x + inter_point.y * inter_point.y >
            geometry.cylinder_radius * geometry.cylinder_radius)
            return false;
        inter_normal = Vec4(0, 0, -1, 0);
        return true;
    }

    double intersec_first = 0;
    double intersec_second = 0;

    if (sphereIntersection(
        geometry.origin_indentation,
        geometry.sphere_radius,
        ray_origin,
        ray_direction,
        intersec_first,
        intersec_second))
    {
        inter_point = ray_origin + intersec_second * ray_direction;

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
