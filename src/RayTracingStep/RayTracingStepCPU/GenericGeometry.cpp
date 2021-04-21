#include "GenericGeometry.hpp"

#include <LDPLAB/Constants.hpp>

#include "IntersectionTests.hpp"

ldplab::rtscpu::RodGeometry::RodGeometry(
    const RodParticleGeometry* geometry)
{ 
    double h, sphere_radius;
    if (geometry->kappa <= constant::rod_particle::min_kappa_threshold)
        h = sphere_radius = 0;
    else
    {
        h = geometry->kappa * geometry->cylinder_radius;
        sphere_radius = 0.5 *
            (h + geometry->cylinder_radius * geometry->cylinder_radius / h);
    }
    m_cylinder_length = geometry->cylinder_length;
    m_cylinder_radius = geometry->cylinder_radius;
    m_sphere_radius = sphere_radius;
    m_origin_cap = Vec3(0, 0, m_cylinder_length + h - m_sphere_radius);
    m_origin_indentation = Vec3(0, 0, h - m_sphere_radius);
}

bool ldplab::rtscpu::RodGeometry::intersects(
    const Ray& ray, 
    Vec3& intersection_point,
    Vec3& intersection_normal)
{
    double min, max;
    if (cylinderIntersection(ray, min, max))
    {
        // Ray intersects the cylinder or lies within it.
        if (min >= 0) // Ray origin outside of infinite cylinder.
        {
            return intersectOutsideCylinder(
                ray, 
                min,
                intersection_point, 
                intersection_normal,
                max);
        }
        else // Ray origin inside the infinite cylinder
        {
            return intersectInsideCylinder(
                ray,
                intersection_point,
                intersection_normal,
                max);
        }
    }
    return false;
}

bool ldplab::rtscpu::RodGeometry::intersectsLineSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    double min, max;
    const Ray ray{ segment_origin, segment_end - segment_origin, -1 };
    if (cylinderIntersection(ray, min, max))
    {
        // Ray intersects the cylinder or lies within it.
        if (min >= 0.0) // Ray origin outside of infinite cylinder.
        {
            bool isec = intersectOutsideCylinder(
                ray,
                min,
                intersection_point,
                intersection_normal,
                max);
            if (isec && max <= 1.0)
                return true;
        }
        else // Ray origin inside the infinite cylinder
        {
            bool isec = intersectInsideCylinder(
                ray,
                intersection_point,
                intersection_normal,
                max);
            if (isec && max <= 1.0)
                return true;
        }
    }
    return false;
}

bool ldplab::rtscpu::RodGeometry::cylinderIntersection(
    const Ray& ray, 
    double& dist_min, 
    double& dist_max)
{
    constexpr double EPSILON = 1e-10;

    const double ray_expansion_length = 
        (ray.direction.x * ray.direction.x + 
            ray.direction.y * ray.direction.y);
    if (ray_expansion_length < EPSILON)
    {
        dist_min = -std::numeric_limits<double>::infinity();
        dist_max = std::numeric_limits<double>::infinity();
        return (ray.origin.x * ray.origin.x + ray.origin.y + ray.origin.y <=
            m_cylinder_radius * m_cylinder_radius);
    }

    const double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
            ray_expansion_length;
    const double q = 
        ((ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) -
            m_cylinder_radius * m_cylinder_radius) / ray_expansion_length;
    
    const double discriminant = p * p - q;
    if (discriminant < 0)
        return false;
    dist_min = -p - std::sqrt(discriminant);
    dist_max = -p + std::sqrt(discriminant);
    return true;
}

bool ldplab::rtscpu::RodGeometry::intersectInsideCylinder(
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    double& isec_dist)
{
    if (ray.origin.z <= 0) // Ray origin below particle
    {
        if (ray.direction.z <= 0)
            return false;
        else
            return IntersectionTest::raySphereOnlyMax(
                ray,
                m_origin_indentation,
                m_sphere_radius,
                intersection_point,
                intersection_normal,
                isec_dist);
    }
    
    const double particle_height = m_origin_cap.z + m_sphere_radius;
    if (ray.origin.z >= particle_height) // Ray above the particle
    {
        if (ray.direction.z >= 0)
            return false;
        else
            return IntersectionTest::raySphereOnlyMin(
                ray,
                m_origin_cap,
                m_sphere_radius,
                intersection_point,
                intersection_normal,
                isec_dist);
    }

    // Ray somewhere in the minimal enclosing cylinder.
    const double cap_dist = glm::length(m_origin_cap - ray.origin);
    if (cap_dist < m_sphere_radius && ray.direction.z >= 0)
    {
        return IntersectionTest::raySphereOnlyMax(
            ray,
            m_origin_cap,
            m_sphere_radius,
            intersection_point,
            intersection_normal,
            isec_dist);
    }
    else if (ray.origin.z >= m_origin_cap.z);
    {
        if (ray.direction.z > 0)
            return false;
        else
            return IntersectionTest::raySphereOnlyMin(
                ray,
                m_origin_cap,
                m_sphere_radius,
                intersection_point,
                intersection_normal,
                isec_dist);
    }

    const double indent_dist = glm::length(m_origin_indentation - ray.origin);
    if (indent_dist < m_sphere_radius)
    {
        if (ray.direction.z < 0)
            return false;
        else
            return IntersectionTest::raySphereOnlyMax(
                ray,
                m_origin_indentation,
                m_sphere_radius,
                intersection_point,
                intersection_normal,
                isec_dist);
    }
    else if (ray.direction.z <= 0)
    {
        return IntersectionTest::raySphereOnlyMin(
            ray,
            m_origin_indentation,
            m_sphere_radius,
            intersection_point,
            intersection_normal,
            isec_dist);
    }
    else
    {
        return IntersectionTest::raySphereOnlyMax(
            ray,
            m_origin_cap,
            m_sphere_radius,
            intersection_point,
            intersection_normal,
            isec_dist);
    }

    return false;
}

bool ldplab::rtscpu::RodGeometry::intersectOutsideCylinder(
    const Ray& ray, 
    double min_dist,
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    double& isec_dist)
{
    intersection_point = ray.origin + min_dist * ray.direction;
    if (intersection_point.z >= 0 &&
        intersection_point.z <= m_cylinder_length)
    {
        intersection_normal = glm::normalize(Vec3(
            intersection_point.x,
            intersection_point.y,
            0));
        isec_dist = min_dist;
        return true;
    }
    else if (intersection_point.z < 0) // First intersection under the cylinder
    {
        return IntersectionTest::raySphereOnlyMax(
            ray, 
            m_origin_indentation,
            m_sphere_radius,
            intersection_point,
            intersection_normal,
            isec_dist);
    }
    else
    {
        return IntersectionTest::raySphereOnlyMin(
            ray,
            m_origin_cap,
            m_sphere_radius,
            intersection_point,
            intersection_normal,
            isec_dist);
    }
    return false;
}

ldplab::rtscpu::SphericalGeometry::SphericalGeometry(
    const SphericalParticleGeometry* geometry)
    :
    m_radius{ geometry->radius }
{ }

bool ldplab::rtscpu::SphericalGeometry::intersects(
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    double min, max;
    if (!IntersectionTest::raySphere(ray, Vec3(0, 0, 0), m_radius, min, max))
        return false;
    if (min < 0)
        min = max;
    intersection_point = ray.origin + min * ray.direction;
    intersection_normal = glm::normalize(intersection_point);
    if (glm::dot(intersection_normal, ray.direction) > 0)
        intersection_normal = -intersection_normal;
    return true;
}

bool ldplab::rtscpu::SphericalGeometry::intersectsLineSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    double min, max;
    const Ray ray{ segment_origin, segment_end - segment_origin, -1.0 };
    if (!IntersectionTest::raySphere(ray, Vec3(0, 0, 0), m_radius, min, max))
        return false;
    if (min < 0)
        min = max;
    if (min > 1.0)
        return false;
    intersection_point = ray.origin + min * ray.direction;
    intersection_normal = glm::normalize(intersection_point);
    if (glm::dot(intersection_normal, ray.direction) > 0)
        intersection_normal = -intersection_normal;
    return true;
}
