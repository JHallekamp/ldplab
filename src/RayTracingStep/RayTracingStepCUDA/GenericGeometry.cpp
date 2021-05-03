#include "GenericGeometry.hpp"

#include <LDPLAB/Constants.hpp>

#include "IntersectionTests.hpp"

bool ldplab::rtscuda::IGenericGeometry::intersectRay(
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    double t;
    return intersectRay(ray, intersection_point, intersection_normal, t);
}

bool ldplab::rtscuda::IGenericGeometry::intersectSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    bool t = false;
    return intersectSegment(
        segment_origin, 
        segment_end, 
        intersection_point, 
        intersection_normal, 
        t);
}

bool ldplab::rtscuda::IGenericGeometry::intersectSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& ray_intersects)
{
    const Vec3 seg = segment_end - segment_origin;
    const double seg_length = glm::length(seg);
    const Ray r{ segment_origin, seg / seg_length, -1 };
    double dist = 0;
    ray_intersects = 
        intersectRay(r, intersection_point, intersection_normal, dist);
    return (ray_intersects && dist <= seg_length);
}

ldplab::rtscuda::RodGeometry::RodGeometry(
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

bool ldplab::rtscuda::RodGeometry::intersectRay(
    const Ray& ray, 
    Vec3& intersection_point,
    Vec3& intersection_normal,
    double& dist)
{
    double min, max;
    if (overlapCylinder(ray, min, max))
    {
        // Ray intersects the cylinder or lies within it.
        if (min > constant::intersection_tests::epsilon) // Ray origin outside of infinite cylinder.
        {
            return intersectOutsideCylinder(
                ray, 
                min,
                intersection_point, 
                intersection_normal,
                dist);
        }
        else // Ray origin inside the infinite cylinder
        {
            return intersectInsideCylinder(
                ray,
                max,
                intersection_point,
                intersection_normal,
                dist);
        }
    }
    return false;
}

bool ldplab::rtscuda::RodGeometry::overlapCylinder(
    const Ray& ray, 
    double& dist_min, 
    double& dist_max)
{
    const double ray_expansion_length = 
        (ray.direction.x * ray.direction.x + 
            ray.direction.y * ray.direction.y);
    if (ray_expansion_length < constant::intersection_tests::epsilon)
    {
        dist_min = -std::numeric_limits<double>::infinity();
        dist_max = std::numeric_limits<double>::infinity();
        return (ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y <
            m_cylinder_radius * m_cylinder_radius);
    }

    const double p =
        (ray.origin.x * ray.direction.x + ray.origin.y * ray.direction.y) /
            ray_expansion_length;
    const double q = 
        ((ray.origin.x * ray.origin.x + ray.origin.y * ray.origin.y) -
            m_cylinder_radius * m_cylinder_radius) / ray_expansion_length;
    
    const double discriminant = p * p - q;
    if (discriminant < constant::intersection_tests::epsilon)
        return false;
    dist_min = -p - std::sqrt(discriminant);
    dist_max = -p + std::sqrt(discriminant);
    return dist_max >= constant::intersection_tests::epsilon;
}

bool ldplab::rtscuda::RodGeometry::intersectInsideCylinder(
    const Ray& ray, 
    double max_dist,
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    double& isec_dist)
{
    // Parallel to cylinder
    Vec3 t_point, t_normal;
    double t;
    bool cap_intersection = false, indent_intersection = false;

    cap_intersection = intersectCap(
        ray, true, intersection_point, intersection_normal, isec_dist);
    indent_intersection = intersectIndent(
        ray, true, t_point, t_normal, t);

    if (indent_intersection && cap_intersection)
    {
        if (t < isec_dist)
        {
            intersection_point = t_point;
            intersection_normal = t_normal;
            isec_dist = t;
        }
        return true;
    }
    else if (indent_intersection)
    {
        intersection_point = t_point;
        intersection_normal = t_normal;
        isec_dist = t;
        return true;
    }
    else if (cap_intersection)
        return true;

    // Check if parallel to cylinder
    if (max_dist < std::numeric_limits<double>::infinity())
    {
        // Not parallel to cylinder
        intersection_point = ray.origin + max_dist * ray.direction;
        if (intersection_point.z < 0 ||
            intersection_point.z > m_cylinder_length)
            return false;
        intersection_normal = glm::normalize(Vec3(
            -intersection_point.x,
            -intersection_point.y,
            0));
        isec_dist = max_dist;
        return true;
    }

    return false;
}

bool ldplab::rtscuda::RodGeometry::intersectOutsideCylinder(
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
        return intersectIndent(
            ray,
            false,
            intersection_point,
            intersection_normal,
            isec_dist);
    }
    else
    {
        return intersectCap(
            ray,
            false,
            intersection_point,
            intersection_normal,
            isec_dist);
    }
    return false;
}

bool ldplab::rtscuda::RodGeometry::intersectCap(
    const Ray& ray, 
    bool inside_cylinder,
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& isec_dist)
{
    // Kappa too small, we have a cylinder
    if (m_sphere_radius <= 0)
    {
        if (ray.direction.z == 0)
            return false;
        else
        {
            isec_dist = (m_origin_cap.z - ray.origin.z) / ray.direction.z;
            if (isec_dist < constant::intersection_tests::epsilon)
                return false;
            intersection_point = ray.origin + isec_dist * ray.direction;
            const double isec_x2 = intersection_point.x * intersection_point.x;
            const double isec_y2 = intersection_point.y * intersection_point.y;
            if (isec_x2 + isec_y2 > m_cylinder_radius * m_cylinder_radius)
                return false;
        }
        intersection_normal = Vec3(0, 0, 1);
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }

    // We have a sphere
    double min, max;
    if (IntersectionTest::intersectRaySphere(
        ray,
        m_origin_cap,
        m_sphere_radius,
        min,
        max))
    {
        if (min < constant::intersection_tests::epsilon)
            isec_dist = max;
        else if (inside_cylinder && ray.origin.z < m_origin_cap.z)
            isec_dist = max;
        else
            isec_dist = min;
        intersection_point = ray.origin + isec_dist * ray.direction;
        if (intersection_point.z <= m_cylinder_length)
            return false;
        //const double isec_x2 = intersection_point.x * intersection_point.x;
        //const double isec_y2 = intersection_point.y * intersection_point.y;
        //if (isec_x2 + isec_y2 > m_cylinder_radius * m_cylinder_radius)
        //    return false;
        intersection_normal = glm::normalize(intersection_point - m_origin_cap);
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }

    return false;
}

bool ldplab::rtscuda::RodGeometry::intersectIndent(
    const Ray& ray, 
    bool inside_cylinder,
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& isec_dist)
{
    // Kappa too small, we have a cylinder
    if (m_sphere_radius <= 0)
    {
        if (ray.direction.z == 0)
            return false;
        else
        {
            isec_dist = -ray.origin.z / ray.direction.z;
            if (isec_dist < constant::intersection_tests::epsilon)
                return false;
            intersection_point = ray.origin + isec_dist * ray.direction;
            const double isec_x2 = intersection_point.x * intersection_point.x;
            const double isec_y2 = intersection_point.y * intersection_point.y;
            if (isec_x2 + isec_y2 > m_cylinder_radius * m_cylinder_radius)
                return false;
        }
        intersection_normal = Vec3(0, 0, 1);
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }

    // We have a sphere
    double min, max;
    if (IntersectionTest::intersectRaySphere(
        ray,
        m_origin_indentation,
        m_sphere_radius,
        min,
        max))
    {
        if (min < constant::intersection_tests::epsilon)
            isec_dist = max;
        else if (inside_cylinder && ray.origin.z > 0)
            isec_dist = min;
        else
            isec_dist = max;
        intersection_point = ray.origin + isec_dist * ray.direction;
        if (intersection_point.z <= 0)
            return false;
        //const double isec_x2 = intersection_point.x * intersection_point.x;
        //const double isec_y2 = intersection_point.y * intersection_point.y;
        //if (isec_x2 + isec_y2 > m_cylinder_radius * m_cylinder_radius)
        //    return false;
        intersection_normal = glm::normalize(intersection_point - m_origin_indentation);
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }

    return false;
}

bool ldplab::rtscuda::RodGeometry::intersectTopBottomPlane(
    const Ray& ray,
    double z,
    Vec3& intersection_point,
    Vec3& intersection_normal,
    double& isec_dist)
{
    if (ray.direction.z == 0)
        return false;
    else
    {
        isec_dist = (z - ray.origin.z) / ray.direction.z;
        if (isec_dist < constant::intersection_tests::epsilon)
            return false;
        intersection_point = ray.origin + isec_dist * ray.direction;
        const double isec_x2 = intersection_point.x * intersection_point.x;
        const double isec_y2 = intersection_point.y * intersection_point.y;
        if (isec_x2 + isec_y2 > m_cylinder_radius * m_cylinder_radius)
            return false;
    }
    intersection_normal = Vec3(0, 0, 1);
    if (glm::dot(intersection_normal, ray.direction) > 0)
        intersection_normal = -intersection_normal;
    return true;
}

ldplab::rtscuda::SphericalGeometry::SphericalGeometry(
    const SphericalParticleGeometry* geometry)
    :
    m_radius{ geometry->radius }
{ }

bool ldplab::rtscuda::SphericalGeometry::intersectRay(
    const Ray& ray, 
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    double& dist)
{
    double max;
    if (!IntersectionTest::intersectRaySphere(ray, Vec3(0, 0, 0), m_radius, dist, max))
        return false;
    if (dist < constant::intersection_tests::epsilon)
        dist = max;
    intersection_point = ray.origin + dist * ray.direction;
    intersection_normal = glm::normalize(intersection_point);
    if (glm::dot(intersection_normal, ray.direction) > 0)
        intersection_normal = -intersection_normal;
    return true;
}