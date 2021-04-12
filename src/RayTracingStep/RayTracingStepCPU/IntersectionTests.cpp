#include "IntersectionTests.hpp"

inline bool ldplab::rtscpu::IntersectionTest::rayTriangle(
    const Ray& ray, 
    const Triangle& triangle, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& dist)
{
    constexpr double EPSILON = 1e-10;
    const Vec3 edge1 = triangle.b - triangle.a;
    const Vec3 edge2 = triangle.c - triangle.a;
    const Vec3 h = glm::cross(ray.direction, edge2);
    const double a = glm::dot(edge1, h);
    // Check parallelisim
    if (a > -EPSILON && a < EPSILON)
        return false;
    const double f = 1.0 / a;
    const Vec3 s = ray.origin - triangle.a;
    const double u = f * glm::dot(s, h);
    if (u < 0.0 || u > 1.0)
        return false;
    const Vec3 q = glm::cross(s, edge1);
    const double v = f * glm::dot(ray.direction, q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // Calculate distance to intersection point
    double t = f * glm::dot(edge2, q);
    if (t > EPSILON)
    {
        dist = t;
        intersection_point = ray.origin + ray.direction * t;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, ray.direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    else
        return false;
}

inline bool ldplab::rtscpu::IntersectionTest::lineTriangle(
    const Vec3& line_start, 
    const Vec3& line_end, 
    const Triangle& triangle, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& dist)
{
    constexpr double EPSILON = 1e-10;
    const Vec3 line_direction = glm::normalize(line_end - line_start);
    const Vec3 edge1 = triangle.b - triangle.a;
    const Vec3 edge2 = triangle.c - triangle.a;
    const Vec3 h = glm::cross(line_direction, edge2);
    const double a = glm::dot(edge1, h);
    // Check parallelisim
    if (a > -EPSILON && a < EPSILON)
        return false;
    const double f = 1.0 / a;
    const Vec3 s = line_start - triangle.a;
    const double u = f * glm::dot(s, h);
    if (u < 0.0 || u > 1.0)
        return false;
    const Vec3 q = glm::cross(s, edge1);
    const double v = f * glm::dot(line_start, q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // Calculate distance to intersection point
    double t = f * glm::dot(edge2, q);
    if (t > EPSILON && t <= glm::length(line_end - line_start))
    {
        dist = t;
        intersection_point = line_start + line_direction * t;
        intersection_normal = glm::normalize(glm::cross(edge1, edge2));
        if (glm::dot(intersection_normal, line_direction) > 0)
            intersection_normal = -intersection_normal;
        return true;
    }
    else
        return false;
}

inline bool ldplab::rtscpu::IntersectionTest::raySphere(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    double& min_dist, 
    double& max_dist)
{
    constexpr double EPSILON = 1e-10;
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) -
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < EPSILON)
        return false;

    min_dist = -p - std::sqrt(discriminant);
    max_dist = -p + std::sqrt(discriminant);
    return true;
}

inline bool ldplab::rtscpu::IntersectionTest::raySphere(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    Vec3& min_intersection_point, 
    Vec3& min_intersection_normal,
    double& min_dist,
    Vec3& max_intersection_point, 
    Vec3& max_intersection_normal, 
    double& max_dist)
{
    constexpr double EPSILON = 1e-10;
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) - 
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < EPSILON)
        return false;
    
    // Calculate intersections
    min_dist = -p - std::sqrt(discriminant);
    min_intersection_point = ray.origin + min_dist * ray.direction;
    min_intersection_normal = 
        glm::normalize(min_intersection_point - sphere_center);
    max_dist = -p + std::sqrt(discriminant);
    max_intersection_point = ray.origin + max_dist * ray.direction;
    max_intersection_normal =
        glm::normalize(max_intersection_point - sphere_center);
    return true;
}

inline bool ldplab::rtscpu::IntersectionTest::raySphereOnlyMin(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& dist)
{
    constexpr double EPSILON = 1e-10;
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) -
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < EPSILON)
        return false;

    // Calculate intersections
    dist = -p - std::sqrt(discriminant);
    intersection_point = ray.origin + dist * ray.direction;
    intersection_normal =
        glm::normalize(intersection_point - sphere_center);
    return true;
}

inline bool ldplab::rtscpu::IntersectionTest::raySphereOnlyMax(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& dist)
{
    constexpr double EPSILON = 1e-10;
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) -
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < EPSILON)
        return false;

    // Calculate intersections
    dist = -p + std::sqrt(discriminant);
    intersection_point = ray.origin + dist * ray.direction;
    intersection_normal =
        glm::normalize(intersection_point - sphere_center);
    return true;
}

inline bool ldplab::rtscpu::IntersectionTest::triangleAABB(
    const Triangle& triangle,
    const AABB& aabb)
{
    const Vec3 aabb_extent = aabb.max - aabb.min;
    const Vec3 aabb_center = aabb.min + aabb_extent;

    // Based on the work of Tomas Akenine-Möller
    // "Fast3DTriangle-BoxOverlapTesting", March 2001

    const Vec3 v0 = triangle.a - aabb_center;
    const Vec3 v1 = triangle.b - aabb_center;
    const Vec3 v2 = triangle.c - aabb_center;

    // ========================================================================
    // Test for triangle aabb intersection
    double min, max;

    // X-Direction
    min = std::fmin(v0.x, std::fmin(v1.x, v2.x));
    max = std::fmax(v0.x, std::fmax(v1.x, v2.x));
    if (min > aabb_extent.x || max < -aabb_extent.x)
        return false;

    // Y-Direction
    min = std::fmin(v0.y, std::fmin(v1.y, v2.y));
    max = std::fmax(v0.y, std::fmax(v1.y, v2.y));
    if (min > aabb_extent.y || max < -aabb_extent.y)
        return false;

    // Z-Direction
    min = std::fmin(v0.z, std::fmin(v1.z, v2.z));
    max = std::fmax(v0.z, std::fmax(v1.z, v2.z));
    if (min > aabb_extent.z || max < -aabb_extent.z)
        return false;

    // ========================================================================
    // 9 Tests
    const Vec3 e0 = v1 - v0;
    const Vec3 e1 = v2 - v1;
    const Vec3 e2 = v0 - v2;

    Vec3 abs;
    double rad;

    // BUILD ABS
    abs.x = std::abs(e0.x);
    abs.y = std::abs(e0.y);
    abs.z = std::abs(e0.z);

    // AXISTEST_X01
    min = e0.z * v0.y - e0.y * v0.z;
    max = e0.z * v2.y - e0.y * v2.z;
    if (min > max) std::swap<double>(min, max);
    rad = abs.z * aabb_extent.y + abs.y * aabb_extent.z;
    if (min > rad || max < -rad) return false;

    // AXISTEST_Y02
    min = -e0.z * v0.x + e0.x * v0.z;
    max = -e0.z * v2.x + e0.x * v2.z;
    if (min > max) std::swap<double>(min, max);
    rad = abs.z * aabb_extent.x + abs.x * aabb_extent.z;
    if (min > rad || max < -rad) return false;

    // AXISTEST_Z12
    min = e0.y * v1.x - e0.x * v1.y;
    max = e0.y * v2.x - e0.x * v2.y;
    if (min > max) std::swap<double>(min, max);
    rad = abs.y * aabb_extent.x + abs.x * aabb_extent.y;
    if (min > rad || max < -rad) return false;

    // BUILD ABS
    abs.x = std::abs(e1.x);
    abs.y = std::abs(e1.y);
    abs.z = std::abs(e1.z);

    // AXISTEST_X01
    min = e1.z * v0.y - e1.y * v0.z;
    max = e1.z * v2.y - e1.y * v2.z;
    if (min > max) std::swap<double>(min, max);
    rad = abs.z * aabb_extent.y + abs.y * aabb_extent.z;
    if (min > rad || max < -rad) return false;

    // AXISTEST_Y02
    min = -e1.z * v0.x + e1.x * v0.z;
    max = -e1.z * v2.x + e1.x * v2.z;
    if (min > max) std::swap<double>(min, max);
    rad = abs.z * aabb_extent.x + abs.x * aabb_extent.z;
    if (min > rad || max < -rad) return false;

    // AXISTEST_Z0
    min = e1.y * v0.x - e1.x * v0.y;				   
    max = e1.y * v1.x - e1.x * v1.y;
    if (min > max) std::swap<double>(min, max);
    rad = abs.y * aabb_extent.x + abs.x * aabb_extent.y;
    if (min > rad || max < -rad) return false;

    // BUILD ABS
    abs.x = std::abs(e2.x);
    abs.y = std::abs(e2.y);
    abs.z = std::abs(e2.z);

    // AXISTEST_X2
    min = e2.z * v0.y - e2.y * v0.z;
    max = e2.z * v1.y - e2.y * v1.z;
    if (min > max) std::swap<double>(min, max);
    rad = abs.z * aabb_extent.y + abs.y * aabb_extent.z;
    if (min > rad || max < -rad) return false;

    // AXISTEST_Y1
    min = -e2.z * v0.x + e2.x * v0.z;
    max = -e2.z * v1.x + e2.x * v1.z;
    if (min > max) std::swap<double>(min, max);
    rad = abs.z * aabb_extent.x + abs.x * aabb_extent.z;
    if (min > rad || max < -rad) return false;

    // AXISTEST_Z12
    min = e2.y * v1.x - e2.x * v1.y;
    max = e2.y * v2.x - e2.x * v2.y;
    if (min > max) std::swap<double>(min, max);
    rad = abs.y * aabb_extent.x + abs.x * aabb_extent.y;
    if (min > rad || max < -rad) return false;

    // ========================================================================
    // Test if aabb does intersect triangle plane
    Vec3 vmin, vmax;
    vmin.x = -aabb_extent.x - v0.x;
    vmax.x = aabb_extent.x - v0.x;
    vmin.y = -aabb_extent.y - v0.y;
    vmax.y = aabb_extent.y - v0.y;
    vmin.z = -aabb_extent.z - v0.z;
    vmax.z = aabb_extent.z - v0.z;
    if (vmin.x > vmax.x) std::swap<double>(vmin.x, vmax.x);
    if (vmin.y > vmax.y) std::swap<double>(vmin.y, vmax.y);
    if (vmin.z > vmax.z) std::swap<double>(vmin.z, vmax.z);

    const Vec3 normal = glm::cross(e0, e1);
    if (glm::dot(normal, vmin) > 0.0)
        return false; // aabb does not overlap => seperation axis found
    else if (glm::dot(normal, vmax) >= 0.0)
        return true; // aabb does overlap => no seperation axis found
    else
        return false; // aabb does not overlap => seperation axis found
    
    // No seperation axis found => Triangle and AABB intersect
    //return true;
}

inline bool ldplab::rtscpu::IntersectionTest::rayAABB(
    const Ray& ray, 
    const AABB& aabb, 
    double& min_dist, 
    double& max_dist)
{
    /*
    min_dist = (aabb.min.x - ray.origin.x) / ray.direction.x;
    max_dist = (aabb.max.x - ray.origin.x) / ray.direction.x;

    if (min_dist > max_dist)
        std::swap<double>(min_dist, max_dist);

    double tymin = (aabb.min.y - ray.origin.y) / ray.direction.y;
    double tymax = (aabb.max.y - ray.origin.y) / ray.direction.y;

    if (tymin > tymax)
        std::swap<double>(tymin, tymax);

    if ((min_dist > tymax) || (tymin > max_dist))
        return false;

    if (tymin > min_dist)
        min_dist = tymin;

    if (tymax < max_dist)
        max_dist = tymax;

    double tzmin = (aabb.min.z - ray.origin.z) / ray.direction.z;
    double tzmax = (aabb.max.z - ray.origin.z) / ray.direction.z;

    if (tzmin > tzmax)
        std::swap<double>(tzmin, tzmax);

    if ((min_dist > tzmax) || (tzmin > max_dist))
        return false;

    if (tzmin > min_dist)
        min_dist = tzmin;

    if (tzmax < max_dist)
        max_dist = tzmax;
    
    return true;
    */
}

inline bool ldplab::rtscpu::IntersectionTest::lineAABB(
    const Vec3& line_start, 
    const Vec3& line_end, 
    const AABB& aabb, 
    double& min_dist, 
    double& max_dist)
{
    return false;
}

inline bool ldplab::rtscpu::IntersectionTest::lineSlab(
    double line_start, 
    double line_end, 
    double slab_min, 
    double slab_max, 
    double& min_dist, 
    double& max_dist)
{
    /*
    constexpr double EPSILON = 1e-10;

    double line_length = line_end - line_start;
    if (line_length < EPSILON)
    {
        if (line_start < slab_min || line_start > slab_max)
            return false;
        else
            return true;
    }

    double dim_min = (slab_min - line_start) / line_length;
    double dim_max = (slab_max - line_start) / line_length;

    if (dim_max < dim_min)
        std::swap<double>(dim_max, dim_min);

    if (dim_max < min_dist)
        return false;
    */
    return false;
}
