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
