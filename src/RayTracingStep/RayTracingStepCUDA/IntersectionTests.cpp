#include "IntersectionTests.hpp"

#include <array>
#include <LDPLAB/Constants.hpp>

bool ldplab::rtscuda::IntersectionTest::intersectRayTriangle(
    const Ray& ray, 
    const Triangle& triangle, 
    double& dist)
{
    const Vec3 edge1 = triangle.b - triangle.a;
    const Vec3 edge2 = triangle.c - triangle.a;
    const Vec3 h = glm::cross(ray.direction, edge2);
    const double a = glm::dot(edge1, h);
    // Check parallelisim
    if (a > -constant::intersection_tests::epsilon && 
        a < constant::intersection_tests::epsilon)
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
    if (t < constant::intersection_tests::epsilon)
        return false;
    dist = t;
    return true;
}

bool ldplab::rtscuda::IntersectionTest::intersectSegmentTriangle(
    const Vec3& segment_start, 
    const Vec3& segment_end, 
    const Triangle& triangle, 
    double& dist)
{
    const Vec3 seg = segment_end - segment_start;
    const Vec3 edge1 = triangle.b - triangle.a;
    const Vec3 edge2 = triangle.c - triangle.a;
    const Vec3 h = glm::cross(glm::normalize(seg), edge2);
    const double a = glm::dot(edge1, h);
    // Check parallelisim
    if (a > -constant::intersection_tests::epsilon && 
        a < constant::intersection_tests::epsilon)
        return false;
    const double f = 1.0 / a;
    const Vec3 s = segment_start - triangle.a;
    const double u = f * glm::dot(s, h);
    if (u < 0.0 || u > 1.0)
        return false;
    const Vec3 q = glm::cross(s, edge1);
    const double v = f * glm::dot(segment_start, q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // Calculate distance to intersection point
    double t = f * glm::dot(edge2, q);
    if (t < constant::intersection_tests::epsilon ||
        t + constant::intersection_tests::epsilon > glm::length(seg))
        return false;
    dist = t;
    return true;
}

bool ldplab::rtscuda::IntersectionTest::intersectRaySphere(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    double& min_dist, 
    double& max_dist)
{
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) -
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < constant::intersection_tests::epsilon)
        return false;

    min_dist = -p - std::sqrt(discriminant);
    max_dist = -p + std::sqrt(discriminant);
    return max_dist >= constant::intersection_tests::epsilon;
}

bool ldplab::rtscuda::IntersectionTest::intersectRaySphereMin(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& dist)
{
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) -
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < constant::intersection_tests::epsilon)
        return false;

    // Calculate intersections
    dist = -p - std::sqrt(discriminant);
    const double max_dist = -p + std::sqrt(discriminant);
    if (max_dist < constant::intersection_tests::epsilon)
        return false;
    intersection_point = ray.origin + dist * ray.direction;
    intersection_normal =
        glm::normalize(intersection_point - sphere_center);
    if (glm::dot(intersection_normal, ray.direction) > 0)
        intersection_normal = -intersection_normal;
    return true;
}

bool ldplab::rtscuda::IntersectionTest::intersectRaySphereMax(
    const Ray& ray, 
    const Vec3& sphere_center, 
    const double sphere_radius, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& dist)
{
    const Vec3 o_minus_c = ray.origin - sphere_center;

    const double p = glm::dot(ray.direction, o_minus_c);
    const double q = glm::dot(o_minus_c, o_minus_c) -
        (sphere_radius * sphere_radius);

    const double discriminant = (p * p) - q;
    if (discriminant < constant::intersection_tests::epsilon)
        return false;

    // Calculate intersections
    dist = -p + std::sqrt(discriminant);
    if (dist < constant::intersection_tests::epsilon)
        return false;
    intersection_point = ray.origin + dist * ray.direction;
    intersection_normal =
        glm::normalize(intersection_point - sphere_center);
    if (glm::dot(intersection_normal, ray.direction) > 0)
        intersection_normal = -intersection_normal;
    return true;
}

bool ldplab::rtscuda::IntersectionTest::overlapTriangleAABB(
    const Triangle& triangle,
    const AABB& aabb)
{
    // Based on AABB triangle intersection test by Thomas Akenine-Möller
    // See below copyright notice

    /********************************************************/
    /* AABB-triangle overlap test code                      */
    /* by Tomas Akenine-Möller                              */
    /* Function: int triBoxOverlap(float boxcenter[3],      */
    /*          float boxhalfsize[3],float triverts[3][3]); */
    /* History:                                             */
    /*   2001-03-05: released the code in its first version */
    /*   2001-06-18: changed the order of the tests, faster */
    /*                                                      */
    /* Acknowledgement: Many thanks to Pierre Terdiman for  */
    /* suggestions and discussions on how to optimize code. */
    /* Thanks to David Hunt for finding a ">="-bug!         */
    /********************************************************/
    
    /*
    Copyright 2020 Tomas Akenine-Möller
    
    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
    documentation files (the "Software"), to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
    to permit persons to whom the Software is furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all copies or substantial
    portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
    OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
    OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    */

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
    const Vec3 normal = glm::cross(e0, e1);
    Vec3 vmin, vmax;
    for (size_t d = 0; d < 3; ++d)
    {
        if (normal[d] >= 0.0)
        {
            vmin[d] = -aabb_extent[d] - v0[d];
            vmax[d] = aabb_extent[d] - v0[d];
        }
        else
        {
            vmin[d] = aabb_extent[d] - v0[d];
            vmax[d] = -aabb_extent[d] - v0[d];
        }
    }
    
    if (glm::dot(normal, vmin) > 0.0)
        return false; // aabb does not overlap => seperation axis found
    else if (glm::dot(normal, vmax) >= 0.0)
        return true; // aabb does overlap => no seperation axis found
    else
        return false; // aabb does not overlap => seperation axis found
    
    // No seperation axis found => Triangle and AABB intersect
    //return true;
}

bool ldplab::rtscuda::IntersectionTest::overlapRayAABB(
    const Ray& ray, 
    const AABB& aabb, 
    double& min_dist)
{
    // Based on original code by Andrew Woo
    // from "Graphics Gems", Academic Press, 1990
    constexpr size_t NUMDIM = 3;
    constexpr char RIGHT = 0;
    constexpr char LEFT = 1;
    constexpr char MIDDLE = 2;

    bool inside = true;
    std::array<char, NUMDIM> quadrant;
    Vec3 candidate_plane;

    // Find candidate planes
    for (size_t i = 0; i < NUMDIM; ++i)
    {
        if (ray.origin[i] < aabb.min[i])
        {
            quadrant[i] = LEFT;
            candidate_plane[i] = aabb.min[i];
            inside = false;
        }
        else if (ray.origin[i] > aabb.max[i])
        {
            quadrant[i] = RIGHT;
            candidate_plane[i] = aabb.max[i];
            inside = false;
        }
        else
            quadrant[i] = MIDDLE;
    }

    // Ray origin inside the AABB
    if (inside)
    {
        min_dist = 0.0;
        return true;
    }

    // Calculate T distances to candidate planes
    Vec3 t_max;
    for (size_t i = 0; i < NUMDIM; ++i)
    {
        if (quadrant[i] != MIDDLE && abs(ray.direction[i]) > 
                constant::intersection_tests::epsilon)
            t_max[i] = (candidate_plane[i] - ray.origin[i]) / ray.direction[i];
        else
            t_max[i] = -1.0;
    }

    // Get largest of the maxs for final choice of intersection
    min_dist = t_max[0];
    size_t which_plane = 0;
    for (size_t i = 1; i < NUMDIM; ++i)
    {
        if (min_dist < t_max[i])
        {
            which_plane = i;
            min_dist = t_max[i];
        }
    }

    // Check final candiate actually inside box
    if (min_dist < 0.0)
        return false;
    for (size_t i = 0; i < NUMDIM; ++i)
    {
        if (which_plane != i)
        {
            double coord = ray.origin[i] + min_dist * ray.direction[i];
            if (coord < aabb.min[i] || coord > aabb.max[i])
                return false;
        }
    }
    return true;
}

bool ldplab::rtscuda::IntersectionTest::overlapSegmentAABB(
    const Vec3& segment_start,
    const Vec3& segment_end, 
    const AABB& aabb, 
    double& min_dist)
{
    const Ray ray{ 
        segment_start, 
        glm::normalize(segment_end - segment_start), 
        0.0 };
    if (!overlapRayAABB(ray, aabb, min_dist))
        return false;
    if (min_dist > glm::length(segment_end - segment_start))
        return false;
    return true;
}
