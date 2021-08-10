#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "ImplGenericGeometry.hpp"

#include <LDPLAB/RayTracingStep/CUDA/DeviceResource.hpp>
#include <LDPLAB/Constants.hpp>
#include <cuda_runtime.h>

#include "IntersectionTests.hpp"

namespace sphere
{
    using namespace ldplab;
    using namespace rtscuda;
    __device__ bool intersectRay(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const void* geometry_data,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        bool& intersects_outside);
    __device__ bool intersectSegment(
        const Vec3& segment_origin,
        const Vec3& segment_end,
        const void* geometry_data,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        bool& end_point_inside);
    __device__ const IGenericGeometry::intersectRay
        intersectRayFp = intersectRay;
    __device__ const IGenericGeometry::intersectSegment
        intersectSegmentFp = intersectSegment;
    // Helper
    __device__ bool intersectRayAtDist(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometrySphereData* particle_data,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        bool& intersects_outside,
        double& dist);
}

ldplab::rtscuda::GenericGeometrySphere::GenericGeometrySphere(
    DeviceBuffer<GeometrySphereData>&& geometry_data)
    :
    m_geometry_data{ std::move(geometry_data) }
{ }

void* ldplab::rtscuda::GenericGeometrySphere::getDeviceData()
{
    return m_geometry_data.getDeviceBuffer();
}

ldplab::rtscuda::IGenericGeometry::intersectRay 
    ldplab::rtscuda::GenericGeometrySphere::getDeviceIntersectRayFunction()
{
    intersectRay kernel_fp = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel_fp,
        sphere::intersectRayFp,
        sizeof(sphere::intersectRayFp))
        != cudaSuccess)
        return nullptr;
    return kernel_fp;
}

ldplab::rtscuda::IGenericGeometry::intersectSegment 
    ldplab::rtscuda::GenericGeometrySphere::getDeviceIntersectSegmentFunction()
{
    intersectSegment kernel_fp = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel_fp,
        sphere::intersectSegmentFp,
        sizeof(sphere::intersectSegmentFp))
        != cudaSuccess)
        return nullptr;
    return kernel_fp;
}

__device__ bool sphere::intersectRay(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const void* geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& intersects_outside)
{
    const GeometrySphereData* particle_data =
        static_cast<const GeometrySphereData*>(geometry_data);
    double t;
    return intersectRayAtDist(
        ray_origin,
        ray_direction,
        particle_data,
        intersection_point,
        intersection_normal,
        intersects_outside,
        t);
}
__device__ bool sphere::intersectSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    const void* geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& end_point_inside)
{
    const GeometrySphereData* particle_data =
        static_cast<const GeometrySphereData*>(geometry_data);
    const Vec3 seg = segment_end - segment_origin;
    const double seg_length = glm::length(seg);
    const Vec3 ray_origin = segment_origin;
    const Vec3 ray_direction = seg / seg_length;
    double dist = 0;
    if (!intersectRayAtDist(
        ray_origin,
        ray_direction,
        particle_data,
        intersection_point,
        intersection_normal,
        end_point_inside,
        dist))
    {
        end_point_inside = false;
        return false;
    }
    else
    {
        if (dist <= seg_length + constant::intersection_tests::epsilon)
            return true;
        end_point_inside = !end_point_inside;
        return false;
    }
}
__device__ bool sphere::intersectRayAtDist(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometrySphereData* particle_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& intersects_outside, 
    double& dist)
{
    double max;
    if (!IntersectionTest::intersectRaySphere(
        ray_origin,
        ray_direction,
        Vec3(0, 0, 0),
        particle_data->radius,
        dist,
        max))
        return false;

    intersects_outside = true;
    if (dist < constant::intersection_tests::epsilon)
    {
        dist = max;
        intersects_outside = false;
    }
    intersection_point = ray_origin + dist * ray_direction;
    intersection_normal = glm::normalize(intersection_point);
    if (!intersects_outside)
        intersection_normal = -intersection_normal;
    return true;
}

namespace rod
{
    using namespace ldplab;
    using namespace rtscuda;
    __device__ bool intersectRay(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const void* geometry_data,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        bool& intersects_outside);
    __device__ bool intersectSegment(
        const Vec3& segment_origin,
        const Vec3& segment_end,
        const void* geometry_data,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        bool& end_point_inside);
    __device__ const IGenericGeometry::intersectRay 
        intersectRayFp = intersectRay;
    __device__ const IGenericGeometry::intersectSegment 
        intersectSegmentFp = intersectSegment;
    // Helper
    __device__ bool intersectRayAtDist(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometryRodData* particle_data,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        bool& intersects_outside,
        double& dist);
    __device__ bool overlapCylinder(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometryRodData* particle_data,
        double& dist_min,
        double& dist_max);
    __device__ bool intersectInsideCylinder(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometryRodData* particle_data,
        double max_dist,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        double& isec_dist,
        bool& intersection_outside);
    __device__ bool intersectOutsideCylinder(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometryRodData* particle_data,
        double min_dist,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        double& isec_dist);
    __device__ bool intersectCap(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometryRodData* particle_data,
        bool inside_cylinder,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        double& isec_dist,
        bool& intersects_outside);
    __device__ bool intersectIndent(
        const Vec3& ray_origin,
        const Vec3& ray_direction,
        const GeometryRodData* particle_data,
        bool inside_cylinder,
        Vec3& intersection_point,
        Vec3& intersection_normal,
        double& isec_dist,
        bool& intersects_outside);
}

ldplab::rtscuda::GenericGeometryRod::GenericGeometryRod(
    DeviceBuffer<GeometryRodData>&& geometry_data)
    :
    m_geometry_data{ std::move(geometry_data) }
{ }

void* ldplab::rtscuda::GenericGeometryRod::getDeviceData()
{
    return m_geometry_data.getDeviceBuffer();
}

ldplab::rtscuda::IGenericGeometry::intersectRay 
    ldplab::rtscuda::GenericGeometryRod::getDeviceIntersectRayFunction()
{
    intersectRay kernel_fp = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel_fp,
        rod::intersectRayFp,
        sizeof(rod::intersectRayFp))
        != cudaSuccess)
        return nullptr;
    return kernel_fp;
}

ldplab::rtscuda::IGenericGeometry::intersectSegment 
    ldplab::rtscuda::GenericGeometryRod::getDeviceIntersectSegmentFunction()
{
    intersectSegment kernel_fp = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel_fp,
        rod::intersectSegmentFp,
        sizeof(rod::intersectSegmentFp))
        != cudaSuccess)
        return nullptr;
    return kernel_fp;
}

__device__ bool rod::intersectRay(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const void* geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& intersects_outside)
{
    const GeometryRodData* particle_data =
        static_cast<const GeometryRodData*>(geometry_data);
    double t = 0;
    return intersectRayAtDist(
        ray_origin,
        ray_direction,
        particle_data,
        intersection_point,
        intersection_normal,
        intersects_outside,
        t);
}

__device__ bool rod::intersectSegment(
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    const void* geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    bool& end_point_inside)
{
    const GeometryRodData* particle_data =
        static_cast<const GeometryRodData*>(geometry_data);
    const Vec3 seg = segment_end - segment_origin;
    const double seg_length = glm::length(seg);
    const Vec3 ray_origin = segment_origin;
    const Vec3 ray_direction = seg / seg_length;
    double dist = 0;
    if (!intersectRayAtDist(
        ray_origin,
        ray_direction,
        particle_data,
        intersection_point,
        intersection_normal,
        end_point_inside,
        dist))
    {
        end_point_inside = false;
        return false;
    }
    else
    {
        if (dist <= seg_length + constant::intersection_tests::epsilon)
            return true;
        end_point_inside = !end_point_inside;
        return false;
    }
}

__device__ bool rod::intersectRayAtDist(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometryRodData* particle_data,
    Vec3& intersection_point, 
    Vec3& intersection_normal,
    bool& intersects_outside,
    double& dist)
{
    double min, max;
    if (overlapCylinder(ray_origin, ray_direction, particle_data, min, max))
    {
        if (min > constant::intersection_tests::epsilon) // Ray origin outside of infinite cylinder.
        {
            intersects_outside = true;
            return intersectOutsideCylinder(
                ray_origin,
                ray_direction,
                particle_data,
                min,
                intersection_point,
                intersection_normal,
                dist);
        }
        else // Ray origin inside the infinite cylinder
        {
            return intersectInsideCylinder(
                ray_origin,
                ray_direction,
                particle_data,
                max,
                intersection_point,
                intersection_normal,
                dist,
                intersects_outside);
        }
    }
    return false;
}

__device__ const double numeric_limits_double_infinity =
    std::numeric_limits<double>::infinity();

__device__ bool rod::overlapCylinder(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometryRodData* particle_data,
    double& dist_min,
    double& dist_max)
{
    const double ray_expansion_length =
        (ray_direction.x * ray_direction.x +
            ray_direction.y * ray_direction.y);
    if (ray_expansion_length < constant::intersection_tests::epsilon)
    {
        dist_min = -numeric_limits_double_infinity;
        dist_max = numeric_limits_double_infinity;
        return (ray_origin.x * ray_origin.x + ray_origin.y * ray_origin.y <
            particle_data->cylinder_radius* particle_data->cylinder_radius);
    }

    const double p =
        (ray_origin.x * ray_direction.x + ray_origin.y * ray_direction.y) /
        ray_expansion_length;
    const double q =
        ((ray_origin.x * ray_origin.x + ray_origin.y * ray_origin.y) -
            particle_data->cylinder_radius * particle_data->cylinder_radius) /
        ray_expansion_length;

    const double discriminant = p * p - q;
    if (discriminant < constant::intersection_tests::epsilon)
        return false;
    dist_min = -p - std::sqrt(discriminant);
    dist_max = -p + std::sqrt(discriminant);
    return dist_max >= constant::intersection_tests::epsilon;
}

__device__ bool rod::intersectInsideCylinder(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometryRodData* particle_data,
    double max_dist, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& isec_dist, 
    bool& intersection_outside)
{
    // Parallel to cylinder
    Vec3 t_point, t_normal;
    double t;
    bool cap_intersection = false, indent_intersection = false;

    cap_intersection = intersectCap(
        ray_origin,
        ray_direction,
        particle_data,
        true,
        intersection_point,
        intersection_normal,
        isec_dist,
        intersection_outside);
    indent_intersection = intersectIndent(
        ray_origin,
        ray_direction,
        particle_data,
        true,
        t_point,
        t_normal,
        t,
        intersection_outside);

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
    if (max_dist < numeric_limits_double_infinity)
    {
        // Not parallel to cylinder
        intersection_point = ray_origin + max_dist * ray_direction;
        if (intersection_point.z < 0 ||
            intersection_point.z > particle_data->cylinder_length)
            return false;
        intersection_outside = false;
        intersection_normal = glm::normalize(Vec3(
            -intersection_point.x,
            -intersection_point.y,
            0));
        isec_dist = max_dist;
        return true;
    }

    return false;
}

__device__ bool rod::intersectOutsideCylinder(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometryRodData* particle_data,
    double min_dist, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& isec_dist)
{
    intersection_point = ray_origin + min_dist * ray_direction;
    if (intersection_point.z >= 0 &&
        intersection_point.z <= particle_data->cylinder_length)
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
        bool outside;
        return intersectIndent(
            ray_origin,
            ray_direction,
            particle_data,
            false,
            intersection_point,
            intersection_normal,
            isec_dist,
            outside);
    }
    else
    {
        bool outside;
        return intersectCap(
            ray_origin,
            ray_direction,
            particle_data,
            false,
            intersection_point,
            intersection_normal,
            isec_dist,
            outside);
    }

    return false;
}

__device__ bool rod::intersectCap(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometryRodData* particle_data,
    bool inside_cylinder, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& isec_dist, 
    bool& intersects_outside)
{
    // Kappa too small, we have a cylinder
    if (particle_data->sphere_radius <= 0)
    {
        if (ray_direction.z == 0)
            return false;
        else
        {
            isec_dist = (particle_data->origin_cap.z - ray_origin.z) / ray_direction.z;
            if (isec_dist < constant::intersection_tests::epsilon)
                return false;
            intersection_point = ray_origin + isec_dist * ray_direction;
            const double isec_x2 = intersection_point.x * intersection_point.x;
            const double isec_y2 = intersection_point.y * intersection_point.y;
            if (isec_x2 + isec_y2 > particle_data->cylinder_radius * particle_data->cylinder_radius)
                return false;
        }
        intersection_normal = Vec3(0, 0, 1);
        intersects_outside = true;
        if (glm::dot(intersection_normal, ray_direction) > 0)
        {
            intersection_normal = -intersection_normal;
            intersects_outside = false;
        }
        return true;
    }

    // We have a sphere
    double min, max;
    if (IntersectionTest::intersectRaySphere(
        ray_origin,
        ray_direction,
        particle_data->origin_cap,
        particle_data->sphere_radius,
        min,
        max))
    {
        if (min < constant::intersection_tests::epsilon)
            isec_dist = max;
        else if (inside_cylinder && ray_origin.z < particle_data->origin_cap.z)
            isec_dist = max;
        else
            isec_dist = min;
        intersection_point = ray_origin + isec_dist * ray_direction;
        if (intersection_point.z <= particle_data->cylinder_length)
            return false;
        intersection_normal = glm::normalize(intersection_point - particle_data->origin_cap);
        intersects_outside = true;
        if (glm::dot(intersection_normal, ray_direction) > 0)
        {
            intersection_normal = -intersection_normal;
            intersects_outside = false;
        }
        return true;
    }

    return false;
}

__device__ bool rod::intersectIndent(
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    const GeometryRodData* particle_data,
    bool inside_cylinder, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    double& isec_dist, 
    bool& intersects_outside)
{
    // Kappa too small, we have a cylinder
    if (particle_data->sphere_radius <= 0)
    {
        if (ray_direction.z == 0)
            return false;
        else
        {
            isec_dist = -ray_origin.z / ray_direction.z;
            if (isec_dist < constant::intersection_tests::epsilon)
                return false;
            intersection_point = ray_origin + isec_dist * ray_direction;
            const double isec_x2 = intersection_point.x * intersection_point.x;
            const double isec_y2 = intersection_point.y * intersection_point.y;
            if (isec_x2 + isec_y2 > particle_data->cylinder_radius * particle_data->cylinder_radius)
                return false;
        }
        intersection_normal = Vec3(0, 0, -1);
        intersects_outside = true;
        if (glm::dot(intersection_normal, ray_direction) > 0)
        {
            intersection_normal = -intersection_normal;
            intersects_outside = false;
        }
        return true;
    }

    // We have a sphere
    double min, max;
    if (IntersectionTest::intersectRaySphere(
        ray_origin,
        ray_direction,
        particle_data->origin_indentation,
        particle_data->sphere_radius,
        min,
        max))
    {
        if (min < constant::intersection_tests::epsilon)
            isec_dist = max;
        else if (inside_cylinder && ray_origin.z > 0)
            isec_dist = min;
        else
            isec_dist = max;
        intersection_point = ray_origin + isec_dist * ray_direction;
        if (intersection_point.z <= 0)
            return false;
        intersection_normal = glm::normalize(intersection_point - particle_data->origin_indentation);
        intersects_outside = false;
        if (glm::dot(intersection_normal, ray_direction) > 0)
        {
            intersection_normal = -intersection_normal;
            intersects_outside = true;
        }
        return true;
    }

    return false;
}

#endif