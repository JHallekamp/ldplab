#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "GenericParticleGeometry.hpp"

#include <LDPLAB/Constants.hpp>

#include "IntersectionTests.hpp"
#include "../../../Utils/Log.hpp"

__device__ bool ldplab::rtscuda::GenericParticleFunctionWrapper::intersectRay(
    intersectRayParticleGeometryFunction_t function, 
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    void* particle_geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal)
{
    double td;
    bool to;
    return function(
        ray_origin, 
        ray_direction, 
        particle_geometry_data, 
        intersection_point, 
        intersection_normal, 
        td, 
        to);
}

__device__ bool ldplab::rtscuda::GenericParticleFunctionWrapper::intersectRay(
    intersectRayParticleGeometryFunction_t function, 
    const Vec3& ray_origin, 
    const Vec3& ray_direction, 
    void* particle_geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& intersects_outside)
{
    double td;
    return function(
        ray_origin,
        ray_direction,
        particle_geometry_data,
        intersection_point,
        intersection_normal,
        td,
        intersects_outside);
}

__device__ bool ldplab::rtscuda::GenericParticleFunctionWrapper::intersectSegment(
    intersectRayParticleGeometryFunction_t function, 
    const Vec3& segment_origin, 
    const Vec3& segment_end, 
    void* particle_geometry_data, 
    Vec3& intersection_point, 
    Vec3& intersection_normal, 
    bool& end_point_inside)
{
    const Vec3 seg = segment_end - segment_origin;
    const double seg_length = glm::length(seg);
    const Vec3 ray_origin = segment_origin;
    const Vec3 ray_direction = seg / seg_length;
    double dist = 0;
    if (!function(
        ray_origin, 
        ray_direction, 
        particle_geometry_data, 
        intersection_point, 
        intersection_normal, 
        dist, 
        end_point_inside))
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

std::shared_ptr<ldplab::rtscuda::GenericParticleGeometry> 
    ldplab::rtscuda::GenericParticleGeometry::create(
        std::shared_ptr<IParticleGeometry> particle_geometry)
{
    std::shared_ptr<ldplab::rtscuda::GenericParticleGeometry> geometry;
    if (particle_geometry->type() == IParticleGeometry::Type::rod_particle)
        geometry = std::make_shared<RodParticle>();
    else if (particle_geometry->type() == IParticleGeometry::Type::sphere)
        geometry = std::make_shared<SphereParticle>();
    else
    {
        // Type not supported.
        LDPLAB_LOG_ERROR("RTSCUDA generic particle geometry: "
            "Could not create resource, unsupported geometry type");
        return nullptr;
    }
    // Allocate resource
    if (!geometry->allocate(particle_geometry))
        return nullptr;
    return geometry;
}

ldplab::rtscuda::GenericParticleGeometryData 
    ldplab::rtscuda::GenericParticleGeometry::getData()
{
    GenericParticleGeometryData data;
    data.type = getGeometryType();
    data.data = getResourcePtr();
    data.intersect_ray_particle = getIsecFunction();
    return data;
}

namespace rod_particle_cuda
{
    /** @brief Intersection kernel. */
    __device__ bool intersectRayKernel(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        void* particle_geometry_data,
        ldplab::Vec3& intersection_point,
        ldplab::Vec3& intersection_normal,
        double& dist,
        bool& intersects_outside);
    /** @brief Actual function pointer. */
    __device__ ldplab::rtscuda::intersectRayParticleGeometryFunction_t
        intersect_ray_kernel_ptr = intersectRayKernel;
    
    __device__ bool overlapCylinder(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        ldplab::rtscuda::RodParticle::Data* particle_data,
        double& dist_min,
        double& dist_max);
    __device__ bool intersectInsideCylinder(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        ldplab::rtscuda::RodParticle::Data* particle_data,
        double max_dist,
        ldplab::Vec3& intersection_point,
        ldplab::Vec3& intersection_normal,
        double& isec_dist,
        bool& intersection_outside);
    __device__ bool intersectOutsideCylinder(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        ldplab::rtscuda::RodParticle::Data* particle_data,
        double min_dist,
        ldplab::Vec3& intersection_point,
        ldplab::Vec3& intersection_normal,
        double& isec_dist);
    __device__ bool intersectCap(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        ldplab::rtscuda::RodParticle::Data* particle_data,
        bool inside_cylinder,
        ldplab::Vec3& intersection_point,
        ldplab::Vec3& intersection_normal,
        double& isec_dist,
        bool& intersects_outside);
    __device__ bool intersectIndent(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        ldplab::rtscuda::RodParticle::Data* particle_data,
        bool inside_cylinder,
        ldplab::Vec3& intersection_point,
        ldplab::Vec3& intersection_normal,
        double& isec_dist,
        bool& intersects_outside);
}

bool ldplab::rtscuda::RodParticle::allocate(
    std::shared_ptr<IParticleGeometry> particle_geometry)
{
    // Allocate data
    if (!m_data.allocate(1))
        return false;
    // Upload data
    RodParticleGeometry* geometry = static_cast<RodParticleGeometry*>(
        particle_geometry.get());
    double h, sphere_radius;
    if (geometry->kappa <= constant::rod_particle::min_kappa_threshold)
        h = sphere_radius = 0;
    else
    {
        h = geometry->kappa * geometry->cylinder_radius;
        sphere_radius = 0.5 *
            (h + geometry->cylinder_radius * geometry->cylinder_radius / h);
    }    
    Data data;
    data.cylinder_length = geometry->cylinder_length;
    data.cylinder_radius = geometry->cylinder_radius;
    data.sphere_radius = sphere_radius;
    data.origin_cap = Vec3(0, 0, data.cylinder_length + h - sphere_radius);
    data.origin_indentation = Vec3(0, 0, h - sphere_radius);
    if (!m_data.upload(&data))
        return false;
    return true;
}

void* ldplab::rtscuda::RodParticle::getResourcePtr()
{
    return m_data.getResource();
}

ldplab::rtscuda::intersectRayParticleGeometryFunction_t 
    ldplab::rtscuda::RodParticle::getIsecFunction()
{
    // Copy the function pointer to the host
    intersectRayParticleGeometryFunction_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        rod_particle_cuda::intersect_ray_kernel_ptr,
        sizeof(rod_particle_cuda::intersect_ray_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

ldplab::rtscuda::GenericParticleGeometryData::Type 
    ldplab::rtscuda::RodParticle::getGeometryType()
{
    return GenericParticleGeometryData::Type::TYPE_ROD;
}

__device__ bool rod_particle_cuda::intersectRayKernel(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    void* particle_geometry_data, 
    ldplab::Vec3& intersection_point, 
    ldplab::Vec3& intersection_normal, 
    double& dist, 
    bool& intersects_outside)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

    RodParticle::Data* particle_data = 
        static_cast<RodParticle::Data*>(particle_geometry_data);

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

__device__ bool rod_particle_cuda::overlapCylinder(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    ldplab::rtscuda::RodParticle::Data* particle_data, 
    double& dist_min, 
    double& dist_max)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

    const double ray_expansion_length =
        (ray_direction.x * ray_direction.x +
            ray_direction.y * ray_direction.y);
    if (ray_expansion_length < constant::intersection_tests::epsilon)
    {
        dist_min = -numeric_limits_double_infinity;
        dist_max = numeric_limits_double_infinity;
        return (ray_origin.x * ray_origin.x + ray_origin.y * ray_origin.y <
            particle_data->cylinder_radius * particle_data->cylinder_radius);
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

__device__ bool rod_particle_cuda::intersectInsideCylinder(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    ldplab::rtscuda::RodParticle::Data* particle_data, 
    double max_dist, 
    ldplab::Vec3& intersection_point, 
    ldplab::Vec3& intersection_normal, 
    double& isec_dist, 
    bool& intersection_outside)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

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

__device__ bool rod_particle_cuda::intersectOutsideCylinder(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    ldplab::rtscuda::RodParticle::Data* particle_data, 
    double min_dist, 
    ldplab::Vec3& intersection_point, 
    ldplab::Vec3& intersection_normal, double& isec_dist)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

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

__device__ bool rod_particle_cuda::intersectCap(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    ldplab::rtscuda::RodParticle::Data* particle_data, 
    bool inside_cylinder, 
    ldplab::Vec3& intersection_point, 
    ldplab::Vec3& intersection_normal, 
    double& isec_dist, 
    bool& intersects_outside)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

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

__device__ bool rod_particle_cuda::intersectIndent(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    ldplab::rtscuda::RodParticle::Data* particle_data, 
    bool inside_cylinder, 
    ldplab::Vec3& intersection_point, 
    ldplab::Vec3& intersection_normal, 
    double& isec_dist, 
    bool& intersects_outside)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;

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

namespace sphere_particle_cuda
{
    /** @brief Intersection kernel. */
    static __device__ bool intersectRayKernel(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        void* particle_geometry_data,
        ldplab::Vec3& intersection_point,
        ldplab::Vec3& intersection_normal,
        double& dist,
        bool& intersects_outside);
    /** @brief Actual function pointer. */
    static __device__ ldplab::rtscuda::intersectRayParticleGeometryFunction_t
        intersect_ray_kernel_ptr = intersectRayKernel;
}

bool ldplab::rtscuda::SphereParticle::allocate(std::shared_ptr<IParticleGeometry> particle_geometry)
{
    // Allocate data
    if (!m_data.allocate(1))
        return false;
    // Upload data
    SphericalParticleGeometry* geometry =
        static_cast<SphericalParticleGeometry*>(particle_geometry.get());
    Data data;
    data.radius = geometry->radius;
    if (!m_data.upload(&data))
        return false;
    return true;
}

void* ldplab::rtscuda::SphereParticle::getResourcePtr()
{
    return m_data.getResource();
}

ldplab::rtscuda::intersectRayParticleGeometryFunction_t 
    ldplab::rtscuda::SphereParticle::getIsecFunction()
{
    // Copy the function pointer to the host
    intersectRayParticleGeometryFunction_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        sphere_particle_cuda::intersect_ray_kernel_ptr,
        sizeof(sphere_particle_cuda::intersect_ray_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

ldplab::rtscuda::GenericParticleGeometryData::Type 
    ldplab::rtscuda::SphereParticle::getGeometryType()
{
    return GenericParticleGeometryData::Type::TYPE_SPHERE;
}

__device__ bool sphere_particle_cuda::intersectRayKernel(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    void* particle_geometry_data, 
    ldplab::Vec3& intersection_point, 
    ldplab::Vec3& intersection_normal, 
    double& dist, 
    bool& intersects_outside)
{
    using namespace ldplab;
    using namespace ldplab::rtscuda;
    SphereParticle::Data* const sphere_data = 
        static_cast<SphereParticle::Data*>(particle_geometry_data);
    double max;
    if (!IntersectionTest::intersectRaySphere(
        ray_origin,
        ray_direction,
        Vec3(0, 0, 0),
        sphere_data->radius,
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

#endif