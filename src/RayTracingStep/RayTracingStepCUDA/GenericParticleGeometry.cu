#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "GenericParticleGeometry.hpp"

#include <LDPLAB/Constants.hpp>

#include "IntersectionTests.hpp"
#include "../../Utils/Log.hpp"

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
    /** @todo */
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