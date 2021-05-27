#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "GenericParticleGeometry.hpp"

#include <LDPLAB/Constants.hpp>

#include "../../Utils/Log.hpp"

__device__ bool ldplab::rtscuda::GenericParticlFunctionWrapper::intersectRay(
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

__device__ bool ldplab::rtscuda::GenericParticlFunctionWrapper::intersectRay(
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

__device__ bool ldplab::rtscuda::GenericParticlFunctionWrapper::intersectSegment(
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
    ldplab::rtscuda::RodParticle::intersect_ray_kernel_ptr =
        ldplab::rtscuda::RodParticle::intersectRayKernel;

ldplab::rtscuda::intersectRayParticleGeometryFunction_t 
    ldplab::rtscuda::RodParticle::getIsecFunction()
{
    // Copy the function pointer to the host
    intersectRayParticleGeometryFunction_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
        &kernel,
        intersect_ray_kernel_ptr,
        sizeof(intersect_ray_kernel_ptr))
        != cudaSuccess)
        return nullptr;
    return kernel;
}

ldplab::rtscuda::GenericParticleGeometryData::Type 
    ldplab::rtscuda::RodParticle::getGeometryType()
{
    return GenericParticleGeometryData::Type::TYPE_ROD;
}

#endif