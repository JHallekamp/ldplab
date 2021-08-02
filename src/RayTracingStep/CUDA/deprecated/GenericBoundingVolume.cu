#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "GenericBoundingVolume.hpp"

#include "IntersectionTests.hpp"
#include "../../../Utils/Log.hpp"


std::shared_ptr<ldplab::rtscuda::GenericBoundingVolume> 
    ldplab::rtscuda::GenericBoundingVolume::create(
        std::shared_ptr<IBoundingVolume> bounding_volume)
{
    // Select the type of the bounding volume.
    std::shared_ptr<ldplab::rtscuda::GenericBoundingVolume> bv;
    if (bounding_volume->type() == IBoundingVolume::Type::sphere)
        bv = std::make_shared<BoundingSphere>();
    else
    {
        // Type not supported.
        LDPLAB_LOG_ERROR("RTSCUDA generic bounding volume: "
            "Could not create resource, unsupported bounding volume type");
        return nullptr;
    }
    // Allocate resource
    if (!bv->allocate(bounding_volume))
        return nullptr;
    return bv;
}

ldplab::rtscuda::GenericBoundingVolumeData 
    ldplab::rtscuda::GenericBoundingVolume::getData()
{
    GenericBoundingVolumeData data;
    data.type = getBoundingVolumeType();
    data.data = getResourcePtr();
    data.intersect_ray_bounding_volume = getIsecFunction();
    return data;
}

namespace bounding_sphere_cuda
{
    /** @brief The intersection kernel. */
    static __device__ bool intersectRayKernel(
        const ldplab::Vec3& ray_origin,
        const ldplab::Vec3& ray_direction,
        void* bounding_volume_data,
        double& dist);
    /** @brief Actual function pointer. */
    static __device__ ldplab::rtscuda::intersectRayBoundingVolumeFunction_t
        intersect_ray_kernel_ptr = intersectRayKernel;
}

bool ldplab::rtscuda::BoundingSphere::allocate(
    std::shared_ptr<IBoundingVolume> bounding_volume)
{
    // Allocate data
    if (!m_data.allocate(1))
        return false;
    // Upload sphere data
    BoundingVolumeSphere* bv = static_cast<BoundingVolumeSphere*>(
        bounding_volume.get());
    Data data;
    data.center = bv->center;
    data.radius = bv->radius;
    if (!m_data.upload(&data))
        return false;
    return true;
}

void* ldplab::rtscuda::BoundingSphere::getResourcePtr()
{
    return m_data.getResource();
}

ldplab::rtscuda::intersectRayBoundingVolumeFunction_t 
    ldplab::rtscuda::BoundingSphere::getIsecFunction()
{
    // Copy the function pointer to the host
    intersectRayBoundingVolumeFunction_t kernel = nullptr;
    if (cudaMemcpyFromSymbol(
            &kernel, 
            bounding_sphere_cuda::intersect_ray_kernel_ptr,
            sizeof(bounding_sphere_cuda::intersect_ray_kernel_ptr))
            != cudaSuccess)
        return nullptr;
    return kernel;
}

ldplab::rtscuda::GenericBoundingVolumeData::Type 
    ldplab::rtscuda::BoundingSphere::getBoundingVolumeType()
{
    return GenericBoundingVolumeData::Type::TYPE_SPHERE;
}

__device__ bool bounding_sphere_cuda::intersectRayKernel(
    const ldplab::Vec3& ray_origin, 
    const ldplab::Vec3& ray_direction, 
    void* bounding_volume_data, 
    double& dist)
{
    double t;
    bool hit = ldplab::rtscuda::IntersectionTest::intersectRaySphere(
        ray_origin,
        ray_direction,
        static_cast<ldplab::rtscuda::BoundingSphere::Data*>(bounding_volume_data)->center,
        static_cast<ldplab::rtscuda::BoundingSphere::Data*>(bounding_volume_data)->radius,
        dist,
        t);
    return hit && dist >= 0;
}

#endif