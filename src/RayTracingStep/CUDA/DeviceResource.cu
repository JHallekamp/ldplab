#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include <LDPLAB/RayTracingStep/CUDA/DeviceResource.hpp>

#include "../../Utils/Log.hpp"

ldplab::rtscuda::IDeviceResource::~IDeviceResource()
{
    free();
}

ldplab::UID<ldplab::rtscuda::IDeviceResource> 
    ldplab::rtscuda::IDeviceResource::uid() const
{
    return m_resource_uid;
}

bool ldplab::rtscuda::IDeviceResource::checkCudaAllocationError(
    cudaError_t cuda_return_value, 
    size_t num_bytes) const
{
    if (cuda_return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not allocate %i "\
            "bytes of device memory, cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            cuda_return_value,
            cudaGetErrorString(cuda_return_value));
        return false;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully allocated %i "\
        "bytes of device memory",
        m_resource_uid,
        resourceTypeName(),
        num_bytes);
    return true;
}

bool ldplab::rtscuda::IDeviceResource::checkCudaFreeError(
    cudaError_t cuda_return_value, 
    void* device_ptr,
    size_t num_bytes) const
{
    if (cuda_return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not free %i bytes "\
            "of device memory at device address %p, cuda returned error code "\
            "%i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            device_ptr,
            cuda_return_value,
            cudaGetErrorString(cuda_return_value));
        return false;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully freed %i bytes "\
        "of device memory at device address %p",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        device_ptr);
    return true;
}

bool ldplab::rtscuda::IDeviceResource::checkCudaMemsetError(
    cudaError_t cuda_return_value, 
    void* device_offset_ptr,
    void* device_base_ptr,
    size_t num_bytes, 
    int value) const
{
    if (cuda_return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not set %i bytes "\
            "of device memory at device address %p (resource base address "\
            "%p) to value %i, cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            device_offset_ptr,
            value,
            cuda_return_value,
            cudaGetErrorString(cuda_return_value));
        return false;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully set %i bytes "\
        "of device memory at device address %p (resource base address %p) "\
        "to value %i",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        device_offset_ptr,
        device_base_ptr,
        value);
    return true;
}

bool ldplab::rtscuda::IDeviceResource::checkCudaUploadError(
    cudaError_t cuda_return_value, 
    void* device_offset_ptr,
    void* device_base_ptr,
    void* host_address, 
    size_t num_bytes) const
{
    if (cuda_return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not upload %i "\
            "bytes of memory from host address %p to device address %p ("\
            "resource base address %p), cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            host_address,
            device_offset_ptr,
            device_base_ptr,
            cuda_return_value,
            cudaGetErrorString(cuda_return_value));
        return false;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully uploaded %i "\
        "bytes of memory from host address %p to device address %p ("\
        "resource base address %p)",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        host_address,
        device_offset_ptr,
        device_base_ptr);
    return true;
}

bool ldplab::rtscuda::IDeviceResource::checkCudaDownloadError(
    cudaError_t cuda_return_value, 
    void* device_offset_ptr,
    void* device_base_ptr,
    void* host_address, 
    size_t num_bytes) const
{
    if (cuda_return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not download %i "\
            "bytes of memory from device address %p (resource base address "\
            "%p) to host address %p, cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            device_offset_ptr,
            device_base_ptr,
            host_address,
            cuda_return_value,
            cudaGetErrorString(cuda_return_value));
        return false;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully downloaded %i "\
        "bytes of memory from device address %p (resource base address %p) "\
        "to host address %p",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        device_base_ptr,
        device_offset_ptr,
        host_address);
    return true;
}

#endif