#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA
#include "CudaResource.hpp"

#include "../../Utils/Log.hpp"

ldplab::rtscuda::ICudaResource::ICudaResource(ICudaResource&& other) noexcept
    :
    m_resource_uid{ other.m_resource_uid },
    m_device_ptr{ other.m_device_ptr }
{
    other.m_device_ptr = 0;
}

bool ldplab::rtscuda::ICudaResource::hasAllocationError(
    cudaError_t return_value, 
    size_t num_bytes) const
{
    if (return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not allocate %i "\
            "bytes of device memory, cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            return_value,
            cudaGetErrorString(return_value));
        return true;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully allocated %i "\
        "bytes of device memory at device address %p",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        m_device_ptr);
    return false;
}

bool ldplab::rtscuda::ICudaResource::hasFreeError(
    cudaError_t return_value, 
    size_t num_bytes) const
{
    if (return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not free %i bytes "\
            "of device memory at device address %p, cuda returned error code "\
            "%i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            m_device_ptr,
            return_value,
            cudaGetErrorString(return_value));
        return true;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully freed %i bytes "\
        "of device memory at device address %p",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        m_device_ptr);
    return false;
}

bool ldplab::rtscuda::ICudaResource::hasMemsetError(
    cudaError_t return_value, 
    void* device_offset_address, 
    size_t num_bytes, 
    int value) const
{
    if (return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not set %i bytes "\
            "of device memory at device address %p (resource base address "\
            "%p) to value %i, cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            device_offset_address,
            m_device_ptr,
            value,
            return_value,
            cudaGetErrorString(return_value));
        return true;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully set %i bytes "\
        "of device memory at device address %p (resource base address %p) "\
        "to value %i",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        device_offset_address,
        m_device_ptr,
        value);
    return false;
}

bool ldplab::rtscuda::ICudaResource::hasUploadError(
    cudaError_t return_value, 
    void* device_offset_address, 
    void* host_address, 
    size_t num_bytes) const
{
    if (return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not upload %i "\
            "bytes of memory from host address %p to device address %p ("\
            "resource base address %p), cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            host_address,
            device_offset_address,
            m_device_ptr,
            return_value,
            cudaGetErrorString(return_value));
        return true;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully uploaded %i "\
        "bytes of memory from host address %p to device address %p ("\
        "resource base address %p)",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        host_address,
        device_offset_address,
        m_device_ptr);
    return false;
}

bool ldplab::rtscuda::ICudaResource::hasDownloadError(
    cudaError_t return_value, 
    void* device_offset_address, 
    void* host_address, 
    size_t num_bytes) const
{
    if (return_value != cudaSuccess)
    {
        LDPLAB_LOG_ERROR("RTSCUDA resource %i (%s): Could not download %i "\
            "bytes of memory from device address %p (resource base address "\
            "%p) to host address %p, cuda returned error code %i: %s",
            m_resource_uid,
            resourceTypeName(),
            num_bytes,
            device_offset_address,
            m_device_ptr,
            host_address,
            return_value,
            cudaGetErrorString(return_value));
        return true;
    }
    LDPLAB_LOG_TRACE("RTSCUDA resource %i (%s): Successfully downloaded %i "\
        "bytes of memory from device address %p (resource base address %p) "\
        "to host address %p",
        m_resource_uid,
        resourceTypeName(),
        num_bytes,
        m_device_ptr,
        device_offset_address,
        host_address);
    return false;
}

#endif