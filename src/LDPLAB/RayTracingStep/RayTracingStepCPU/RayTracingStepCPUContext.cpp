#include "RayTracingStepCPUContext.hpp"

void ldplab::RayTracingStepCPUContext::initialRayBufferReset(
    size_t num_rays)
{
    std::unique_lock<std::mutex> lck(m_ray_buffer_queue_mutex);
    if (m_initial_rays.size() < num_rays)
    {
        m_initial_rays.clear();
        m_initial_rays.resize(num_rays);
    }
}

ldplab::RayBuffer ldplab::RayTracingStepCPUContext::initialRayBufferGetBuffer()
{
    std::unique_lock<std::mutex> lck(m_ray_buffer_queue_mutex);
    
    if (m_initial_rays_offset + 1 > m_initial_rays_count)
        return RayBuffer{ nullptr, 0 };
    
    RayBuffer buffer;
    buffer.data = m_initial_rays.data() + m_initial_rays_offset;
    if (m_initial_rays_offset + m_ray_buffer_size + 1 >= m_initial_rays_count)
        buffer.num_rays = m_initial_rays_count - m_initial_rays_offset;
    else
        buffer.num_rays = m_ray_buffer_size;
    
    m_initial_rays_offset += m_ray_buffer_size;
    return buffer;
}
