#include "BufferControl.hpp"
#include "Context.hpp"
#include "../../Log.hpp"

ldplab::rtscpu::BufferControl::BufferControl(std::shared_ptr<Context> context)
    :
    m_context{ context }
{    
    const size_t num_rays = m_context->number_rays_per_buffer *
        (m_context->maximum_depth + 1) * 2;
    m_ray_data.resize(num_rays);
    m_index_data.resize(num_rays);
    m_min_bounding_sphere_distance_data.resize(num_rays);
    m_point_data.resize(m_context->number_rays_per_buffer);
    m_normal_data.resize(m_context->number_rays_per_buffer);
    m_intersected_particle_index_data.resize(m_context->number_rays_per_buffer);
    m_force_data.resize(m_context->particles.size());
    m_torque_data.resize(m_context->particles.size());
    initializeBuffers();

    LDPLAB_LOG_INFO("RTSCPU context %i: "\
        "BufferControl instance created with %i individual buffers for a "\
        "maximum of %i rays (branching depth %i)",
        m_context->uid, m_buffers.size(), num_rays, m_context->maximum_depth);
}

ldplab::rtscpu::RayBuffer& ldplab::rtscpu::BufferControl::initialBuffer()
{
    return m_buffers[0];
}

ldplab::rtscpu::RayBuffer& 
    ldplab::rtscpu::BufferControl::getReflectionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context->maximum_depth)
        return m_buffers[1]; //return m_dummy;
    return m_buffers[2 * buffer.depth + 2];
}

ldplab::rtscpu::RayBuffer& 
    ldplab::rtscpu::BufferControl::getTransmissionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context->maximum_depth)
        return m_buffers[1]; //return m_dummy;
    return m_buffers[2 * buffer.depth + 3];
}

ldplab::rtscpu::IntersectionBuffer& 
    ldplab::rtscpu::BufferControl::getIntersectionBuffer()
{
    return m_intersection_buffer;
}

ldplab::rtscpu::OutputBuffer& ldplab::rtscpu::BufferControl::getOutputBuffer()
{
    return m_output_buffer;
}

void ldplab::rtscpu::BufferControl::resetOutputBuffer()
{
    for (size_t i = 0; i < m_output_buffer.size; ++i)
    {
        m_output_buffer.force[i] = { 0.0, 0.0, 0.0 };
        m_output_buffer.torque[i] = { 0.0, 0.0, 0.0 };
    }
}

size_t ldplab::rtscpu::BufferControl::dummyBufferUID()
{
    return m_buffers[1].uid; //m_dummy.uid;
}

void ldplab::rtscpu::BufferControl::initializeBuffers()
{
    // Initial buffer
    m_buffers.emplace_back(0, m_context->number_rays_per_buffer);
    m_buffers.back().ray_data = m_ray_data.data();
    m_buffers.back().index_data = m_index_data.data();
    m_buffers.back().min_bounding_volume_distance_data =
        m_min_bounding_sphere_distance_data.data();

    // Dummy buffer
    m_buffers.emplace_back(
        m_context->maximum_depth + 1, m_context->number_rays_per_buffer);
    m_buffers.back().ray_data =
        m_ray_data.data() + m_context->number_rays_per_buffer;
    m_buffers.back().index_data =
        m_index_data.data() + m_context->number_rays_per_buffer;
    m_buffers.back().min_bounding_volume_distance_data =
        m_min_bounding_sphere_distance_data.data() +
        m_context->number_rays_per_buffer;

    // Branching buffers
    for (size_t i = 1; i <= m_context->maximum_depth; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            m_buffers.emplace_back(i, m_context->number_rays_per_buffer);
            const size_t offset =
                (2 * i + j) * m_context->number_rays_per_buffer;
            m_buffers.back().ray_data = m_ray_data.data() + offset;
            m_buffers.back().index_data = m_index_data.data() + offset;
            m_buffers.back().min_bounding_volume_distance_data
                = m_min_bounding_sphere_distance_data.data() + offset;
        }
    }

    m_intersection_buffer.size = m_context->number_rays_per_buffer;
    m_intersection_buffer.point = m_point_data.data();
    m_intersection_buffer.normal = m_normal_data.data();
    m_intersection_buffer.particle_index =
        m_intersected_particle_index_data.data();

    m_output_buffer.size = m_context->particles.size();
    m_output_buffer.force = m_force_data.data();
    m_output_buffer.torque = m_torque_data.data();
}
