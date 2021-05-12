#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSCUDA

#include "BufferControl.hpp"
#include "Context.hpp"
#include "../../Utils/Log.hpp"

ldplab::rtscuda::BufferControl::BufferControl(Context& context)
    :
    m_context{ context }
{    
    const size_t num_rays = m_context.parameters.number_rays_per_buffer *
        (m_context.parameters.maximum_branching_depth + 1) * 2;
    m_ray_data.resize(num_rays);
    m_index_data.resize(num_rays);
    m_min_bounding_sphere_distance_data.resize(num_rays);
    m_point_data.resize(m_context.parameters.number_rays_per_buffer);
    m_normal_data.resize(m_context.parameters.number_rays_per_buffer);
    m_intersected_particle_index_data.resize(
        m_context.parameters.number_rays_per_buffer);
    m_force_data.resize(m_context.particles.size());
    m_torque_data.resize(m_context.particles.size());
    initializeBuffers();

    LDPLAB_LOG_INFO("RTSCUDA context %i: "\
        "BufferControl instance created with %i individual buffers for a "\
        "maximum of %i rays (branching depth %i)",
        m_context.uid, 
        m_buffers.size(), 
        num_rays, 
        m_context.parameters.maximum_branching_depth);
}

ldplab::rtscuda::RayBuffer& ldplab::rtscuda::BufferControl::initialBuffer()
{
    return m_buffers[0];
}

ldplab::rtscuda::RayBuffer& 
    ldplab::rtscuda::BufferControl::getReflectionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context.parameters.maximum_branching_depth)
        return m_buffers[1]; //return m_dummy;
    return m_buffers[2 * buffer.depth + 2];
}

ldplab::rtscuda::RayBuffer& 
    ldplab::rtscuda::BufferControl::getTransmissionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context.parameters.maximum_branching_depth)
        return m_buffers[1]; //return m_dummy;
    return m_buffers[2 * buffer.depth + 3];
}

ldplab::rtscuda::IntersectionBuffer& 
    ldplab::rtscuda::BufferControl::getIntersectionBuffer()
{
    return m_intersection_buffer;
}

ldplab::rtscuda::OutputBuffer& ldplab::rtscuda::BufferControl::getOutputBuffer()
{
    return m_output_buffer;
}

void ldplab::rtscuda::BufferControl::resetOutputBuffer()
{
    for (size_t i = 0; i < m_output_buffer.size; ++i)
    {
        m_output_buffer.force[i] = { 0.0, 0.0, 0.0 };
        m_output_buffer.torque[i] = { 0.0, 0.0, 0.0 };
    }
}

size_t ldplab::rtscuda::BufferControl::dummyBufferUID()
{
    return m_buffers[1].uid; //m_dummy.uid;
}

void ldplab::rtscuda::BufferControl::initializeBuffers()
{
    // Initial buffer
    m_buffers.emplace_back(0, m_context.parameters.number_rays_per_buffer);
    m_buffers.back().ray_data = m_ray_data.data();
    m_buffers.back().index_data = m_index_data.data();
    m_buffers.back().min_bounding_volume_distance_data =
        m_min_bounding_sphere_distance_data.data();

    // Dummy buffer
    m_buffers.emplace_back(
        m_context.parameters.maximum_branching_depth + 1, 
        m_context.parameters.number_rays_per_buffer);
    m_buffers.back().ray_data =
        m_ray_data.data() + m_context.parameters.number_rays_per_buffer;
    m_buffers.back().index_data =
        m_index_data.data() + m_context.parameters.number_rays_per_buffer;
    m_buffers.back().min_bounding_volume_distance_data =
        m_min_bounding_sphere_distance_data.data() +
        m_context.parameters.number_rays_per_buffer;

    // Branching buffers
    for (size_t i = 1; i <= m_context.parameters.maximum_branching_depth; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            m_buffers.emplace_back(i, 
                m_context.parameters.number_rays_per_buffer);
            const size_t offset =
                (2 * i + j) * m_context.parameters.number_rays_per_buffer;
            m_buffers.back().ray_data = m_ray_data.data() + offset;
            m_buffers.back().index_data = m_index_data.data() + offset;
            m_buffers.back().min_bounding_volume_distance_data
                = m_min_bounding_sphere_distance_data.data() + offset;
        }
    }

    m_intersection_buffer.size = m_context.parameters.number_rays_per_buffer;
    m_intersection_buffer.point = m_point_data.data();
    m_intersection_buffer.normal = m_normal_data.data();
    m_intersection_buffer.particle_index =
        m_intersected_particle_index_data.data();

    m_output_buffer.size = m_context.particles.size();
    m_output_buffer.force = m_force_data.data();
    m_output_buffer.torque = m_torque_data.data();
}

#endif