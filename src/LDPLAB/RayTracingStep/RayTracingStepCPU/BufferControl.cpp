#include "BufferControl.hpp"
#include "Context.hpp"
#include "../../Log.hpp"

size_t exp2i(size_t x)
{
    // Calculate 2 to the power of x: 2^x
    return (static_cast<size_t>(1) << x);
}

ldplab::rtscpu::BufferControl::BufferControl(std::shared_ptr<Context> context)
    :
    m_context{ context },
    m_dummy{ 
        exp2i(context->maximum_depth + 1), 
        context->maximum_depth + 1, 
        context->number_rays_per_buffer }
{    
    const size_t num_rays = m_context->number_rays_per_buffer * 
        (exp2i(m_context->maximum_depth + 1));
    m_ray_data.resize(num_rays);
    m_index_data.resize(num_rays);
    m_point_data.resize(context->number_rays_per_buffer);
    m_normal_data.resize(context->number_rays_per_buffer);
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
        return m_dummy;

    //const size_t layer_buffer_index = buffer.index - (exp2i(buffer.depth) - 1);
    //const size_t next_layer_offset = exp2i(buffer.depth + 1) - 1;
    //return m_buffers[next_layer_offset + 2 * layer_buffer_index];
    
    return m_buffers[2 * buffer.index + 1];
}

ldplab::rtscpu::RayBuffer& 
    ldplab::rtscpu::BufferControl::getTransmissionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context->maximum_depth)
        return m_dummy;

    //const size_t layer_buffer_index = buffer.index - (exp2i(buffer.depth) - 1);
    //const size_t next_layer_offset = exp2i(buffer.depth + 1) - 1;
    //return m_buffers[next_layer_offset + 2 * layer_buffer_index + 1];

    return m_buffers[2 * buffer.index + 2];
}

ldplab::rtscpu::IntersectionBuffer& 
    ldplab::rtscpu::BufferControl::getIntersectionBuffer()
{
    return m_intersection_buffer;
}

size_t ldplab::rtscpu::BufferControl::dummyBufferIndex()
{
    return m_dummy.index;
}

void ldplab::rtscpu::BufferControl::initializeBuffers()
{
    for (size_t i = 0; i <= m_context->maximum_depth; ++i)
    {
        const size_t layer_buffer_count = exp2i(i);
        for (size_t j = 0; j < layer_buffer_count; ++i)
        {
            const size_t index = layer_buffer_count - 1 + j;
            const size_t depth = i;
            m_buffers.push_back(
                RayBuffer(index, depth, m_context->number_rays_per_buffer));

            const size_t offset = index * m_context->number_rays_per_buffer;
            m_buffers.back().ray_data = m_ray_data.data() + offset;
            m_buffers.back().index_data = m_index_data.data() + offset;
        }
    }

    const size_t offset = m_dummy.index * m_context->number_rays_per_buffer;
    m_dummy.ray_data = m_ray_data.data() + offset;
    m_dummy.index_data = m_index_data.data() + offset;

    m_intersection_buffer.size = m_context->number_rays_per_buffer;
    m_intersection_buffer.point = m_point_data.data();
    m_intersection_buffer.normal = m_normal_data.data();
}
