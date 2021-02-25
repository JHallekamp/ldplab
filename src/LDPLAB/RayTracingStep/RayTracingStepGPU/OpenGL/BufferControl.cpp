#include "BufferControl.hpp"
#include "Context.hpp"
#include "../../../Log.hpp"

#include <mutex>

ldplab::rtsgpu_ogl::BufferControl::BufferControl(std::shared_ptr<Context> context)
    :
    m_context{ context }
{    
    const size_t num_rays = m_context->parameters.number_rays_per_buffer *
        (m_context->parameters.maximum_branching_depth * 2 + 3);
    
    m_ray_properties_data.resize(num_rays);
    m_ray_particle_index_data.resize(num_rays);
    
    m_intersection_properties_data.resize(
        m_context->parameters.number_rays_per_buffer);
    m_intersection_particle_index_data.resize(
        m_context->parameters.number_rays_per_buffer);
    
    m_output_force_data.resize(m_context->particles.size());
    m_output_torque_data.resize(m_context->particles.size());
    m_output_scattered_data.resize(
        m_context->parameters.number_rays_per_buffer);
    
    initializeBuffers();
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "BufferControl instance created with %i individual buffers for a "\
        "maximum of %i rays (branching depth %i)",
        m_context->uid, 
        m_ray_buffers.size(), 
        num_rays, 
        m_context->parameters.maximum_branching_depth);
}

ldplab::rtsgpu_ogl::RayBuffer& ldplab::rtsgpu_ogl::BufferControl::initialBuffer()
{
    return m_ray_buffers[0];
}

ldplab::rtsgpu_ogl::RayBuffer& 
    ldplab::rtsgpu_ogl::BufferControl::getReflectionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context->parameters.maximum_branching_depth)
        return m_ray_buffers[1]; //return m_dummy;
    return m_ray_buffers[2 * buffer.depth + 3];
}

ldplab::rtsgpu_ogl::RayBuffer& 
    ldplab::rtsgpu_ogl::BufferControl::getTransmissionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context->parameters.maximum_branching_depth)
        return m_ray_buffers[2]; //return m_dummy;
    return m_ray_buffers[2 * buffer.depth + 4];
}

ldplab::rtsgpu_ogl::IntersectionBuffer& 
    ldplab::rtsgpu_ogl::BufferControl::getIntersectionBuffer()
{
    return m_intersection_buffer;
}

ldplab::rtsgpu_ogl::OutputBuffer& ldplab::rtsgpu_ogl::BufferControl::getOutputBuffer()
{
    return m_output_buffer;
}

void ldplab::rtsgpu_ogl::BufferControl::resetOutputBuffer()
{
    for (size_t i = 0; i < m_output_buffer.size; ++i)
    {
        m_output_buffer.force_data[i] = { 0.0, 0.0, 0.0 };
        m_output_buffer.torque_data[i] = { 0.0, 0.0, 0.0 };
    }
}

size_t ldplab::rtsgpu_ogl::BufferControl::dummyBufferDepth()
{
    return m_ray_buffers[1].depth;
}

void ldplab::rtsgpu_ogl::BufferControl::initializeBuffers()
{
    // Initial buffer
    m_ray_buffers.emplace_back(0, m_context->parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_properties_data = m_ray_properties_data.data();
    m_ray_buffers.back().particle_index_data = m_ray_particle_index_data.data();
    // Dummy buffers 1
    m_ray_buffers.emplace_back(
        m_context->parameters.maximum_branching_depth + 1, 
        m_context->parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_properties_data =
        m_ray_properties_data.data() +
        m_context->parameters.number_rays_per_buffer;
    m_ray_buffers.back().particle_index_data =
        m_ray_particle_index_data.data() +
        m_context->parameters.number_rays_per_buffer;
    // Dummy buffer 2
    m_ray_buffers.emplace_back(
        m_context->parameters.maximum_branching_depth + 1,
        m_context->parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_properties_data =
        m_ray_properties_data.data() +
        2 * m_context->parameters.number_rays_per_buffer;
    m_ray_buffers.back().particle_index_data =
        m_ray_particle_index_data.data() +
        2 * m_context->parameters.number_rays_per_buffer;

    // Branching buffers
    for (size_t i = 0; i < m_context->parameters.maximum_branching_depth; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            m_ray_buffers.emplace_back(i + 1, 
                m_context->parameters.number_rays_per_buffer);
            const size_t offset =
                (2 * i + j + 3) * m_context->parameters.number_rays_per_buffer;
            m_ray_buffers.back().ray_properties_data = 
                m_ray_properties_data.data() + offset;
            m_ray_buffers.back().particle_index_data = 
                m_ray_particle_index_data.data() + offset;
        }
    }

    // Intersection buffer
    m_intersection_buffer.size = m_context->parameters.number_rays_per_buffer;
    m_intersection_buffer.intersection_properties_data =
        m_intersection_properties_data.data();
    m_intersection_buffer.particle_index_data =
        m_intersection_particle_index_data.data();

    // Output buffer
    m_output_buffer.size = m_context->particles.size();
    m_output_buffer.force_data = m_output_force_data.data();
    m_output_buffer.torque_data = m_output_torque_data.data();
    m_output_buffer.output_per_ray_data = m_output_scattered_data.data();

    // Create ssbos
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::lock_guard<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();

    const size_t num_rays_per_buffer = 
        m_context->parameters.number_rays_per_buffer;
    for (size_t i = 0; i < m_ray_buffers.size(); ++i)
    {
        m_ray_buffers[i].ssbo.particle_index =
            m_context->ogl->createShaderStorageBuffer(num_rays_per_buffer *
                sizeof(int32_t));
        m_ray_buffers[i].ssbo.ray_properties =
            m_context->ogl->createShaderStorageBuffer(num_rays_per_buffer *
                sizeof(RayBuffer::RayProperties));
    }

    m_intersection_buffer.ssbo.particle_index = 
        m_context->ogl->createShaderStorageBuffer(num_rays_per_buffer *
            sizeof(int32_t));
    m_intersection_buffer.ssbo.intersection_properties =
        m_context->ogl->createShaderStorageBuffer(num_rays_per_buffer * 
            sizeof(IntersectionBuffer::IntersectionProperties));

    m_output_buffer.ssbo.output_per_ray =
        m_context->ogl->createShaderStorageBuffer(num_rays_per_buffer * 
            sizeof(OutputBuffer::ScatteredOutput));

    m_context->ogl->unbindGlContext();
}
