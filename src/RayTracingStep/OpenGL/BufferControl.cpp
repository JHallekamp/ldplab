#ifdef LDPLAB_BUILD_OPTION_ENABLE_RTSOGL

#include "BufferControl.hpp"
#include "Context.hpp"
#include "../../Utils/Log.hpp"

#include <mutex>

ldplab::rtsogl::BufferControl::BufferControl(Context& context)
    :
    m_context{ context }
{    
    const size_t num_rays = m_context.parameters.number_rays_per_buffer *
        (m_context.parameters.maximum_branching_depth * 2 + 3);
    
    m_ray_properties_data.resize(num_rays);
    m_ray_particle_index_data.resize(num_rays);
    
    m_intersection_properties_data.resize(
        m_context.parameters.number_rays_per_buffer);
    m_intersection_particle_index_data.resize(
        m_context.parameters.number_rays_per_buffer);
    
    m_output_scattered_data.resize(
        m_context.parameters.number_rays_per_buffer);
    m_output_gathered_data.resize(
        m_context.particles.size());
    
    initializeBuffers();
    LDPLAB_LOG_INFO("RTSOGL context %i: "\
        "BufferControl instance created with %i individual buffers for a "\
        "maximum of %i rays (branching depth %i)",
        m_context.uid, 
        m_ray_buffers.size(), 
        num_rays, 
        m_context.parameters.maximum_branching_depth);
}

void ldplab::rtsogl::BufferControl::initSSBO()
{
    // Create ssbos
    const size_t num_rays_per_buffer =
        m_context.parameters.number_rays_per_buffer;
    for (size_t i = 0; i < m_ray_buffers.size(); ++i)
    {
        m_ray_buffers[i].ssbo.particle_index =
            m_context.ogl->createShaderStorageBuffer(num_rays_per_buffer *
                sizeof(int32_t));
        m_ray_buffers[i].ssbo.ray_properties =
            m_context.ogl->createShaderStorageBuffer(num_rays_per_buffer *
                sizeof(RayBuffer::RayProperties));
    }

    m_intersection_buffer.ssbo.particle_index =
        m_context.ogl->createShaderStorageBuffer(num_rays_per_buffer *
            sizeof(int32_t));
    m_intersection_buffer.ssbo.intersection_properties =
        m_context.ogl->createShaderStorageBuffer(num_rays_per_buffer *
            sizeof(IntersectionBuffer::IntersectionProperties));

    m_output_buffer.ssbo.output_per_ray =
        m_context.ogl->createShaderStorageBuffer(num_rays_per_buffer *
            sizeof(OutputBuffer::OutputData));
    m_output_buffer.ssbo.output_gathered =
        m_context.ogl->createShaderStorageBuffer(m_context.particles.size() *
            sizeof(OutputBuffer::OutputData));
    m_output_buffer.ssbo.gather_temp =
        m_context.ogl->createShaderStorageBuffer(num_rays_per_buffer *
            sizeof(OutputBuffer::OutputData));
}

ldplab::rtsogl::RayBuffer& ldplab::rtsogl::BufferControl::initialBuffer()
{
    return m_ray_buffers[0];
}

ldplab::rtsogl::RayBuffer& 
    ldplab::rtsogl::BufferControl::getReflectionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context.parameters.maximum_branching_depth)
        return m_ray_buffers[1]; //return m_dummy;
    return m_ray_buffers[2 * buffer.depth + 3];
}

ldplab::rtsogl::RayBuffer& 
    ldplab::rtsogl::BufferControl::getTransmissionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context.parameters.maximum_branching_depth)
        return m_ray_buffers[2]; //return m_dummy;
    return m_ray_buffers[2 * buffer.depth + 4];
}

ldplab::rtsogl::IntersectionBuffer& 
    ldplab::rtsogl::BufferControl::getIntersectionBuffer()
{
    return m_intersection_buffer;
}

ldplab::rtsogl::OutputBuffer& ldplab::rtsogl::BufferControl::getOutputBuffer()
{
    return m_output_buffer;
}

void ldplab::rtsogl::BufferControl::resetOutputBuffer()
{
    for (size_t i = 0; i < m_output_buffer.size; ++i)
    {
        m_output_buffer.output_gathered_data[i].force = Vec3{ 0, 0, 0 };
        m_output_buffer.output_gathered_data[i].torque = Vec3{ 0, 0, 0 };
    }

    std::unique_lock<std::mutex> gpu_lck{ m_context.ogl->getGPUMutex() };
    m_context.ogl->bindGlContext();
    m_output_buffer.ssbo.output_gathered->upload(
        m_output_buffer.output_gathered_data);
    m_context.ogl->unbindGlContext();
}

size_t ldplab::rtsogl::BufferControl::dummyBufferDepth()
{
    return m_ray_buffers[1].depth;
}

void ldplab::rtsogl::BufferControl::initializeBuffers()
{
    // Initial buffer
    m_ray_buffers.emplace_back(0, m_context.parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_properties_data = m_ray_properties_data.data();
    m_ray_buffers.back().particle_index_data = m_ray_particle_index_data.data();
    // Dummy buffers 1
    m_ray_buffers.emplace_back(
        m_context.parameters.maximum_branching_depth + 1, 
        m_context.parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_properties_data =
        m_ray_properties_data.data() +
        m_context.parameters.number_rays_per_buffer;
    m_ray_buffers.back().particle_index_data =
        m_ray_particle_index_data.data() +
        m_context.parameters.number_rays_per_buffer;
    // Dummy buffer 2
    m_ray_buffers.emplace_back(
        m_context.parameters.maximum_branching_depth + 1,
        m_context.parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_properties_data =
        m_ray_properties_data.data() +
        2 * m_context.parameters.number_rays_per_buffer;
    m_ray_buffers.back().particle_index_data =
        m_ray_particle_index_data.data() +
        2 * m_context.parameters.number_rays_per_buffer;

    // Branching buffers
    for (size_t i = 0; i < m_context.parameters.maximum_branching_depth; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            m_ray_buffers.emplace_back(i + 1, 
                m_context.parameters.number_rays_per_buffer);
            const size_t offset =
                (2 * i + j + 3) * m_context.parameters.number_rays_per_buffer;
            m_ray_buffers.back().ray_properties_data = 
                m_ray_properties_data.data() + offset;
            m_ray_buffers.back().particle_index_data = 
                m_ray_particle_index_data.data() + offset;
        }
    }

    // Intersection buffer
    m_intersection_buffer.size = m_context.parameters.number_rays_per_buffer;
    m_intersection_buffer.intersection_properties_data =
        m_intersection_properties_data.data();
    m_intersection_buffer.particle_index_data =
        m_intersection_particle_index_data.data();

    // Output buffer
    m_output_buffer.size = m_context.particles.size();
    m_output_buffer.output_per_ray_data = m_output_scattered_data.data();
    m_output_buffer.output_gathered_data = m_output_gathered_data.data();
}

#endif