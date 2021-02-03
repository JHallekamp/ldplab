#include "BufferControl.hpp"
#include "Context.hpp"
#include "../../../Log.hpp"

ldplab::rtsgpu_ogl::BufferControl::BufferControl(std::shared_ptr<Context> context)
    :
    m_context{ context }
{    
    const size_t num_rays = m_context->parameters.number_rays_per_buffer *
        (m_context->parameters.maximum_branching_depth + 1) * 2;
    
    m_ray_origin_data.resize(num_rays);
    m_ray_direction_data.resize(num_rays);
    m_ray_intensity_data.resize(num_rays);
    m_ray_index_data.resize(num_rays);
    m_ray_min_bounding_sphere_distance_data.resize(num_rays);
    
    m_intersection_point_data.resize(
        m_context->parameters.number_rays_per_buffer);
    m_intersection_normal_data.resize(
        m_context->parameters.number_rays_per_buffer);
    m_intersection_particle_index_data.resize(
        m_context->parameters.number_rays_per_buffer);
    
    m_output_force_data.resize(m_context->particles.size());
    m_output_torque_data.resize(m_context->particles.size());
    
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
    return m_ray_buffers[2 * buffer.depth + 2];
}

ldplab::rtsgpu_ogl::RayBuffer& 
    ldplab::rtsgpu_ogl::BufferControl::getTransmissionBuffer(RayBuffer& buffer)
{
    if (buffer.depth >= m_context->parameters.maximum_branching_depth)
        return m_ray_buffers[1]; //return m_dummy;
    return m_ray_buffers[2 * buffer.depth + 3];
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
        m_output_buffer.force_data[i] = { 0.0, 0.0, 0.0, 0.0 };
        m_output_buffer.torque_data[i] = { 0.0, 0.0, 0.0, 0.0 };
    }
}

size_t ldplab::rtsgpu_ogl::BufferControl::dummyBufferUID()
{
    return m_ray_buffers[1].uid; //m_dummy.uid;
}

void ldplab::rtsgpu_ogl::BufferControl::initializeBuffers()
{
    // Initial buffer
    m_ray_buffers.emplace_back(0, m_context->parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_origin_data = m_ray_origin_data.data();
    m_ray_buffers.back().ray_direction_data = m_ray_direction_data.data();
    m_ray_buffers.back().ray_intensity_data = m_ray_intensity_data.data();
    m_ray_buffers.back().index_data = m_ray_index_data.data();
    m_ray_buffers.back().min_bounding_volume_distance_data =
        m_ray_min_bounding_sphere_distance_data.data();
    // Dummy buffer
    m_ray_buffers.emplace_back(
        m_context->parameters.maximum_branching_depth + 1, 
        m_context->parameters.number_rays_per_buffer);
    m_ray_buffers.back().ray_origin_data = m_ray_origin_data.data() + 
        m_context->parameters.number_rays_per_buffer;
    m_ray_buffers.back().ray_direction_data = m_ray_direction_data.data() +
        m_context->parameters.number_rays_per_buffer;
    m_ray_buffers.back().ray_intensity_data = m_ray_intensity_data.data() +
        m_context->parameters.number_rays_per_buffer;
    m_ray_buffers.back().index_data = m_ray_index_data.data() + 
        m_context->parameters.number_rays_per_buffer;
    m_ray_buffers.back().min_bounding_volume_distance_data =
        m_ray_min_bounding_sphere_distance_data.data() +
        m_context->parameters.number_rays_per_buffer;

    // Branching buffers
    for (size_t i = 1; i <= m_context->parameters.maximum_branching_depth; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            m_ray_buffers.emplace_back(i, 
                m_context->parameters.number_rays_per_buffer);
            const size_t offset =
                (2 * i + j) * m_context->parameters.number_rays_per_buffer;
            m_ray_buffers.back().ray_origin_data = m_ray_origin_data.data() +
                offset;
            m_ray_buffers.back().ray_direction_data = m_ray_direction_data.data() +
                offset;
            m_ray_buffers.back().ray_intensity_data = m_ray_intensity_data.data() +
                offset;
            m_ray_buffers.back().index_data = m_ray_index_data.data() +
                offset;
            m_ray_buffers.back().min_bounding_volume_distance_data =
                m_ray_min_bounding_sphere_distance_data.data() +
                offset;
        }
    }

    // Intersection buffer
    m_intersection_buffer.size = m_context->parameters.number_rays_per_buffer;
    m_intersection_buffer.point_data = m_intersection_point_data.data();
    m_intersection_buffer.normal_data = m_intersection_normal_data.data();
    m_intersection_buffer.particle_index_data =
        m_intersection_particle_index_data.data();

    // Output buffer
    m_output_buffer.size = m_context->particles.size();
    m_output_buffer.force_data = m_output_force_data.data();
    m_output_buffer.torque_data = m_output_torque_data.data();

    // Create ssbos
    for (size_t i = 0; i < m_ray_buffers.size(); ++i)
    {
        m_ray_buffers[i].ray_origin_ssbo = 
            m_context->ogl->createShaderStorageBuffer(
                m_context->parameters.number_rays_per_buffer * sizeof(Vec3));
        m_ray_buffers[i].ray_direction_ssbo =
            m_context->ogl->createShaderStorageBuffer(
                m_context->parameters.number_rays_per_buffer * sizeof(Vec3));
        m_ray_buffers[i].ray_intensity_ssbo =
            m_context->ogl->createShaderStorageBuffer(
                m_context->parameters.number_rays_per_buffer * sizeof(double));
        m_ray_buffers[i].index_ssbo =
            m_context->ogl->createShaderStorageBuffer(
                m_context->parameters.number_rays_per_buffer * sizeof(int32_t));
        m_ray_buffers[i].min_bounding_volume_distance_ssbo =
            m_context->ogl->createShaderStorageBuffer(
                m_context->parameters.number_rays_per_buffer * sizeof(double));
    }

    m_intersection_buffer.point_ssbo = 
        m_context->ogl->createShaderStorageBuffer(
            m_context->parameters.number_rays_per_buffer * sizeof(Vec3));
    m_intersection_buffer.normal_ssbo =
        m_context->ogl->createShaderStorageBuffer(
            m_context->parameters.number_rays_per_buffer * sizeof(Vec3));
    m_intersection_buffer.particle_index_ssbo =
        m_context->ogl->createShaderStorageBuffer(
            m_context->parameters.number_rays_per_buffer * sizeof(int32_t));

    m_output_buffer.force_per_ray_ssbo =
        m_context->ogl->createShaderStorageBuffer(
            m_context->parameters.number_rays_per_buffer * sizeof(Vec3));
    m_output_buffer.torque_per_ray_ssbo =
        m_context->ogl->createShaderStorageBuffer(
            m_context->parameters.number_rays_per_buffer * sizeof(Vec3));
}
