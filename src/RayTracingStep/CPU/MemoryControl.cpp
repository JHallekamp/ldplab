#include "MemoryControl.hpp"

bool ldplab::rtscpu::MemoryControl::allocateBuffers(
    const RayTracingStepCPUInfo& info,
    const ExperimentalSetup& setup,
    StageDependentData&& stage_dependent_data)
{
    // Move stage dependent data
    m_stage_dependent_data = std::move(stage_dependent_data);

    // Initialize ray buffers
    const size_t num_ray_buffers = info.maximum_branching_depth + 2;
    const size_t num_rays = num_ray_buffers * info.number_rays_per_buffer;
    m_ray_data.resize(num_rays);
    m_ray_index_data.resize(num_rays);
    m_ray_min_bounding_sphere_distance_data.resize(num_rays);
    for (size_t i = 0; i < num_ray_buffers; ++i)
    {
        m_ray_buffers.emplace_back(i, i, info.number_rays_per_buffer);
        const size_t offset = i * info.number_rays_per_buffer;
        m_ray_buffers.back().ray_data = &m_ray_data[offset];
        m_ray_buffers.back().index_data = &m_ray_index_data[offset];
        m_ray_buffers.back().min_bounding_volume_distance_data = 
            &m_ray_min_bounding_sphere_distance_data[offset];
    }

    // Intialize intersection buffers
    const size_t num_isec_buffers = info.maximum_branching_depth + 1;
    const size_t num_isec_data = num_isec_buffers * info.number_rays_per_buffer;
    m_intersection_point_data.resize(num_isec_data);
    m_intersection_normal_data.resize(num_isec_data);
    m_intersection_particle_index_data.resize(num_isec_data);
    for (size_t i = 0; i < num_isec_buffers; ++i)
    {
        m_intersection_buffers.emplace_back(i, info.number_rays_per_buffer);
        const size_t offset = i * info.number_rays_per_buffer;
        m_intersection_buffers.back().point = 
            &m_intersection_point_data[offset];
        m_intersection_buffers.back().normal =
            &m_intersection_normal_data[offset];
        m_intersection_buffers.back().particle_index =
            &m_intersection_particle_index_data[offset];
    }

    // Initialize the output buffer
    const size_t num_particles = setup.particles.size();
    m_output_force_data.resize(num_particles);
    m_output_torque_data.resize(num_particles);
    m_output_buffer.size = num_particles;
    m_output_buffer.force = m_output_force_data.data();
    m_output_buffer.torque = m_output_torque_data.data();

    return true;
}

void ldplab::rtscpu::MemoryControl::resetOutputBuffer()
{
    for (size_t i = 0; i < m_output_buffer.size; ++i)
    {
        m_output_buffer.force[i] = { 0, 0, 0 };
        m_output_buffer.torque[i] = { 0, 0, 0 };
    }
}
