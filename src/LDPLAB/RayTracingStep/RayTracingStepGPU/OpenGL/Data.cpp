#include "Data.hpp"
#include "Context.hpp"
#include "../../../Log.hpp"

void ldplab::rtsgpu_ogl::RodParticleData::uploadSSBO()
{
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Begin uploading rod "\
        "particle data to SSBO", m_context->uid);
    const size_t num_particles = particle_data.size();

    std::vector<double> temp_double_vec;
    temp_double_vec.resize(num_particles);

    // Upload cylinder length
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_double_vec[i] = particle_data[i].
            cylinder_length;
    }
    ssbo.cylinder_length->upload(temp_double_vec.data());

    // Upload cylinder radius
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_double_vec[i] = particle_data[i].
            cylinder_radius;
    }
    ssbo.cylinder_radius->upload(temp_double_vec.data());

    // Upload sphere radius
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_double_vec[i] = particle_data[i].
            sphere_radius;
    }
    ssbo.sphere_radius->upload(temp_double_vec.data());
    temp_double_vec.clear();

    // Upload origin cap
    std::vector<Vec4> temp_vec4_vec;
    temp_vec4_vec.resize(num_particles);
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_vec4_vec[i] = particle_data[i].
            origin_cap;
    }
    ssbo.origin_cap->upload(temp_vec4_vec.data());

    // Upload origin indentation
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_vec4_vec[i] = particle_data[i].
            origin_indentation;
    }
    ssbo.origin_indentation->upload(temp_vec4_vec.data());

    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Finished uploading rod "\
        "particle data to SSBO", m_context->uid);
}

void ldplab::rtsgpu_ogl::ParticleMaterialLinearOneDirectionalData::uploadSSBO()
{
    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Begin uploading particle "\
        "material data to SSBO", m_context->uid);
    const size_t num_particles = material_data.size();

    // Upload direction times gradient
    std::vector<Vec4> temp_direction_times_gradient;
    temp_direction_times_gradient.resize(num_particles);
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_direction_times_gradient[i] = Vec4(
            material_data[i].direction_times_gradient, 0);
    }
    ssbo.direction_times_gradient->upload(temp_direction_times_gradient.data());
    temp_direction_times_gradient.clear();

    // Upload sum term
    std::vector<double> temp_sum_term;
    temp_sum_term.resize(num_particles);
    for (size_t i = 0; i < num_particles; ++i)
    {
        temp_sum_term[i] = material_data[i].
            index_of_refraction_minus_partial_dot;
    }
    ssbo.index_of_refraction_sum_term->upload(temp_sum_term.data());

    LDPLAB_LOG_DEBUG("RTSGPU (OpenGL) context %i: Finished uploading particle "\
        "material data to SSBO", m_context->uid);
}
