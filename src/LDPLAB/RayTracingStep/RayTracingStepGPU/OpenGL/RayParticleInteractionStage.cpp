#ifndef LDPLAB_BUILD_OPTION_DISABLE_RTSGPU_OGL

#include "RayParticleInteractionStage.hpp"

#include "Constants.hpp"
#include "Context.hpp"
#include "Data.hpp"
#include "../../../Utils/Log.hpp"
#include "../../../Utils/ComputeHelper.hpp"
#include "../../../Utils/Profiler.hpp"

#include <glm/glm.hpp>
#include <sstream>

ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::
    UnpolirzedLight1DLinearIndexGradientInteraction(
        std::shared_ptr<Context> context)
    :
    m_context{ context }
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "UnpolirzedLight1DLinearIndexGradientInteraction instance created",
        m_context->uid);
}

bool ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::
    initShaders()
{
    // Create shader
    if (!m_context->shared_shaders.createShaderByName(
            constant::glsl_shader_name::unpolarized_light_1d_linear_index_gradient_interaction,
            m_cs_interaction.shader))
        return false;
    if (!m_context->shared_shaders.createShaderByName(
            constant::glsl_shader_name::gather_output_pre_stage,
            m_cs_gather_output_pre_stage.shader))
        return false;
    if (!m_context->shared_shaders.createShaderByName(
        constant::glsl_shader_name::gather_output_post_stage,
        m_cs_gather_output_post_stage.shader))
        return false;
    if (!m_context->shared_shaders.createShaderByName(
        constant::glsl_shader_name::gather_output_reduction_stage,
        m_cs_gather_output_reduction_stage.shader))
        return false;

    // Set work group size
    m_cs_interaction.num_work_groups = utils::ComputeHelper::getNumWorkGroups(
        m_context->parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::unpolarized_light_1d_linear_index_gradient_interaction);
    m_cs_gather_output_pre_stage.num_work_groups = utils::ComputeHelper::getNumWorkGroups(
        m_context->parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::gather_output_pre_stage);
    m_cs_gather_output_post_stage.num_work_groups = utils::ComputeHelper::getNumWorkGroups(
        m_context->parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::gather_output_post_stage);

    // Update interaction shader uniforms
    m_cs_interaction.uniform_inner_particle_rays =
        m_cs_interaction.shader->getUniformLocation("inner_particle_rays");
    m_cs_interaction.uniform_num_rays_per_buffer =
        m_cs_interaction.shader->getUniformLocation("num_rays_per_buffer");
    m_cs_interaction.uniform_parameter_intensity_cutoff =
        m_cs_interaction.shader->getUniformLocation("parameter_intensity_cutoff");
    m_cs_interaction.uniform_parameter_medium_reflection_index =
        m_cs_interaction.shader->getUniformLocation("parameter_medium_reflection_index");
    
    m_cs_interaction.shader->use();
    glUniform1ui(m_cs_interaction.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));
    glUniform1d(m_cs_interaction.uniform_parameter_intensity_cutoff,
        m_context->parameters.intensity_cutoff);
    glUniform1d(m_cs_interaction.uniform_parameter_medium_reflection_index,
        m_context->parameters.medium_reflection_index);

    // Update gather output shader uniforms
    m_cs_gather_output_pre_stage.uniform_num_particles =
        m_cs_gather_output_pre_stage.shader->getUniformLocation("num_particles");
    m_cs_gather_output_pre_stage.uniform_num_rays_per_buffer =
        m_cs_gather_output_pre_stage.shader->getUniformLocation("num_rays_per_buffer");

    m_cs_gather_output_reduction_stage.uniform_num_particles =
        m_cs_gather_output_reduction_stage.shader->getUniformLocation("num_particles");
    m_cs_gather_output_reduction_stage.uniform_buffer_size =
        m_cs_gather_output_reduction_stage.shader->getUniformLocation("buffer_size");
    m_cs_gather_output_reduction_stage.uniform_source_offset =
        m_cs_gather_output_reduction_stage.shader->getUniformLocation("source_offset");

    m_cs_gather_output_post_stage.uniform_num_particles =
        m_cs_gather_output_post_stage.shader->getUniformLocation("num_particles");

    m_cs_gather_output_pre_stage.shader->use();
    glUniform1ui(m_cs_gather_output_pre_stage.uniform_num_particles,
        static_cast<GLuint>(m_context->particles.size()));
    glUniform1ui(m_cs_gather_output_pre_stage.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));
    
    m_cs_gather_output_post_stage.shader->use();
    glUniform1ui(m_cs_gather_output_post_stage.uniform_num_particles,
        static_cast<GLuint>(m_context->particles.size()));

    m_cs_gather_output_reduction_stage.shader->use();
    glUniform1ui(m_cs_gather_output_reduction_stage.uniform_num_particles,
        static_cast<GLuint>(m_context->particles.size()));

    // Finished
    return true;
}

void ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::execute(
    const IntersectionBuffer& intersection,
    const RayBuffer& rays,
    RayBuffer& reflected_rays,
    RayBuffer& refracted_rays,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute ray particle interaction "\
        "on batch buffer %i",
        m_context->uid, rays.uid);

    // Bind interaction shaders
    LDPLAB_PROFILING_START(ray_particle_interaction_shader_binding);
    m_cs_interaction.shader->use();
    LDPLAB_PROFILING_STOP(ray_particle_interaction_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(ray_particle_interaction_ssbo_binding);
    rays.ssbo.ray_properties->bindToIndex(0);
    rays.ssbo.particle_index->bindToIndex(1);
    reflected_rays.ssbo.ray_properties->bindToIndex(2);
    reflected_rays.ssbo.particle_index->bindToIndex(3);
    refracted_rays.ssbo.ray_properties->bindToIndex(4);
    refracted_rays.ssbo.particle_index->bindToIndex(5);
    intersection.ssbo.intersection_properties->bindToIndex(6);
    ParticleMaterialLinearOneDirectionalData* pmd =
        (ParticleMaterialLinearOneDirectionalData*)
        m_context->particle_material_data.get();
    pmd->ssbo.material->bindToIndex(7);
    output.ssbo.output_per_ray->bindToIndex(8);
    LDPLAB_PROFILING_STOP(ray_particle_interaction_ssbo_binding);

    // Bind uniforms
    LDPLAB_PROFILING_START(ray_particle_interaction_uniform_binding);
    if (rays.inner_particle_rays)
        glUniform1i(m_cs_interaction.uniform_inner_particle_rays, 1);
    else
        glUniform1i(m_cs_interaction.uniform_inner_particle_rays, 0);
    LDPLAB_PROFILING_STOP(ray_particle_interaction_uniform_binding);

    // Execute shader
    LDPLAB_PROFILING_START(ray_particle_interaction_shader_execution);
    m_cs_interaction.shader->execute(m_cs_interaction.num_work_groups);
    LDPLAB_PROFILING_STOP(ray_particle_interaction_shader_execution);

    // Gather output pre stage
    LDPLAB_PROFILING_START(gather_output_pre_stage);
    m_cs_gather_output_pre_stage.shader->use();
    output.ssbo.output_per_ray->bindToIndex(0);
    rays.ssbo.particle_index->bindToIndex(1);
    output.ssbo.gather_temp->bindToIndex(2);
    m_cs_gather_output_pre_stage.shader->execute(
        m_cs_gather_output_pre_stage.num_work_groups);
    LDPLAB_PROFILING_STOP(gather_output_pre_stage);

    // Gather output reduction stage
    LDPLAB_PROFILING_START(gather_output_reduction_stage);
    m_cs_gather_output_reduction_stage.shader->use();
    output.ssbo.gather_temp->bindToIndex(0);
    const size_t num_particles = m_context->particles.size();
    size_t num_threads = m_context->parameters.number_rays_per_buffer;
    GLuint buffer_size, source_offset;
    do
    {
        buffer_size = static_cast<GLuint>(num_threads * num_particles);
        num_threads = num_threads / 2 + num_threads % 2;
        source_offset = static_cast<GLuint>(num_threads * num_particles);
        glUniform1ui(m_cs_gather_output_reduction_stage.uniform_buffer_size,
            buffer_size);
        glUniform1ui(m_cs_gather_output_reduction_stage.uniform_source_offset,
            source_offset);
        m_cs_gather_output_reduction_stage.shader->execute(
            utils::ComputeHelper::getNumWorkGroups(num_threads,
                constant::glsl_local_group_size::gather_output_reduction_stage));
    } while (num_threads > 1);
    LDPLAB_PROFILING_STOP(gather_output_reduction_stage);

    // Gather output post stage
    LDPLAB_PROFILING_START(gather_output_post_stage);
    m_cs_gather_output_post_stage.shader->use();
    output.ssbo.gather_temp->bindToIndex(0);
    output.ssbo.output_gathered->bindToIndex(1);
    m_cs_gather_output_post_stage.shader->execute(
        m_cs_gather_output_post_stage.num_work_groups);
    LDPLAB_PROFILING_STOP(gather_output_post_stage);

    // Update ray buffers
    reflected_rays.inner_particle_rays = rays.inner_particle_rays;
    refracted_rays.inner_particle_rays = !rays.inner_particle_rays;

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Ray particle interaction on batch "\
        "buffer %i executed, buffer %i now holds %i reflected rays, buffer "\
        "%i now holds %i refracted rays",
        m_context->uid, 
        rays.uid, 
        reflected_rays.uid, 
        reflected_rays.active_rays, 
        refracted_rays.uid, 
        refracted_rays.active_rays);
}

#endif