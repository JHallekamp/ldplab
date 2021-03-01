#include "RayParticleInteractionStage.hpp"

#include "Constants.hpp"
#include "Context.hpp"
#include "Data.hpp"
#include "../../../Log.hpp"
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
            constant::glsl_shader_name::gather_output,
            m_cs_gather_output.shader))
        return false;

    // Set work group size
    m_cs_interaction.num_work_groups = ComputeHelper::getNumWorkGroups(
        m_context->parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::unpolarized_light_1d_linear_index_gradient_interaction);
    m_cs_gather_output.num_work_groups = ComputeHelper::getNumWorkGroups(
        m_context->parameters.number_rays_per_buffer,
        constant::glsl_local_group_size::gather_output);

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
    m_cs_gather_output.uniform_num_particles =
        m_cs_gather_output.shader->getUniformLocation("num_particles");
    m_cs_gather_output.uniform_num_rays_per_buffer =
        m_cs_gather_output.shader->getUniformLocation("num_rays_per_buffer");

    m_cs_gather_output.shader->use();
    glUniform1ui(m_cs_gather_output.uniform_num_particles,
        static_cast<GLuint>(m_context->particles.size()));
    glUniform1ui(m_cs_gather_output.uniform_num_rays_per_buffer,
        static_cast<GLuint>(m_context->parameters.number_rays_per_buffer));
    
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

    // Bind gather shader
    LDPLAB_PROFILING_START(scattered_output_gather_shader_binding);
    m_cs_gather_output.shader->use();
    LDPLAB_PROFILING_STOP(scattered_output_gather_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(scattered_output_gather_shader_ssbo_binding);
    output.ssbo.output_per_ray->bindToIndex(0);
    rays.ssbo.particle_index->bindToIndex(1);
    output.ssbo.gather_temp->bindToIndex(2);
    output.ssbo.output_gathered->bindToIndex(3);
    LDPLAB_PROFILING_STOP(scattered_output_gather_shader_ssbo_binding);

    // Execute shader
    LDPLAB_PROFILING_START(scattered_output_gather_shader_execution);
    m_cs_gather_output.shader->execute(m_cs_gather_output.num_work_groups);
    LDPLAB_PROFILING_STOP(scattered_output_gather_shader_execution);

    // Update ray buffers
    m_context->shared_shaders.updateRayBufferState(reflected_rays);
    m_context->shared_shaders.updateRayBufferState(refracted_rays);

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