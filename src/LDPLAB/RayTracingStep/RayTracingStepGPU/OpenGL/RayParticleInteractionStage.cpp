#include "RayParticleInteractionStage.hpp"

#include "Context.hpp"
#include "Data.hpp"

#include "../../../Log.hpp"
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

bool ldplab::rtsgpu_ogl::UnpolirzedLight1DLinearIndexGradientInteraction::initShaders(
    const RayTracingStepGPUOpenGLInfo& info)
{
    std::string shader_path = info.shader_base_directory_path +
        "glsl/ldplab_cs_unpolarized_light_1d_linear_index_gradient_interaction.glsl";

    // Load file
    std::ifstream file(shader_path);
    if (!file)
    {
        LDPLAB_LOG_ERROR("RTSGPU (OpenGL) context %i: Unable to open shader "\
            "source file \"%s\"", m_context->uid, shader_path.c_str());
        return false;
    }
    std::stringstream code;
    code << file.rdbuf();

    // Create shader
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();

    m_compute_shader =
        m_context->ogl->createComputeShader(
            "UnpolirzedLight1DLinearIndexGradientInteraction", code.str());
    file.close();

    // Set uniforms
    if (m_compute_shader != nullptr)
    {
        m_shader_uniform_location_num_rays_per_buffer =
            m_compute_shader->getUniformLocation("num_rays_per_buffer");
        m_shader_uniform_location_num_particles =
            m_compute_shader->getUniformLocation("num_particles");
        m_shader_uniform_location_parameter_medium_reflection_index =
            m_compute_shader->getUniformLocation("parameter_medium_reflection_index");
        m_shader_uniform_location_parameter_intensity_cutoff =
            m_compute_shader->getUniformLocation("parameter_intensity_cutoff");
        m_shader_uniform_location_inner_particle_rays =
            m_compute_shader->getUniformLocation("inner_particle_rays");

        m_compute_shader->use();
        glUniform1ui(m_shader_uniform_location_num_rays_per_buffer,
            m_context->parameters.number_rays_per_buffer);
        glUniform1ui(m_shader_uniform_location_num_particles,
            m_context->particles.size());
        glUniform1d(m_shader_uniform_location_parameter_medium_reflection_index,
            m_context->parameters.medium_reflection_index);
        glUniform1d(m_shader_uniform_location_parameter_intensity_cutoff,
            m_context->parameters.intensity_cutoff);
    }

    m_context->ogl->unbindGlContext();
    return (m_compute_shader != nullptr);
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

    // Bind GPU to this thread
    LDPLAB_PROFILING_START(ray_particle_interaction_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(ray_particle_interaction_gl_context_binding);

    // Bind shaders
    LDPLAB_PROFILING_START(ray_particle_interaction_shader_binding);
    m_compute_shader->use();
    LDPLAB_PROFILING_STOP(ray_particle_interaction_shader_binding);

    // Bind SSBOs
    LDPLAB_PROFILING_START(ray_particle_interaction_ssbo_binding);
    // TODO
    LDPLAB_PROFILING_STOP(ray_particle_interaction_ssbo_binding);

    // Bind uniforms
    LDPLAB_PROFILING_START(ray_particle_interaction_uniform_binding);
    if (rays.inner_particle_rays)
        glUniform1i(m_shader_uniform_location_inner_particle_rays, 1);
    else
        glUniform1i(m_shader_uniform_location_inner_particle_rays, 0);
    LDPLAB_PROFILING_STOP(ray_particle_interaction_uniform_binding);

    // Execute shader
    LDPLAB_PROFILING_START(ray_particle_interaction_shader_execution);
    m_compute_shader->execute(rays.size / 64);
    LDPLAB_PROFILING_STOP(ray_particle_interaction_shader_execution);

    // Download data
    LDPLAB_PROFILING_START(ray_particle_interaction_data_download);
    reflected_rays.ssbo.particle_index->download(
        reflected_rays.particle_index_data);
    refracted_rays.ssbo.particle_index->download(
        reflected_rays.particle_index_data);
    output.ssbo.output_per_ray->download(output.output_per_ray_data);
    LDPLAB_PROFILING_STOP(ray_particle_interaction_data_download);

    // Unbind gl context
    LDPLAB_PROFILING_START(ray_particle_interaction_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(ray_particle_interaction_gl_context_unbinding);

    // Get number of active rays for reflected and refracted rays
    reflected_rays.inner_particle_rays = rays.inner_particle_rays;
    refracted_rays.inner_particle_rays = !rays.inner_particle_rays;
    reflected_rays.active_rays = 0;
    refracted_rays.active_rays = 0;
    for (size_t i = 0; i < rays.size; ++i)
    {
        if (reflected_rays.particle_index_data[i] >= 0)
            ++reflected_rays.active_rays;
        if (refracted_rays.particle_index_data[i] >= 0)
            ++refracted_rays.active_rays;
    }

    // Gather output
    for (size_t i = 0; i < rays.size; ++i)
    {
        output.force_data[rays.particle_index_data[i]] += 
            output.output_per_ray_data[i].force;
        output.torque_data[rays.particle_index_data[i]] += 
            output.output_per_ray_data[i].torque;
    }

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