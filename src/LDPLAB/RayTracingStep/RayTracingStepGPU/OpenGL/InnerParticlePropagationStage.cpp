#include "InnerParticlePropagationStage.hpp"

#include "Data.hpp"
#include "Context.hpp"
#include "../../../ExperimentalSetup/Particle.hpp"
#include "../../../ExperimentalSetup/ParticleMaterial.hpp"
#include "../../../Log.hpp"
#include "../../../Utils/Profiler.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::
    LinearIndexGradientRodParticlePropagation(
        std::shared_ptr<Context> context,
        RK45 parameters)
    :
    m_context{ context },
    m_parameters{parameters}
{
    LDPLAB_LOG_INFO("RTSGPU (OpenGL) context %i: "\
        "LinearIndexGradientRodParticlePropagation instance created",
        m_context->uid);
}

bool ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::initShaders(
    const ldplab::RayTracingStepGPUOpenGLInfo& info)
{
    std::string shader_path = info.shader_base_directory_path + 
        "glsl/ldplab_cs_linear_index_gradient_rod_particle_propagation.glsl";

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
            "LinearIndexGradientRodParticlePropagation", code.str());
    file.close();

    // Get uniform locations
    if (m_compute_shader != nullptr)
    {
        m_shader_uniform_location_num_rays_per_buffer =
            m_compute_shader->getUniformLocation("num_rays_per_buffer");
        m_shader_uniform_location_parameter_epsilon =
            m_compute_shader->getUniformLocation("parameter_epsilon");
        m_shader_uniform_location_parameter_initial_step_size =
            m_compute_shader->getUniformLocation("parameter_initial_step_size");
        m_shader_uniform_location_parameter_safety_factor =
            m_compute_shader->getUniformLocation("parameter_safety_factor");

        // Set uniforms
        m_compute_shader->use();
        glUniform1ui(m_shader_uniform_location_num_rays_per_buffer,
            m_context->parameters.number_rays_per_buffer);
        glUniform1d(m_shader_uniform_location_parameter_epsilon,
            m_parameters.epsilon);
        glUniform1d(m_shader_uniform_location_parameter_initial_step_size,
            m_parameters.initial_step_size);
        glUniform1d(m_shader_uniform_location_parameter_safety_factor,
            m_parameters.safety_factor);
    }

    // Finish it
    m_context->ogl->unbindGlContext();
    return (m_compute_shader != nullptr);
}

void ldplab::rtsgpu_ogl::LinearIndexGradientRodParticlePropagation::execute(
    RayBuffer& rays,
    IntersectionBuffer& intersection,
    OutputBuffer& output)
{
    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Execute inner particle ray "\
        "propagation on batch buffer %i",
        m_context->uid, rays.uid);
    
    // Bind GPU to this thread
    LDPLAB_PROFILING_START(inner_particle_propagation_gl_context_binding);
    std::mutex& gpu_mutex = m_context->ogl->getGPUMutex();
    std::unique_lock<std::mutex> gpu_lock{ gpu_mutex };
    m_context->ogl->bindGlContext();
    LDPLAB_PROFILING_STOP(inner_particle_propagation_gl_context_binding);
    
    // Upload intersection and ray buffers
    //LDPLAB_PROFILING_START(inner_particle_propagation_data_upload);
    //rays.index_ssbo->upload(rays.index_data);
    //rays.ray_direction_ssbo->upload(rays.ray_direction_data);
    //rays.ray_intensity_ssbo->upload(rays.ray_intensity_data);
    //rays.ray_origin_ssbo->upload(rays.ray_origin_data);
    //LDPLAB_PROFILING_STOP(inner_particle_propagation_data_upload);
    
    // Bind shaders
    LDPLAB_PROFILING_START(inner_particle_propagation_shader_binding);
    m_compute_shader->use();
    LDPLAB_PROFILING_STOP(inner_particle_propagation_shader_binding);
    
    // Bind buffers
    LDPLAB_PROFILING_START(inner_particle_propagation_ssbo_binding);
    rays.index_ssbo->bindToIndex(0);
    rays.ray_origin_ssbo->bindToIndex(1);
    rays.ray_direction_ssbo->bindToIndex(2);
    rays.ray_intensity_ssbo->bindToIndex(3);
    intersection.point_ssbo->bindToIndex(4);
    intersection.normal_ssbo->bindToIndex(5);
    output.force_per_ray_ssbo->bindToIndex(6);
    output.torque_per_ray_ssbo->bindToIndex(7);
    const RodParticleData* pd = 
        ((RodParticleData*)m_context->particle_data.get());
    const ParticleMaterialLinearOneDirectionalData* pmd =
        ((ParticleMaterialLinearOneDirectionalData*)
            m_context->particle_material_data.get());
    pd->ssbo.cylinder_radius->bindToIndex(8);
    pd->ssbo.cylinder_length->bindToIndex(9);
    pd->ssbo.sphere_radius->bindToIndex(10);
    pd->ssbo.origin_cap->bindToIndex(11);
    pd->ssbo.origin_indentation->bindToIndex(12);
    pmd->ssbo.index_of_refraction_sum_term->bindToIndex(13);
    pmd->ssbo.direction_times_gradient->bindToIndex(14);
    LDPLAB_PROFILING_STOP(inner_particle_propagation_ssbo_binding);
    
    // Execute shader
    LDPLAB_PROFILING_START(inner_particle_propagation_shader_execution);
    m_compute_shader->execute(rays.size / 64);
    LDPLAB_PROFILING_STOP(inner_particle_propagation_shader_execution);
    
    // Download intersection and output
    LDPLAB_PROFILING_START(inner_particle_propagation_data_download);
    //rays.ray_origin_ssbo->download(rays.ray_origin_data);
    //rays.ray_direction_ssbo->download(rays.ray_direction_data);
    //intersection.point_ssbo->download(intersection.point_data);
    //intersection.normal_ssbo->download(intersection.normal_data);
    output.force_per_ray_ssbo->download(output.force_per_ray_data);
    output.torque_per_ray_ssbo->download(output.torque_per_ray_data);
    LDPLAB_PROFILING_STOP(inner_particle_propagation_data_download);
    
    // Unbind gl context
    LDPLAB_PROFILING_START(inner_particle_propagation_gl_context_unbinding);
    m_context->ogl->unbindGlContext();
    gpu_lock.unlock();
    LDPLAB_PROFILING_STOP(inner_particle_propagation_gl_context_unbinding);
    
    // Gather output
    //LDPLAB_PROFILING_START(inner_particle_propagation_gather_output);
    //for (size_t i = 0; i < rays.size; ++i)
    //{
    //    if (rays.index_data[i] < 0 ||
    //        rays.index_data[i] >= m_context->particles.size())
    //        continue;
    //    output.force_data[rays.index_data[i]] += output.force_per_ray_data[i];
    //    output.torque_data[rays.index_data[i]] += output.torque_per_ray_data[i];
    //}
    //LDPLAB_PROFILING_STOP(inner_particle_propagation_gather_output);

    LDPLAB_LOG_TRACE("RTSGPU (OpenGL) context %i: Inner particle ray propagation on "\
        "buffer %i completed",
        m_context->uid,
        rays.uid);
}
